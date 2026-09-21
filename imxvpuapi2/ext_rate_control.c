/* External rate control. See the header for the rationale.
 *
 * Author: Andrii Voznytsia <andrew@auterion.com>
 *
 * No encoder types, no logging, no I/O: this is a controller that takes
 * numbers and returns a QP, so it can be replayed offline against a recorded
 * stream and compared against the reference implementation in
 * tools/rcprobe.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "imxvpuapi2_priv.h"
#include "ext_rate_control.h"


static void observe_complexity(ExtRateControl *rc, double b, int is_intra, ExtRateControlStats const *cu_stats);


/* Every tunable can be overridden from the environment, because the useful
 * ones are only found by sweeping them on the target against real footage.
 * The defaults below are the settings that came out of that sweep. */
static double env_double(char const *name, double fallback)
{
	char const *v = getenv(name);
	char *end;
	double d;

	if ((v == NULL) || (v[0] == '\0'))
		return fallback;

	d = strtod(v, &end);
	return (end == v) ? fallback : d;
}


static int env_int(char const *name, int fallback)
{
	char const *v = getenv(name);
	char *end;
	long l;

	if ((v == NULL) || (v[0] == '\0'))
		return fallback;

	l = strtol(v, &end, 0);
	return (end == v) ? fallback : (int)l;
}


int ext_rate_control_init(ExtRateControl *rc, ExtRateControlParams const *params)
{
	double fps;
	unsigned int buffer_bits;

	if ((rc == NULL) || (params == NULL))
		return -1;
	if ((params->bitrate_bps == 0) || (params->frame_rate_numerator == 0) || (params->frame_rate_denominator == 0))
		return -1;
	if ((params->frame_width < 16) || (params->frame_height < 16))
		return -1;

	memset(rc, 0, sizeof(*rc));

	fps = (double)(params->frame_rate_numerator) / (double)(params->frame_rate_denominator);
	/* 0 means one second of bitrate. A second is a lot of latency to allow
	 * and more than the smoothing gains justify - 250 to 500 ms is the range
	 * worth tuning in - but it is the one default that cannot surprise
	 * anyone: the buffer holds exactly what the link carries in a second, so
	 * a caller who has not thought about the buffer gets a bucket that
	 * scales with the rate rather than a fixed bit count that means half a
	 * second at one bitrate and two at another. */
	buffer_bits = (params->buffer_bits > 0)
	            ? params->buffer_bits
	            : params->bitrate_bps;
	rc->frame_rate = fps;
	rc->bit_per_pic = (double)(params->bitrate_bps) / fps;
	rc->bucket_cap = (double)buffer_bits;
	rc->total_blocks = (uint32_t)((params->frame_width / 8) * (params->frame_height / 8));

	/* Half the buffer. Not exposed as a setting: a picture is either small
	 * enough for the buffer to absorb or it is not, and that is answered by
	 * the buffer size the caller already gave. EXT_RC_CAP overrides it, in
	 * percent, for debugging only. */
	rc->cap_share = env_int("EXT_RC_CAP", 50) / 100.0;
	rc->keyframe_mode = (params->keyframe_mode != 0);
	rc->qp_min_inter = (int)params->qp_min_inter;
	rc->qp_max_inter = (params->qp_max_inter > 0) ? (int)params->qp_max_inter : 51;
	rc->qp_min_intra = (int)params->qp_min_intra;
	rc->qp_max_intra = (params->qp_max_intra > 0) ? (int)params->qp_max_intra : 51;
	if (rc->qp_max_inter > 51) rc->qp_max_inter = 51;
	if (rc->qp_max_intra > 51) rc->qp_max_intra = 51;
	if (rc->qp_max_inter < rc->qp_min_inter) rc->qp_max_inter = rc->qp_min_inter;
	if (rc->qp_max_intra < rc->qp_min_intra) rc->qp_max_intra = rc->qp_min_intra;

	/* How hard the bucket level pulls the target around, and the level it is
	 * held at.
	 *
	 * The bucket is the accumulated difference between what was coded and
	 * what the link drained, so this term is integral action on rate error
	 * and the setpoint is a standing queue the controller is content to
	 * carry. Both matter more than they look: the queue it settles at *is*
	 * the latency. It settles where gain * (fill - setpoint) cancels
	 * whatever the content term is asking for above budget, so a low gain
	 * buys a high equilibrium fill.
	 *
	 * At the original 0.5 and 0.15 that equilibrium was far away from the
	 * setpoint. On harbour_4cif at 600 kbps the fill settled at 0.82 of the
	 * buffer, the stream ran 5.4% over the target rate, and p99 queueing
	 * delay was 588 ms against the encoder's own rate control at 211 ms -
	 * with the frame sizes themselves innocent: drained at the rate it
	 * actually produced, that same stream measures 190 ms. The excess *was*
	 * the standing queue.
	 *
	 * 4.0 and 0.05 pull the equilibrium down to 0.21 and the rate to within
	 * 1%, and p99 to 157 ms. Measured over the classic sequences the trade
	 * is latency roughly halved for 0.06-0.63 dB, with per-picture PSNR
	 * spread no wider than the encoder's own; on the 720p aerial footage it
	 * is better on both axes (fpv 339 -> 142 ms and mountain 297 -> 123 ms
	 * at a 714 ms buffer, PSNR within 0.01 dB). Raising the gain further
	 * keeps buying a little latency for a little quality - 8.0 reaches
	 * 120 ms on harbour - so this is a chosen operating point rather than an
	 * optimum. */
	rc->gain = env_double("EXT_RC_GAIN", 4.0);
	rc->setpoint = env_double("EXT_RC_SET", 0.05);
	/* Floor and ceiling on the target, as fractions of the nominal budget.
	 * The floor bounds how far the content terms may pull the target down;
	 * at 0.25 it capped them at 4x, i.e. 7.4 QP, which pinned easy content
	 * near QP 19 no matter how anything else was tuned. */
	rc->target_min = env_double("EXT_RC_TMIN", 0.15);
	rc->target_max = env_double("EXT_RC_TMAX", 2.50);
	/* Ceiling on the very first picture, as a share of the buffer.
	 *
	 * It is the one picture with no model behind it: pre() has nothing to
	 * predict from and falls back to a fixed QP 32, and the discretionary
	 * cap in ext_rate_control_cap() exempts intra pictures, so the only
	 * thing bounding it is buffer overflow - the whole buffer. That guess
	 * suits 720p at 1400 kbps and nothing else. On 4CIF at 600 kbps it
	 * produced 253 kbit into a 429 kbit buffer, 59% of the buffer in one
	 * picture, which then takes ~250 pictures to drain and sets p99 for the
	 * entire run: 471 ms against the encoder's own rate control at 159 ms.
	 *
	 * 0.15 is the standing queue the bucket term carries anyway, so an intra
	 * picture that fits inside it cannot be what sets the delay. Measured on
	 * soccer_4cif: p99 471 -> 227 ms at the same bitrate and the same mean
	 * QP, because it is one picture in 600. The knee is at about 19% of the
	 * buffer, so this has a little margin. 0 disables it. */
	rc->first_intra_share = env_double("EXT_RC_FIRST_INTRA", 0.15);
	/* Bounds on the relative-complexity term, so one anomalous picture
	 * cannot hand the next one an unbounded budget or starve it. */
	rc->cplx_min = env_double("EXT_RC_CPLX_MIN", 0.25);
	rc->cplx_max = env_double("EXT_RC_CPLX_MAX", 4.00);
	rc->alpha = env_double("EXT_RC_ALPHA", 0.3);
	/* Repaying link capacity the encoder left unused.
	 *
	 * The bucket term above is integral action on rate error, but it is
	 * clipped at empty: once the link has drained everything, the bucket
	 * cannot record that the stream is *still* behind, so its bias saturates
	 * at 1 + gain * setpoint - 1.20 at the defaults - and a content term that
	 * asks for a quarter of the budget wins permanently. Measured on
	 * near-static VGA footage at 100 kbps: 63 kbps delivered, the buffer
	 * empty on 227 of 350 pictures, and mean fill sitting exactly on the
	 * setpoint. The controller believed it was in equilibrium while giving
	 * away a third of the link, and a constant-QP encode of the same clip
	 * showed the missing bits were worth 1.8 dB, so they were not bits
	 * nobody needed.
	 *
	 * This carries the shortfall the bucket cannot, and repays it only where
	 * repaying is free. A target *below* one frame budget is raised towards
	 * one frame budget and never above it; a picture of exactly one frame
	 * budget leaves the bucket where it found it, so repayment cannot raise
	 * buffer occupancy - mean or peak - by construction. The peaks are left
	 * to the cap. The missing bits are in the troughs anyway, and the troughs
	 * are the latency-free place to put them back.
	 *
	 * Because the debt integrates the shortfall rather than the lift it
	 * granted, it settles where the delivered rate equals the budget: repay
	 * too little and it keeps growing, too much and it shrinks. Repaying the
	 * lift instead makes it proportional action, which splits the difference
	 * and stalls halfway - measured, that reached 57% of the budget and
	 * stopped.
	 *
	 * debt_cap bounds how much idle capacity may be remembered so a long calm
	 * passage cannot bank a burst for the motion after it, and debt_gain how
	 * much is offered back per picture. Neither is critical once the gate in
	 * ext_rate_control_pre() is in place: cap 4 / gain 0.5 and cap 8 / gain
	 * 1.0 measure identically, because it is the gate and not the reservoir
	 * that decides when repayment happens.
	 *
	 * debt_fill is the one that matters. It stops a repaid target short of the
	 * full frame budget, which leaves the buffer still draining on a repaid
	 * picture and so leaves room for the QP model to overshoot - and on this
	 * content it overshoots by up to 95%. Measured over five clips, against
	 * the shipped library: at 1.0 the mean rate error goes -17.1% -> -11.7%
	 * for +2.6 ms on p95 buffer occupancy; at 0.8 it goes -17.1% -> -12.6%
	 * for +0.3 ms, which is noise, with peak occupancy 1 ms lower than the
	 * baseline's. 0.8 is the operating point because latency is the priority;
	 * raise it towards 1.0 to trade about a point of rate error per 2 ms. */
	rc->debt_cap  = env_double("EXT_RC_DEBT_CAP", 4.0) * rc->bit_per_pic;
	rc->debt_gain = env_double("EXT_RC_DEBT_GAIN", 0.5);
	/* The repayment ceiling is progressive rather than fixed.
	 *
	 * A fixed 0.8 is the right resting value - it leaves the buffer draining
	 * on a repaid picture, which is what keeps the QP model's overshoot from
	 * turning into latency. But on content the rate control genuinely cannot
	 * keep up with, 0.8 *is* the binding constraint: measured on an aerial
	 * VGA pan at 700 kbps, the mean picture landed on 0.801 frame budgets,
	 * exactly the ceiling, for a 19.9% rate shortfall.
	 *
	 * So it stretches towards debt_fill_max while the delivered rate is
	 * persistently short and relaxes back as soon as it is not. The signal is
	 * a slow EMA of the relative shortfall, not the debt: the debt is an
	 * integrator and parks wherever it must to produce the lift being asked
	 * for, so it saturates on well behaved content too and cannot tell the
	 * two apart. The rate error can.
	 *
	 * Dead band below debt_err_lo, full stretch at debt_err_hi, linear
	 * between. alpha is slow on purpose - about a 100 picture window - so one
	 * hard passage does not spend the latency margin. */
	rc->debt_fill      = env_double("EXT_RC_DEBT_FILL",      0.80);
	rc->debt_fill_max  = env_double("EXT_RC_DEBT_FILL_MAX",  0.95);
	rc->debt_err_alpha = env_double("EXT_RC_DEBT_ERR_ALPHA", 0.05);
	/* Worst band first. Thresholds are relative rate shortfall; the counts are
	 * how many consecutive pictures must agree before the step is granted.
	 *
	 * Chosen by measurement, and deliberately not the fastest set tried.
	 * Tighter thresholds and counts (0.120/0.060/0.025 with 2/5/12) do react
	 * far sooner - the ceiling reaches 0.90 by picture 8 rather than 66 - but
	 * over 36 operating points they bought 0.07 points of rate for 0.02 dB of
	 * PSNR and 0.36 ms of p95 occupancy, and they cost 0.11-0.35 dB on dense
	 * detail footage by redistributing bits badly: at 700 kbps they were worse
	 * on 130 pictures and better on 2, rescuing two pictures the slower set
	 * had starved to 3.4 kbit while shaving 130 decent ones, with the measured
	 * minimum PSNR unchanged. Fast reaction has a real case on content that
	 * changes - a cut, a camera switch, a mid-stream bitrate change - but a
	 * single-scene benchmark cannot show it, so the slower set is the default
	 * until a transient test says otherwise. The knobs below make that a
	 * one line experiment.
	 *
	 * A faster signal is not the answer either: at alpha 0.10 the ceiling
	 * slams to its maximum on the first picture and then chatters through 19
	 * steps, delivering less rate than the slowest setting here. */
	rc->debt_t[0] = env_double("EXT_RC_DEBT_T1", 0.20); rc->debt_n[0] = (unsigned)env_int("EXT_RC_DEBT_N1",  5); rc->debt_f[0] = env_double("EXT_RC_DEBT_F1", 0.95);
	rc->debt_t[1] = env_double("EXT_RC_DEBT_T2", 0.10); rc->debt_n[1] = (unsigned)env_int("EXT_RC_DEBT_N2", 15); rc->debt_f[1] = env_double("EXT_RC_DEBT_F2", 0.90);
	rc->debt_t[2] = env_double("EXT_RC_DEBT_T3", 0.05); rc->debt_n[2] = (unsigned)env_int("EXT_RC_DEBT_N3", 40); rc->debt_f[2] = env_double("EXT_RC_DEBT_F3", 0.85);

	/* The safety valve, on by default. What it prevents is unbounded
	 * latency, and a caller whose qp_max is low enough to reach it has no
	 * other protection: the re-encode ladder has already run out of rungs
	 * by then. It should not open at all on a feasible operating point -
	 * the threshold is the buffer overflow limit, which a working stream
	 * does not reach - so leaving it armed costs nothing until it is
	 * needed. EXT_RC_SKIP=0 disarms it. */
	rc->skip_enable = env_int("EXT_RC_SKIP", 1);
	/* The valve asks the same question at two points, and both are worth
	 * having.
	 *
	 * ext_rate_control_should_drop() is the rule proper: the picture has
	 * been encoded, it is at qp_max, it does not fit, so it is not sent.
	 * Nothing is forecast, so nothing can be forecast wrong.
	 *
	 * ext_rate_control_should_skip() asks before the encode, off the
	 * content model, and saves the encode when the answer is already
	 * obvious.
	 *
	 * The pre-check is off by default, because measuring turned out to be
	 * both simpler and better. The two are not interchangeable - a picture
	 * refused before the encode is never measured, so the content model does
	 * not move for it and the pictures after it are quantised differently -
	 * and on the stress clip at qp_max 44, against a 107 ms buffer:
	 *
	 *   pre-check only    165 refused   17 overflows   peak 118 ms
	 *   measured only     127 refused    0 overflows   peak 107 ms
	 *   both              152 refused    0 overflows   peak 107 ms
	 *
	 * The measured rule alone cannot overflow the buffer, which is the whole
	 * point, and adding the pre-check to it only costs 25 more dropped
	 * pictures - 0.5 fps - for nothing. The pre-check survives for the case
	 * this board is not in: if encode time is ever the binding constraint,
	 * refusing a picture before spending an encode on it is worth having.
	 *
	 * EXT_RC_SKIP_PREDICT=1 arms it; EXT_RC_SKIP_DROP=0 disarms the measured
	 * rule. That is how the table above was taken. */
	rc->skip_predict = env_int("EXT_RC_SKIP_PREDICT", 0);
	rc->skip_drop = env_int("EXT_RC_SKIP_DROP", 1);
	rc->skip_room_share = env_double("EXT_RC_SKIP_ROOM", 1.0);
	rc->cap_attempt_limit = env_int("EXT_RC_CAP_ATTEMPTS", 4);
	/* With the valve off there is nothing left to stop a picture the
	 * quantiser cannot shrink from overflowing the buffer: the ladder's
	 * give-up paths become the end of the line. That is a legitimate thing
	 * to ask for in an experiment and a bad thing to discover in the field,
	 * so it is said out loud rather than inferred from a missing knob. */
	if (!rc->skip_enable || !rc->skip_drop)
		IMX_VPU_API_WARNING(
			"new CBR: the frame skipping valve is %s (EXT_RC_SKIP=%d, EXT_RC_SKIP_DROP=%d) - "
			"the HRD buffer can no longer be guaranteed, because a picture the quantiser "
			"cannot shrink to fit has nothing left to stop it",
			rc->skip_enable ? "half disabled" : "disabled",
			rc->skip_enable, rc->skip_drop);

	/* Above 1.0 this would license the very thing the valve exists to
	 * prevent, so it is a ceiling and not a free parameter. */
	if (rc->skip_room_share > 1.0) rc->skip_room_share = 1.0;
	if (rc->skip_room_share < 0.0) rc->skip_room_share = 0.0;
	if (rc->debt_gain < 0.0) rc->debt_gain = 0.0;
	if (rc->debt_fill < 0.0) rc->debt_fill = 0.0;
	if (rc->debt_fill > 1.0) rc->debt_fill = 1.0;
	if (rc->debt_fill_max < rc->debt_fill) rc->debt_fill_max = rc->debt_fill;
	if (rc->debt_fill_max > 1.0) rc->debt_fill_max = 1.0;
	rc->debt_fill_now = rc->debt_fill;
	rc->debt_err_run = 0;

	/* QP per doubling of rate. Measured on a VC8000E by coding 250 pictures
	 * at every QP from 20 to 42: rate follows Qstep^-1.62, i.e. 3.71 QP per
	 * doubling, not the 6.00 that a plain R ~ 1/Qstep proportion assumes.
	 * Other encoders will want their own value here. */
	rc->slope = env_double("EXT_RC_SLOPE", 3.71);
	if (rc->slope < 1.0) rc->slope = 1.0;
	rc->slope_corrections = 0;
	rc->obs_n = 0;
	rc->obs_head = 0;
	rc->prev_attempt_qp = -1;
	rc->prev_attempt_bits = 0;

	rc->prev_qp = -1;
	rc->intra_bootstrap_pending = 1;

	return 0;
}


/* What the next picture is predicted to cost at the coarsest quantiser the
 * controller may use, and the room the buffer has for it.
 *
 * The room is what is left in the buffer *now*, with no credit for the
 * draining that happens while the picture is going out. That is deliberately
 * stricter than the ceiling ext_rate_control_check() re-encodes against,
 * which subtracts one frame budget of drain first and so bounds the level at
 * the *end* of the picture's period rather than the peak during it. The two
 * differ by exactly one frame budget, and for a re-encode that is the right
 * call - it is aiming a quantiser at a size, and the end-of-period level is
 * what the controller's own accounting carries forward.
 *
 * For the valve it is the wrong call, and measurably so. Replaying the stress
 * clip at 1400 kbps into a 107 ms buffer with qp_max at 44: of 344 pictures
 * that overflowed the buffer at their peak, 308 had "fitted" under the
 * end-of-period rule and only 36 were the prediction genuinely missing. The
 * question a valve has to answer is whether the picture fits in the buffer as
 * it stands, so that is the question it asks.
 *
 * Being a share of the level rather than a fixed margin also makes the test
 * adaptive in the right direction on its own: with the buffer nearly empty it
 * is barely stricter than the re-encode ceiling, and it tightens sharply as
 * the buffer fills, which is where a picture that does not fit does real
 * damage.
 *
 * Returns 0 when there is nothing to predict from, which is a reason to code
 * the picture rather than to refuse it.
 */
static int skip_pressure(ExtRateControl const *rc, int is_intra, double *est, double *room)
{
	*room = rc->bucket_cap - rc->bucket;
	if (*room < 0.0) *room = 0.0;
	*est = 0.0;

	/* There is nothing to predict an intra picture from: what an inter
	 * picture cost says nothing about one coded from scratch. Intra pictures
	 * are handled by the measured rule instead, on their real coded size,
	 * and deferred rather than dropped - see
	 * ext_rate_control_should_drop(). */
	if (is_intra)
		return 0;

	if (!rc->have_complexity_x)
		return 0;

	/* complexity_x is log2(bits) + qp/slope, i.e. what this content would
	 * cost at QP 0, so this inverts it at qp_max. */
	*est = pow(2.0, rc->complexity_x - (double)(rc->qp_max_inter) / rc->slope);
	return 1;
}


void ext_rate_control_set_forced_sync_point(ExtRateControl *rc)
{
	if (rc != NULL)
		rc->forced_sync_point = 1;
}


double ext_rate_control_room(ExtRateControl const *rc)
{
	double room;

	if (rc == NULL)
		return 0.0;

	room = rc->bucket_cap - rc->bucket;
	return (room > 0.0) ? room : 0.0;
}


int ext_rate_control_keyframe_ready(ExtRateControl const *rc)
{
	if (rc == NULL)
		return 1;

	/* Only a sync point the caller asked for waits. A periodic IDR is on the
	 * schedule the controller has been budgeting for, and the ladder bounds
	 * it; sweeping a refresh, a keyframe is one band of an ordinary picture
	 * and there is nothing to drain for either. */
	if (!rc->forced_sync_point)
		return 1;

	/* Nothing left to drain: the keyframe is being offered the whole buffer,
	 * so waiting longer cannot help and withholding it any further only
	 * delays the sync point a decoder is waiting for. Same condition the
	 * valve uses; see ext_rate_control_should_skip(). */
	if (rc->bucket <= 0.0)
		return 1;

	/* No intra picture coded yet, so no forecast. Code it and measure it -
	 * that measurement is what makes every later decision possible, and the
	 * bootstrap picture has its own ceiling (first_intra_share) anyway. */
	if (rc->last_intra_bits <= 0.0)
		return 1;

	return (ext_rate_control_room(rc) >= rc->last_intra_bits);
}


int ext_rate_control_should_skip(ExtRateControl const *rc, int is_intra)
{
	double est, room;

	if ((rc == NULL) || !rc->skip_enable || !rc->skip_predict)
		return 0;

	/* The one condition under which a picture that does not fit is sent
	 * anyway, and it is a physical one rather than a countdown.
	 *
	 * Skipping is worth doing while it can still buy something: each skipped
	 * picture drains one frame budget out of the bucket. Once the bucket is
	 * empty there is nothing left to buy - the picture is being offered the
	 * whole buffer and still does not fit, and the quantiser is already at
	 * its ceiling - so no further skipping and no coarser quantiser can
	 * change the outcome. Sending it is then the least bad option: take the
	 * overflow, and drain it back off over the pictures that follow, which
	 * this same test does on its own because room stays 0 until the bucket
	 * comes back under the cap.
	 *
	 * Liveness comes from the bucket, not from a limit: it drains a frame
	 * budget per skipped picture unconditionally, so this condition is always
	 * reached and the stream can never stall. That is why this replaced a
	 * fixed run limit, which stopped refusing after a set count whether or
	 * not anything could still be done - on an imx8mp at 1600 kbps into a
	 * 180 kbit buffer that count was 4, and it let 58 of 1653 pictures
	 * overflow, one of them 315 kbit against a 180 kbit buffer. With the
	 * drain rule the same run measured 1 of 1255, and that one was a 200
	 * kbit picture against a 180 kbit buffer, which nothing can place. */
	if (rc->bucket <= 0.0)
		return 0;

	/* Draining the buffer so a keyframe can be placed. This is the one case
	 * where an intra picture is refused before being encoded: normally there
	 * is nothing to forecast one from, but here there is - the last intra
	 * picture's measured cost - and the request stays armed, so refusing the
	 * picture defers the keyframe rather than losing it.
	 *
	 * Refusing it costs nothing and saves everything the alternative spends:
	 * an encode whose result cannot be used, and a ladder that would coarsen
	 * the one picture a decoder can start from, once per attempt, while the
	 * buffer drains underneath it. */
	if (rc->intra_drain_bits > 0.0)
		return (ext_rate_control_room(rc) < rc->intra_drain_bits);

	if (!skip_pressure(rc, is_intra, &est, &room))
		return 0;

	return (est > (room * rc->skip_room_share));
}


int ext_rate_control_should_drop(ExtRateControl const *rc, size_t bits, int is_intra)
{
	double room;

	if ((rc == NULL) || !rc->skip_enable || !rc->skip_drop)
		return 0;
	/* The one state in which a picture that does not fit has to be sent, and
	 * it takes both halves: the buffer empty, so no further skipping can make
	 * room, AND the quantiser at its ceiling, so no coarser attempt can make
	 * the picture smaller. Either alone is not enough - emitting on an empty
	 * buffer while quantiser headroom remained is what let a keyframe go out
	 * at qp 41 and 148.9 kbit into a 108 kbit buffer when qp 51 was
	 * available. ext_rate_control_check() lifts its attempt limit in exactly
	 * this situation, so the ceiling is always reachable and this stays
	 * live. */
	if ((rc->bucket <= 0.0)
	 && (rc->current_qp >= (is_intra ? rc->qp_max_intra : rc->qp_max_inter)))
		return 0;

	/* The quantiser must have no moves left, and there are two ways to run
	 * out of them. The obvious one is qp_max. The other is the re-encode
	 * ladder spending its attempts: ext_rate_control_check() deliberately
	 * accepts a picture it has failed to shrink four times over, because
	 * grinding it down further costs encode time and looks worse than the
	 * overshoot - but at that point "accept it" was the only option it had,
	 * and dropping it is the better one.
	 *
	 * That second route is not a corner case. Over the 137 standard clips,
	 * every picture that overflowed the buffer was *below* qp_max and had
	 * come out of the ladder's attempt limit - 25 of them on
	 * deadline_176x144 alone. Gating only on qp_max left the valve watching
	 * them go past.
	 *
	 * Below both limits the picture is still the ladder's problem, and
	 * taking it here would trade frame rate for something quality could
	 * have bought. */
	if (!(rc->forced_sync_point && is_intra && !rc->intra_bootstrap_pending)
	 && (rc->current_qp < (is_intra ? rc->qp_max_intra : rc->qp_max_inter))
	 && (rc->cap_attempts < rc->cap_attempt_limit))
		return 0;

	/* A requested sync point is the exception, and it has to be: such a
	 * picture is deferred on the strength of not fitting alone.
	 * ext_rate_control_check() deliberately does not coarsen it, so neither
	 * route above can ever fire for one - the quantiser never moves and no
	 * attempt is spent - and the test would hand the oversized picture
	 * straight to the buffer. Measured on an imx8mp before this was added:
	 * two of 45 keyframes went in over the ceiling and the buffer peaked at
	 * 108% of cap. The remedy for a keyframe is the drain that follows the
	 * deferral, not a coarser picture. */

	/* And it must not fit in the buffer as it stands. */
	room = rc->bucket_cap - rc->bucket;
	if (room < 0.0) room = 0.0;

	return ((double)bits > (room * rc->skip_room_share));
}


void ext_rate_control_skip(ExtRateControl *rc, int is_intra, size_t bits, ExtRateControlStats const *cu_stats)
{
	double est = 0.0, room = 0.0;

	if (rc == NULL)
		return;

	skip_pressure(rc, is_intra, &est, &room);
	if (bits > 0)
		est = (double)bits;

	IMX_VPU_API_LOG(
		"new CBR: picture %lu skipped - %.1f kbit %s at qp %d, and the HRD buffer "
		"has room for %.1f kbit (%.1f of %.1f kbit in flight, refusal %u, %u more "
		"would empty the bucket)",
		rc->num_pictures,
		est / 1000.0,
		(bits > 0) ? "coded" : "predicted",
		rc->qp_max_inter,
		room / 1000.0,
		rc->bucket / 1000.0, rc->bucket_cap / 1000.0,
		rc->skip_run + 1, (unsigned int)ceil(rc->bucket / (rc->bit_per_pic > 0.0 ? rc->bit_per_pic : 1.0))
	);

	/* The link drains over this picture's period whether or not anything
	 * was coded into it, and that is the whole mechanism. */
	rc->bucket -= rc->bit_per_pic;
	if (rc->bucket < 0.0)
	{
		rc->bucket = 0.0;
		rc->num_bucket_empty++;
	}

	/* Deliberately not fed to the debt integrator or to the standing rate
	 * error. Those exist to repay capacity the encoder left unused, and
	 * these bits were not left unused - they were refused, because the
	 * buffer could not take them. Counting them as a shortfall would have
	 * the controller raise later targets to put back exactly the bits it
	 * had just declined, refilling the buffer the skip drained. */

	/* A picture that was encoded and then discarded still measured the
	 * content, and that measurement is worth keeping even though the bits
	 * are not going anywhere: it is what the next picture's QP is predicted
	 * from, and throwing it away would mean paying for the encode twice.
	 * The bucket above is the only thing the bits must not touch. */
	if (bits > 0)
		observe_complexity(rc, (double)bits, is_intra, cu_stats);

	/* Arm or sustain the drain. An intra picture that was encoded and then
	 * discarded tells us exactly what it costs, which is a better forecast
	 * than the previous keyframe's; a picture refused before the encode
	 * leaves the existing forecast in place. Either way the request is still
	 * armed, so the drain ends with the keyframe coded. */
	if (is_intra)
	{
		int const was_draining = (rc->intra_drain_bits > 0.0);

		if (bits > 0)
			rc->intra_drain_bits = (double)bits;
		else if (!was_draining)
			rc->intra_drain_bits = rc->last_intra_bits;

		if (rc->intra_drain_bits > 0.0)
		{
			if (!was_draining)
				rc->num_intra_drains++;
			rc->num_intra_drain_pics++;
			IMX_VPU_API_LOG("new CBR:   draining for a keyframe of %.1f kbit, room %.1f kbit",
			                rc->intra_drain_bits / 1000.0,
			                ext_rate_control_room(rc) / 1000.0);
		}
	}

	/* Edge: refusing has begun. The per-picture line above stays, at a level
	 * that has to be asked for, so the default output carries the decision
	 * and not every picture it applies to.
	 *
	 * It is reported after the drain is armed above and not before: a
	 * keyframe refused ahead of its encode has no measured size, and the
	 * figure the refusal is actually waiting on is the forecast that arming
	 * has just put in intra_drain_bits. Emitted first, this line read
	 * "does not fit - 0.0 kbit" on every one of them. */
	if (rc->skip_state == 0)
	{
		rc->skip_state = 1;
		IMX_VPU_API_INFO(
			"new CBR: picture %lu does not fit - %.1f kbit against %.1f kbit of room "
			"(qp_max %d); refusing pictures to drain the buffer",
			rc->num_pictures,
			((est > 0.0) ? est : rc->intra_drain_bits) / 1000.0, room / 1000.0,
			is_intra ? rc->qp_max_intra : rc->qp_max_inter);
	}

	rc->forced_sync_point = 0;

	rc->skip_run++;
	if (rc->skip_run > rc->max_skip_run)
		rc->max_skip_run = rc->skip_run;
	rc->num_skipped++;
	if (bits > 0)
		rc->num_dropped++;
}


void ext_rate_control_set_bitrate(ExtRateControl *rc, unsigned int bitrate_bps)
{
	if ((rc == NULL) || (bitrate_bps == 0) || (rc->frame_rate <= 0.0))
		return;

	rc->bit_per_pic = (double)bitrate_bps / rc->frame_rate;
	/* Expressed in frame budgets, so it follows the new rate. The debt itself
	 * is a bit count already incurred and stays, only re-clamped. */
	rc->debt_cap = env_double("EXT_RC_DEBT_CAP", 4.0) * rc->bit_per_pic;
	if (rc->debt > rc->debt_cap) rc->debt = rc->debt_cap;

	/* bucket_cap is deliberately left alone: it is a buffer size in bits,
	 * set by whoever knows how much coded data may be in flight, and that
	 * does not change because the encoder was asked for a different rate.
	 * The buffer measured in time therefore grows as the rate falls, which
	 * is what a fixed CPB does.
	 *
	 * The complexity terms are in bits and were learned at the old rate, but
	 * they describe the content, not the budget - they stay valid. */
}


int ext_rate_control_pre(ExtRateControl *rc, int is_intra, size_t overhead_bits)
{
	double fill, scale, target, qp;

	if (rc == NULL)
		return -1;

	fill = (rc->bucket_cap > 0.0) ? (rc->bucket / rc->bucket_cap) : 0.0;
	rc->sum_fill += fill;
	if (fill > rc->max_fill) rc->max_fill = fill;

	/* Two content terms, both read off the previous picture. The relative
	 * one asks whether this stretch of video is harder or easier than the
	 * recent average; the absolute one asks how much of the picture there
	 * was to code at all. Neither has to be right - a picture that breaches
	 * the ceiling is re-encoded. */
	scale = 1.0;

	if ((rc->cplx_ema > 0.0) && (rc->cplx_prev > 0.0))
	{
		double r = rc->cplx_prev / rc->cplx_ema;
		if (r < rc->cplx_min) r = rc->cplx_min;
		if (r > rc->cplx_max) r = rc->cplx_max;
		scale *= r;
	}

	/* The bucket biases the target: full pushes it down, empty lets it up. */
	target = rc->bit_per_pic * scale * (1.0 - rc->gain * (fill - rc->setpoint));

	if (target < rc->bit_per_pic * rc->target_min) target = rc->bit_per_pic * rc->target_min;
	if (target > rc->bit_per_pic * rc->target_max) target = rc->bit_per_pic * rc->target_max;

	/* Repay idle link capacity, in the troughs only. After the clamps above,
	 * so the ceiling on a repaid target is exactly one frame budget and not
	 * something target_max could lift further.
	 *
	 * Gated on the buffer being no fuller than the setpoint. Without that
	 * gate the repayment fights the very term that bounds latency: the bucket
	 * term lowers the target as the buffer fills, and a lift that ignores it
	 * puts the bits back into a buffer that already has a queue in it.
	 * Measured on 720p aerial footage, ungated repayment moved peak occupancy
	 * from 74 ms to 99 ms for 7 points of rate; gated, the rate is the same
	 * and the peak is not. The setpoint is the standing queue the controller
	 * already accepts, so repaying below it cannot add to what the design
	 * already allows. */
	if ((rc->debt > 0.0) && (rc->debt_gain > 0.0) && (fill <= rc->setpoint))
	{
		double const fill_share = rc->debt_fill_now;
		double const fill_to = rc->bit_per_pic * fill_share;

		if (target < fill_to)
		{
			double lift = rc->debt * rc->debt_gain;
			if (lift > (fill_to - target)) lift = fill_to - target;
			target += lift;
			rc->sum_lift += lift;
			rc->sum_fill_used += fill_share;
			rc->num_debt_lifts++;
		}
	}

	/* The target so far is a budget for what goes on the link, and the
	 * caller is about to put overhead_bits of parameter sets and prefix SEI
	 * in front of the picture. Aim the picture at what is left, so the
	 * access unit lands on the target instead of overshooting it by the size
	 * of a parameter set at every GOP boundary.
	 *
	 * Charging the overhead in ext_rate_control_post() keeps the bucket
	 * honest about what the link carried, but it cannot prevent the
	 * overshoot - by then the picture has been coded. Only telling the
	 * controller in advance does that, which is what this is. On the stress
	 * clip the tax is ~800 bits on one picture in 30, so it moves that
	 * picture's QP by about a tenth of a step: the point is not the quality,
	 * it is that the budget stops being systematically wrong. */
	if (overhead_bits > 0)
	{
		target -= (double)overhead_bits;
		if (target < 1.0) target = 1.0;
	}

	rc->current_target = (int)target;

	/* complexity_x is what the stream costs at QP 0, so subtracting the
	 * target in the same log domain gives the QP that lands on it. Intra
	 * pictures are not predicted from it - their cost says nothing about
	 * what an inter picture needs and vice versa - so they simply reuse the
	 * QP of the picture before, which by then reflects the same content. */
	if (rc->have_complexity_x && !is_intra)
	{
		qp = rc->slope * (rc->complexity_x - log2(target));
		rc->current_qp = (int)lround(qp);
	}
	else
		rc->current_qp = (rc->prev_qp >= 0) ? rc->prev_qp : 32;

	{
		int const lo = is_intra ? rc->qp_min_intra : rc->qp_min_inter;
		int const hi = is_intra ? rc->qp_max_intra : rc->qp_max_inter;
		if (rc->current_qp < lo) rc->current_qp = lo;
		if (rc->current_qp > hi) rc->current_qp = hi;
	}

	rc->cap_attempts = 0;
	rc->prev_attempt_qp = -1;
	rc->prev_attempt_bits = 0;

	return rc->current_qp;
}


int ext_rate_control_target(ExtRateControl const *rc)
{
	return (rc != NULL) ? rc->current_target : 0;
}


/* How much evidence a correction needs, and how far it may move on it - both
 * scaled by how wrong the gradient looks. A gradient off by more than a factor
 * of two is unmistakable and is taken at once and in full: it is the difference
 * between hitting the target and a third of it. Half that could be the estimate
 * wandering around a value that is already right, so it waits longer and only
 * moves part of the way; below a quarter it is ignored. Measured, a clip whose
 * gradient really is 3.7 threw estimates as low as 2.6, and chasing those cost
 * 32 corrections and nothing else.
 *
 * Ratios rather than differences, because a gradient is multiplicative - being
 * 1 apart means something different at 1.5 than at 7. The window is not
 * cleared on a correction: doing that starves the cautious bands of the
 * evidence they ask for, which turned into a correction every few pictures. */
static unsigned int slope_band(double slope, double est, double *step)
{
	double const r = (est > slope) ? (est / slope) : (slope / est);

	if (r >= 2.5) { *step = 1.00; return 3; }
	if (r >= 1.6) { *step = 0.50; return 8; }
	if (r >= 1.3) { *step = 0.25; return 16; }
	*step = 0.0;
	return 0;
}


/* Bands on the standing rate shortfall: the worse and the more sustained it
 * is, the further the repayment ceiling stretches. Same graduated shape as
 * slope_band() above, and for the same reason - a big miss is unmistakable and
 * is answered quickly, a small one could be the estimate wandering and waits.
 *
 * Asymmetric on purpose: slow to stretch, immediate to relax. The margin
 * between the resting ceiling and one frame budget is latency headroom, so it
 * is given up reluctantly and taken back as soon as the rate is delivered.
 *
 * Returns the ceiling this shortfall justifies and how many consecutive
 * pictures of evidence it needs before being granted. */
static double debt_fill_band(ExtRateControl const *rc, double err, unsigned int *needed)
{
	unsigned int i;

	for (i = 0; i < 3; ++i)
	{
		if (err >= rc->debt_t[i])
		{
			*needed = rc->debt_n[i];
			return rc->debt_f[i];
		}
	}
	*needed = 0;
	return rc->debt_fill;
}


static void slope_apply(ExtRateControl *rc, double est)
{
	unsigned int needed;
	double step, moved;

	if (est < 1.0) est = 1.0;
	if (est > 8.0) est = 8.0;

	needed = slope_band(rc->slope, est, &step);
	if ((needed == 0) || (rc->obs_n < needed))
		return;

	moved = rc->slope + step * (est - rc->slope);
	if (moved < 1.0) moved = 1.0;
	if (moved > 8.0) moved = 8.0;

	IMX_VPU_API_LOG("new CBR: QP per doubling %.2f -> %.2f (measured %.2f, %.0f%% of the way, %u observations)",
	                rc->slope, moved, est, step * 100.0, rc->obs_n);
	rc->slope = moved;
	rc->slope_corrections++;
}


/* One attempt and the next at the same picture: same content, two quantisers,
 * two sizes, so the gradient between them is measured rather than assumed. */
/* The gradient measured on this picture's own two most recent attempts, or 0
 * when there is no usable pair yet. Same content at two quantisers, so this is
 * measured rather than assumed - and it is specific to the picture in hand,
 * which the learned slope cannot be. */
static double local_slope(ExtRateControl const *rc, size_t bits)
{
	double dqp, ratio, obs;

	if ((rc->prev_attempt_qp < 0) || (bits == 0) || (rc->prev_attempt_bits == 0))
		return 0.0;

	dqp = (double)(rc->current_qp) - (double)(rc->prev_attempt_qp);
	ratio = (double)(rc->prev_attempt_bits) / (double)bits;

	/* At least one quantiser step and a size that actually moved: below that
	 * the logarithm turns rounding into an arbitrary gradient. */
	if ((dqp < 1.0) || (ratio < 1.1))
		return 0.0;

	obs = dqp / log2(ratio);
	return ((obs < 0.2) || (obs > 12.0)) ? 0.0 : obs;
}


static void slope_observe(ExtRateControl *rc, size_t bits)
{
	double obs, sorted[16];
	unsigned int i, j;

	obs = local_slope(rc, bits);
	if (obs <= 0.0)
		return;

	rc->obs[rc->obs_head % 16u] = obs;
	rc->obs_head++;
	if (rc->obs_n < 16u) rc->obs_n++;

	if (rc->obs_n < 3u)
		return;

	for (i = 0; i < rc->obs_n; ++i) sorted[i] = rc->obs[i];
	for (i = 1; i < rc->obs_n; ++i)
	{
		double const v = sorted[i];
		for (j = i; (j > 0) && (sorted[j - 1] > v); --j) sorted[j] = sorted[j - 1];
		sorted[j] = v;
	}
	slope_apply(rc, sorted[rc->obs_n / 2u]);
}


int ext_rate_control_check(ExtRateControl *rc, size_t bits, size_t overhead_bits, int is_intra)
{
	double ceiling, remaining;
	double wire;
	int qp, qp_max;

	if (rc == NULL)
		return 0;
	/* The ceiling bounds what goes on the link, so it is measured against
	 * the whole access unit; the gradient below is learned from the picture
	 * alone. Adding a constant to both halves of a re-encode pair would
	 * compress their ratio and so overstate the QP per doubling, which is
	 * the one number here that is fitted rather than configured. */
	wire = (double)bits + (double)overhead_bits;
	/* The ceiling is a soft one by design. A picture the encoder simply
	 * cannot shrink gets through rather than being ground down attempt
	 * after attempt, which costs encode time and looks worse than the
	 * overshoot. */
	if (rc->cap_attempts >= rc->cap_attempt_limit)
	{
		/* The limit exists to stop a picture being ground down attempt after
		 * attempt for no gain. It must not, however, bound encode time into a
		 * guaranteed buffer breach: with the bucket already empty, a picture
		 * still larger than the whole buffer, and quantiser headroom left,
		 * accepting here overflows for certain while a coarser attempt would
		 * not. Keep going in that one case, and only that one. */
		double const room_now = (rc->bucket_cap > rc->bucket)
		                      ? (rc->bucket_cap - rc->bucket) : 0.0;
		int const qmax_now = is_intra ? rc->qp_max_intra : rc->qp_max_inter;

		if (!((rc->bucket <= 0.0) && (wire > room_now) && (rc->current_qp < qmax_now)))
		{
			IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu - attempt limit, accepted",
			                rc->cap_attempts, rc->current_qp, bits);
			return 0;
		}
		IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu - past the attempt limit, but "
		                "the buffer is empty and qp %d is still below %d, so accepting would "
		                "overflow for certain",
		                rc->cap_attempts, rc->current_qp, bits, rc->current_qp, qmax_now);
	}

	slope_observe(rc, bits);

	qp_max = is_intra ? rc->qp_max_intra : rc->qp_max_inter;
	if (rc->current_qp >= qp_max)
	{
		IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu - at qp_max, accepted",
		                rc->cap_attempts, rc->current_qp, bits);
		/* Edge: the ladder is out of moves. Worth one line, because from here
		 * the only remaining lever is refusing pictures. */
		if (!rc->qp_max_reported)
		{
			double const room = (rc->bucket_cap > rc->bucket)
			                  ? (rc->bucket_cap - rc->bucket) : 0.0;
			rc->qp_max_reported = 1;
			IMX_VPU_API_INFO(
				"new CBR: quantiser at its ceiling (qp %d) on picture %lu - %.1f kbit "
				"against %.1f kbit of room; the valve is the only lever left",
				rc->current_qp, rc->num_pictures, wire / 1000.0, room / 1000.0);
		}
		return 0;
	}

	/* What the buffer can still take: what is left in it right now. A
	 * picture bigger than this overflows the buffer, which is the one thing
	 * that is never allowed - so unlike the discretionary cap below, this
	 * applies to intra pictures too.
	 *
	 * "Right now", with no credit for the frame budget the link drains while
	 * the picture is going out. Crediting it was the original form, and it
	 * bounds the level at the *end* of the picture's period rather than its
	 * peak during it - which lets through a picture that overflows the
	 * buffer by up to one frame budget. Measured on the stress clip in IDR
	 * mode at qp_max 44, that was 44 of 46 buffer overflows: every one under
	 * a frame budget over, and every one with quantiser headroom left, so
	 * the ladder had the means to shrink them and simply was not asking.
	 *
	 * It is also the same line ext_rate_control_should_drop() measures
	 * against, which is what makes the two agree: the ladder shrinks a
	 * picture until it fits the buffer, and the valve is reached only when
	 * the quantiser has run out of ways to do that. */
	remaining = rc->bucket_cap - rc->bucket;
	if (remaining < 0.0) remaining = 0.0;
	ceiling = remaining;

	/* The discretionary cap, a share of the whole buffer, applies to inter
	 * pictures only - an intra picture carries a whole refresh and is held
	 * to the overflow limit alone. */
	if (!is_intra && !rc->forced_sync_point && (rc->cap_share > 0.0))
	{
		double const soft = rc->bucket_cap * rc->cap_share;
		if (soft < ceiling) ceiling = soft;
	}

	/* Except one with no model behind it, which would otherwise be bounded
	 * only by the whole buffer. Only that one: every later intra picture is
	 * predicted from a complexity estimate that exists by then, and holding
	 * those to a fraction of the buffer would be the refresh-starving
	 * mistake this cap is not trying to make.
	 *
	 * That is the first picture of a stream, and also the first picture at a
	 * new resolution - nothing coded before it says what a picture of that
	 * size costs, and the caller re-arms the flag when it continues a stream
	 * into a differently sized one. Measured on a 720p to 1080p switch at a
	 * 100 ms buffer: without this the switch picture takes 0.91 of the
	 * buffer on its own, with it 0.16. */
	if (is_intra && rc->intra_bootstrap_pending && (rc->first_intra_share > 0.0))
	{
		double const first = rc->bucket_cap * rc->first_intra_share;
		if (first < ceiling) ceiling = first;
	}

	if (wire <= ceiling)
	{
		IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu ceiling %.0f - accepted",
		                rc->cap_attempts, rc->current_qp, bits, ceiling);
		return 0;
	}

	/* A sync point the caller asked for out of band, and the buffer has room
	 * left to drain. Do not coarsen it yet.
	 *
	 * Only a requested one. A periodic IDR arrives on a schedule the rate
	 * control has been budgeting for all along, and the ordinary ladder
	 * bounds it perfectly well - measured on the clip corpus, holding
	 * periodic IDRs to the ladder gives 53 overflows and a largest picture
	 * of 6.5x the frame budget, while deferring their coarsening the way a
	 * requested one is deferred gives 459 overflows and 8.4x, for no
	 * measurable quality gain: on the clips where neither arm dropped a
	 * picture, PSNR was 34.97 dB against 34.94. A requested sync point is
	 * different because its timing is not the encoder's to choose, so
	 * coarsening it would be a quality pulse imposed by whatever the buffer
	 * happened to hold when a viewer joined.
	 *
	 * The ladder's only lever on an intra picture is its quantiser, and the
	 * buffer it has to fit into refills every picture, so a keyframe that
	 * arrives when the buffer is nearly full gets ground down hard - and the
	 * picture a decoder starts from is the worst possible one to spend
	 * quality on. Worse, the amount of grinding depends on how full the
	 * buffer happened to be, so consecutive keyframes come out at visibly
	 * different quality: a pulse once per GOP.
	 *
	 * Accepting the attempt as-is hands it to ext_rate_control_should_drop(),
	 * which defers it, and the drain then makes room for it at the quantiser
	 * it was predicted at. The bootstrap picture is excluded: nothing has
	 * been coded yet, so there is no drain forecast to wait for and its own
	 * first_intra_share ceiling is what bounds it. */
	if (rc->forced_sync_point && is_intra && !rc->intra_bootstrap_pending
	 && rc->skip_enable && rc->skip_drop
	 /* Only while draining can still achieve something. Once the bucket is
	  * empty the picture is being offered the whole buffer and still does
	  * not fit, so it is genuinely too big rather than merely unlucky in its
	  * timing, and coarsening is then the right remedy instead of a quality
	  * pulse imposed by a transient. Letting the ladder run only from here
	  * is what bounds the picture: skipping alone cannot, because no amount
	  * of draining places a picture larger than the buffer.
	  *
	  * Measured on the clip corpus in IDR mode with this condition absent -
	  * i.e. never coarsening a keyframe - against the shipping library:
	  * 5309 overflows against 53, p95 occupancy 836 kbit against 336, peak
	  * 1735 ms against 146, 1130 pictures dropped against 203, and mean
	  * PSNR 22.89 dB against 31.54. The largest picture reached 25x the
	  * frame budget into a buffer holding 3.3 of them. */
	 && (rc->bucket > 0.0))
	{
		IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu ceiling %.0f - keyframe over by "
		                "%.2fx, not coarsened; draining the buffer for it instead",
		                rc->cap_attempts, rc->current_qp, bits, ceiling, wire / ceiling);
		return 0;
	}

	/* With no room left at all there is nothing to aim at, so go straight
	 * to the coarsest quantiser allowed and let the guard end it. */
	if (ceiling < 1.0)
		qp = qp_max;
	else
	{
		/* Steer by this picture's own gradient once it has one, and fall back
		 * on the learned slope only for the first step, where nothing has
		 * been measured yet.
		 *
		 * The learned slope is a median over a window that mixes intra and
		 * inter pictures, and their gradients differ by roughly threefold, so
		 * on an intra picture it understates the step badly. Measured on
		 * oldtown_720 picture 50 at a 108 kbit buffer: the gradient between
		 * the picture's own first two attempts was 6.36 QP per doubling while
		 * the learned slope was 2.02, so every step was a third of the size
		 * it needed - qp 33, 36, 38, 40, 41 - and the ladder spent its
		 * attempt limit at qp 41 and 148.9 kbit with qp 51 still available
		 * and the buffer overflowing as a result. At the measured gradient
		 * the second step lands on qp 45, which fits. */
		double const g = local_slope(rc, bits);

		qp = rc->current_qp + (int)lround((g > 0.0 ? g : rc->slope) * log2(wire / ceiling));
		if (qp <= rc->current_qp) qp = rc->current_qp + 1;
		if (qp > qp_max) qp = qp_max;
	}

	IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu ceiling %.0f - over by %.2fx, re-encoding at qp %d",
	                rc->cap_attempts, rc->current_qp, bits, ceiling, wire / ceiling, qp);

	rc->prev_attempt_qp = rc->current_qp;
	rc->prev_attempt_bits = bits;

	rc->current_qp = qp;
	rc->cap_attempts++;
	rc->num_reencodes++;

	return qp;
}


/* What this picture says about the content, which is independent of whether
 * the picture is going to be sent. Called for every picture the encoder
 * produced, including one the valve then discards. */
static void observe_complexity(ExtRateControl *rc, double b, int is_intra, ExtRateControlStats const *cu_stats)
{
	if ((b > 0.0) && !is_intra)
	{
		double coded = (double)(rc->total_blocks);
		double per_block, x;

		if ((cu_stats != NULL) && (cu_stats->total_blocks > cu_stats->skip_blocks))
			coded = (double)(cu_stats->total_blocks - cu_stats->skip_blocks);
		if (coded < 1.0)
			coded = 1.0;

		/* Cost of one coded block at Qstep 1. The next picture's budget is
		 * that times the blocks it is expected to code, which separates
		 * "this content is expensive" from "there is a lot of it" - the
		 * two move independently, and a single per-picture complexity
		 * figure cannot tell them apart. */
		per_block = b * pow(2.0, ((double)(rc->current_qp) - 4.0) / 6.0) / coded;
		rc->cplx_per_block = (rc->cplx_per_block > 0.0)
		                   ? (rc->cplx_per_block + rc->alpha * (per_block - rc->cplx_per_block))
		                   : per_block;
		rc->cplx_prev = rc->cplx_per_block * coded;
		rc->cplx_ema = (rc->cplx_ema > 0.0)
		             ? (rc->cplx_ema + rc->alpha * (rc->cplx_prev - rc->cplx_ema))
		             : rc->cplx_prev;

		/* Predicting QP from one previous picture amplifies every
		 * fluctuation, so this is smoothed too. */
		x = log2(b) + (double)(rc->current_qp) / rc->slope;
		if (!rc->have_complexity_x)
		{
			rc->complexity_x = x;
			rc->have_complexity_x = 1;
		}
		else
			rc->complexity_x += rc->alpha * (x - rc->complexity_x);
	}
}


void ext_rate_control_post(ExtRateControl *rc, size_t bits, size_t overhead_bits, int is_intra, ExtRateControlStats const *cu_stats)
{
	/* Two different sizes, and they are not interchangeable.
	 *
	 * The content model is told what the *picture* cost, because that is
	 * what a quantiser has any influence over: charging it for a parameter
	 * set would teach it that a picture carrying one is more expensive
	 * content than it is.
	 *
	 * The bucket is charged the whole access unit - picture, parameter sets,
	 * prefix SEI - because the bucket models the link, and the link carries
	 * all of it. Leaving the overhead out is not the small rounding error it
	 * looks like: measured on the stress clip it averages only 3.3 bytes a
	 * picture, but it arrives in bursts of about 800 bits at each GOP
	 * boundary, and until the buffer next drains to empty the bucket sits
	 * that much below the truth. That is enough to overflow a picture the
	 * controller had correctly judged to fit, which is how a frame skipping
	 * valve that provably cannot overflow the buffer was still measured
	 * overflowing it 24 times. */
	double b = (double)bits;
	double const wire = (double)bits + (double)overhead_bits;

	if (rc == NULL)
		return;

	observe_complexity(rc, b, is_intra, cu_stats);

	/* The forecast for the next keyframe, and the end of any drain that was
	 * waiting for this one. Recorded from the picture rather than the whole
	 * access unit, to match what ext_rate_control_check() measures a
	 * re-encode against; the parameter sets an IDR carries are accounted
	 * separately by the caller and are small next to the picture. */
	if (is_intra)
		rc->last_intra_bits = b;
	rc->intra_drain_bits = 0.0;
	rc->forced_sync_point = 0;

	b = wire;

	/* Integrate the rate error before the bucket clips it away. Positive
	 * means the link was ready to carry bits the encoder did not produce. */
	rc->debt += rc->bit_per_pic - b;
	if (rc->debt < 0.0) rc->debt = 0.0;
	if (rc->debt > rc->debt_cap) rc->debt = rc->debt_cap;

	/* Standing relative shortfall, the signal the repayment ceiling follows.
	 * Signed, so running over target pulls it back down again. */
	if (rc->bit_per_pic > 0.0)
	{
		double const e = (rc->bit_per_pic - b) / rc->bit_per_pic;
		unsigned int needed;
		double cand;

		rc->rate_err_ema += rc->debt_err_alpha * (e - rc->rate_err_ema);

		cand = debt_fill_band(rc, rc->rate_err_ema, &needed);
		if (cand < rc->debt_fill) cand = rc->debt_fill;
		if (cand > rc->debt_fill_max) cand = rc->debt_fill_max;

		if (cand > rc->debt_fill_now)
		{
			rc->debt_err_run++;
			if (rc->debt_err_run >= needed)
			{
				IMX_VPU_API_LOG("new CBR: picture %lu rate short by %.1f%%, repayment ceiling %.2f -> %.2f",
				                rc->num_pictures, rc->rate_err_ema * 100.0, rc->debt_fill_now, cand);
				rc->debt_fill_now = cand;
				rc->debt_err_run = 0;
			}
		}
		else
		{
			if (cand < rc->debt_fill_now)
				rc->debt_fill_now = cand;      /* rate is back: relax at once */
			rc->debt_err_run = 0;
		}
	}

	{
		/* Whether the rule was honoured is decided here, because here is where
		 * the bucket learns what the picture cost. Logged once per episode -
		 * on the transition into overflow, not on every picture that stays
		 * there - so a stream at a hard operating point reports each event
		 * rather than a line per frame. */
		double const room_before = (rc->bucket_cap > rc->bucket)
		                         ? (rc->bucket_cap - rc->bucket) : 0.0;
		int const was_empty = (rc->bucket <= 0.0);
		int const was_over = (rc->bucket > rc->bucket_cap);

		rc->bucket += b - rc->bit_per_pic;
		if (rc->bucket < 0.0)
		{
			rc->bucket = 0.0;
			rc->num_bucket_empty++;
		}

		if (rc->bucket > rc->bucket_cap)
		{
			double const fill = rc->bucket / rc->bucket_cap;

			rc->num_overflows++;
			if (was_empty)
				rc->num_forced_overflows++;
			if (fill > rc->max_overflow_fill)
				rc->max_overflow_fill = fill;

			if (!was_over)
			{
				rc->skip_state = 2;
				if (was_empty)
					/* ERROR, not WARNING: the library's default threshold is
					 * ERROR, and a breached buffer is the one thing an
					 * operator must not have to opt in to see. Rare by
					 * construction - once per hard operating point - so it
					 * cannot spam. The text, not the level, says whether the
					 * rule held. */
					IMX_VPU_API_ERROR(
						"new CBR: picture %lu of %.1f kbit does not fit even an empty "
						"%.1f kbit HRD buffer, so it was sent rather than withheld "
						"further; buffer now %.0f%% of cap and pictures will be refused "
						"until it drains",
						rc->num_pictures, b / 1000.0, rc->bucket_cap / 1000.0,
						fill * 100.0);
				else
					IMX_VPU_API_ERROR(
						"new CBR: HRD buffer overflowed to %.0f%% of cap on picture %lu "
						"- %.1f kbit into %.1f kbit of room, with the buffer not empty, "
						"so something could still have been done. The no-overflow rule "
						"was broken; this is a bug, not a hard operating point",
						fill * 100.0, rc->num_pictures, b / 1000.0, room_before / 1000.0);
			}
		}
	}
	rc->prev_qp = rc->current_qp;
	rc->num_pictures++;
	rc->sum_bits += b;
	/* A picture got through, so the run of refusals is over. Counting runs
	 * rather than a total is what keeps the valve from starving the stream:
	 * the limit is on consecutive skips, and one coded picture clears it. */
	/* Edge: back to normal. Either the picture fit again or the buffer has
	 * drained back under its cap - both mean the excursion is over, and it is
	 * the one edge that says the stream recovered rather than that it is
	 * still in trouble. */
	if ((rc->skip_state != 0) && (rc->bucket <= rc->bucket_cap))
	{
		IMX_VPU_API_INFO(
			"new CBR: picture %lu fits again - %.1f kbit with the buffer at %.0f%% of "
			"cap after %u refusals; no longer skipping",
			rc->num_pictures, b / 1000.0, rc->bucket * 100.0 / rc->bucket_cap,
			rc->skip_run);
		rc->skip_state = 0;
	}
	if (rc->current_qp < (is_intra ? rc->qp_max_intra : rc->qp_max_inter))
		rc->qp_max_reported = 0;

	rc->skip_run = 0;
	if (is_intra)
		rc->intra_bootstrap_pending = 0;
}
