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


void ext_rate_control_set_bitrate(ExtRateControl *rc, unsigned int bitrate_bps)
{
	if ((rc == NULL) || (bitrate_bps == 0) || (rc->frame_rate <= 0.0))
		return;

	rc->bit_per_pic = (double)bitrate_bps / rc->frame_rate;

	/* bucket_cap is deliberately left alone: it is a buffer size in bits,
	 * set by whoever knows how much coded data may be in flight, and that
	 * does not change because the encoder was asked for a different rate.
	 * The buffer measured in time therefore grows as the rate falls, which
	 * is what a fixed CPB does.
	 *
	 * The complexity terms are in bits and were learned at the old rate, but
	 * they describe the content, not the budget - they stay valid. */
}


int ext_rate_control_pre(ExtRateControl *rc, int is_intra)
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
static void slope_observe(ExtRateControl *rc, size_t bits)
{
	double dqp, ratio, obs, sorted[16];
	unsigned int i, j;

	if ((rc->prev_attempt_qp < 0) || (bits == 0) || (rc->prev_attempt_bits == 0))
		return;

	dqp = (double)(rc->current_qp) - (double)(rc->prev_attempt_qp);
	ratio = (double)(rc->prev_attempt_bits) / (double)bits;

	/* At least one quantiser step and a size that actually moved: below that
	 * the logarithm turns rounding into an arbitrary gradient. */
	if ((dqp < 1.0) || (ratio < 1.1))
		return;

	obs = dqp / log2(ratio);
	if ((obs < 0.2) || (obs > 12.0))
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


int ext_rate_control_check(ExtRateControl *rc, size_t bits, int is_intra)
{
	double ceiling, drained, remaining;
	int qp, qp_max;

	if (rc == NULL)
		return 0;
	/* The ceiling is a soft one by design. A picture the encoder simply
	 * cannot shrink gets through rather than being ground down attempt
	 * after attempt, which costs encode time and looks worse than the
	 * overshoot. */
	if (rc->cap_attempts >= 4)
	{
		IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu - attempt limit, accepted",
		                rc->cap_attempts, rc->current_qp, bits);
		return 0;
	}

	slope_observe(rc, bits);

	qp_max = is_intra ? rc->qp_max_intra : rc->qp_max_inter;
	if (rc->current_qp >= qp_max)
	{
		IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu - at qp_max, accepted",
		                rc->cap_attempts, rc->current_qp, bits);
		return 0;
	}

	/* What the buffer can still take. The link drains one frame budget
	 * over this picture's period, so the space available when the picture
	 * goes in is the level left after that drain. A picture bigger than
	 * this overflows the buffer, which is the one thing that is never
	 * allowed - so unlike the discretionary cap below, this applies to
	 * intra pictures too. */
	drained = rc->bucket - rc->bit_per_pic;
	if (drained < 0.0) drained = 0.0;
	remaining = rc->bucket_cap - drained;
	if (remaining < 0.0) remaining = 0.0;
	ceiling = remaining;

	/* The discretionary cap, a share of the whole buffer, applies to inter
	 * pictures only - an intra picture carries a whole refresh and is held
	 * to the overflow limit alone. */
	if (!is_intra && (rc->cap_share > 0.0))
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

	if ((double)bits <= ceiling)
	{
		IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu ceiling %.0f - accepted",
		                rc->cap_attempts, rc->current_qp, bits, ceiling);
		return 0;
	}

	/* With no room left at all there is nothing to aim at, so go straight
	 * to the coarsest quantiser allowed and let the guard end it. */
	if (ceiling < 1.0)
		qp = qp_max;
	else
	{
		qp = rc->current_qp + (int)lround(rc->slope * log2((double)bits / ceiling));
		if (qp <= rc->current_qp) qp = rc->current_qp + 1;
		if (qp > qp_max) qp = qp_max;
	}

	IMX_VPU_API_LOG("new CBR:   attempt %d qp %d bits %zu ceiling %.0f - over by %.2fx, re-encoding at qp %d",
	                rc->cap_attempts, rc->current_qp, bits, ceiling, (double)bits / ceiling, qp);

	rc->prev_attempt_qp = rc->current_qp;
	rc->prev_attempt_bits = bits;

	rc->current_qp = qp;
	rc->cap_attempts++;
	rc->num_reencodes++;

	return qp;
}


void ext_rate_control_post(ExtRateControl *rc, size_t bits, int is_intra, ExtRateControlStats const *cu_stats)
{
	double b = (double)bits;

	if (rc == NULL)
		return;

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

	rc->bucket += b - rc->bit_per_pic;
	if (rc->bucket < 0.0)
	{
		rc->bucket = 0.0;
		rc->num_bucket_empty++;
	}
	rc->prev_qp = rc->current_qp;
	rc->num_pictures++;
	rc->sum_bits += b;
	if (is_intra)
		rc->intra_bootstrap_pending = 0;
}
