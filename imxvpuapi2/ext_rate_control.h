/* External rate control: picks a QP per picture from outside the encoder.
 *
 * Author: Andrii Voznytsia <andrew@auterion.com>
 *
 * Nothing here is specific to a codec or to a particular encoder. It needs a
 * bitrate, a frame rate, a frame size, the coded size of each picture and a
 * count of how many blocks that picture actually had to code; it returns a QP.
 * The defaults were measured on a VC8000E and are called out as such where it
 * matters.
 *
 * One naming note, because two words for one thing has caused confusion: the
 * "leaky bucket" below is what everything user facing - the hrd-buffer-size
 * property, the logs, the documentation - calls the HRD buffer. They are the
 * same buffer. This file keeps the metaphor because the code models a bucket
 * that fills and drains; anything a user reads says HRD buffer, and says it
 * in kbits so it can be compared against a bitrate in kbps.
 *
 * The built-in rate control it replaces aims a quadratic R-Q model at a bit
 * budget every picture. That has two costs on a buffer-constrained radio
 * link: the model misses by ~30% on a single shot, and forcing every picture
 * to the same size crushes the hard ones, which is visible as flicker. This
 * one instead
 *
 *   - budgets by content, from what the previous picture cost per block it
 *     actually had to code, so calm scenes ask for less rather than spending
 *     a quota they do not need,
 *   - predicts QP once from the linear Qstep relationship measured on the
 *     stream so far, and lets the size land where it lands,
 *   - absorbs the error in a leaky bucket whose level biases the next
 *     target, so a miss is repaid over several pictures instead of out of
 *     the very next one,
 *   - and only re-encodes a picture that breaches a hard ceiling, which is
 *     the one case that actually threatens the link.
 *
 * The result adapts across roughly a 3x range of scene difficulty with no
 * retuning, which is what a drone that hovers and then does aerobatics
 * needs.
 *
 * This module holds no encoder state and does no I/O. The caller drives it:
 *
 *     qp = ext_rate_control_pre(rc, is_intra);
 *     ... encode at qp ...
 *     while ((qp = ext_rate_control_check(rc, bits, is_intra)) > 0)
 *         ... re-encode at qp, update bits ...
 *     ext_rate_control_post(rc, bits, is_intra, &cu_stats);
 */
#ifndef EXT_RATE_CONTROL_H
#define EXT_RATE_CONTROL_H

#include <stddef.h>
#include <stdint.h>



#ifdef __cplusplus
extern "C" {
#endif

/* Per-picture block statistics the controller budgets against. Counts are in
 * whatever fixed-size block the encoder reports, so long as the unit is
 * consistent: only ratios of these are used. */
typedef struct
{
	uint32_t total_blocks;
	/* Blocks the encoder had to code as intra. On a predicted picture a high
	 * share means prediction did not hold - a cut, or a region an intra
	 * refresh has not reached yet. */
	uint32_t intra_blocks;
	/* Blocks that cost nothing beyond their signalling. total - skip is what
	 * the picture actually had to code, and is what the budget is set
	 * against. */
	uint32_t skip_blocks;
	/* An encoder-reported cost figure for the picture, if it has one.
	 * Recorded but not yet used. */
	uint32_t rd_cost;
}
ExtRateControlStats;



typedef struct
{
	/* Link rate the bucket drains at, in bits per second. This is a ceiling,
	 * not a quota: the rate control does not try to spend it. */
	unsigned int bitrate_bps;
	unsigned int frame_rate_numerator, frame_rate_denominator;
	unsigned int frame_width, frame_height;

	/* Leaky bucket size in bits - the coded data that may be in flight
	 * before the link has drained it, i.e. exactly what a CPB/HRD buffer
	 * size describes. 0 selects 100 ms of bitrate_bps. */
	unsigned int buffer_bits;
	/* QP range the rate control may use, separately for intra and inter
	 * pictures. A maximum of 0 means "let the codec decide", i.e. 51 - no
	 * ceiling. A minimum of 0 is taken as written, and is worth having:
	 * easy content at a generous bitrate genuinely needs the bottom of the
	 * range, and an arbitrary floor pins it there and starves the stream. */
	unsigned int qp_min_inter, qp_max_inter;
	unsigned int qp_min_intra, qp_max_intra;

	/* Non-zero when the encoder places whole intra pictures - a periodic or
	 * requested IDR - and zero when it sweeps an intra refresh instead.
	 *
	 * It changes what happens to an intra picture the buffer cannot take.
	 * Sweeping, every picture carries a band and no single one is large, so
	 * the ordinary ladder applies. Placing whole intra pictures, the
	 * ladder's only move is to coarsen the one picture a decoder can start
	 * from, and coarsening it repeatedly as the buffer slowly drains is
	 * visible as a quality pulse on every keyframe. So in that mode the
	 * controller drains the buffer first and codes the picture after -
	 * see ext_rate_control_keyframe_ready(). */
	unsigned int keyframe_mode;

	/* How far the quantiser may fall in one picture, relative to the last
	 * picture actually coded. 0 means unbounded, which is the right setting
	 * wherever an oversized picture can be re-encoded.
	 *
	 * It exists for the encoders where it cannot. The QP comes from
	 * inverting an exponential model, so an error in the model is an error
	 * in the exponent: on vtc1nw_720x480 the complexity EMA sat at 19.52
	 * between a cheap mode at 18.7 and an expensive one at 21.8, asked for
	 * qp 17 ten steps below the last measured picture, predicted 31 kbit and
	 * got 149 - 4.8x, which is 2^2.25 of model error. Bounding the step
	 * bounds the extrapolation, and with it the size of the mistake a single
	 * picture can make. Raising the quantiser stays unbounded: that
	 * direction makes pictures smaller, and refusing to let the controller
	 * retreat quickly is what would cause the overflow.
	 *
	 * Set on the H1, where a predicted picture is committed the moment
	 * H264EncStrmEncode() returns and the model error reaches the wire. Left
	 * at 0 on the VC8000E, whose ladder re-encodes the mistake away. */
	unsigned int qp_down_step;
}
ExtRateControlParams;


typedef struct
{
	/* --- configuration, derived from ExtRateControlParams --- */
	double frame_rate;
	double bit_per_pic;
	double bucket_cap;
	uint32_t total_blocks;
	/* Per-picture ceiling, as a share of the buffer: a picture that would
	 * take more than this much of the buffer on its own is re-encoded
	 * coarser. Tied to the buffer rather than to the frame budget because
	 * what a single oversized picture threatens is the buffer, and a
	 * caller who sizes the buffer has already said how much may be in
	 * flight. Never applied to intra pictures, which carry a whole refresh.
	 * Debug override only - see EXT_RC_CAP in ext_rate_control_init(). */
	double cap_share;
	int qp_min_inter, qp_max_inter;
	int qp_min_intra, qp_max_intra;
	int keyframe_mode;
	/* Largest fall in QP allowed from one coded picture to the next; 0 is
	 * unbounded. See ExtRateControlParams. */
	int qp_down_step;

	/* --- tuning; see ext_rate_control_init() for what each one does --- */
	double gain, setpoint, target_min, target_max;
	/* Ceiling on the bootstrap intra picture, as a share of the buffer. See
	 * ext_rate_control_init(). */
	double first_intra_share;
	double cplx_min, cplx_max;
	double alpha;
	/* QP per doubling of picture size: how far the quantiser has to move to
	 * halve the bits. Not a constant of the codec - it is the local gradient,
	 * and it depends on how much of the picture is being coded at all.
	 * Measured 3.7 on a clip alternating stills with motion and 1.7 on
	 * uniformly detailed footage, so this starts at a default and is then
	 * corrected from what the encoder actually does; see slope_band(). */
	double slope;
	unsigned int slope_corrections;
	/* Observations of the gradient, each from one re-encode: two quantisers
	 * and two sizes for the same picture, so only QP differs. Per-picture
	 * samples cannot substitute - normalising them by coded blocks removes
	 * part of the QP dependence being measured, because a coarser quantiser
	 * skips more blocks, and the fit then runs to the clamp.
	 *
	 * Kept as a window and reduced by median rather than averaged: single
	 * observations ranged 0.98 to 6.01 on a clip whose gradient is 1.7, and
	 * an EMA of that made the correction hunt between 1.2 and 2.4. */
	double obs[16];
	unsigned int obs_n, obs_head;
	/* The previous attempt at this picture - the other half of a pair. */
	int prev_attempt_qp;
	size_t prev_attempt_bits;

	/* Repayment of link capacity the encoder left unused. See
	 * ext_rate_control_init() for why the bucket alone cannot do this.
	 * debt_cap bounds how much idle capacity may be remembered; debt_gain is
	 * the share of the outstanding debt offered back per picture; debt_fill
	 * is how far towards one frame budget a repaid target may be raised. */
	double debt_cap, debt_gain, debt_fill, debt_fill_max, debt_err_alpha;
	/* The three shortfall bands: entry threshold, consecutive pictures of
	 * evidence required, and the ceiling each grants. Tunable because the
	 * right reaction speed depends on how fast the content changes. */
	double debt_t[3], debt_f[3];
	unsigned int debt_n[3];

	/* --- the frame skipping valve --- */
	/* The re-encode ladder in ext_rate_control_check() ends at qp_max, and
	 * past that rung the controller has no authority left. If the content
	 * still needs more bits than the buffer can take, every further picture
	 * goes in on top of a buffer that is already full and the queue - which
	 * is the latency - grows without bound.
	 *
	 * That failure is invisible in a mean bitrate. Measured on the
	 * still/motion stress clip at 1400 kbps into a 107 ms buffer with
	 * qp_max pinned at its own median of 44: the stream came out at -4.5%
	 * of target, which looks healthy, while p95 buffer occupancy was
	 * 1025 ms and 619 of 1320 pictures overflowed. The bits were not too
	 * many, they were in the wrong place in time.
	 *
	 * So the last rung is to not code the picture at all. This is a bypass
	 * valve and not a rate control tool: it trades frame rate for latency
	 * at the point where there is nothing left to trade quality for, and it
	 * is meant never to open on an operating point that is actually
	 * feasible.
	 *
	 * skip_enable turns the valve off. skip_room_share scales its threshold
	 * in units of the room the buffer has left, so below 1.0 the valve opens
	 * before a picture would strictly have overflowed; it is clamped at 1.0,
	 * because above that it would license the overflow it exists to prevent.
	 *
	 * The run is not bounded by a count. A picture is refused while refusing
	 * it can still achieve something - each refusal drains a frame budget -
	 * and sent once the bucket is empty, because the whole buffer is then on
	 * offer and the quantiser is at its ceiling, so nothing further can be
	 * done. The bucket drains unconditionally, so that condition is always
	 * reached and the stream cannot stall.
	 *
	 * A fixed cap on the run was tried and removed: it cannot be set
	 * correctly. Too low and a buffer hovering just over its cap refuses
	 * most pictures indefinitely without ever draining, which measured worse
	 * on both axes at once - p95 buffer occupancy 352 ms against 238 ms at
	 * 6000 kbps on the stress clip, while dropping *more* pictures, 356
	 * against 347. One longer run drains the buffer and stops; a capped one
	 * never finishes the job. Draining to empty is that observation taken to
	 * its conclusion. */
	int skip_enable;
	/* The valve's two decision points, separately disableable. Both are on
	 * by default and both change which pictures are refused - see
	 * ext_rate_control_init(). */
	int skip_predict, skip_drop;
	double skip_room_share;
	/* How many times ext_rate_control_check() may re-encode one picture
	 * before accepting whatever it has. Shared with the valve, which treats
	 * reaching it as the quantiser having run out of moves. */
	int cap_attempt_limit;

	/* --- state --- */
	/* Bits handed to the link that it has not drained yet. */
	double bucket;
	/* Integral of the rate shortfall: bits the link was ready to carry that
	 * the encoder did not produce. One sided - it remembers falling behind,
	 * not running ahead - and bounded by debt_cap. */
	double debt;
	/* Slow EMA of the relative rate shortfall, (budget - coded) / budget.
	 * Near zero means the rate is being delivered. This is what decides how
	 * far the repayment ceiling stretches; it is deliberately slower than the
	 * debt so a single hard passage does not stretch it. */
	double rate_err_ema;
	/* The repayment ceiling currently in force, and how many consecutive
	 * pictures of evidence have accumulated for the next step up. */
	double debt_fill_now;
	unsigned int debt_err_run;
	/* Pictures the valve has refused back to back. Reset by the next
	 * picture that is coded. */
	unsigned int skip_run;

	/* --- placing a keyframe into a buffer that is too full --- */
	/* What the last coded intra picture cost, in bits. This is the forecast
	 * for the next one: an intra picture has no content model of its own to
	 * be predicted from - that is why the predictive valve never refuses
	 * one - but the previous intra picture of the same stream is a far
	 * better estimate than nothing, and it is the only one available before
	 * the encode.
	 *
	 * Deciding before the encode is not forced - both encoders can redo a
	 * keyframe - but it is what saves the work: an encode whose result is
	 * already known not to fit costs a full picture on the hardware and, on
	 * H1, a vendor instance restart per attempt.
	 *
	 * 0 until an intra picture has been coded, which reads as "no forecast",
	 * and the keyframe is then placed without waiting. */
	double last_intra_bits;
	/* Set for one picture that the caller has been asked to make a sync
	 * point out of band - a decoder joining mid-stream needs somewhere to
	 * start, and while sweeping a refresh that is a picture with every
	 * coding unit intra rather than an IDR.
	 *
	 * Such a picture is coded as a predicted one, because that is what it
	 * is structurally: no IDR, no reset of the reference list or the picture
	 * order. So without this flag the controller sees an inter picture and
	 * holds it to the discretionary cap, which exists to stop an ordinary
	 * inter picture monopolising the buffer - and this one is not ordinary.
	 * Measured on an imx8mp: the cap pinned it to 90128 bit against a
	 * 180000 bit buffer, 0.5007 of it, so an all-intra picture came out the
	 * size of a refresh band and the sync point it was asked for was worth
	 * much less than it should have been.
	 *
	 * Raises the ceiling to the whole buffer for that picture only. Cleared
	 * once the picture is accounted for. */
	int forced_sync_point;

	/* Bits an intra picture is waiting for room for. Non-zero means pictures
	 * are being skipped to drain the buffer and the keyframe request is
	 * still outstanding; the run ends when the bucket empties, so the picture
	 * is always coded in the end. */
	double intra_drain_bits;
	/* Cost of one coded block at Qstep 1, smoothed, and the previous
	 * picture's cost derived from it. */
	double cplx_per_block, cplx_prev, cplx_ema;
	/* log2(bits) + qp/slope: what the picture would have cost at QP 0, and
	 * therefore independent of the QP it was coded at. */
	double complexity_x;
	int have_complexity_x;

	int prev_qp;
	int current_qp;
	int current_target;
	int cap_attempts;
	/* Set while the next intra picture is one with no model behind it, which
	 * is the case at the start of a stream and again after a resolution
	 * change: nothing coded so far says what a picture of the new size
	 * costs. first_intra_share bounds that one picture; see
	 * ext_rate_control_check(). Cleared as soon as an intra picture has been
	 * coded. */
	int intra_bootstrap_pending;

	/* --- counters, for logging only --- */
	unsigned long num_pictures;
	unsigned long num_reencodes;
	/* Pictures where the bucket hit its floor, i.e. where the link had
	 * drained everything and the accounting identity above stops holding. */
	unsigned long num_bucket_empty;
	/* Pictures whose target was raised to repay idle capacity, and the bits
	 * that repayment added. */
	unsigned long num_debt_lifts;
	double sum_lift;
	/* Sum of the repayment ceiling actually used, for reporting its mean. */
	double sum_fill_used;
	/* Pictures the valve refused to send, how many of those had been encoded
	 * first and were then discarded on their measured size rather than
	 * refused on the forecast, and the longest run. */
	unsigned long num_skipped;
	unsigned long num_dropped;
	/* Edge reporting for the skip and overflow decisions. These are states,
	 * not counters: each is logged when it is entered and not again until it
	 * is left, so a stream at a hard operating point reports the handful of
	 * decisions it took rather than a line per picture.
	 *
	 * skip_state: 0 coding normally, 1 refusing pictures to drain, 2 sent a
	 * picture over the cap because nothing else was left to try.
	 * qp_max_reported: the quantiser ceiling has been reported for this
	 * excursion, and is re-armed when the quantiser comes back off it. */
	int skip_state;
	int qp_max_reported;

	/* Pictures whose accounting left the buffer over its cap, and how many of
	 * those were the one permitted case: a picture that could not be made to
	 * fit even with the whole buffer on offer and the quantiser at its
	 * ceiling, so sending it was the only remaining option. The two being
	 * equal means the rule held; num_overflows exceeding
	 * num_forced_overflows means it did not, and that is a bug rather than a
	 * hard operating point. max_overflow_fill is the worst level reached, as
	 * a share of the cap. */
	unsigned long num_overflows;
	/* How many pictures had their QP fall clamped by qp_down_step. */
	unsigned long num_qp_down_clamped;
	unsigned long num_forced_overflows;
	double max_overflow_fill;

	/* Times the buffer had to be drained before a keyframe could be placed,
	 * and pictures spent doing it. */
	unsigned long num_intra_drains;
	unsigned long num_intra_drain_pics;
	unsigned int max_skip_run;
	double sum_bits;
	double sum_fill;
	double max_fill;
}
ExtRateControl;


/* Sets up rc from params. Returns 0 on success, -1 if params are unusable
 * (no bitrate, no frame rate, no frame size). */
int ext_rate_control_init(ExtRateControl *rc, ExtRateControlParams const *params);

/* QP to encode the next picture at. overhead_bits is what the caller will
 * prepend to this picture on its way to the link - parameter sets, prefix SEI
 * - and is taken off the picture's budget so that the access unit as a whole
 * lands on the target. Pass 0 if the picture is the whole access unit. */
int ext_rate_control_pre(ExtRateControl *rc, int is_intra, size_t overhead_bits);

/* Bits the rate control aimed for. Valid after ext_rate_control_pre();
 * for logging and diagnostics only. */
int ext_rate_control_target(ExtRateControl const *rc);

/* Call with the size of the picture just encoded, and with whatever the
 * caller will prepend to it on the way to the link. Returns a QP to re-encode
 * it at, or 0 to keep it. Loop until it returns 0.
 *
 * The ceiling is applied to the two together, because that is what the buffer
 * has to hold; the QP gradient is learned from the picture alone. */
int ext_rate_control_check(ExtRateControl *rc, size_t bits, size_t overhead_bits, int is_intra);

/* Call once per picture with the attempt that was kept. bits is the coded
 * picture; overhead_bits is everything else the access unit carries onto the
 * link - parameter sets, prefix SEI - which the leaky bucket is charged for
 * and the content model is not. cu_stats may be NULL, in which case the
 * picture is assumed to have coded every block. */
void ext_rate_control_post(ExtRateControl *rc, size_t bits, size_t overhead_bits, int is_intra, ExtRateControlStats const *cu_stats);

/* Whether the next picture should be skipped rather than coded, because even
 * at the coarsest quantiser allowed it cannot be made to fit in the buffer.
 *
 * Call this *before* advancing anything that carries per-picture state - an
 * intra refresh sweep, a GOP position, a picture counter - because a skipped
 * picture has to leave all of it exactly where it was, so that the next
 * picture takes the slot this one would have had.
 *
 * Never true for an intra picture: there is nothing to predict one from. The
 * measured rule takes those, and defers rather than drops them.
 *
 * Pure: it reads rc and changes nothing. Tell the controller what was
 * actually done with ext_rate_control_skip().
 *
 * This runs before the encode, so all it has to go on is the content model.
 * Its purpose is to save an encode when the answer is already obvious, but it
 * is not merely an optimisation on top of ext_rate_control_should_drop(): a
 * picture refused here is never measured, so the content model does not move
 * for it and later pictures are quantised differently. The two paths select
 * different pictures, and having both is measurably better than either -
 * see ext_rate_control_init(). */
int ext_rate_control_should_skip(ExtRateControl const *rc, int is_intra);

/* The rule itself: whether the picture that was just encoded must be
 * discarded rather than sent, because the quantiser is already at its maximum
 * and the picture still does not fit in the buffer. Nothing is predicted
 * here - these are the bits the encoder produced.
 *
 * Discarding is safe as long as the caller has not yet advanced the coded
 * picture's own bookkeeping: the next input frame is then encoded in the slot
 * this one would have had, predicting from the same reference picture and
 * overwriting the discarded reconstruction, so the encoder and the decoder
 * stay in step. Advancing first and discarding afterwards would leave the
 * encoder predicting from a picture the decoder never received.
 *
 * An intra picture is the one every encoder can redo, whatever else it
 * cannot. Discarding or re-encoding an *inter* picture needs the encoder to
 * roll back its reference state, which not all of them can - H1 commits
 * frameCnt, frameNum and the reference rotation inside H264EncStrmEncode().
 * An intra picture has no reference dependency, so on that hardware it is
 * redone by restarting the vendor instance, which is byte-deterministic for
 * the same input (see h1_h264_restart()). So the keyframe rules below apply
 * uniformly: both encoders can re-encode a keyframe as many times as asked,
 * and only predicted pictures differ.
 *
 * This applies to intra pictures too, and for them the caller must read it
 * as "defer", not "drop": re-arm the intra request so the next picture coded
 * carries it. An intra picture is the only one a decoder can start from, so it
 * must never be lost - but it can arrive a frame or two later, and it has to
 * be allowed to, because an intra picture the quantiser cannot shrink to fit
 * is otherwise a guaranteed buffer overflow. A periodic IDR at a scene cut is
 * exactly that: measured at 222 kbit against 122 kbit of buffer with
 * qp_max_intra already reached. The run limit bounds the deferral, so the
 * picture is always coded in the end.
 *
 * Pure, like should_skip(). Report what was done with
 * ext_rate_control_skip(). */
int ext_rate_control_should_drop(ExtRateControl const *rc, size_t bits, int is_intra);

/* Account for a picture the caller did not send. The link drains over that
 * picture's period, and that drain is the entire mechanism - it is how
 * skipping buys the buffer back. The bucket is never charged the bits.
 *
 * bits is 0 for a picture that was never encoded, and the coded size for one
 * that was encoded and then discarded; in the second case the measurement is
 * still fed to the content model, because it describes the content whether or
 * not the picture is sent. cu_stats may be NULL. */
void ext_rate_control_skip(ExtRateControl *rc, int is_intra, size_t bits, ExtRateControlStats const *cu_stats);

/* Declare the next picture a forced sync point; see forced_sync_point above.
 * Call before ext_rate_control_pre() for that picture, and again for each
 * retry if it is deferred - the flag covers one accounted picture. */
void ext_rate_control_set_forced_sync_point(ExtRateControl *rc);

/* Bits the HRD buffer can still take right now, i.e. what one picture may
 * add without overflowing it. Never negative. Pure. */
double ext_rate_control_room(ExtRateControl const *rc);

/* Whether an intra picture can be placed in the next coded picture, or
 * whether the caller should skip pictures to drain the buffer first.
 *
 * Returns non-zero when the keyframe should be coded now. Returns zero to ask
 * the caller to skip this input picture - without encoding it, and leaving the
 * keyframe request armed - so that the buffer drains by one frame budget and
 * the question can be asked again on the next one.
 *
 * Only ever zero in keyframe mode, and only while a forecast exists and the
 * buffer has less room than that forecast. Sweeping an intra refresh, a
 * keyframe is a band rather than a whole picture and this always says yes.
 *
 * Liveness comes from the drain: once the bucket is empty this says yes
 * regardless, because waiting longer cannot make more room, so a keyframe can
 * never be withheld indefinitely.
 *
 * Pure. Report a picture skipped for this reason with
 * ext_rate_control_skip(), which is also what advances the drain. */
int ext_rate_control_keyframe_ready(ExtRateControl const *rc);

/* Retarget at a new link rate. The bucket keeps both its level and its size:
 * the level is a debt already incurred and is still owed, and the size is how
 * much coded data may be in flight, which is a property of the link and its
 * receiver rather than of the rate the encoder was asked for. */
void ext_rate_control_set_bitrate(ExtRateControl *rc, unsigned int bitrate_bps);


#ifdef __cplusplus
}
#endif


#endif /* EXT_RATE_CONTROL_H */
