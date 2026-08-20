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

	/* --- tuning; see ext_rate_control_init() for what each one does --- */
	double gain, setpoint, target_min, target_max;
	double deadband, unspent_share;
	double cplx_min, cplx_max, coded_ref, coded_pow;
	double alpha;
	double slope;
	int deadband_first;
	int bucket_clamp;
	int unspent_capped;

	/* --- state --- */
	/* Bits handed to the link that it has not drained yet. */
	double bucket;
	/* Cost of one coded block at Qstep 1, smoothed, and the previous
	 * picture's cost and coded-block count derived from it. */
	double cplx_per_block, cplx_prev, cplx_ema, coded_prev;
	/* Bits deliberately not requested for the current picture. Charged to
	 * the bucket so a calm stretch does not bank credit for the first hard
	 * picture to spend all at once. */
	double unspent;
	/* log2(bits) + qp/slope: what the picture would have cost at QP 0, and
	 * therefore independent of the QP it was coded at. */
	double complexity_x;
	int have_complexity_x;

	int prev_qp;
	int current_qp;
	int current_target;
	int cap_attempts;

	/* --- counters, for logging only --- */
	unsigned long num_pictures;
	unsigned long num_reencodes;
	/* Pictures whose deliberately-unspent bits were charged to the bucket,
	 * and how many bits that was in total. In steady state the stream's
	 * rate is the configured rate minus this, so it is the first thing to
	 * look at when the output lands under the bitrate that was asked for. */
	unsigned long num_trimmed;
	double sum_unspent;
	/* Pictures where the bucket hit its floor, i.e. where the link had
	 * drained everything and the accounting identity above stops holding. */
	unsigned long num_bucket_empty;
	unsigned long num_bucket_full;
	double sum_bits;
	double sum_fill;
	double max_fill;
	double min_headroom;
}
ExtRateControl;


/* Sets up rc from params. Returns 0 on success, -1 if params are unusable
 * (no bitrate, no frame rate, no frame size). */
int ext_rate_control_init(ExtRateControl *rc, ExtRateControlParams const *params);

/* QP to encode the next picture at. */
int ext_rate_control_pre(ExtRateControl *rc, int is_intra);

/* Bits the rate control aimed for. Valid after ext_rate_control_pre();
 * for logging and diagnostics only. */
int ext_rate_control_target(ExtRateControl const *rc);

/* Call with the size of the picture just encoded. Returns a QP to re-encode
 * it at, or 0 to keep it. Loop until it returns 0. */
int ext_rate_control_check(ExtRateControl *rc, size_t bits, int is_intra);

/* Call once per picture with the attempt that was kept. cu_stats may be NULL,
 * in which case the picture is assumed to have coded every block. */
void ext_rate_control_post(ExtRateControl *rc, size_t bits, int is_intra, ExtRateControlStats const *cu_stats);

/* Retarget at a new link rate. The bucket keeps both its level and its size:
 * the level is a debt already incurred and is still owed, and the size is how
 * much coded data may be in flight, which is a property of the link and its
 * receiver rather than of the rate the encoder was asked for. */
void ext_rate_control_set_bitrate(ExtRateControl *rc, unsigned int bitrate_bps);


#ifdef __cplusplus
}
#endif


#endif /* EXT_RATE_CONTROL_H */
