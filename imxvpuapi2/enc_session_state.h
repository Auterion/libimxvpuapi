/* What a stream carries from one encoder instance to the next.
 *
 * Author: Andrii Voznytsia <andrew@auterion.com>
 *
 * An encoder instance is per resolution: the hardware is initialised with the
 * frame size, so a resolution change means closing the encoder and opening
 * another. Two things must not restart with it, and both backends need the
 * same two, which is why they live here rather than once per backend:
 *
 *   - the rate control's leaky bucket, which holds coded bits the link has
 *     not drained yet - a property of the link, not of the picture size;
 *   - the parameter set ids, so that the sets of the new resolution cannot be
 *     confused with the old ones by a decoder that lost the switch point.
 *
 * The caller moves the state across the gap with
 * imx_vpu_api_enc_get_session_state() and imx_vpu_api_enc_set_session_state();
 * this is what those two marshal.
 */
#ifndef IMXVPUAPI2_ENC_SESSION_STATE_H
#define IMXVPUAPI2_ENC_SESSION_STATE_H

#include <stdint.h>
#include "imxvpuapi2.h"
#include "ext_rate_control.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Bumped whenever the layout below changes. A state written by another
 * version is refused rather than misread. */
#define IMX_VPU_API_ENC_SESSION_VERSION 1

/* How many distinct resolutions get an id of their own before ids start being
 * reused. h.265 allows 16 SPS ids and 64 PPS ids, h.264 32 and 256, so eight
 * fits both with room to spare; a session that visits more than eight
 * resolutions gets the id of the one it saw eight switches ago, by which point
 * no decoder is still holding that one's parameter sets. */
#define IMX_VPU_API_ENC_MAX_PARAM_SET_IDS 8


typedef struct
{
	/* Which parameter set id each resolution seen so far owns, oldest entry
	 * first, and the id the next unseen resolution gets. The first resolution
	 * of a session takes id 0, so a stream that never changes resolution
	 * carries the ids the encoder writes by itself and needs no renumbering. */
	uint16_t ps_width[IMX_VPU_API_ENC_MAX_PARAM_SET_IDS];
	uint16_t ps_height[IMX_VPU_API_ENC_MAX_PARAM_SET_IDS];
	uint8_t ps_id[IMX_VPU_API_ENC_MAX_PARAM_SET_IDS];
	uint8_t ps_num;
	uint8_t ps_next_id;

	/* The rate control's state. Set only once a picture has been encoded;
	 * before that there is nothing to carry. */
	uint8_t rc_valid;
	/* Frame size the quantities below were learned at, in blocks. The ones
	 * that scale with it are rescaled on restore. */
	uint32_t rc_total_blocks;
	double rc_bucket;
	double rc_cplx_per_block, rc_cplx_prev, rc_cplx_ema;
	double rc_complexity_x;
	int32_t rc_have_complexity_x;
	int32_t rc_prev_qp, rc_current_qp;
	uint32_t rc_num_pictures, rc_num_reencodes, rc_num_bucket_empty;
	double rc_sum_bits, rc_sum_fill, rc_max_fill;
}
ImxVpuApiEncSession;

/* The state has to fit in the caller's buffer, and that buffer is public ABI. */
typedef char imx_vpu_api_enc_session_fits[(sizeof(ImxVpuApiEncSession) <= IMX_VPU_API_ENC_SESSION_STATE_SIZE) ? 1 : -1];


/* Which parameter set id this resolution owns, assigning one if it has not
 * been seen before. Returns the id; 0 for the first resolution of a session,
 * which is what the encoder would have written unaided. */
int imx_vpu_api_enc_session_param_set_id(ImxVpuApiEncSession *session,
                                         unsigned int width, unsigned int height);

/* Copy the rate control's state in and out. Restoring rescales the quantities
 * that are per picture onto the new frame size; the bucket level is not one of
 * them, being a debt owed to the link. */
void imx_vpu_api_enc_session_save_rc(ImxVpuApiEncSession *session, ExtRateControl const *rc);
void imx_vpu_api_enc_session_restore_rc(ImxVpuApiEncSession const *session, ExtRateControl *rc);

/* Marshal to and from the caller's opaque buffer. imx_vpu_api_enc_session_unpack()
 * returns 0 if the buffer was written by a different version or size, in which
 * case session is left alone. */
void imx_vpu_api_enc_session_pack(ImxVpuApiEncSession const *session, ImxVpuApiEncSessionState *state);
int imx_vpu_api_enc_session_unpack(ImxVpuApiEncSession *session, ImxVpuApiEncSessionState const *state);

/* The HRD buffer size in bits, with 0 resolved to one second of bitrate and a
 * floor of min_frame_budgets frame budgets applied.
 *
 * The floor differs by who enforces the buffer, which is why it is a
 * parameter. The hardware HRD model enforces by *skipping pictures* and
 * starves below about four frame budgets - a frozen stream - so it gets five.
 * The rate control in ext_rate_control.c enforces by choosing a QP and never
 * skips, so three is enough; below that it has less than a group of pictures
 * to work with and spends the stream reacting instead of tracking. */
unsigned int imx_vpu_api_enc_hrd_buffer_bits(ImxVpuApiEncOpenParams const *open_params,
                                             double min_frame_budgets);

/* The rate control's parameters, as the unified open_params spell them. */
void imx_vpu_api_enc_session_rc_params(ExtRateControlParams *params,
                                       ImxVpuApiEncOpenParams const *open_params);


#ifdef __cplusplus
}
#endif

#endif /* IMXVPUAPI2_ENC_SESSION_STATE_H */
