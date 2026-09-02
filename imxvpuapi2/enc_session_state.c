/* Session state shared by both encoder backends. See the header.
 *
 * Author: Andrii Voznytsia <andrew@auterion.com>
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "imxvpuapi2_priv.h"
#include "enc_session_state.h"


int imx_vpu_api_enc_session_param_set_id(ImxVpuApiEncSession *session,
                                         unsigned int width, unsigned int height)
{
	uint16_t const w = (uint16_t)width;
	uint16_t const h = (uint16_t)height;
	int i;

	if (session == NULL)
		return 0;

	for (i = 0; i < (int)(session->ps_num); ++i)
	{
		if ((session->ps_width[i] == w) && (session->ps_height[i] == h))
		{
			IMX_VPU_API_DEBUG("parameter set id %d: %ux%u again", (int)(session->ps_id[i]), width, height);
			return (int)(session->ps_id[i]);
		}
	}

	if (session->ps_num < IMX_VPU_API_ENC_MAX_PARAM_SET_IDS)
		i = session->ps_num++;
	else
	{
		/* Oldest entry out, and its id comes back around. */
		memmove(&(session->ps_width[0]), &(session->ps_width[1]),
		        (IMX_VPU_API_ENC_MAX_PARAM_SET_IDS - 1) * sizeof(session->ps_width[0]));
		memmove(&(session->ps_height[0]), &(session->ps_height[1]),
		        (IMX_VPU_API_ENC_MAX_PARAM_SET_IDS - 1) * sizeof(session->ps_height[0]));
		memmove(&(session->ps_id[0]), &(session->ps_id[1]),
		        (IMX_VPU_API_ENC_MAX_PARAM_SET_IDS - 1) * sizeof(session->ps_id[0]));
		i = IMX_VPU_API_ENC_MAX_PARAM_SET_IDS - 1;
	}

	session->ps_width[i] = w;
	session->ps_height[i] = h;
	session->ps_id[i] = session->ps_next_id;
	session->ps_next_id = (uint8_t)((session->ps_next_id + 1) % IMX_VPU_API_ENC_MAX_PARAM_SET_IDS);

	if (session->ps_id[i] != 0)
		IMX_VPU_API_INFO("parameter set id %d for %ux%u; %d resolution(s) in this stream so far",
		                 (int)(session->ps_id[i]), width, height, (int)(session->ps_num));

	return (int)(session->ps_id[i]);
}


void imx_vpu_api_enc_session_save_rc(ImxVpuApiEncSession *session, ExtRateControl const *rc)
{
	if ((session == NULL) || (rc == NULL))
		return;

	session->rc_valid = 1;
	session->rc_total_blocks = rc->total_blocks;
	session->rc_bucket = rc->bucket;
	session->rc_cplx_per_block = rc->cplx_per_block;
	session->rc_cplx_prev = rc->cplx_prev;
	session->rc_cplx_ema = rc->cplx_ema;
	session->rc_complexity_x = rc->complexity_x;
	session->rc_have_complexity_x = rc->have_complexity_x;
	session->rc_prev_qp = rc->prev_qp;
	session->rc_current_qp = rc->current_qp;
	session->rc_num_pictures = (uint32_t)(rc->num_pictures);
	session->rc_num_reencodes = (uint32_t)(rc->num_reencodes);
	session->rc_num_bucket_empty = (uint32_t)(rc->num_bucket_empty);
	session->rc_sum_bits = rc->sum_bits;
	session->rc_sum_fill = rc->sum_fill;
	session->rc_max_fill = rc->max_fill;
}


void imx_vpu_api_enc_session_restore_rc(ImxVpuApiEncSession const *session, ExtRateControl *rc)
{
	double ratio;

	if ((session == NULL) || (rc == NULL) || !session->rc_valid)
		return;

	/* Everything the controller holds is either a property of the link, which
	 * the frame size does not enter into, or a cost per coded block, which it
	 * does. The second kind is rescaled by the change in block count so that
	 * the first picture at the new resolution is budgeted from a model in the
	 * new picture's units.
	 *
	 * cplx_prev and cplx_ema are only ever used as a ratio of each other, so
	 * scaling both leaves the controller's reading of "is this stretch harder
	 * than the recent average" untouched while putting both in the new scale.
	 * complexity_x is log2(bits) at QP 0, and bits scale with the block count,
	 * so the correction there is additive. */
	ratio = ((session->rc_total_blocks > 0) && (rc->total_blocks > 0))
	      ? ((double)(rc->total_blocks) / (double)(session->rc_total_blocks))
	      : 1.0;

	rc->bucket = session->rc_bucket;
	if (rc->bucket > rc->bucket_cap)
	{
		IMX_VPU_API_WARNING("carried HRD buffer level %.0f kbit exceeds the new %.0f kbit buffer; clamped",
		                    rc->bucket / 1000.0, rc->bucket_cap / 1000.0);
		rc->bucket = rc->bucket_cap;
	}

	rc->cplx_per_block = session->rc_cplx_per_block;
	rc->cplx_prev = session->rc_cplx_prev * ratio;
	rc->cplx_ema = session->rc_cplx_ema * ratio;
	rc->complexity_x = session->rc_complexity_x + ((ratio > 0.0) ? log2(ratio) : 0.0);
	rc->have_complexity_x = session->rc_have_complexity_x;
	rc->prev_qp = session->rc_prev_qp;
	rc->current_qp = session->rc_current_qp;
	rc->num_pictures = session->rc_num_pictures;
	/* The picture that opens the new resolution has no model behind it -
	 * nothing coded at the old size says what one of the new size costs - so
	 * it gets the same bootstrap allowance as the first picture of a stream.
	 * Without this it is bounded only by what is left in the buffer, and on a
	 * 720p to 1080p switch at a 100 ms buffer it takes 0.91 of the buffer on
	 * its own instead of 0.16. */
	rc->intra_bootstrap_pending = 1;
	rc->num_reencodes = session->rc_num_reencodes;
	rc->num_bucket_empty = session->rc_num_bucket_empty;
	rc->sum_bits = session->rc_sum_bits;
	rc->sum_fill = session->rc_sum_fill;
	rc->max_fill = session->rc_max_fill;

	IMX_VPU_API_INFO(
		"rate control resumed after %lu pictures: HRD buffer %.0f of %.0f kbit (%.2f full), QP %d, "
		"content model rescaled by %.3f, first picture held to %.0f kbit",
		rc->num_pictures, rc->bucket / 1000.0, rc->bucket_cap / 1000.0,
		(rc->bucket_cap > 0.0) ? (rc->bucket / rc->bucket_cap) : 0.0,
		rc->current_qp, ratio, (rc->bucket_cap * rc->first_intra_share) / 1000.0
	);
}


void imx_vpu_api_enc_session_pack(ImxVpuApiEncSession const *session, ImxVpuApiEncSessionState *state)
{
	memset(state, 0, sizeof(*state));
	state->version = IMX_VPU_API_ENC_SESSION_VERSION;
	state->size = (uint32_t)sizeof(ImxVpuApiEncSession);
	memcpy(state->data, session, sizeof(ImxVpuApiEncSession));
}


int imx_vpu_api_enc_session_unpack(ImxVpuApiEncSession *session, ImxVpuApiEncSessionState const *state)
{
	if ((state->version != IMX_VPU_API_ENC_SESSION_VERSION)
	 || (state->size != (uint32_t)sizeof(ImxVpuApiEncSession)))
	{
		IMX_VPU_API_WARNING("session state is version %u size %u, this encoder writes version %d size %zu; "
		                    "starting a new session instead",
		                    state->version, state->size,
		                    IMX_VPU_API_ENC_SESSION_VERSION, sizeof(ImxVpuApiEncSession));
		return 0;
	}

	memcpy(session, state->data, sizeof(ImxVpuApiEncSession));
	return 1;
}


unsigned int imx_vpu_api_enc_hrd_buffer_bits(ImxVpuApiEncOpenParams const *open_params,
                                             double min_frame_budgets)
{
	unsigned int const bitrate_bps = open_params->bitrate * 1000u;
	unsigned int bits;

	/* 0 means one second of bitrate. The two are numerically equal - one is
	 * kbits, the other kbps - which is the point: a buffer that scales with
	 * the rate, rather than a fixed kbit count meaning half a second at one
	 * setting and two at another. */
	bits = (open_params->hrd_buffer_size > 0)
	     ? (((unsigned int)(open_params->hrd_buffer_size)) * 1000u)
	     : bitrate_bps;

	if ((min_frame_budgets > 0.0)
	 && (open_params->frame_rate_numerator > 0) && (open_params->frame_rate_denominator > 0))
	{
		unsigned int const min_bits = (unsigned int)((double)(bitrate_bps) * min_frame_budgets
		                                             * open_params->frame_rate_denominator
		                                             / open_params->frame_rate_numerator);

		if (bits < min_bits)
		{
			IMX_VPU_API_WARNING("hrd-buffer-size %u kbit is below %.1f frame budgets (%u kbit) at "
			                    "%u kbps and %u/%u fps; clamping",
			                    bits / 1000u, min_frame_budgets, min_bits / 1000u,
			                    open_params->bitrate,
			                    open_params->frame_rate_numerator,
			                    open_params->frame_rate_denominator);
			bits = min_bits;
		}
	}

	return bits;
}


void imx_vpu_api_enc_session_rc_params(ExtRateControlParams *params,
                                       ImxVpuApiEncOpenParams const *open_params)
{
	memset(params, 0, sizeof(*params));
	params->bitrate_bps = open_params->bitrate * 1000;
	params->frame_rate_numerator = open_params->frame_rate_numerator;
	params->frame_rate_denominator = open_params->frame_rate_denominator;
	params->frame_width = open_params->frame_width;
	params->frame_height = open_params->frame_height;
	/* Two and a half frame budgets, not three. The buffer is the caller's
	 * latency bound, so the floor exists only to keep the rate control out of
	 * territory where it cannot function at all - not to second-guess a
	 * deliberate setting. Measured at 1400 kbps 720p25: a 150 kbit bucket
	 * holds the target to within 5% on the rcstress clip, and clamping it to
	 * three budgets (168 kbit) cost 24% on the queueing delay tail - 93 ms
	 * peak against 75 - for nothing measurable on the rate. The hardware HRD
	 * is a different matter and keeps its own floor; see the callers that
	 * pass one. */
	params->buffer_bits = imx_vpu_api_enc_hrd_buffer_bits(open_params, 2.5);

	params->qp_min_inter = open_params->qp_min_inter;
	params->qp_max_inter = open_params->qp_max_inter;
	params->qp_min_intra = open_params->qp_min_intra;
	params->qp_max_intra = open_params->qp_max_intra;
}
