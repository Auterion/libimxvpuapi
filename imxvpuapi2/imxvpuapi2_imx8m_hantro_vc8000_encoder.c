#include <config.h>
#include "imxvpuapi2.h"
#include "imxvpuapi2_priv.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>

#include <imxdmabuffer/imxdmabuffer.h>

/* This encoder uses the VC8000E API in slightly unusual ways:
 * - There is just one GOP config with a single P frame. B frames would
 *   be problematic with the zerolatency nature of libimxvpuapi's encoder
 *   interface.
 * - Draining does not call VCEncFlush(), because there isn't actually
 *   anything to drain (due to the zerolatency encoding). This may
 *   change if B frames are introduced (see above).
 * - gopSize in VCEncIn is always set to 1 due to the GOP config
 *   (as mentioned above). GOPs are emulated by manually setting
 *   VCEncOut's codingType field to VCENC_INTRA_FRAME every Nth frame
 *   (N being the gop_size value of ImxVpuApiEncOpenParams).
 * - For h.264 encoding, only the byte-stream stream format is supported.
 *   This is for ABI/API compatibility reasons.
 * - Interlaced encoding is not supported.
 * - SSIM is hardcoded to be always enabled.
 * - Horizontal and vertical sample aspect ratio are set to 0
 *   (= undefined).
 *
 * Future libimxvpuapi versions may change these unusual ways
 * of using the API, or remove them altogether.
 *
 * Also, these driver features are turned off because they produce corrupted data:
 * - Access unit delimiters
 * - Resending SPS/PPS/VPS (this is instead done manually by re-sending
 *   the header data that is generated at the beginning of the stream)
 *
 * and, these features are not used because the encoder hardware does not support them:
 * - 10-bit encoding
 * */

/* hevcencapi.h includes the h.264 encoder API as well.
 * It also includes AV1 and VP9 encoder APIs, but these
 * are unused in the imx8mp variant of the driver. */
#include "hantro_VC8000E_enc/hevcencapi.h"
#include "hantro_VC8000E_enc/enccommon.h"
#include "hantro_VC8000E_enc/base_type.h"

/* New CBR (open_params.rate_control_mode == 1). Include order matters:
 * vc8000e-shim.h (pulled in by the rate control header) only defines its
 * VCEncOut conversion helper if the vendor header was seen first. */
#include "vc8000e-shim.h"
#include "ext_rate_control.h"
#include "intra_refresh.h"
#include "enc_session_state.h"




/******************************************************/
/******* MISCELLANEOUS STRUCTURES AND FUNCTIONS *******/
/******************************************************/


/* Define the stream buffer size to be able to hold one big
 * uncompressed YUV 4:4:4 frame, plus 256 kB of extra headroom
 * (the latter is arbitrarily chosen). This is more than what
 * the encoder will ever produce, which is needed to prevent
 * the encoder from running out of stream buffer memory. */
#define VPU_ENC_MIN_REQUIRED_STREAM_BUFFER_SIZE  (1920*1088*3 + 262144)
#define STREAM_BUFFER_PHYSADDR_ALIGNMENT         (0x10)
#define STREAM_BUFFER_SIZE_ALIGNMENT             (1024)
#define FRAME_WIDTH_ALIGNMENT                    (2)
#define FRAME_HEIGHT_ALIGNMENT                   (2)
#define EXP_OF_INPUT_ALIGNMENT                   (4)
#define INPUT_ALIGNMENT                          (1 << (EXP_OF_INPUT_ALIGNMENT))


static char const * vcenc_retval_to_string(VCEncRet retval)
{
	switch (retval)
	{
		case VCENC_OK: return "ok";
		case VCENC_FRAME_READY: return "frame ready";
		case VCENC_FRAME_ENQUEUE: return "";

		case VCENC_ERROR: return "frame enqueue";
		case VCENC_NULL_ARGUMENT: return "null argument";
		case VCENC_INVALID_ARGUMENT: return "invalid argument";
		case VCENC_MEMORY_ERROR: return "memory error";
		case VCENC_EWL_ERROR: return "EWL error";
		case VCENC_EWL_MEMORY_ERROR: return "EWL memory error";
		case VCENC_INVALID_STATUS: return "invalid status";
		case VCENC_OUTPUT_BUFFER_OVERFLOW: return "output buffer overflow";
		case VCENC_HW_BUS_ERROR: return "hw bus error";
		case VCENC_HW_DATA_ERROR: return "hw data error";
		case VCENC_HW_TIMEOUT: return "hw timeout";
		case VCENC_HW_RESERVED: return "hw reserved";
		case VCENC_SYSTEM_ERROR: return "system error";
		case VCENC_INSTANCE_ERROR: return "instance error";
		case VCENC_HRD_ERROR: return "hrd error";
		case VCENC_HW_RESET: return "hw reset";
		default: return "<unknown>";
	}
}


static char const * vcenc_picture_coding_type_to_string(VCEncPictureCodingType coding_type)
{
	switch (coding_type)
	{
		case VCENC_INTRA_FRAME: return "intra";
		case VCENC_PREDICTED_FRAME: return "predicted";
		case VCENC_BIDIR_PREDICTED_FRAME: return "bidirectional predicted";
		case VCENC_NOTCODED_FRAME: return "not-coded";
		default: return "<unknown>";
	}
}


static VCEncPictureType convert_to_vc8000e_pixel_format(ImxVpuApiColorFormat color_format)
{
	switch (color_format)
	{
		case IMX_VPU_API_COLOR_FORMAT_FULLY_PLANAR_YUV420_8BIT: return VCENC_YUV420_PLANAR;
		case IMX_VPU_API_COLOR_FORMAT_SEMI_PLANAR_YUV420_8BIT: return VCENC_YUV420_SEMIPLANAR;
		case IMX_VPU_API_COLOR_FORMAT_PACKED_YUV422_UYVY_8BIT: return VCENC_YUV422_INTERLEAVED_UYVY;
		case IMX_VPU_API_COLOR_FORMAT_PACKED_YUV422_YUYV_8BIT: return VCENC_YUV422_INTERLEAVED_YUYV;
		case IMX_VPU_API_COLOR_FORMAT_RGB565: return VCENC_RGB565;
		case IMX_VPU_API_COLOR_FORMAT_BGR565: return VCENC_BGR565;
		case IMX_VPU_API_COLOR_FORMAT_RGB444: return VCENC_RGB444;
		case IMX_VPU_API_COLOR_FORMAT_ARGB4444: return VCENC_RGB444;
		case IMX_VPU_API_COLOR_FORMAT_ARGB1555: return VCENC_BGR555;
		case IMX_VPU_API_COLOR_FORMAT_RGBA8888: return VCENC_BGR888;
		case IMX_VPU_API_COLOR_FORMAT_BGRA8888: return VCENC_RGB888;

		default: return VCENC_FORMAT_MAX;
	}
}




/************************************************/
/******* ENCODER STRUCTURES AND FUNCTIONS *******/
/************************************************/


#define FORCED_INTRA_QUEUE_SIZE 32

struct _ImxVpuApiEncoder
{
	/* Hantro VC8000E encoder that is in use. */
	VCEncInst encoder;

	/* Stream buffer. Holds data coming from the encoder. */
	ImxDmaBuffer *stream_buffer;
	/* Due to the way the Hantro encoder operates, we have to map
	 * the stream buffer until it is encoded and no longer staged.
	 * The mapped virtual address is stored in this field. */
	uint8_t *stream_buffer_virtual_address;
	/* Physical address of the stream buffer. Stored here to avoid
	 * redundant imx_dma_buffer_get_physical_address() calls. */
	imx_physical_address_t stream_buffer_physical_address;
	/* Size of the stream buffer, in bytes. Stored here to avoid
	 * redundant imx_dma_buffer_get_size() calls. */
	size_t stream_buffer_size;

	/* Copy of the open_params passed to imx_vpu_api_enc_open(). */
	ImxVpuApiEncOpenParams open_params;

	/* Stream information that is generated by imx_vpu_api_enc_open(). */
	ImxVpuApiEncStreamInfo stream_info;

	/* GOP config for the encoder. Currently, this is set up to produce
	 * P frames only, since the libimxvpuapi interface is not designed
	 * for non-zerolatency encoding. Future libimxvpuapi versions might
	 * also allow B frames to be used. */
	VCEncConfig encoder_config;
	VCEncGopPicConfig gop_pic_config[MAX_GOP_PIC_CONFIG_NUM];
	VCEncGopPicSpecialConfig gop_pic_special_config[MAX_GOP_SPIC_CONFIG_NUM];

	/* Structure with information for the VC8000E encoder. Used
	 * by the VCEncStrmStart() and VCEncStrmEncode() functions. */
	VCEncIn encoder_input;

	/* Encoded picture counter. This is needed for setting the
	 * encoder_input->picture_cnt value and for detecting the
	 * very first picture (which requires some special handling). */
	int32_t num_encoded_pictures;

	/* The coding type to use for the next VCEncStrmEncode() call.
	 * This is set to the VCEncFindNextPic() call's return value. */
	VCEncPictureCodingType next_coding_type;

	/* DEPRECATED. This is kept here for backwards compatibility. */
	BOOL drain_mode_enabled;

	/* New bitrate to use. This is only used when rate control is active.
	 * If this is nonzero, the bitrate will be adjusted during the next
	 * imx_vpu_api_enc_encode() call. */
	unsigned int new_bitrate;

	/* h.264/h.265 SPS/PPS/VPS header data generated by the encoder. This
	 * is prepended to the main frame data if has_header is set to TRUE. */
	uint8_t *header_data;
	size_t header_data_size;

	/* TRUE if a header generated by the encoder is also to be included in
	 * the data of the encoded frame that will be output next. This is also
	 * needed for setting the has_header field in ImxVpuApiEncEncodedFrame. */
	BOOL has_header;

	/* TRUE if the next frame shall be forcibly encoded as an IDR frame.
	 * This is used after flushing to make sure the next frame is an
	 * I/IDR frame. */
	BOOL force_IDR_frame;
	/* An intra picture the frame skipping valve deferred rather than let
	 * through, so that the request outlives the input frame it arrived on
	 * and is applied to the next picture that is coded.
	 * IMX_VPU_API_FRAME_TYPE_UNKNOWN means none is pending. */
	ImxVpuApiFrameType postponed_frame_type;

	/* How many bytes of encoded frame data are currently stored in
	 * the stream buffer. This number is always less than or equal to
	 * stream_buffer_size. */
	size_t num_bytes_in_stream_buffer;

	/* The raw frame that is staged for encoding. */
	ImxVpuApiRawFrame staged_raw_frame;
	/* Physical address of the staged raw frame. Stored here to avoid
	 * redundant imx_dma_buffer_get_physical_address() calls. */
	imx_physical_address_t staged_raw_frame_physical_address;
	/* TRUE if a frame is staged, FALSE otherwise (the staged frame
	 * fields above are invalid if this is FALSE). */
	BOOL staged_raw_frame_set;

	/* TRUE is an encoded frame is available, FALSE otherwise.
	 * If set to FALSE, then the fields below about the encoded frame
	 * are invalid. */
	BOOL encoded_frame_available;
	/* Context, PTS, DTS copied from the input raw frame. */
	void *encoded_frame_context;
	uint64_t encoded_frame_pts, encoded_frame_dts;
	/* What encoded frame type the input raw frame was encoded into.
	 * Filled in imx_vpu_api_enc_encode(). */
	ImxVpuApiFrameType encoded_frame_type;
	/* Size of the resulting encoded frame, in bytes. If a header
	 * was prepended, then its size is included in this. This value
	 * is used for setting the data_size field of the
	 * ImxVpuApiEncEncodedFrame structure when getting the encoded
	 * frame with imx_vpu_api_enc_get_encoded_frame(). */
	size_t encoded_frame_data_size;

	/* cached_coding_ctrl holds the VCEncCodingCtrl captured right after the
	 * initial VCEncSetCodingCtrl call so that roiMapDeltaQpEnable is
	 * correct for all subsequent mid-stream SetCodingCtrl calls. */
	VCEncCodingCtrl cached_coding_ctrl;

	struct { uint32_t first; uint32_t num; } forced_intra_q[FORCED_INTRA_QUEUE_SIZE];
	int forced_intra_q_head;
	int forced_intra_q_count;

	/* Intra refresh. Two mechanisms can produce it, and
	 * resolve_refresh_config() picks one of them - never both.
	 *
	 * refresh_active: the sweep is driven from here, because the vendor's own
	 * cannot be configured to do what the new rate control needs - see
	 * intra_refresh.c. One scheduler for what used to be three separate
	 * modes; the deprecated open_params fields are mapped onto refresh_cfg by
	 * resolve_refresh_config().
	 *
	 * vendor_gdr_active: the vendor library places the band itself from
	 * gdrDuration. That is what this encoder did before any of the above, and
	 * what rate_control_mode 0 still gets, down to the bytes. */
	BOOL refresh_active;
	/* An out-of-order full-picture refresh is owed, because a keyframe was
	 * asked for while sweeping. Survives a skipped or discarded picture -
	 * it is cleared only once the picture carrying it has been coded - so
	 * the refresh cannot be lost to the rate control deferring a frame. */
	BOOL pending_full_refresh;
	BOOL vendor_gdr_active;
	ImxVpuApiIntraRefreshCfg refresh_cfg;
	ImxVpuApiIntraRefreshState refresh_state;
	/* Slice height in CTB rows that was programmed, 0 = one slice. Kept
	 * because sliceSize is set once at open time and the refresh schedule
	 * no longer implies it. */
	int slice_size;
	/* recovery_point SEI emitted ahead of the picture that starts a sweep.
	 * The encoder's own GDR used to produce this; driving the sweep from
	 * here means producing it here too, or a decoder joining mid-stream has
	 * nothing standard telling it when the picture will be complete. */
	uint8_t recovery_sei[IMX_VPU_API_RECOVERY_POINT_SEI_MAX];
	size_t recovery_sei_size;

	BOOL skipped_frame_available;
	void *skipped_frame_context;
	uint64_t skipped_frame_pts, skipped_frame_dts;

	/* New CBR (open_params.rate_control_mode == 1). The encoder's own
	 * picture rate control is switched off in that mode, and new_cbr picks
	 * the QP for every picture instead. cached_rate_ctrl is the full
	 * VCEncRateCtrl that was accepted at open time; per-picture updates
	 * modify only its qpHdr and set it back, because VCEncGetRateCtrl does
	 * not write every field of the struct (crf in particular) and a
	 * get-modify-set would therefore apply values nobody asked for. */
	BOOL new_cbr_active;
	ExtRateControl new_cbr;
	VCEncRateCtrl cached_rate_ctrl;

	/* Stream continuity across encoder instances - see
	 * ImxVpuApiEncSessionState. Only filled in at rate_control_mode 1, where
	 * a resolution change must not restart either the rate control or the
	 * parameter set numbering. */
	ImxVpuApiEncSession session;
	/* Parameter set id this instance stamps into its SPS and PPS. 0 means the
	 * ids the encoder itself wrote are left alone, which is the case for the
	 * first resolution of a session and therefore for every stream that never
	 * changes resolution. */
	int param_set_id;
	/* The vendor PPS carrying param_set_id is created and activated so that
	 * slice headers reference it too; without that the parameter sets would
	 * carry the new id and the slices would still point at the previous
	 * resolution's PPS. Creating it makes the vendor library insert a copy of
	 * it into the next picture, ahead of the slice, and that copy says the
	 * picture uses the SPS the old resolution used - contradicting the
	 * header - so it is taken back out again. */
	BOOL drop_inserted_pps;
};


/* Static, invariant global & compression format information. */

static ImxVpuApiCompressionFormat const enc_supported_compression_formats[] =
{
	IMX_VPU_API_COMPRESSION_FORMAT_H264,
	IMX_VPU_API_COMPRESSION_FORMAT_H265
};

static ImxVpuApiEncGlobalInfo const enc_global_info = {
	.flags = IMX_VPU_API_ENC_GLOBAL_INFO_FLAG_HAS_ENCODER
	       | IMX_VPU_API_ENC_GLOBAL_INFO_FLAG_SEMI_PLANAR_FRAMES_SUPPORTED
	       | IMX_VPU_API_ENC_GLOBAL_INFO_FLAG_FULLY_PLANAR_FRAMES_SUPPORTED
	       | IMX_VPU_API_ENC_GLOBAL_INFO_FLAG_ENCODER_SUPPORTS_RGB_FORMATS,
	.hardware_type = IMX_VPU_API_HARDWARE_TYPE_HANTRO,
	.min_required_stream_buffer_size = VPU_ENC_MIN_REQUIRED_STREAM_BUFFER_SIZE,
	.required_stream_buffer_physaddr_alignment = STREAM_BUFFER_PHYSADDR_ALIGNMENT,
	.required_stream_buffer_size_alignment = STREAM_BUFFER_SIZE_ALIGNMENT,
	.supported_compression_formats = enc_supported_compression_formats,
	.num_supported_compression_formats = sizeof(enc_supported_compression_formats) / sizeof(ImxVpuApiCompressionFormat)
};

ImxVpuApiEncGlobalInfo const * imx_vpu_api_enc_get_global_info(void)
{
	return &enc_global_info;
}


static ImxVpuApiColorFormat const enc_supported_basic_color_formats[] =
{
	/* 8-bit YUV formats */
	IMX_VPU_API_COLOR_FORMAT_FULLY_PLANAR_YUV420_8BIT,
	IMX_VPU_API_COLOR_FORMAT_SEMI_PLANAR_YUV420_8BIT,
	IMX_VPU_API_COLOR_FORMAT_PACKED_YUV422_UYVY_8BIT,
	IMX_VPU_API_COLOR_FORMAT_PACKED_YUV422_YUYV_8BIT,

	/* RGB formats */
	IMX_VPU_API_COLOR_FORMAT_RGB565,
	IMX_VPU_API_COLOR_FORMAT_BGR565,
	IMX_VPU_API_COLOR_FORMAT_RGB444,
	IMX_VPU_API_COLOR_FORMAT_ARGB4444,
	IMX_VPU_API_COLOR_FORMAT_ARGB1555,
	IMX_VPU_API_COLOR_FORMAT_RGBA8888,
	IMX_VPU_API_COLOR_FORMAT_BGRA8888
};

static ImxVpuApiH264SupportDetails const enc_h264_support_details = {
	.parent = {
		.min_width = 32, .max_width = 1920,
		.min_height = 32, .max_height = 1080,
		.supported_color_formats = enc_supported_basic_color_formats,
		.num_supported_color_formats = sizeof(enc_supported_basic_color_formats) / sizeof(ImxVpuApiColorFormat),
		.min_quantization = 1, .max_quantization = 51
	},

	.max_constrained_baseline_profile_level = IMX_VPU_API_H264_LEVEL_UNDEFINED,
	.max_baseline_profile_level = IMX_VPU_API_H264_LEVEL_5_1,
	.max_main_profile_level = IMX_VPU_API_H264_LEVEL_5_1,
	.max_high_profile_level = IMX_VPU_API_H264_LEVEL_5_1,
	.max_high10_profile_level = IMX_VPU_API_H264_LEVEL_5_1,

	.flags = 0
};

static ImxVpuApiH265SupportDetails const enc_h265_support_details = {
	.parent = {
		.min_width = 32, .max_width = 1920,
		.min_height = 32, .max_height = 1080,
		.supported_color_formats = enc_supported_basic_color_formats,
		.num_supported_color_formats = sizeof(enc_supported_basic_color_formats) / sizeof(ImxVpuApiColorFormat),
		.min_quantization = 1, .max_quantization = 51
	},

	.max_main_profile_level = IMX_VPU_API_H265_LEVEL_5_2,
	.max_main10_profile_level = IMX_VPU_API_H265_LEVEL_5_2,

	.flags = IMX_VPU_API_H265_FLAG_ACCESS_UNITS_SUPPORTED
};


ImxVpuApiCompressionFormatSupportDetails const * imx_vpu_api_enc_get_compression_format_support_details(ImxVpuApiCompressionFormat compression_format)
{
	switch (compression_format)
	{
		case IMX_VPU_API_COMPRESSION_FORMAT_H264:
			return (ImxVpuApiCompressionFormatSupportDetails const *)(&enc_h264_support_details);

		case IMX_VPU_API_COMPRESSION_FORMAT_H265:
			return (ImxVpuApiCompressionFormatSupportDetails const *)(&enc_h265_support_details);

		default:
			return NULL;
	}

	return NULL;
}


void imx_vpu_api_enc_set_default_open_params(ImxVpuApiCompressionFormat compression_format, ImxVpuApiColorFormat color_format, size_t frame_width, size_t frame_height, ImxVpuApiEncOpenParams *open_params)
{
	assert(open_params != NULL);

	open_params->frame_width = frame_width;
	open_params->frame_height = frame_height;
	open_params->compression_format = compression_format;
	open_params->color_format = color_format;
	open_params->bitrate = 256;
	open_params->quantization = 0;
	open_params->gop_size = 16;
	open_params->min_intra_refresh_mb_count = 0;
	open_params->frame_rate_numerator = 25;
	open_params->frame_rate_denominator = 1;
	open_params->flags &= ~IMX_VPU_API_ENC_H26x_OPEN_PARAMS_FLAG_USE_HRD;
	open_params->hrd_buffer_size = 0;
	open_params->intra_qp_delta = 0;

	switch (compression_format)
	{
		case IMX_VPU_API_COMPRESSION_FORMAT_H264:
			open_params->format_specific_open_params.h264_open_params.profile = IMX_VPU_API_H264_PROFILE_BASELINE;
			open_params->format_specific_open_params.h264_open_params.level = IMX_VPU_API_H264_LEVEL_UNDEFINED;
			open_params->format_specific_open_params.h264_open_params.enable_access_unit_delimiters = 0;
			break;

		case IMX_VPU_API_COMPRESSION_FORMAT_H265:
			open_params->format_specific_open_params.h265_open_params.profile = IMX_VPU_API_H265_PROFILE_MAIN;
			open_params->format_specific_open_params.h265_open_params.level = IMX_VPU_API_H265_LEVEL_UNDEFINED;
			open_params->format_specific_open_params.h265_open_params.enable_access_unit_delimiters = 0;
			break;

		default:
			break;
	}
}


typedef struct { const uint8_t *d; size_t len; size_t bit; } VcBR;
static unsigned vc_br_u1(VcBR *b){ unsigned v=0; if((b->bit>>3)<b->len) v=(b->d[b->bit>>3]>>(7-(b->bit&7)))&1u; b->bit++; return v; }
static unsigned vc_br_un(VcBR *b,int n){ unsigned v=0; while(n-->0) v=(v<<1)|vc_br_u1(b); return v; }
static unsigned vc_br_ue(VcBR *b){ int z=0; while(vc_br_u1(b)==0 && (b->bit>>3)<b->len) z++; unsigned v=(z<32)?((1u<<z)-1u):0xffffffffu; v+=vc_br_un(b,z); return v; }
typedef struct { uint8_t *d; size_t cap; size_t bit; } VcBW;
static void vc_bw_u1(VcBW *w,unsigned v){ size_t i=w->bit>>3; if(i<w->cap){ uint8_t m=(uint8_t)(1u<<(7-(w->bit&7))); if(v) w->d[i]|=m; else w->d[i]&=(uint8_t)~m; } w->bit++; }
static void vc_bw_un(VcBW *w,unsigned v,int n){ for(int i=n-1;i>=0;i--) vc_bw_u1(w,(v>>i)&1u); }
static void vc_bw_ue(VcBW *w,unsigned v){ unsigned val=v+1u; int n=0; unsigned t=val; while(t>1u){t>>=1;n++;} for(int i=0;i<n;i++) vc_bw_u1(w,0); for(int i=n;i>=0;i--) vc_bw_u1(w,(val>>i)&1u); }
static void vc_cp_bits(VcBR*r,VcBW*w,int n){ while(n-->0) vc_bw_u1(w,vc_br_u1(r)); }
static unsigned vc_cp_ue(VcBR*r,VcBW*w){ unsigned v=vc_br_ue(r); vc_bw_ue(w,v); return v; }
static unsigned vc_cp_un(VcBR*r,VcBW*w,int n){ unsigned v=vc_br_un(r,n); vc_bw_un(w,v,n); return v; }
static size_t vc_deemulate(const uint8_t*p,size_t n,uint8_t*rbsp,size_t cap){ size_t o=0,z=0; for(size_t i=0;i<n&&o<cap;i++){ if(z>=2&&p[i]==3){z=0;continue;} rbsp[o++]=p[i]; if(p[i]==0)z++; else z=0; } return o; }
static size_t vc_emulate(const uint8_t*rbsp,size_t n,uint8_t*out,size_t cap){ size_t o=0,z=0; for(size_t i=0;i<n;i++){ if(z>=2&&rbsp[i]<=3){ if(o<cap)out[o++]=3; z=0; } if(o<cap)out[o++]=rbsp[i]; if(rbsp[i]==0)z++; else z=0; } return o; }
static size_t vc_sps_rewrite_rbsp(const uint8_t*in,size_t in_len,uint8_t*out,size_t out_cap)
{
	long L=(long)in_len-1; while(L>=0&&in[L]==0)L--; if(L<0)return 0;
	int r=0; while(((in[L]>>r)&1u)==0)r++;
	size_t stop_bit=(size_t)L*8+(size_t)(7-r);
	VcBR br={in,in_len,0}; VcBW bw={out,out_cap,0}; memset(out,0,out_cap);
	vc_cp_un(&br,&bw,4); unsigned max_sub=vc_cp_un(&br,&bw,3); vc_cp_un(&br,&bw,1);
	if(max_sub!=0) return 0;
	vc_cp_bits(&br,&bw,96);
	vc_cp_ue(&br,&bw);
	unsigned chroma=vc_cp_ue(&br,&bw); if(chroma==3) vc_cp_un(&br,&bw,1);
	vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw);
	unsigned conf=vc_cp_un(&br,&bw,1); if(conf){ vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); }
	vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw);
	vc_cp_un(&br,&bw,1);
	vc_cp_ue(&br,&bw);
	unsigned reorder=vc_br_ue(&br); vc_bw_ue(&bw,0);
	vc_cp_ue(&br,&bw);
	if(reorder==0) return 0;
	while(br.bit<stop_bit) vc_bw_u1(&bw,vc_br_u1(&br));
	vc_bw_u1(&bw,1); while(bw.bit&7) vc_bw_u1(&bw,0);
	return bw.bit>>3;
}
static void vc8000_hevc_force_no_reorder(uint8_t *data,size_t *size)
{
	size_t sz=*size,p=0;
	while(p+3<=sz && !(data[p]==0&&data[p+1]==0&&data[p+2]==1)) p++;
	while(p+3<=sz && data[p]==0&&data[p+1]==0&&data[p+2]==1){
		size_t nal=p+3,q=nal;
		while(q+3<=sz && !(data[q]==0&&data[q+1]==0&&data[q+2]==1)) q++;
		size_t nal_end=(q+3<=sz)?q:sz;
		int type=(data[nal]>>1)&0x3f;
		if(type==33 && nal_end-nal>2){
			uint8_t rbsp[512],nr[512],em[560];
			size_t rl=vc_deemulate(data+nal+2,(nal_end-nal)-2,rbsp,sizeof rbsp);
			size_t nl=vc_sps_rewrite_rbsp(rbsp,rl,nr,sizeof nr);
			if(nl>0){
				size_t el=vc_emulate(nr,nl,em,sizeof em);
				size_t new_nal=2+el, old_nal=nal_end-nal;
				if(new_nal<=old_nal){
					memcpy(data+nal+2,em,el);
					size_t delta=old_nal-new_nal;
					if(delta){ memmove(data+nal+new_nal,data+nal_end,sz-nal_end); sz-=delta; }
					*size=sz;
				}
			}
			return;
		}
		p=nal_end;
	}
}

static int vc_cp_hrd(VcBR*r,VcBW*w){
	unsigned cpb=vc_cp_ue(r,w); vc_cp_un(r,w,4); vc_cp_un(r,w,4);
	for(unsigned i=0;i<=cpb && i<32;i++){ vc_cp_ue(r,w); vc_cp_ue(r,w); vc_cp_un(r,w,1); }
	vc_cp_un(r,w,5); vc_cp_un(r,w,5); vc_cp_un(r,w,5); vc_cp_un(r,w,5);
	return 1;
}
static size_t vc_sps_rewrite_rbsp_h264(const uint8_t*in,size_t in_len,uint8_t*out,size_t out_cap)
{
	long L=(long)in_len-1; while(L>=0&&in[L]==0)L--; if(L<0)return 0;
	int rr=0; while(((in[L]>>rr)&1u)==0)rr++;
	size_t stop_bit=(size_t)L*8+(size_t)(7-rr);
	VcBR br={in,in_len,0}; VcBW bw={out,out_cap,0}; memset(out,0,out_cap);
	unsigned prof=vc_cp_un(&br,&bw,8); vc_cp_un(&br,&bw,8); vc_cp_un(&br,&bw,8);
	vc_cp_ue(&br,&bw);
	if(prof==100||prof==110||prof==122||prof==244||prof==44||prof==83||prof==86||prof==118||prof==128||prof==138||prof==139||prof==134||prof==135){
		unsigned cf=vc_cp_ue(&br,&bw); if(cf==3) vc_cp_un(&br,&bw,1);
		vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); vc_cp_un(&br,&bw,1);
		if(vc_cp_un(&br,&bw,1)) return 0;
	}
	vc_cp_ue(&br,&bw);
	unsigned poc=vc_cp_ue(&br,&bw);
	if(poc==0) vc_cp_ue(&br,&bw);
	else if(poc==1){ vc_cp_un(&br,&bw,1); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); unsigned n=vc_cp_ue(&br,&bw); for(unsigned i=0;i<n && i<256;i++) vc_cp_ue(&br,&bw); }
	vc_cp_ue(&br,&bw); vc_cp_un(&br,&bw,1); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw);
	if(!vc_cp_un(&br,&bw,1)) vc_cp_un(&br,&bw,1);
	vc_cp_un(&br,&bw,1);
	if(vc_cp_un(&br,&bw,1)){ vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); }
	if(!vc_cp_un(&br,&bw,1)) return 0;
	if(vc_cp_un(&br,&bw,1)){ if(vc_cp_un(&br,&bw,8)==255){ vc_cp_un(&br,&bw,16); vc_cp_un(&br,&bw,16); } }
	if(vc_cp_un(&br,&bw,1)) vc_cp_un(&br,&bw,1);
	if(vc_cp_un(&br,&bw,1)){ vc_cp_un(&br,&bw,3); vc_cp_un(&br,&bw,1); if(vc_cp_un(&br,&bw,1)){ vc_cp_un(&br,&bw,8); vc_cp_un(&br,&bw,8); vc_cp_un(&br,&bw,8); } }
	if(vc_cp_un(&br,&bw,1)){ vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); }
	if(vc_cp_un(&br,&bw,1)){ vc_cp_bits(&br,&bw,32); vc_cp_bits(&br,&bw,32); vc_cp_un(&br,&bw,1); }
	unsigned nal_hrd=vc_cp_un(&br,&bw,1); if(nal_hrd) vc_cp_hrd(&br,&bw);
	unsigned vcl_hrd=vc_cp_un(&br,&bw,1); if(vcl_hrd) vc_cp_hrd(&br,&bw);
	if(nal_hrd||vcl_hrd) vc_cp_un(&br,&bw,1);
	vc_cp_un(&br,&bw,1);
	if(!vc_cp_un(&br,&bw,1)) return 0;
	vc_cp_un(&br,&bw,1); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw); vc_cp_ue(&br,&bw);
	unsigned reorder=vc_br_ue(&br); vc_bw_ue(&bw,0);
	vc_cp_ue(&br,&bw);
	if(reorder==0) return 0;
	if(br.bit>stop_bit+1) return 0;
	while(br.bit<stop_bit) vc_bw_u1(&bw,vc_br_u1(&br));
	vc_bw_u1(&bw,1); while(bw.bit&7) vc_bw_u1(&bw,0);
	return bw.bit>>3;
}
static void vc8000_h264_force_no_reorder(uint8_t *data,size_t *size)
{
	size_t sz=*size,p=0;
	while(p+3<=sz && !(data[p]==0&&data[p+1]==0&&data[p+2]==1)) p++;
	while(p+3<=sz && data[p]==0&&data[p+1]==0&&data[p+2]==1){
		size_t nal=p+3,q=nal;
		while(q+3<=sz && !(data[q]==0&&data[q+1]==0&&data[q+2]==1)) q++;
		size_t nal_end=(q+3<=sz)?q:sz;
		int type=data[nal]&0x1f;
		if(type==7 && nal_end-nal>1){
			uint8_t rbsp[512],nr[512],em[560];
			size_t rl=vc_deemulate(data+nal+1,(nal_end-nal)-1,rbsp,sizeof rbsp);
			size_t nl=vc_sps_rewrite_rbsp_h264(rbsp,rl,nr,sizeof nr);
			if(nl>0){
				size_t el=vc_emulate(nr,nl,em,sizeof em);
				size_t new_nal=1+el, old_nal=nal_end-nal;
				if(new_nal<=old_nal){
					memcpy(data+nal+1,em,el);
					size_t delta=old_nal-new_nal;
					if(delta){ memmove(data+nal+new_nal,data+nal_end,sz-nal_end); sz-=delta; }
					*size=sz;
				}
			}
			return;
		}
		p=nal_end;
	}
}


/* Parameter set ids.
 *
 * A resolution change means a new encoder instance, and the vendor library
 * numbers the parameter sets of every instance from zero - VCEncStrmStart()
 * writes SPS 0 and PPS 0 and resets the active PPS id to 0. The stream that
 * comes out therefore describes its new frame size with the same ids as the
 * old one, and a decoder that loses the parameter sets sent at the switch
 * point goes on using the ones it already has: it decodes the new resolution
 * as the old one, with nothing in the bitstream to tell it otherwise.
 *
 * So the ids are restamped, per resolution. Two of the three places they
 * appear are in the cached SPS/PPS header, which is ours to rewrite; the third
 * is slice_pic_parameter_set_id in every slice header, written by the hardware
 * from the active PPS id, and that one is moved by creating a PPS with the id
 * this resolution owns and activating it (see activate_param_set_pps()).
 *
 * Both halves are needed. With only the SPS id moved, a decoder that receives
 * the PPS but not the SPS is left with a PPS pointing at an SPS it does not
 * have and drops cleanly - but one that receives the SPS and not the PPS still
 * has the previous PPS, which now points at the wrong SPS, and that is worse
 * than before the change. With the PPS id moved too, both cases end in a
 * reference the decoder cannot resolve, which is what it can act on. */

/* Position of the rbsp_stop_one_bit, i.e. of the end of the syntax. 0 if the
 * data does not end in one. */
static size_t vc_rbsp_stop_bit(const uint8_t *in, size_t in_len)
{
	long L = (long)in_len - 1;
	int r = 0;

	while ((L >= 0) && (in[L] == 0)) L--;
	if (L < 0) return 0;
	while (((in[L] >> r) & 1u) == 0) r++;

	return (size_t)L * 8 + (size_t)(7 - r);
}

/* Copies what is left of the syntax across verbatim and terminates the RBSP.
 * Returns the byte length written, or 0 if the reader has already overrun. */
static size_t vc_restamp_tail(VcBR *br, VcBW *bw, size_t stop_bit)
{
	if (br->bit > stop_bit) return 0;
	while (br->bit < stop_bit) vc_bw_u1(bw, vc_br_u1(br));
	vc_bw_u1(bw, 1);
	while (bw->bit & 7) vc_bw_u1(bw, 0);
	/* vc_bw_u1() drops what does not fit rather than failing, so the length
	 * is the only thing that says whether all of it was written. */
	if ((bw->bit >> 3) > bw->cap) return 0;
	return bw->bit >> 3;
}

static size_t vc_h265_sps_restamp(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap, unsigned id)
{
	size_t const stop_bit = vc_rbsp_stop_bit(in, in_len);
	VcBR br = { in, in_len, 0 };
	VcBW bw = { out, out_cap, 0 };
	unsigned max_sub;

	if (stop_bit == 0) return 0;
	memset(out, 0, out_cap);

	vc_cp_un(&br, &bw, 4);                     /* sps_video_parameter_set_id */
	max_sub = vc_cp_un(&br, &bw, 3);           /* sps_max_sub_layers_minus1 */
	vc_cp_un(&br, &bw, 1);                     /* sps_temporal_id_nesting_flag */
	/* profile_tier_level() is 96 bits only with a single sub layer, which is
	 * all this encoder produces. */
	if (max_sub != 0) return 0;
	vc_cp_bits(&br, &bw, 96);
	(void)vc_br_ue(&br); vc_bw_ue(&bw, id);    /* sps_seq_parameter_set_id */

	return vc_restamp_tail(&br, &bw, stop_bit);
}

static size_t vc_h264_sps_restamp(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap, unsigned id)
{
	size_t const stop_bit = vc_rbsp_stop_bit(in, in_len);
	VcBR br = { in, in_len, 0 };
	VcBW bw = { out, out_cap, 0 };

	if (stop_bit == 0) return 0;
	memset(out, 0, out_cap);

	vc_cp_un(&br, &bw, 8);                     /* profile_idc */
	vc_cp_un(&br, &bw, 8);                     /* constraint flags + reserved */
	vc_cp_un(&br, &bw, 8);                     /* level_idc */
	(void)vc_br_ue(&br); vc_bw_ue(&bw, id);    /* seq_parameter_set_id */

	return vc_restamp_tail(&br, &bw, stop_bit);
}

/* The first two fields of a PPS are the same in both codecs: its own id, then
 * the id of the SPS it activates. Both move together, so that the PPS this
 * resolution owns points at the SPS this resolution owns. */
static size_t vc_pps_restamp(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap, unsigned id)
{
	size_t const stop_bit = vc_rbsp_stop_bit(in, in_len);
	VcBR br = { in, in_len, 0 };
	VcBW bw = { out, out_cap, 0 };

	if (stop_bit == 0) return 0;
	memset(out, 0, out_cap);

	(void)vc_br_ue(&br); vc_bw_ue(&bw, id);    /* pps_pic_parameter_set_id */
	(void)vc_br_ue(&br); vc_bw_ue(&bw, id);    /* pps_seq_parameter_set_id */

	return vc_restamp_tail(&br, &bw, stop_bit);
}

/* Restamps the SPS and PPS of a cached annex B header with id. Everything else
 * in the header - the start codes, whatever their length, the nal unit headers,
 * the VPS - is copied through byte for byte.
 *
 * An id costs more bits than the 0 it replaces, so this cannot be done in
 * place; on success *header_data points at a new buffer and the old one has
 * been freed. Returns 0 and leaves the header untouched if anything about it
 * does not parse, which is the safe outcome: the stream then carries the ids
 * the encoder wrote, as it did before this existed. */
static int vc8000_restamp_param_set_ids(uint8_t **header_data, size_t *header_size, BOOL is_h264, unsigned id)
{
	uint8_t *in = *header_data;
	size_t const in_size = *header_size;
	size_t const out_cap = in_size + 16;
	uint8_t *out;
	size_t out_len = 0;
	size_t p = 0;
	int num_sps = 0, num_pps = 0;

	if ((in == NULL) || (in_size < 4)) return 0;

	out = malloc(out_cap);
	if (out == NULL) return 0;

	/* Anything ahead of the first start code (there is none in practice). */
	while ((p + 3 <= in_size) && !((in[p] == 0) && (in[p+1] == 0) && (in[p+2] == 1))) p++;
	if (p + 3 > in_size) goto fail;
	memcpy(out, in, p);
	out_len = p;

	while (p + 3 <= in_size)
	{
		size_t nal, q, nal_end, pad, payload_len, hdr_len;
		int type;
		size_t rewritten = 0;

		nal = p + 3;                     /* first byte of the nal unit header */
		q = nal;
		while ((q + 3 <= in_size) && !((in[q] == 0) && (in[q+1] == 0) && (in[q+2] == 1))) q++;
		nal_end = (q + 3 <= in_size) ? q : in_size;

		/* Trailing zero bytes belong to the next start code, not to this nal
		 * unit: an RBSP always ends in its stop bit, so its last byte is
		 * never zero. Held back and re-emitted so that a four byte start
		 * code stays a four byte start code. */
		hdr_len = is_h264 ? 1 : 2;
		pad = 0;
		while ((nal_end - pad > nal + hdr_len) && (in[nal_end - pad - 1] == 0)) pad++;

		type = is_h264 ? (in[nal] & 0x1f) : ((in[nal] >> 1) & 0x3f);
		payload_len = (nal_end - pad > nal + hdr_len) ? (nal_end - pad - nal - hdr_len) : 0;

		if ((payload_len > 0) &&
		    ((is_h264 && ((type == 7) || (type == 8))) || (!is_h264 && ((type == 33) || (type == 34)))))
		{
			uint8_t rbsp[512], stamped[544], em[640];
			size_t rl;
			size_t sl = 0;

			/* Nothing this encoder emits comes close, and a parameter set
			 * that did would be truncated by the de-emulation rather than
			 * rejected by it. */
			if (payload_len > sizeof(rbsp))
				goto fail;

			rl = vc_deemulate(in + nal + hdr_len, payload_len, rbsp, sizeof(rbsp));

			if (rl > 0)
			{
				if (is_h264)
					sl = (type == 7) ? vc_h264_sps_restamp(rbsp, rl, stamped, sizeof(stamped), id)
					                 : vc_pps_restamp(rbsp, rl, stamped, sizeof(stamped), id);
				else
					sl = (type == 33) ? vc_h265_sps_restamp(rbsp, rl, stamped, sizeof(stamped), id)
					                  : vc_pps_restamp(rbsp, rl, stamped, sizeof(stamped), id);
			}

			if (sl == 0)
				goto fail;

			{
				size_t el = vc_emulate(stamped, sl, em, sizeof(em));
				size_t const chunk = (nal - p) + hdr_len + el + pad;

				if ((el < sl) || (out_len + chunk > out_cap))
					goto fail;

				memcpy(out + out_len, in + p, (nal - p) + hdr_len);
				out_len += (nal - p) + hdr_len;
				memcpy(out + out_len, em, el);
				out_len += el;
				memset(out + out_len, 0, pad);
				out_len += pad;
				rewritten = 1;
			}

			if ((is_h264 && (type == 7)) || (!is_h264 && (type == 33)))
				num_sps++;
			else
				num_pps++;
		}

		if (!rewritten)
		{
			if (out_len + (nal_end - p) > out_cap)
				goto fail;
			memcpy(out + out_len, in + p, nal_end - p);
			out_len += nal_end - p;
		}

		p = nal_end;
	}

	if ((num_sps != 1) || (num_pps != 1))
	{
		IMX_VPU_API_WARNING("header carries %d SPS and %d PPS; expected one of each", num_sps, num_pps);
		if ((num_sps == 0) || (num_pps == 0))
			goto fail;
	}

	free(in);
	*header_data = out;
	*header_size = out_len;

	return 1;

fail:
	free(out);
	return 0;
}



/* Work out what shape the refresh sweep has, from the unified open_params
 * fields plus the deprecated ones they replaced, and program the slice height
 * that used to be a side effect of the rolling modes.
 *
 * The mapping itself lives in intra_refresh.c so that this file, rcprobe and
 * anything else configuring the encoder cannot disagree about what
 * use-rolling-slices meant. The three modes those fields selected -
 * use_intra_refresh, rolling slices and rolling tiles - were never actually
 * different mechanisms: all three forced VCEncCodingCtrl's intraArea over a
 * band of CTB rows, and differed only in how the band was placed and whether
 * they also set sliceSize. So they become configurations of one scheduler, and
 * the schedules they produced are unchanged - which
 * tools/common/intra_refresh_test.c checks band for band. */
static void resolve_refresh_config(ImxVpuApiEncoder *encoder)
{
	ImxVpuApiEncOpenParams *open_params = &encoder->open_params;
	ImxVpuApiIntraRefreshRequest req;
	ImxVpuApiIntraRefreshPlan plan;

	/* The encoder counts geometry in its own coding units, and they are
	 * codec dependent: 64 pixels for h.265, 16 for h.264
	 * (hevcencapi.c:8593-8613, ceil on each dimension). Using the h.265
	 * unit on an h.264 stream does not fail - the band lands inside the
	 * picture and is accepted - it just refreshes the top quarter of it
	 * forever. */
	int const unit = (open_params->compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H264)
	               ? 16 : 64;

	memset(&req, 0, sizeof(req));
	req.unit_pixels = unit;
	req.ctb_rows = ((int)(open_params->frame_height) + unit - 1) / unit;
	req.ctb_cols = ((int)(open_params->frame_width) + unit - 1) / unit;
	req.gop_size = (int)(open_params->gop_size);
	req.enable = !!(open_params->flags & IMX_VPU_API_ENC_OPEN_PARAMS_FLAG_USE_INTRA_REFRESH);
	req.period = open_params->intra_refresh_period;
	req.duration = open_params->intra_refresh_duration;
	req.rows = open_params->intra_refresh_height;
	req.slice_height = open_params->slice_height;
	req.slice_count = open_params->slice_count;
	req.gdr_refresh_period = open_params->gdr_refresh_period;
	req.num_rolling_slices = open_params->num_rolling_slices;
	req.num_rolling_tiles = open_params->num_rolling_tiles;
	req.roll_size = open_params->roll_size;

	imx_vpu_api_intra_refresh_plan(&req, &(encoder->refresh_cfg), &plan);

	/* Which of the two mechanisms runs the sweep.
	 *
	 * rate_control_mode 0 is this encoder as it shipped, and that includes
	 * its own GDR: the vendor library places the intra band from gdrDuration,
	 * and neither the scheduler here nor the two SEIs that come with it
	 * appear in the stream. Only rate_control_mode 1 gets the caller-driven
	 * sweep - it exists because the vendor's cannot be paced independently of
	 * the IDR interval, and because it takes intraArea away from everything
	 * else that wants it.
	 *
	 * The deprecated rolling slices and tiles aliases are the exception. They
	 * were always caller driven, the vendor sweep can place neither a band
	 * per slice nor a half width region, and reproducing what they did before
	 * is the whole point of keeping them - so they stay on the scheduler in
	 * either mode.
	 *
	 * This keys off the requested rate control mode and not off whether the
	 * new CBR actually came up, so that a zero bitrate or a failed
	 * ext_rate_control_init() cannot quietly change which refresh mechanism
	 * the stream uses on top of losing the rate control. */
	encoder->vendor_gdr_active = plan.active
	                          && (open_params->rate_control_mode != 1)
	                          && (req.num_rolling_slices == 0)
	                          && (req.num_rolling_tiles == 0);
	encoder->refresh_active = plan.active && !encoder->vendor_gdr_active;
	encoder->slice_size = plan.slice_size;
	memset(&(encoder->refresh_state), 0, sizeof(encoder->refresh_state));

	if (!plan.active)
		return;

	/* Slices are programmed the same way whoever runs the sweep, and asking
	 * for a count the hardware cannot derive silently changed it before. */
	if ((open_params->slice_count > 1) && (plan.slice_count != (int)(open_params->slice_count)))
		IMX_VPU_API_INFO("slice-count %u is not achievable at %d CTB rows; "
		                 "using %d slices of %d rows",
		                 open_params->slice_count, encoder->refresh_cfg.ctb_rows,
		                 plan.slice_count, plan.slice_size);

	if (encoder->vendor_gdr_active)
	{
		/* The vendor sweep has one dial - the period - and derives the
		 * band height from it and the picture size. There is nowhere to
		 * put the rest, so say they are being dropped rather than report
		 * a schedule that is not the one running. */
		if ((open_params->intra_refresh_duration > 0)
		 && ((int)(open_params->intra_refresh_duration) != encoder->refresh_cfg.period))
			IMX_VPU_API_WARNING("intra-refresh-duration %u is ignored with the encoder's "
			                    "own GDR (rate-control=0): the sweep is spread over the "
			                    "whole %d picture period",
			                    open_params->intra_refresh_duration,
			                    encoder->refresh_cfg.period);
		if (open_params->intra_refresh_height > 0)
			IMX_VPU_API_WARNING("intra-refresh-height %u is ignored with the encoder's own "
			                    "GDR (rate-control=0): it derives the band height from "
			                    "the period", open_params->intra_refresh_height);

		IMX_VPU_API_DEBUG("intra refresh: the encoder's own GDR, period %d, "
		                  "%d slice(s) of %d row(s)",
		                  encoder->refresh_cfg.period, plan.slice_count, plan.slice_size);
		return;
	}

	imx_vpu_api_intra_refresh_init(&(encoder->refresh_state), &(encoder->refresh_cfg));

	/* This silently changed the caller's configuration before, and it changes
	 * what the result means, so say so. */
	if (plan.wanted_steps > plan.num_steps)
		IMX_VPU_API_INFO("intra refresh: a %d picture sweep cannot hold %d regions; "
		                 "using %d row(s) of %d, so bands are %d CTB rows not %d",
		                 encoder->refresh_cfg.duration, plan.wanted_steps,
		                 plan.row_steps, plan.col_steps,
		                 encoder->refresh_cfg.ctb_rows / plan.row_steps,
		                 encoder->refresh_cfg.rows);

	IMX_VPU_API_DEBUG("intra refresh: period %d, duration %d, %d region(s) of "
	                  "%d row(s) x %d col(s) of %d px, %d slice(s) of %d row(s)",
	                  encoder->refresh_cfg.period, encoder->refresh_cfg.duration,
	                  plan.num_steps, encoder->refresh_cfg.rows,
	                  encoder->refresh_cfg.columns, unit,
	                  plan.slice_count, plan.slice_size);
}


static void init_encoder_input(ImxVpuApiEncoder *encoder)
{
	ImxVpuApiEncOpenParams *open_params = &encoder->open_params;
	VCEncIn *encoder_input = &encoder->encoder_input;
	int i;

	memset(encoder_input, 0, sizeof(VCEncIn));

	encoder_input->gopConfig.pGopPicCfg = &encoder->gop_pic_config[0];
	encoder_input->gopConfig.size = 1;
	encoder_input->gopConfig.special_size = 0;
	encoder_input->gopConfig.pGopPicSpecialCfg = &encoder->gop_pic_special_config[0];
	/* Whose GDR. The vendor's is one setting, and it is tied to the IDR
	 * interval: VCEncFindNextPic() starts a sweep after each IDR and asks for
	 * the intra picture that begins it, so the period has to be both.
	 *
	 * A sweep driven from this file wants the opposite - no periodic IDR, and
	 * gdrDuration left at zero so VCEncFindNextPic() never asks for an intra
	 * picture. Leaving the encoder's own GDR on as well would take the intra
	 * area away: it overwrites intraArea and roi1Area on every picture
	 * (hevcencapi.c:6340-6394) and rejects any intraArea set from here. */
	if (encoder->vendor_gdr_active)
	{
		encoder_input->gopConfig.gdrDuration = encoder->refresh_cfg.period;
		encoder_input->gopConfig.idr_interval = encoder->refresh_cfg.period;
	}
	else
	{
		encoder_input->gopConfig.idr_interval = encoder->refresh_active
		                                      ? INT32_MAX : open_params->gop_size;
		encoder_input->gopConfig.gdrDuration = 0;
	}
	encoder_input->gopConfig.firstPic = 0;
	encoder_input->gopConfig.lastPic = INT32_MAX;
	encoder_input->gopConfig.outputRateNumer = open_params->frame_rate_numerator;
	encoder_input->gopConfig.outputRateDenom = open_params->frame_rate_denominator;
	encoder_input->gopConfig.inputRateNumer = open_params->frame_rate_numerator;
	encoder_input->gopConfig.inputRateDenom = open_params->frame_rate_denominator;
	encoder_input->gopConfig.gopCfgOffset[0] = 0;

	encoder_input->gopCurrPicConfig.codingType = FRAME_TYPE_RESERVED;
	encoder_input->gopCurrPicConfig.numRefPics = NUMREFPICS_RESERVED;
	encoder_input->gopCurrPicConfig.poc = -1;
	encoder_input->gopCurrPicConfig.QpFactor = QPFACTOR_RESERVED;
	encoder_input->gopCurrPicConfig.QpOffset = QPOFFSET_RESERVED;
	encoder_input->gopCurrPicConfig.temporalId = TEMPORALID_RESERVED;
	for (i = 0; i < VCENC_MAX_REF_FRAMES; ++i)
	{
		encoder_input->gopCurrPicConfig.refPics[i].ref_pic = -1;
		encoder_input->gopCurrPicConfig.refPics[i].used_by_cur = 0;
	}

	encoder_input->bIsPeriodUsingLTR = HANTRO_TRUE;
	encoder_input->bIsPeriodUpdateLTR = HANTRO_TRUE;
	for (i = 0; i < VCENC_MAX_LT_REF_FRAMES; ++i)
		encoder_input->long_term_ref_pic[i] = -1;

	encoder_input->vui_timing_info_enable = 1;
	encoder_input->poc = 0;
	encoder_input->gopSize = 1;
	encoder_input->picture_cnt = 0;
	encoder_input->last_idr_picture_cnt = 0;
	encoder_input->bIsIDR = HANTRO_TRUE;
	encoder_input->sendAUD = (open_params->compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H264 || open_params->compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H265)
	                      ? 0
	                      : open_params->format_specific_open_params.h265_open_params.enable_access_unit_delimiters;
	encoder_input->i8SpecialRpsIdx = -1;

	encoder_input->pOutBuf[1] = NULL;
	encoder_input->busOutBuf[1] = 0;
	encoder_input->outBufSize[1] = 0;
}


static ImxVpuApiEncReturnCodes init_vcenc_instance(ImxVpuApiEncoder *encoder,
                                                   VCEncPictureType encoder_pixel_format,
                                                   ImxVpuApiFramebufferMetrics const *fb_metrics)
{
	ImxVpuApiEncOpenParams *open_params = &encoder->open_params;
	VCEncConfig *encoder_config = &encoder->encoder_config;
	VCEncRet enc_ret;

	/* Initialize the actual encoder. */
	enc_ret = VCEncInit(encoder_config, &encoder->encoder);
	if (enc_ret != VCENC_OK)
	{
		IMX_VPU_API_ERROR("could not initialize encoder: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
		encoder->encoder = NULL;
		return IMX_VPU_API_ENC_RETURN_CODE_ERROR;
	}

	/* Set up the encoder's coding configuration. */
	{
		VCEncCodingCtrl coding_config;
		memset(&coding_config, 0, sizeof(coding_config));

		/* Use the full 0..255 range for RGB -> YUV color space conversions.
		 * Otherwise, the encoder assumes that the range is 16..235 for Y
		 * and 16..240 for U and V. */
		coding_config.videoFullRange = imx_vpu_api_is_color_format_rgb(open_params->color_format) ? 1 : 0;
		/* Enable the sample adaptive offset (SAO) filter. */
		coding_config.enableSao = 1;
		/* h.264 baseline profile uses CAVLC instead of CABAC. */
		coding_config.enableCabac = (open_params->compression_format != IMX_VPU_API_COMPRESSION_FORMAT_H264)
		                         || (open_params->format_specific_open_params.h264_open_params.profile != IMX_VPU_API_H264_PROFILE_BASELINE);
		coding_config.cirStart = 0;
		coding_config.cirInterval = open_params->min_intra_refresh_mb_count;
		/* Nonzero only for the encoder's own GDR. It also forces
		 * cu_qp_delta_enabled_flag on in the PPS (hevcencapi.c:2324) and
		 * stamps roi1DeltaQp onto the refresh band - part of what that
		 * mechanism is, and neither of which a sweep driven from this file
		 * wants. */
		coding_config.gdrDuration = encoder->vendor_gdr_active
		                          ? encoder->refresh_cfg.period : 0;

		/* These are set to the defaults specified in hevcencapi.h */
		coding_config.noiseLow = 10;
		coding_config.firstFrameSigma = 11;
		/* Set these to what VCEncGetCodingCtrl() returns when called
		 * right after opening the encoder. (In other words, set these
		 * to the defaults of the encoder.) Only nonzero defaults are
		 * assigned here; fields that are set to zero by default are
		 * already zero due to the memset() call above. */
		coding_config.tc_Offset = -2;
		coding_config.beta_Offset = 5;
		coding_config.RoiQpDelta_ver = 1;
		coding_config.streamMultiSegmentAmount = 1;


		/* Slices are full width horizontal bands and nothing else, and
		 * only the height is programmable - the hardware derives the
		 * count as ceil(ctbPerCol / sliceSize) (hevcencapi.c:1780).
		 * resolve_refresh_config() has already turned a requested count
		 * into a height that exists. */
		coding_config.sliceSize = encoder->slice_size;

		enc_ret = VCEncSetCodingCtrl(encoder->encoder, &coding_config);
		if (enc_ret != VCENC_OK)
		{
			IMX_VPU_API_ERROR("could not set coding configuration: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
			VCEncRelease(encoder->encoder);
			encoder->encoder = NULL;
			return IMX_VPU_API_ENC_RETURN_CODE_ERROR;
		}

		VCEncGetCodingCtrl(encoder->encoder, &encoder->cached_coding_ctrl);

		imx_vpu_api_intra_refresh_init(&(encoder->refresh_state), &(encoder->refresh_cfg));
		encoder->forced_intra_q_head = 0;
		encoder->forced_intra_q_count = 0;
	}



	/* Set up rate control. */

	{
		VCEncRateCtrl rate_control_config;
		memset(&rate_control_config, 0, sizeof(rate_control_config));
		BOOL use_rate_control = (open_params->bitrate != 0);
		BOOL use_new_cbr = (open_params->rate_control_mode == 1) && use_rate_control;

		/* This runs again on an auto-recovery restart, so the controller is
		 * always rebuilt from scratch rather than resumed - a restart is a
		 * new stream, and the bucket level from before it describes a link
		 * state that no longer applies. */
		encoder->new_cbr_active = FALSE;

		if (use_new_cbr)
		{
			ExtRateControlParams rc_params;

			imx_vpu_api_enc_session_rc_params(&rc_params, open_params);
			/* Whole intra pictures, or a sweep? The controller treats a
			 * keyframe it cannot fit differently in each case, and only
			 * the encoder knows which mechanism the plan resolved to. */
			rc_params.keyframe_mode = !encoder->refresh_active;

			if (ext_rate_control_init(&encoder->new_cbr, &rc_params) != 0)
			{
				IMX_VPU_API_ERROR("could not initialize the new CBR rate control; falling back to the encoder's own");
				use_new_cbr = FALSE;
			}
			else
			{
				encoder->new_cbr_active = TRUE;
				IMX_VPU_API_INFO(
					"new CBR: %u kbps, %.2f fps, %.0f kbit HRD buffer (%.0f ms at this rate), cap %.0f kbit/picture, QP %d..%d",
					open_params->bitrate,
					encoder->new_cbr.frame_rate,
					encoder->new_cbr.bucket_cap / 1000.0,
					encoder->new_cbr.bucket_cap * 1000.0 / (double)(open_params->bitrate * 1000),
					(encoder->new_cbr.bucket_cap * encoder->new_cbr.cap_share) / 1000.0,
					encoder->new_cbr.qp_min_inter,
					encoder->new_cbr.qp_max_inter
				);
			}
		}

		if (use_rate_control)
			IMX_VPU_API_INFO("using constant bitrate encoding with bitrate set to %u kbps", open_params->bitrate);
		else
			IMX_VPU_API_INFO("using constant quality encoding with quantization set to %u", open_params->quantization);

		/* Enable rate control if a bitrate is given. */
		rate_control_config.pictureRc = use_rate_control ? 1 : 0;
		/* If rate control is disabled, use the quantization
		 * value for the QP values. */
		rate_control_config.qpHdr = use_rate_control ? -1 : ((int)(open_params->quantization));
		rate_control_config.qpMinI = use_rate_control ? (int)open_params->qp_min_intra : (int)open_params->quantization;
		rate_control_config.qpMinPB = use_rate_control ? (int)open_params->qp_min_inter : (int)open_params->quantization;
		rate_control_config.qpMaxI = use_rate_control
		                           ? ((open_params->qp_max_intra > 0) ? (int)open_params->qp_max_intra : 51)
		                           : (int)open_params->quantization;
		rate_control_config.qpMaxPB = use_rate_control
		                            ? ((open_params->qp_max_inter > 0) ? (int)open_params->qp_max_inter : 51)
		                            : (int)open_params->quantization;
		/* Set the bitrate, in bps. open_params->bitrate is given
		 * in kbps, so a multiplication by 1000 is necessary. */
		rate_control_config.bitPerSecond = open_params->bitrate * 1000;
		/* Number of frames to monitor for a moving bitrate. Use
		 * a timespan of one second. Since this is an integer,
		 * we must convert the fps numerator/denominator
		 * fraction and round it up. */
		rate_control_config.monitorFrames = (open_params->frame_rate_numerator + open_params->frame_rate_denominator - 1) / open_params->frame_rate_denominator;
		/* Enforce a minimum of 5 frames to monitor, otherwise
		 * rate control may produce garbage. */
		if (rate_control_config.monitorFrames < 5)
			rate_control_config.monitorFrames = 5;
		/* Enable VBR only if bitrate based rate control is not being used. */
		rate_control_config.vbr = use_rate_control ? 0 : 1;

		/* These defaults were taken from NXP's imx-vpuwrap library. */
		rate_control_config.bitVarRangeI = 10000;
		rate_control_config.bitVarRangeP = 10000;
		rate_control_config.bitVarRangeB = 10000;
		rate_control_config.u32StaticSceneIbitPercent = 80;
		if (open_params->compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H264)
		{
			rate_control_config.blockRCSize = 2;
			rate_control_config.ctbRcRowQpStep = 4;
		}
		else
		{
			rate_control_config.blockRCSize = 0;
			rate_control_config.ctbRcRowQpStep = 16;
		}

		/* Set these to what VCEncGetRateCtrl() returns when called
		 * right after opening the encoder. (In other words, set these
		 * to the defaults of the encoder.) Only nonzero defaults are
		 * assigned here; fields that are set to zero by default are
		 * already zero due to the memset() call above. */
		rate_control_config.hrd = !!(open_params->flags & IMX_VPU_API_ENC_H26x_OPEN_PARAMS_FLAG_USE_HRD);
		/* 0 means one second of bitrate, and the hardware model gets the
		 * five frame budget floor it needs - see
		 * imx_vpu_api_enc_hrd_buffer_bits(). */
		rate_control_config.hrdCpbSize = imx_vpu_api_enc_hrd_buffer_bits(open_params, 5);
		rate_control_config.bitrateWindow = open_params->gop_size;
		rate_control_config.intraQpDelta = open_params->intra_qp_delta;
		rate_control_config.tolMovingBitRate = 2000;
		rate_control_config.rcQpDeltaRange = 10;
		rate_control_config.rcBaseMBComplexity = 15;
		rate_control_config.picQpDeltaMin = -2;
		rate_control_config.picQpDeltaMax = +3;
		rate_control_config.tolCtbRcIntra = -1;

		/* Smooth-CBR mode: when HRD/VBV is enabled (use-hrd=1), tighten the otherwise very
		 * loose rate-control limits so the output bitrate stays flat (low jitter for RTP over
		 * constrained/radio links). hrdCpbSize (from hrd-buffer-size) is the hard CPB bound;
		 * these reinforce it. Pair with use-intra-refresh so there are no IDR/full-intra
		 * spikes either. Values are conservative defaults -- tune on-device for the
		 * smoothness/quality trade-off. Only active when HRD is requested, so other modes
		 * keep the original behaviour. */
		if (rate_control_config.hrd && !use_new_cbr)
		{
			rate_control_config.ctbRc = 1;             /* per-CTB QP: spread bits within a frame */
			if (rate_control_config.blockRCSize == 0)
				rate_control_config.blockRCSize = 1;    /* 32x32 CTB-RC blocks for HEVC */
			rate_control_config.bitVarRangeI = 100;    /* was 10000: clamp per-frame variation */
			rate_control_config.bitVarRangeP = 100;
			rate_control_config.bitVarRangeB = 100;
			rate_control_config.tolMovingBitRate = 50; /* was 2000: track target bitrate tightly */
			/* Asymmetric per-frame QP authority (Hantro defaults -2..+3): fast UP
			 * so the RC can re-price the periodic GDR refresh frame and motion
			 * onsets within one frame (flat standstill bitrate; symmetric -10..+10
			 * did this too), slow DOWN so the recovery cannot overshoot and
			 * re-spike (-10 caused RC hunting under sustained motion). */
			rate_control_config.picQpDeltaMin = -3;
			rate_control_config.picQpDeltaMax = 10;
			/* Cap the static-scene refresh spike: the VC8000E boosts intra bits in detected
			 * static scenes (u32StaticSceneIbitPercent, default 80), and under CBR with idle
			 * P-frames the rate control has surplus budget it dumps into the periodic GDR
			 * refresh frame at a low QP -> a fat periodic spike when nothing moves. Disable
			 * the static-scene intra boost and floor the P/B QP (qpMaxPB is already 51) so the
			 * refresh P-frame cannot balloon; surplus then sags the rate slightly, not spikes. */
			rate_control_config.u32StaticSceneIbitPercent = open_params->static_scene_ibit_percent; /* config: static-scene-ibit-percent */
			/* qpMinPB comes from open_params->qp_min_inter (config: qp-min); set above. */

			/* NOTE: on CPB overflow VCEncStrmEncode returns VCENC_HRD_ERROR;
			 * that is the HRD's graceful per-picture skip and is handled as a
			 * skipped frame in imx_vpu_api_enc_encode() - it must NOT trigger
			 * the lost-IRQ auto-recovery restart (restart storm = frozen
			 * stream under sustained overload, e.g. darkness sensor noise). */
		}

		if (use_new_cbr)
		{
			/* Everything that would otherwise decide a QP is switched off
			 * here, because the QP comes from ext_rate_control_pre()
			 * instead and any second opinion silently overrides it. */

			/* With pictureRc on, qpHdr is only a suggestion: the encoder
			 * quietly uses its own value, and every setting we make below
			 * produces byte-identical output. */
			rate_control_config.pictureRc = 0;
			rate_control_config.ctbRc = 0;
			rate_control_config.vbr = 0;
			rate_control_config.pictureSkip = 0;
			/* Our own leaky bucket does this job, and it does it without
			 * dropping pictures: the hardware HRD skips a picture after
			 * committing its POC, which leaves the decoder hunting for a
			 * reference that was never coded. */
			rate_control_config.hrd = 0;
			rate_control_config.hrdCpbSize = 0;
			/* VCEncGetRateCtrl does not write this field, so after the
			 * memset above it reads 0, which the encoder takes as "CRF
			 * enabled at QP 0". It has to be disabled explicitly. */
			rate_control_config.crf = -1;
			/* The intra QP bias would code intra pictures at a QP nobody
			 * asked for, which is exactly what the caller is trying to
			 * control here. */
			rate_control_config.intraQpDelta = 0;
			rate_control_config.fixedIntraQp = 0;
			rate_control_config.u32StaticSceneIbitPercent = 0;
			/* The full 0..51 range, bounded by the rate control itself.
			 * An arbitrary floor is not harmless: easy content at a
			 * generous bitrate genuinely needs the bottom of the range,
			 * and with a floor of 10 most pictures of gentle aerial
			 * footage sit pinned at it and the stream lands far under the
			 * requested rate. */
			rate_control_config.qpMinI  = encoder->new_cbr.qp_min_intra;
			rate_control_config.qpMaxI  = encoder->new_cbr.qp_max_intra;
			rate_control_config.qpMinPB = encoder->new_cbr.qp_min_inter;
			rate_control_config.qpMaxPB = encoder->new_cbr.qp_max_inter;
			/* 0: the header does not exist yet - VCEncStrmStart() below is
			 * what produces it - and the bootstrap allowance bounds this
			 * picture anyway. */
			rate_control_config.qpHdr = ext_rate_control_pre(&encoder->new_cbr, 1, 0);
		}

		enc_ret = VCEncSetRateCtrl(encoder->encoder, &rate_control_config);
		if (enc_ret != VCENC_OK)
		{
			IMX_VPU_API_ERROR("could not set rate control configuration: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
			VCEncRelease(encoder->encoder);
			encoder->encoder = NULL;
			return IMX_VPU_API_ENC_RETURN_CODE_ERROR;
		}

		/* Keep the configuration that was accepted, so per-picture QP
		 * updates can be made against it rather than against whatever
		 * VCEncGetRateCtrl happens to fill in. */
		encoder->cached_rate_ctrl = rate_control_config;
	}


	/* Set up preprocessing configuration. */

	{
		VCEncPreProcessingCfg preprocessing_config;
		memset(&preprocessing_config, 0, sizeof(preprocessing_config));

		preprocessing_config.origWidth = fb_metrics->aligned_frame_width;
		preprocessing_config.origHeight = fb_metrics->aligned_frame_height;
		preprocessing_config.xOffset = 0;
		preprocessing_config.yOffset = 0;
		preprocessing_config.inputType = encoder_pixel_format;
		/* The VC8000E PP rotates in its internal RGB domain while reading the
		 * input frame: no extra DDR pass, chroma-exact for any input format. */
		preprocessing_config.rotation = open_params->rotation_180 ? VCENC_ROTATE_180R : VCENC_ROTATE_0;
		preprocessing_config.mirror = VCENC_MIRROR_NO;
		preprocessing_config.colorConversion.type = VCENC_RGBTOYUV_BT601_FULL_RANGE;
		preprocessing_config.input_alignment = INPUT_ALIGNMENT;

		/* Set these to what VCEncGetPreProcessing() returns when called
		 * right after opening the encoder. (In other words, set these
		 * to the defaults of the encoder.) Only nonzero defaults are
		 * assigned here; fields that are set to zero by default are
		 * already zero due to the memset() call above. */
		preprocessing_config.constCb = 128;
		preprocessing_config.constCr = 128;

		enc_ret = VCEncSetPreProcessing(encoder->encoder, &preprocessing_config);
		if (enc_ret != VCENC_OK)
		{
			IMX_VPU_API_ERROR("could not set preprocessing configuration: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
			VCEncRelease(encoder->encoder);
			encoder->encoder = NULL;
			return IMX_VPU_API_ENC_RETURN_CODE_ERROR;
		}
	}

	return IMX_VPU_API_ENC_RETURN_CODE_OK;
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_open(ImxVpuApiEncoder **encoder, ImxVpuApiEncOpenParams *open_params, ImxDmaBuffer *stream_buffer)
{
	int err;
	ImxVpuApiEncReturnCodes ret = IMX_VPU_API_ENC_RETURN_CODE_OK;
	ImxVpuApiFramebufferMetrics *fb_metrics;
	BOOL semi_planar;
	VCEncConfig *encoder_config;
	VCEncGopPicConfig *gop_pic_config;
	VCEncGopPicSpecialConfig *gop_pic_special_config;
	VCEncPictureType encoder_pixel_format;
	size_t stream_buffer_size;

	assert(encoder != NULL);
	assert(open_params != NULL);
	assert(stream_buffer != NULL);


	IMX_VPU_API_DEBUG("opening encoder");


	/* Check that the allocated stream buffer is big enough */
	{
		stream_buffer_size = imx_dma_buffer_get_size(stream_buffer);
		if (stream_buffer_size < VPU_ENC_MIN_REQUIRED_STREAM_BUFFER_SIZE) 
		{
			IMX_VPU_API_ERROR("stream buffer size is %zu bytes; need at least %zu bytes", stream_buffer_size, (size_t)VPU_ENC_MIN_REQUIRED_STREAM_BUFFER_SIZE);
			return IMX_VPU_API_ENC_RETURN_CODE_INSUFFICIENT_STREAM_BUFFER_SIZE;
		}
		IMX_VPU_API_DEBUG("stream buffer size is %zu bytes", stream_buffer_size);
	}


	/* Allocate encoder instance. */
	*encoder = malloc(sizeof(ImxVpuApiEncoder));
	assert((*encoder) != NULL);


	/* Set default encoder values. */
	memset(*encoder, 0, sizeof(ImxVpuApiEncoder));

	(*encoder)->next_coding_type = VCENC_NOTCODED_FRAME;


	/* Map the stream buffer. We need to keep it mapped always so we can
	 * keep updating it. It is mapped as readwrite so we can shift data
	 * inside it later with memmove() if necessary. */
	(*encoder)->stream_buffer_virtual_address = imx_dma_buffer_map(stream_buffer, IMX_DMA_BUFFER_MAPPING_FLAG_WRITE | IMX_DMA_BUFFER_MAPPING_FLAG_READ | IMX_DMA_BUFFER_MAPPING_FLAG_MANUAL_SYNC, &err);
	if ((*encoder)->stream_buffer_virtual_address == NULL)
	{
			IMX_VPU_API_ERROR("mapping  buffer to virtual address space failed: %s (%d)", strerror(err), err);
			ret = IMX_VPU_API_ENC_RETURN_CODE_DMA_MEMORY_ACCESS_ERROR;
			goto cleanup_after_error;
	}

	(*encoder)->stream_buffer_physical_address = imx_dma_buffer_get_physical_address(stream_buffer);
	(*encoder)->stream_buffer_size = stream_buffer_size;
	(*encoder)->stream_buffer = stream_buffer;

	IMX_VPU_API_DEBUG(
		"mapped stream buffer: virtual address: %p"
		"  physical address: %" IMX_PHYSICAL_ADDRESS_FORMAT
		"  buffer size: %zu",
		(*encoder)->stream_buffer_virtual_address,
		(*encoder)->stream_buffer_physical_address,
		(*encoder)->stream_buffer_size
	);


	/* Make a copy of the open_params for later use. */
	(*encoder)->open_params = *open_params;


	/* Calculate framebuffer metrics. */

	fb_metrics = &((*encoder)->stream_info.frame_encoding_framebuffer_metrics);

	fb_metrics->actual_frame_width = open_params->frame_width;
	fb_metrics->actual_frame_height = open_params->frame_height;
	fb_metrics->aligned_frame_width = IMX_VPU_API_ALIGN_VAL_TO(fb_metrics->actual_frame_width, FRAME_WIDTH_ALIGNMENT);
	fb_metrics->aligned_frame_height = IMX_VPU_API_ALIGN_VAL_TO(fb_metrics->actual_frame_height, FRAME_HEIGHT_ALIGNMENT);

	semi_planar = imx_vpu_api_is_color_format_semi_planar(open_params->color_format);

	encoder_pixel_format = convert_to_vc8000e_pixel_format(open_params->color_format);

	{
		u32 luma_stride, chroma_stride;
		VCEncGetAlignedStride(fb_metrics->aligned_frame_width, encoder_pixel_format, &luma_stride, &chroma_stride, INPUT_ALIGNMENT);
		fb_metrics->y_stride = luma_stride;
		fb_metrics->uv_stride = chroma_stride;
	}

	switch (open_params->color_format)
	{
		case IMX_VPU_API_COLOR_FORMAT_FULLY_PLANAR_YUV420_10BIT:
		case IMX_VPU_API_COLOR_FORMAT_FULLY_PLANAR_YUV420_8BIT:
		case IMX_VPU_API_COLOR_FORMAT_SEMI_PLANAR_YUV420_10BIT:
		case IMX_VPU_API_COLOR_FORMAT_SEMI_PLANAR_YUV420_8BIT:
			fb_metrics->y_size = fb_metrics->y_stride * fb_metrics->aligned_frame_height;
			fb_metrics->uv_size = fb_metrics->uv_stride * fb_metrics->aligned_frame_height / 2;
			break;

		case IMX_VPU_API_COLOR_FORMAT_PACKED_YUV422_UYVY_8BIT:
		case IMX_VPU_API_COLOR_FORMAT_PACKED_YUV422_YUYV_8BIT:
			fb_metrics->y_size = fb_metrics->y_stride * fb_metrics->aligned_frame_height;
			fb_metrics->uv_stride = 0;
			fb_metrics->uv_size = 0;
			break;

		case IMX_VPU_API_COLOR_FORMAT_SEMI_PLANAR_P010_10BIT:
			fb_metrics->y_size = fb_metrics->y_stride * fb_metrics->aligned_frame_height;
			fb_metrics->uv_size = fb_metrics->uv_stride * fb_metrics->aligned_frame_height / 2;
			break;

		case IMX_VPU_API_COLOR_FORMAT_RGB565:
		case IMX_VPU_API_COLOR_FORMAT_BGR565:
		case IMX_VPU_API_COLOR_FORMAT_RGB444:
		case IMX_VPU_API_COLOR_FORMAT_ARGB4444:
		case IMX_VPU_API_COLOR_FORMAT_ARGB1555:
		case IMX_VPU_API_COLOR_FORMAT_RGBA8888:
		case IMX_VPU_API_COLOR_FORMAT_BGRA8888:
			fb_metrics->y_size = fb_metrics->y_stride * fb_metrics->aligned_frame_height;
			fb_metrics->uv_stride = 0;
			fb_metrics->uv_size = 0;
			break;

		default:
			/* User specified an unknown format. */
			IMX_VPU_API_ERROR("unknown/unsupported color format %s (%d)", imx_vpu_api_color_format_string(open_params->color_format), open_params->color_format);
			ret = IMX_VPU_API_ENC_RETURN_CODE_UNSUPPORTED_COLOR_FORMAT;
			goto cleanup_after_error;
	}

	/* Adjust the uv_stride and uv_size values in case we are using semi-planar chroma. */
	if (semi_planar)
	{
		fb_metrics->uv_stride *= 2;
		fb_metrics->uv_size *= 2;
	}

	fb_metrics->y_offset = 0;
	fb_metrics->u_offset = fb_metrics->y_size;
	fb_metrics->v_offset = fb_metrics->u_offset + fb_metrics->uv_size;

	IMX_VPU_API_DEBUG(
		"framebuffer metrics:  "
		"actual width/height: %zu/%zu  aligned width/height: %zu/%zu  "
		"semi planar: %d  Y/UV stride: %zu/%zu  "
		"Y/UV size: %zu/%zu  Y/U/V offset: %zu/%zu/%zu",
		fb_metrics->actual_frame_width, fb_metrics->actual_frame_height,
		fb_metrics->aligned_frame_width, fb_metrics->aligned_frame_height,
		semi_planar, fb_metrics->y_stride, fb_metrics->uv_stride,
		fb_metrics->y_size, fb_metrics->uv_size,
		fb_metrics->y_offset, fb_metrics->u_offset, fb_metrics->v_offset
	);


	/* Main encoder configuration. */

	/* The Hantro VC8000E encoder does not use a framebuffer pool, so set this to 0. */
	(*encoder)->stream_info.min_num_required_framebuffers = 0;
	(*encoder)->stream_info.min_framebuffer_size = (semi_planar ? fb_metrics->u_offset : fb_metrics->v_offset) + fb_metrics->uv_size;
	(*encoder)->stream_info.framebuffer_alignment = INPUT_ALIGNMENT;
	(*encoder)->stream_info.frame_rate_numerator = open_params->frame_rate_numerator;
	(*encoder)->stream_info.frame_rate_denominator = open_params->frame_rate_denominator;

	encoder_config = &((*encoder)->encoder_config);
	memset(encoder_config, 0, sizeof(VCEncConfig));
	encoder_config->width = fb_metrics->aligned_frame_width;
	encoder_config->height = fb_metrics->aligned_frame_height;
	encoder_config->frameRateNum = open_params->frame_rate_numerator;
	encoder_config->frameRateDenom = open_params->frame_rate_denominator;
	/* Set to 1 since the encoder's own GOP size is hardcoded to 1.
	 * See the comments at the top of this source for the reason why. */
	encoder_config->refFrameAmount = 1;
	/* Set to 1 since the maximum temporal ID in the GOP config is 0,
	 * and maxTLayers = max temporalID + 1. */
	encoder_config->maxTLayers = 1;
	encoder_config->strongIntraSmoothing = 0;
	encoder_config->compressor = 0;
	encoder_config->interlacedFrame = 0;
	/* Even though the driver could handle 10 bit, the VC8000E does not,
	 * at least not the version in the imx8m plus, so always set this to 8. */
	encoder_config->bitDepthLuma = encoder_config->bitDepthChroma = 8;
	/* Enable SSIM (structural similarity). */
	encoder_config->enableSsim = 1;
	/* RDO = Rate-Distotion optimization. Pick a balance between quality and performance. */
	encoder_config->rdoLevel = 1;
	encoder_config->exp_of_input_alignment = EXP_OF_INPUT_ALIGNMENT;
	encoder_config->parallelCoreNum = 1;
	encoder_config->log2MaxPicOrderCntLsb = 16;
	encoder_config->log2MaxFrameNum = 12;
	/* Adaptive GOP is useful for when no explicit GOP size is given. But, we
	 * do require one to always be set, so we don't use this adaptive feature. */
	encoder_config->bPass1AdaptiveGop = 0;
	encoder_config->cuInfoVersion = -1;
	/* Always use VCENC_CHROMA_IDC_420, even when the source color format
	 * is a 4:2:2 YUV one like UYVY. The preprocessor will convert it to
	 * a 4:2:0 format, so using VCENC_CHROMA_IDC_422 here won't work. */
	encoder_config->codedChromaIdc = VCENC_CHROMA_IDC_420;

	/* GOP config, set up for a simple GOP with 1 P frame. */
	gop_pic_config = &((*encoder)->gop_pic_config[0]);
	gop_pic_config->poc = 1;
	gop_pic_config->QpOffset = 0;
	gop_pic_config->QpFactor = sqrt((open_params->compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H264) ? 0.4 : 0.578);
	gop_pic_config->temporalId = 0;
	gop_pic_config->codingType = VCENC_PREDICTED_FRAME;
	gop_pic_config->numRefPics = 1;
	gop_pic_config->refPics[0].ref_pic = -1;
	gop_pic_config->refPics[0].used_by_cur = 1;

	gop_pic_special_config = &((*encoder)->gop_pic_special_config[0]);
	memset(gop_pic_special_config, 0, sizeof(VCEncGopPicSpecialConfig) * MAX_GOP_SPIC_CONFIG_NUM);

	/* h.264 / h.265 specific config. */
	switch (open_params->compression_format)
	{
		case IMX_VPU_API_COMPRESSION_FORMAT_H264:
		{
			ImxVpuApiH264Level level;

			(*encoder)->stream_info.format_specific_open_params.h264_open_params = open_params->format_specific_open_params.h264_open_params;


			/* Estimate the max level if none is specified. The VC8000E
			 * encoder requires the level to be set to a valid value. */

			if (open_params->format_specific_open_params.h264_open_params.level == IMX_VPU_API_H264_LEVEL_UNDEFINED)
			{
				ImxVpuApiH264Level level;
				level = imx_vpu_api_estimate_max_h264_level(
					fb_metrics->aligned_frame_width, fb_metrics->aligned_frame_height,
					open_params->bitrate,
					open_params->frame_rate_numerator,
					open_params->frame_rate_denominator,
					open_params->format_specific_open_params.h264_open_params.profile
				);
				IMX_VPU_API_DEBUG(
					"no h.264 level given; estimated level %s out of width, height, bitrate, framerate, profile",
					imx_vpu_api_h264_level_string(level)
				);
				(*encoder)->stream_info.format_specific_open_params.h264_open_params.level = level;
			}

			level = (*encoder)->stream_info.format_specific_open_params.h264_open_params.level;


			encoder_config->codecFormat = VCENC_VIDEO_CODEC_H264;
			/* h.264 has no tiers; just set this as the default. */
			encoder_config->tier = VCENC_HEVC_MAIN_TIER;
			/* For libimxvpuapi2 API/ABI compatibility reasons, it is
			 * not possible to select anything else when encoding to h.264. */
			encoder_config->streamType = VCENC_BYTE_STREAM;

			switch (open_params->format_specific_open_params.h264_open_params.profile)
			{
				case IMX_VPU_API_H264_PROFILE_BASELINE:
					encoder_config->profile = VCENC_H264_BASE_PROFILE;
					break;

				case IMX_VPU_API_H264_PROFILE_MAIN:
					encoder_config->profile = VCENC_H264_MAIN_PROFILE;
					break;

				case IMX_VPU_API_H264_PROFILE_HIGH:
					encoder_config->profile = VCENC_H264_HIGH_PROFILE;
					break;

				case IMX_VPU_API_H264_PROFILE_HIGH10:
					encoder_config->profile = VCENC_H264_HIGH_10_PROFILE;
					break;

				default:
					/* User specified an unknown profile. */
					IMX_VPU_API_ERROR("unknown/unsupported h.264 profile");
					ret = IMX_VPU_API_ENC_RETURN_CODE_UNSUPPORTED_COMPRESSION_FORMAT_PARAMS;
					goto cleanup_after_error;
			}

			switch (level)
			{
				case IMX_VPU_API_H264_LEVEL_1:   encoder_config->level = VCENC_H264_LEVEL_1;   break;
				case IMX_VPU_API_H264_LEVEL_1B:  encoder_config->level = VCENC_H264_LEVEL_1_b; break;
				case IMX_VPU_API_H264_LEVEL_1_1: encoder_config->level = VCENC_H264_LEVEL_1_1; break;
				case IMX_VPU_API_H264_LEVEL_1_2: encoder_config->level = VCENC_H264_LEVEL_1_2; break;
				case IMX_VPU_API_H264_LEVEL_1_3: encoder_config->level = VCENC_H264_LEVEL_1_3; break;
				case IMX_VPU_API_H264_LEVEL_2:   encoder_config->level = VCENC_H264_LEVEL_2;   break;
				case IMX_VPU_API_H264_LEVEL_2_1: encoder_config->level = VCENC_H264_LEVEL_2_1; break;
				case IMX_VPU_API_H264_LEVEL_2_2: encoder_config->level = VCENC_H264_LEVEL_2_2; break;
				case IMX_VPU_API_H264_LEVEL_3:   encoder_config->level = VCENC_H264_LEVEL_3;   break;
				case IMX_VPU_API_H264_LEVEL_3_1: encoder_config->level = VCENC_H264_LEVEL_3_1; break;
				case IMX_VPU_API_H264_LEVEL_3_2: encoder_config->level = VCENC_H264_LEVEL_3_2; break;
				case IMX_VPU_API_H264_LEVEL_4:   encoder_config->level = VCENC_H264_LEVEL_4;   break;
				case IMX_VPU_API_H264_LEVEL_4_1: encoder_config->level = VCENC_H264_LEVEL_4_1; break;
				case IMX_VPU_API_H264_LEVEL_4_2: encoder_config->level = VCENC_H264_LEVEL_4_2; break;
				case IMX_VPU_API_H264_LEVEL_5:   encoder_config->level = VCENC_H264_LEVEL_5;   break;
				case IMX_VPU_API_H264_LEVEL_5_1: encoder_config->level = VCENC_H264_LEVEL_5_1; break;
				case IMX_VPU_API_H264_LEVEL_5_2: encoder_config->level = VCENC_H264_LEVEL_5_2; break;
				case IMX_VPU_API_H264_LEVEL_6:   encoder_config->level = VCENC_H264_LEVEL_6;   break;
				case IMX_VPU_API_H264_LEVEL_6_1: encoder_config->level = VCENC_H264_LEVEL_6_1; break;
				case IMX_VPU_API_H264_LEVEL_6_2: encoder_config->level = VCENC_H264_LEVEL_6_2; break;

				default:
					/* User specified an unknown level. */
					IMX_VPU_API_ERROR("unknown/unsupported h.264 level");
					ret = IMX_VPU_API_ENC_RETURN_CODE_UNSUPPORTED_COMPRESSION_FORMAT_PARAMS;
					goto cleanup_after_error;
			}

			break;
		}

		case IMX_VPU_API_COMPRESSION_FORMAT_H265:
		{
			ImxVpuApiH265Level level;

			(*encoder)->stream_info.format_specific_open_params.h265_open_params = open_params->format_specific_open_params.h265_open_params;


			/* Estimate the max level if none is specified. The VC8000E
			 * encoder requires the level to be set to a valid value. */

			if (open_params->format_specific_open_params.h265_open_params.level == IMX_VPU_API_H265_LEVEL_UNDEFINED)
			{
				ImxVpuApiH265Level level;
				level = imx_vpu_api_estimate_max_h265_level(
					fb_metrics->aligned_frame_width, fb_metrics->aligned_frame_height,
					open_params->bitrate,
					open_params->frame_rate_numerator,
					open_params->frame_rate_denominator,
					open_params->format_specific_open_params.h265_open_params.profile
				);
				IMX_VPU_API_DEBUG(
					"no h.265 level given; estimated level %s out of width, height, bitrate, framerate, profile",
					imx_vpu_api_h265_level_string(level)
				);
				(*encoder)->stream_info.format_specific_open_params.h265_open_params.level = level;
			}

			level = (*encoder)->stream_info.format_specific_open_params.h265_open_params.level;


			encoder_config->codecFormat = VCENC_VIDEO_CODEC_HEVC;
			encoder_config->tier = (open_params->format_specific_open_params.h265_open_params.tier == IMX_VPU_API_H265_TIER_HIGH) ? VCENC_HEVC_HIGH_TIER : VCENC_HEVC_MAIN_TIER;
			/* For libimxvpuapi2 API/ABI compatibility reasons, it is
			 * not possible to select anything else when encoding to h.265. */
			encoder_config->streamType = VCENC_BYTE_STREAM;

			switch (open_params->format_specific_open_params.h265_open_params.profile)
			{
				case IMX_VPU_API_H265_PROFILE_MAIN:
					encoder_config->profile = VCENC_HEVC_MAIN_PROFILE;
					break;

				case IMX_VPU_API_H265_PROFILE_MAIN10:
					encoder_config->profile = VCENC_HEVC_MAIN_10_PROFILE;
					break;

				default:
					/* User specified an unknown profile. */
					IMX_VPU_API_ERROR("unknown/unsupported h.265 profile");
					ret = IMX_VPU_API_ENC_RETURN_CODE_UNSUPPORTED_COMPRESSION_FORMAT_PARAMS;
					goto cleanup_after_error;
			}

			switch (level)
			{
				case IMX_VPU_API_H265_LEVEL_1:   encoder_config->level = VCENC_HEVC_LEVEL_1;   break;
				case IMX_VPU_API_H265_LEVEL_2:   encoder_config->level = VCENC_HEVC_LEVEL_2;   break;
				case IMX_VPU_API_H265_LEVEL_2_1: encoder_config->level = VCENC_HEVC_LEVEL_2_1; break;
				case IMX_VPU_API_H265_LEVEL_3:   encoder_config->level = VCENC_HEVC_LEVEL_3;   break;
				case IMX_VPU_API_H265_LEVEL_3_1: encoder_config->level = VCENC_HEVC_LEVEL_3_1; break;
				case IMX_VPU_API_H265_LEVEL_4:   encoder_config->level = VCENC_HEVC_LEVEL_4;   break;
				case IMX_VPU_API_H265_LEVEL_4_1: encoder_config->level = VCENC_HEVC_LEVEL_4_1; break;
				case IMX_VPU_API_H265_LEVEL_5:   encoder_config->level = VCENC_HEVC_LEVEL_5;   break;
				case IMX_VPU_API_H265_LEVEL_5_1: encoder_config->level = VCENC_HEVC_LEVEL_5_1; break;
				case IMX_VPU_API_H265_LEVEL_5_2: encoder_config->level = VCENC_HEVC_LEVEL_5_2; break;
				case IMX_VPU_API_H265_LEVEL_6:   encoder_config->level = VCENC_HEVC_LEVEL_6;   break;
				case IMX_VPU_API_H265_LEVEL_6_1: encoder_config->level = VCENC_HEVC_LEVEL_6_1; break;
				case IMX_VPU_API_H265_LEVEL_6_2: encoder_config->level = VCENC_HEVC_LEVEL_6_2; break;

				default:
					/* User specified an unknown level. */
					IMX_VPU_API_ERROR("unknown/unsupported h.265 level");
					ret = IMX_VPU_API_ENC_RETURN_CODE_UNSUPPORTED_COMPRESSION_FORMAT_PARAMS;
					goto cleanup_after_error;
			}

			break;
		}

		default:
			IMX_VPU_API_ERROR("invalid/unsupported compression format %s", imx_vpu_api_compression_format_string(open_params->compression_format));
			ret = IMX_VPU_API_ENC_RETURN_CODE_UNSUPPORTED_COMPRESSION_FORMAT;
			goto cleanup_after_error;
	}


	/* Prepare the encoder input information that will be used by encode(). */
	resolve_refresh_config(*encoder);

	init_encoder_input(*encoder);

	ret = init_vcenc_instance(*encoder, encoder_pixel_format, fb_metrics);
	if (ret != IMX_VPU_API_ENC_RETURN_CODE_OK)
		goto cleanup_after_error;


	/* Finish & cleanup. */
finish:
	if (ret == IMX_VPU_API_ENC_RETURN_CODE_OK)
		IMX_VPU_API_DEBUG("successfully opened encoder");

	return ret;

cleanup_after_error:
	if ((*encoder) != NULL)
	{
		imx_vpu_api_enc_close(*encoder);
		*encoder = NULL;
	}

	if (ret == IMX_VPU_API_ENC_RETURN_CODE_OK)
		ret = IMX_VPU_API_ENC_RETURN_CODE_ERROR;

	goto finish;
}


void imx_vpu_api_enc_close(ImxVpuApiEncoder *encoder)
{
	assert(encoder != NULL);

	IMX_VPU_API_DEBUG("closing encoder");

	/* The rate control's own account of the stream it just produced. The
	 * bucket is charged the coded bits and drained one picture's budget, so
	 * fill mean is where the loop settled - and that standing level is the
	 * queueing delay, which makes it the first number to look at when the
	 * latency is wrong. */
	if (encoder->new_cbr_active && (encoder->new_cbr.num_pictures > 0))
	{
		ExtRateControl const *rc = &encoder->new_cbr;
		double const pictures = (double)(rc->num_pictures);
		/* The rate is bits over elapsed time, and a skipped picture still
		 * takes up its slot on the timeline. Dividing by coded pictures
		 * alone would report a stream that dropped a third of its
		 * pictures at the same kbps as one that kept them. */
		double const slots = (double)(rc->num_pictures + rc->num_skipped);

		IMX_VPU_API_INFO(
			"new CBR summary: %lu pictures, %.0f kbps of %.0f kbps configured, "
			"%lu re-encodes, "
			"HRD buffer emptied on %lu pictures, fill mean %.2f max %.2f, "
			"idle capacity repaid on %lu pictures (%.0f kbit, %.1f%% of the stream), "
			"standing rate error %.1f%%, mean repayment ceiling %.2f",
			rc->num_pictures,
			rc->sum_bits / slots * rc->frame_rate / 1000.0,
			rc->bit_per_pic * rc->frame_rate / 1000.0,
			rc->num_reencodes,
			rc->num_bucket_empty,
			rc->sum_fill / pictures,
			rc->max_fill,
			rc->num_debt_lifts,
			rc->sum_lift / 1000.0,
			(rc->sum_bits > 0.0) ? (rc->sum_lift * 100.0 / rc->sum_bits) : 0.0,
			rc->rate_err_ema * 100.0,
			(rc->num_debt_lifts > 0) ? (rc->sum_fill_used / (double)(rc->num_debt_lifts)) : 0.0
		);

		/* Only when it opened. A line saying "0 pictures skipped" on every
		 * healthy stream trains people to stop reading it, and this is
		 * the one number that says the operating point was not feasible. */
		if (rc->num_skipped > 0)
			IMX_VPU_API_INFO(
				"new CBR summary: the frame skipping valve refused %lu of %.0f pictures "
				"(%.1f%%, %.1f fps delivered of %.1f, longest run %u, %lu of them "
				"encoded first and discarded on their measured size) - qp_max %d was "
				"not enough to hold the configured rate, so frame rate was traded for "
				"latency",
				rc->num_skipped, slots,
				rc->num_skipped * 100.0 / slots,
				pictures / slots * rc->frame_rate, rc->frame_rate,
				rc->max_skip_run, rc->num_dropped,
				rc->qp_max_inter
			);
	}

	if (encoder->encoder != NULL)
		VCEncRelease(encoder->encoder);

	if (encoder->stream_buffer != NULL)
		imx_dma_buffer_unmap(encoder->stream_buffer);

	free(encoder->header_data);

	free(encoder);
}


ImxVpuApiEncStreamInfo const * imx_vpu_api_enc_get_stream_info(ImxVpuApiEncoder *encoder)
{
	assert(encoder != NULL);
	return &(encoder->stream_info);
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_add_framebuffers_to_pool(ImxVpuApiEncoder *encoder, ImxDmaBuffer **fb_dma_buffers, size_t num_framebuffers)
{
	IMX_VPU_API_UNUSED_PARAM(encoder);
	IMX_VPU_API_UNUSED_PARAM(fb_dma_buffers);
	IMX_VPU_API_UNUSED_PARAM(num_framebuffers);
	IMX_VPU_API_ERROR("tried to add framebuffers, but this encoder does not use a framebuffer pool");
	return IMX_VPU_API_ENC_RETURN_CODE_INVALID_CALL;
}


void imx_vpu_api_enc_enable_drain_mode(ImxVpuApiEncoder *encoder)
{
	assert(encoder != NULL);
	encoder->drain_mode_enabled = TRUE;
}


int imx_vpu_api_enc_is_drain_mode_enabled(ImxVpuApiEncoder *encoder)
{
	assert(encoder != NULL);
	return encoder->drain_mode_enabled;
}


void imx_vpu_api_enc_flush(ImxVpuApiEncoder *encoder)
{
	assert(encoder != NULL);

	/* Force the first frame after the flush to be an IDR frame. This
	 * makes sure that decoders can show a video signal right away
	 * after the encoder got flushed. */
	encoder->force_IDR_frame = TRUE;
	encoder->staged_raw_frame_set = FALSE;
	encoder->encoded_frame_available = FALSE;

	/* That IDR refreshes the whole picture, so the sweep starts over with
	 * it rather than carrying on from wherever it had got to - and any
	 * region a receiver had asked for is moot now. */
	imx_vpu_api_intra_refresh_init(&(encoder->refresh_state), &(encoder->refresh_cfg));
	encoder->forced_intra_q_head = 0;
	encoder->forced_intra_q_count = 0;
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_set_bitrate(ImxVpuApiEncoder *encoder, unsigned int bitrate)
{
	assert(encoder != NULL);
	assert(bitrate > 0);

	if (encoder->open_params.bitrate == 0)
	{
		IMX_VPU_API_ERROR("rate control disabled in the imx_vpu_api_enc_open() parameters");
		return IMX_VPU_API_ENC_RETURN_CODE_INVALID_CALL;
	}

	IMX_VPU_API_TRACE("setting bitrate to %u kbps", bitrate);

	/* We specify the bitrate in kbps, the encoder expects bps, so multiply by 1000.
	 * The new bitrate is set in imx_vpu_api_enc_encode(). */
	encoder->new_bitrate = bitrate * 1000;

	return IMX_VPU_API_ENC_RETURN_CODE_OK;
}


void imx_vpu_api_enc_set_intra_refresh_region(ImxVpuApiEncoder *encoder, unsigned int first_ctb_row, unsigned int num_ctb_rows)
{
	assert(encoder != NULL);

	if (num_ctb_rows == 0)
	{
		encoder->forced_intra_q_count = 0;
		encoder->forced_intra_q_head = 0;
		return;
	}

	/* The encoder's own GDR sweeps on its own schedule and overwrites
	 * intraArea on every picture, so there is nothing here that can serve a
	 * request. Say so once instead of filling a queue nobody drains. */
	if (encoder->vendor_gdr_active)
	{
		IMX_VPU_API_DEBUG("ignoring the intra refresh request for rows %u..%u: the "
		                  "encoder's own GDR drives the sweep (rate-control=0)",
		                  first_ctb_row, first_ctb_row + num_ctb_rows - 1);
		return;
	}

	for (int i = 0; i < encoder->forced_intra_q_count; i++)
	{
		int idx = (encoder->forced_intra_q_head + i) % FORCED_INTRA_QUEUE_SIZE;
		if ((encoder->forced_intra_q[idx].first == first_ctb_row) &&
		    (encoder->forced_intra_q[idx].num == num_ctb_rows))
			return;
	}

	if (encoder->forced_intra_q_count >= FORCED_INTRA_QUEUE_SIZE)
	{
		IMX_VPU_API_WARNING("forced intra refresh queue full; dropping rows %u..%u",
		                    first_ctb_row, first_ctb_row + num_ctb_rows - 1);
		return;
	}

	{
		int tail = (encoder->forced_intra_q_head + encoder->forced_intra_q_count) % FORCED_INTRA_QUEUE_SIZE;
		encoder->forced_intra_q[tail].first = first_ctb_row;
		encoder->forced_intra_q[tail].num = num_ctb_rows;
		encoder->forced_intra_q_count++;
	}

	IMX_VPU_API_TRACE("forced intra refresh queued: rows %u..%u (queue depth %d)",
	                  first_ctb_row, first_ctb_row + num_ctb_rows - 1, encoder->forced_intra_q_count);
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_get_session_state(ImxVpuApiEncoder *encoder, ImxVpuApiEncSessionState *state)
{
	ImxVpuApiEncSession *session;

	assert(encoder != NULL);
	assert(state != NULL);

	if (encoder->open_params.rate_control_mode != 1)
	{
		/* Nothing to carry: mode 0 is the unmodified encoder, and its stream
		 * restarts exactly as it always did. */
		IMX_VPU_API_DEBUG("no session state at rate control mode %d", (int)(encoder->open_params.rate_control_mode));
		return IMX_VPU_API_ENC_RETURN_CODE_INVALID_CALL;
	}

	/* Debug lever: with nothing carried, a resolution change behaves the way
	 * it did before any of this - the rate control restarts from an empty
	 * buffer and the parameter sets keep the ids of the resolution before
	 * them. It is here so that the difference can be measured on the target
	 * rather than argued about. */
	if (getenv("VC8000E_NO_SESSION_CARRY") != NULL)
	{
		IMX_VPU_API_INFO("VC8000E_NO_SESSION_CARRY is set; the stream will restart at the next resolution change");
		return IMX_VPU_API_ENC_RETURN_CODE_INVALID_CALL;
	}

	session = &(encoder->session);

	if (encoder->new_cbr_active && (encoder->new_cbr.num_pictures > 0))
		imx_vpu_api_enc_session_save_rc(session, &(encoder->new_cbr));

	imx_vpu_api_enc_session_pack(session, state);

	IMX_VPU_API_DEBUG("session state read out: %d resolution(s), rate control %s",
	                  (int)(session->ps_num), session->rc_valid ? "carried" : "not started yet");

	return IMX_VPU_API_ENC_RETURN_CODE_OK;
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_set_session_state(ImxVpuApiEncoder *encoder, ImxVpuApiEncSessionState const *state)
{
	ImxVpuApiEncSession const *session;

	assert(encoder != NULL);
	assert(state != NULL);

	if (encoder->open_params.rate_control_mode != 1)
	{
		IMX_VPU_API_DEBUG("session state ignored at rate control mode %d", (int)(encoder->open_params.rate_control_mode));
		return IMX_VPU_API_ENC_RETURN_CODE_INVALID_CALL;
	}

	if ((encoder->num_encoded_pictures != 0) || encoder->has_header)
	{
		IMX_VPU_API_ERROR("tried to continue a stream in an encoder that has already encoded");
		return IMX_VPU_API_ENC_RETURN_CODE_INVALID_CALL;
	}

	if (!imx_vpu_api_enc_session_unpack(&(encoder->session), state))
		return IMX_VPU_API_ENC_RETURN_CODE_ERROR;

	session = &(encoder->session);

	if (encoder->new_cbr_active && session->rc_valid)
		imx_vpu_api_enc_session_restore_rc(session, &(encoder->new_cbr));
	else if (session->rc_valid)
		IMX_VPU_API_WARNING("session state carries rate control state, but this encoder has no new CBR to resume it in");

	return IMX_VPU_API_ENC_RETURN_CODE_OK;
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_set_frame_rate(ImxVpuApiEncoder *encoder, unsigned int frame_rate_numerator, unsigned int frame_rate_denominator)
{
	// TODO
	return IMX_VPU_API_ENC_RETURN_CODE_OK;
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_push_raw_frame(ImxVpuApiEncoder *encoder, ImxVpuApiRawFrame const *raw_frame)
{
	assert(encoder != NULL);
	assert(raw_frame != NULL);

	if (encoder->staged_raw_frame_set)
	{
		IMX_VPU_API_ERROR("tried to push a raw frame before a previous one was encoded");
		return IMX_VPU_API_ENC_RETURN_CODE_INVALID_CALL;
	}

	IMX_VPU_API_LOG("staged raw frame");

	/* Stage the raw frame. We cannot use it here right away, since the
	 * encoder has no separate function to push raw frames into it. Instead,
	 * just keep track of it here, and actually use it in imx_vpu_api_enc_encode(). */
	encoder->staged_raw_frame = *raw_frame;
	encoder->staged_raw_frame_physical_address = imx_dma_buffer_get_physical_address(encoder->staged_raw_frame.fb_dma_buffer);

	encoder->staged_raw_frame_set = TRUE;

	return IMX_VPU_API_ENC_RETURN_CODE_OK;
}


/* Which parameter set id this resolution owns. Same resolution as an earlier
 * one in the session means the same id, so returning to a resolution the
 * decoder still has parameter sets for costs nothing. The first resolution
 * takes id 0, which is what the encoder writes anyway: a stream that never
 * changes resolution is therefore not rewritten at all. */
static void resolve_param_set_id(ImxVpuApiEncoder *encoder)
{
	/* Mode 0 is the unmodified encoder, down to the bytes. */
	if (encoder->open_params.rate_control_mode != 1)
	{
		encoder->param_set_id = 0;
		return;
	}

	encoder->param_set_id = imx_vpu_api_enc_session_param_set_id(
		&(encoder->session),
		encoder->open_params.frame_width,
		encoder->open_params.frame_height);
}


/* Moves slice_pic_parameter_set_id onto the id this resolution owns, by
 * creating the PPS with that id and making it the active one. The hardware
 * writes the active id into every slice header, so this is the only way to
 * move it without rewriting picture data.
 *
 * Clears param_set_id if it cannot be done, which leaves the ids exactly as
 * the encoder wrote them - a working stream without the improvement, rather
 * than a header and a picture that disagree. */
static void activate_param_set_pps(ImxVpuApiEncoder *encoder)
{
	VCEncPPSCfg pps_cfg;
	i32 created_id = 0;
	int guard;

	memset(&pps_cfg, 0, sizeof(pps_cfg));

	/* Cloned from PPS 0, so the PPS the pictures are coded against carries
	 * the same chroma and deblocking offsets as the one in the header.
	 * VCEncGetPPSData() reports them in the units VCEncCreateNewPPS()
	 * expects. */
	if (VCEncGetPPSData(encoder->encoder, &pps_cfg, 0) != VCENC_OK)
	{
		IMX_VPU_API_WARNING("could not read PPS 0; keeping the parameter set ids the encoder wrote");
		encoder->param_set_id = 0;
		return;
	}

	/* The ids handed out are the lowest free ones, from 1 up, so this walks
	 * up to the one this resolution owns. The ones passed on the way are
	 * never written to the stream: after VCEncStrmStart() the vendor library
	 * emits a PPS only for a newly created one - taken back out again by
	 * drop_inserted_pps_nal() - and for the resend flags, which this encoder
	 * does not use. */
	for (guard = 0; (created_id < encoder->param_set_id) && (guard < IMX_VPU_API_ENC_MAX_PARAM_SET_IDS); ++guard)
	{
		i32 const previous_id = created_id;

		if (VCEncCreateNewPPS(encoder->encoder, &pps_cfg, &created_id) != VCENC_OK)
			break;
		if (created_id <= previous_id)
			break;
	}

	if ((created_id != encoder->param_set_id) ||
	    (VCEncActiveAnotherPPS(encoder->encoder, encoder->param_set_id) != VCENC_OK))
	{
		IMX_VPU_API_WARNING("could not activate PPS %d; keeping the parameter set ids the encoder wrote",
		                    encoder->param_set_id);
		encoder->param_set_id = 0;
		return;
	}

	/* Creating it queued a copy of it for the next picture. */
	encoder->drop_inserted_pps = TRUE;
}


/* Takes the PPS the vendor library inserted ahead of the slice back out of the
 * picture. It is a copy of PPS 0 under the new id, so it says the picture uses
 * the SPS of the previous resolution - the one thing the restamped header
 * exists to stop the decoder believing. Dropping it also keeps the parameter
 * sets on the wire coming from one place, the cached header. */
static void drop_inserted_pps_nal(ImxVpuApiEncoder *encoder, VCEncOut *encoder_output)
{
	uint8_t *data = encoder->stream_buffer_virtual_address;
	size_t size = (size_t)(encoder_output->streamSize);
	BOOL const is_h264 = (encoder->open_params.compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H264);
	int const pps_nal_type = is_h264 ? 8 : 34;
	size_t p = 0;
	int nals_examined = 0;

	imx_dma_buffer_start_sync_session(encoder->stream_buffer);

	while ((p + 3 <= size) && !((data[p] == 0) && (data[p+1] == 0) && (data[p+2] == 1))) p++;

	/* It goes in ahead of the slice, behind at most an access unit delimiter,
	 * so there is no reason to walk into the picture data looking for it. */
	while ((p + 3 <= size) && (nals_examined < 4))
	{
		size_t nal = p + 3, q = p + 3, nal_end;
		int type;

		while ((q + 3 <= size) && !((data[q] == 0) && (data[q+1] == 0) && (data[q+2] == 1))) q++;
		nal_end = (q + 3 <= size) ? q : size;
		type = is_h264 ? (data[nal] & 0x1f) : ((data[nal] >> 1) & 0x3f);

		if (type == pps_nal_type)
		{
			size_t const dropped = nal_end - p;

			memmove(data + p, data + nal_end, size - nal_end);
			size -= dropped;
			encoder_output->streamSize = (u32)size;
			encoder->drop_inserted_pps = FALSE;
			IMX_VPU_API_DEBUG("dropped the %zu byte PPS the encoder inserted with parameter set id %d",
			                  dropped, encoder->param_set_id);
			break;
		}

		nals_examined++;
		p = nal_end;
	}

	imx_dma_buffer_stop_sync_session(encoder->stream_buffer);

	if (encoder->drop_inserted_pps)
	{
		IMX_VPU_API_WARNING("no inserted PPS found at the start of the picture; the stream carries two PPS %d",
		                    encoder->param_set_id);
		encoder->drop_inserted_pps = FALSE;
	}
}


/* The Hantro VC8000E blob prints unconditional stdout debug while assembling the
 * bitstream ("RecoveryPoint sei size=%d", "PicTiming sei size=%d", "BufferingSei
 * sei size=%d", "UserDataUnreg sei size=%d", ...). With a per-picture recovery SEI
 * (use-intra-refresh / gdr_refresh_period) and/or use-hrd timing SEIs this is several
 * lines per frame, flooding the service journal. The blob has no trace toggle (it does
 * not link getenv) and reports real errors via return codes + stderr, not these prints,
 * so mute only fd 1 (stdout) for the duration of this one call.
 * NOTE: fd 1 is process-global. This is safe because the encode loop is single-threaded
 * per encoder instance; concurrent encoder instances in one process would need a shared
 * lock around the swap to avoid clobbering each other's saved fd. */
static VCEncRet encode_one_picture(ImxVpuApiEncoder *encoder, VCEncIn *encoder_input, VCEncOut *encoder_output)
{
	VCEncRet enc_ret;
	int saved_stdout_fd = -1;
	int devnull_fd = open("/dev/null", O_WRONLY);

	if (devnull_fd >= 0)
	{
		fflush(stdout);
		saved_stdout_fd = dup(STDOUT_FILENO);
		(void)dup2(devnull_fd, STDOUT_FILENO);
	}

	enc_ret = VCEncStrmEncode(encoder->encoder, encoder_input, encoder_output, NULL, NULL);

	if (saved_stdout_fd >= 0)
	{
		fflush(stdout);
		(void)dup2(saved_stdout_fd, STDOUT_FILENO);
		close(saved_stdout_fd);
	}
	if (devnull_fd >= 0)
		close(devnull_fd);

	return enc_ret;
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_encode(ImxVpuApiEncoder *encoder, size_t *encoded_frame_size, ImxVpuApiEncOutputCodes *output_code)
{
	ImxVpuApiFramebufferMetrics *fb_metrics;
	imx_physical_address_t raw_frame_phys_addr;
	fb_metrics = &(encoder->stream_info.frame_encoding_framebuffer_metrics);
	BOOL is_first_picture;
	BOOL is_idr;
	VCEncIn *encoder_input;
	VCEncOut encoder_output;
	ImxVpuApiEncReturnCodes ret = IMX_VPU_API_ENC_RETURN_CODE_OK;
	VCEncRet enc_ret;
	ImxVpuApiFrameType requested_frame_type;

	assert(encoder != NULL);
	assert(encoded_frame_size != NULL);
	assert(output_code != NULL);

	if (!(encoder->staged_raw_frame_set))
	{
		IMX_VPU_API_TRACE("no data left to encode");
		*output_code = IMX_VPU_API_ENC_OUTPUT_CODE_MORE_INPUT_DATA_NEEDED;
		ret = IMX_VPU_API_ENC_RETURN_CODE_OK;
		goto finish;
	}

	encoder_input = &(encoder->encoder_input);

	is_first_picture = (encoder->num_encoded_pictures == 0);

	fb_metrics = &(encoder->stream_info.frame_encoding_framebuffer_metrics);
	*output_code = IMX_VPU_API_ENC_OUTPUT_CODE_NO_OUTPUT_YET_AVAILABLE;

	raw_frame_phys_addr = imx_dma_buffer_get_physical_address(encoder->staged_raw_frame.fb_dma_buffer);
	IMX_VPU_API_LOG("encoding raw_frame with physical address %" IMX_PHYSICAL_ADDRESS_FORMAT, raw_frame_phys_addr);

	encoder_input->busLuma = (ptr_t)( ((uint8_t *)raw_frame_phys_addr) + fb_metrics->y_offset);
	encoder_input->busChromaU = (ptr_t)( ((uint8_t *)raw_frame_phys_addr) + fb_metrics->u_offset);
	encoder_input->busChromaV = (ptr_t)( ((uint8_t *)raw_frame_phys_addr) + fb_metrics->v_offset);
	/* Duration of the _previous_ picture, in 1/frame_rate_numerator units.
	 * (We use a fixed framerate here, so this is just set to the frame rate
	 * denominator.) Since there is no "previous picture" when encoding the
	 * first one, use an increment of 0 for the first picture. */
	encoder_input->timeIncrement = is_first_picture ? 0 : encoder->open_params.frame_rate_denominator;
	encoder_input->picture_cnt = encoder->num_encoded_pictures;

	requested_frame_type = encoder->staged_raw_frame.frame_types[0];

	/* An intra request the valve deferred outranks whatever this input frame
	 * asked for. The deferral only ever moves an intra picture to a later
	 * one, so this can promote a picture to intra and never demote one. */
	if (((encoder->postponed_frame_type == IMX_VPU_API_FRAME_TYPE_I)
	  || (encoder->postponed_frame_type == IMX_VPU_API_FRAME_TYPE_IDR))
	 && (requested_frame_type != IMX_VPU_API_FRAME_TYPE_I)
	 && (requested_frame_type != IMX_VPU_API_FRAME_TYPE_IDR))
		requested_frame_type = encoder->postponed_frame_type;

	if (is_first_picture)
	{
		if (encoder->refresh_active)
		{
			IMX_VPU_API_DEBUG("refresh mode: no IDR — first picture encoded as non-IDR I-frame");
			requested_frame_type = IMX_VPU_API_FRAME_TYPE_I;
		}
		else
		{
			IMX_VPU_API_DEBUG("encoding the first picture as IDR frame");
			requested_frame_type = IMX_VPU_API_FRAME_TYPE_IDR;
		}
	}
	else if (encoder->force_IDR_frame)
	{
		IMX_VPU_API_DEBUG("forcing this frame to be encoded as IDR");
		requested_frame_type = IMX_VPU_API_FRAME_TYPE_IDR;
	}

	/* Sweeping a refresh, a keyframe request is served with one picture coded
	 * entirely intra, not with an IDR. An IDR here would be exactly the
	 * once-per-GOP bitrate spike the sweep exists to avoid, and it would
	 * reset the reference structure and the picture order as well. A
	 * full-picture intra region gives a decoder the same clean start and
	 * leaves both alone, and the refresh tick is re-anchored to it so the
	 * sweeps that follow are aligned to this picture rather than to the
	 * schedule it interrupted. */
	if (encoder->refresh_active && !is_first_picture
	 && ((requested_frame_type == IMX_VPU_API_FRAME_TYPE_I)
	  || (requested_frame_type == IMX_VPU_API_FRAME_TYPE_IDR)))
	{
		IMX_VPU_API_DEBUG("refresh mode: keyframe request served as an out-of-order "
		                  "full-picture refresh rather than an IDR");
		encoder->pending_full_refresh = TRUE;
		requested_frame_type = IMX_VPU_API_FRAME_TYPE_UNKNOWN;
	}

	switch (requested_frame_type)
	{
		case IMX_VPU_API_FRAME_TYPE_I:
			encoder_input->codingType = VCENC_INTRA_FRAME;
			encoder_input->bIsIDR = HANTRO_FALSE;
			break;

		case IMX_VPU_API_FRAME_TYPE_IDR:
			encoder_input->codingType = VCENC_INTRA_FRAME;
			encoder_input->bIsIDR = HANTRO_TRUE;
			encoder_input->last_idr_picture_cnt = encoder->num_encoded_pictures;
			break;

		default:
			/* Use the coding type that the previous
			 * VCEncFindNextPic() call indicated. */
			encoder_input->codingType = encoder->next_coding_type;
			break;
	}

	if ((requested_frame_type == IMX_VPU_API_FRAME_TYPE_IDR) && (encoder->num_encoded_pictures != 0))
	{
		encoder_input->poc = 0;
		/* Set this to TRUE to make imx_vpu_api_enc_get_encoded_frame()
		 * prepend the header data to the encoded IDR frame. This is
		 * crucial for cases where a decoder receives the encoded signal
		 * mid-stream, after the initial header data has been sent. */
		encoder->has_header = TRUE;
	}

	/* Re-send the parameter sets every GOP, so a receiver that joined late or
	 * lost them can start decoding without waiting for a new stream.
	 *
	 * In every mode, not just the refresh ones. gstimxvpuenc does this too -
	 * it caches the first frame's parameter sets and appends them to the
	 * output buffer every gop-size frames when the frame does not already
	 * carry them - and doing it there is invisible to the rate control,
	 * because it happens after imx_vpu_api_enc_get_encoded_frame_ext() has
	 * returned and the bits have been accounted. That is a real leak: 89
	 * bytes of VPS, SPS and PPS reach the link that the leaky bucket was
	 * never charged for, once per GOP. Measured in IDR mode on the stress
	 * clip, 39 insertions over 1187 pictures, 27768 bits the rate control
	 * could not see - enough to put a picture over a 150 kbit buffer.
	 *
	 * Emitting them here instead costs the same bytes in the same place and
	 * charges them, and it also stops the element from adding a second copy:
	 * its guard is `!encoded_frame.has_header`. This condition used to
	 * require a refresh mode, which is why the leak only ever showed in IDR
	 * mode - with a sweep running, this branch already fired and the element
	 * stayed out of it. */
	if (!encoder->has_header && (encoder->num_encoded_pictures > 0) &&
	    (encoder->open_params.gop_size > 0) &&
	    (((int)(encoder->num_encoded_pictures) % (int)(encoder->open_params.gop_size)) == 0))
	{
		encoder->has_header = TRUE;
	}

	/* Record the bIsIDR value for logging further below. */
	is_idr = !!(encoder_input->bIsIDR);

	/* Only setting the first items in these arrays because this version
	 * of the VC8000E encoder does not support two-stream buffers. */
	encoder_input->pOutBuf[0] = (u32 *)(encoder->stream_buffer_virtual_address);
	encoder_input->busOutBuf[0] = encoder->stream_buffer_physical_address;
	encoder_input->outBufSize[0] = encoder->stream_buffer_size;

	*encoded_frame_size = 0;
	encoder->num_bytes_in_stream_buffer = 0;
	encoder->recovery_sei_size = 0;

	if (is_first_picture)
	{
		/* Start the stream if we are encoding the very first picture.
		 * This is where the SPS/PPS/VPS header data is generated. */

		resolve_param_set_id(encoder);

		memset(&encoder_output, 0, sizeof(encoder_output));
		enc_ret = VCEncStrmStart(encoder->encoder, encoder_input, &encoder_output);
		if (enc_ret != VCENC_OK)
			goto error;

		/* Copy the header data so we can insert it later if necessary. */
		encoder->header_data = malloc(encoder_output.streamSize);

		/* Use synced access since we have to copy the
		 * header data out of the stream buffer. */
		imx_dma_buffer_start_sync_session(encoder->stream_buffer);
		memcpy(encoder->header_data, encoder_input->pOutBuf[0], encoder_output.streamSize);
		imx_dma_buffer_stop_sync_session(encoder->stream_buffer);

		encoder->header_data_size = encoder_output.streamSize;

		if (encoder->open_params.compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H265)
		{
			size_t before = encoder->header_data_size;
			vc8000_hevc_force_no_reorder(encoder->header_data, &encoder->header_data_size);
			IMX_VPU_API_LOG("HEVC SPS no-reorder rewrite: header %zu -> %zu bytes",
			                before, encoder->header_data_size);
		}
		else if (encoder->open_params.compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H264)
		{
			size_t before = encoder->header_data_size;
			vc8000_h264_force_no_reorder(encoder->header_data, &encoder->header_data_size);
			IMX_VPU_API_LOG("H264 SPS no-reorder rewrite: header %zu -> %zu bytes",
			                before, encoder->header_data_size);
		}

		/* Give this resolution's parameter sets an id of their own, so that a
		 * decoder which loses them at a resolution change cannot go on
		 * decoding with the previous resolution's. Both halves have to
		 * succeed together: the slice headers point at the new PPS, so the
		 * header must define it. */
		if (encoder->param_set_id != 0)
		{
			BOOL const is_h264 = (encoder->open_params.compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H264);
			size_t const before = encoder->header_data_size;

			activate_param_set_pps(encoder);

			if ((encoder->param_set_id != 0)
			 && !vc8000_restamp_param_set_ids(&(encoder->header_data), &(encoder->header_data_size),
			                                  is_h264, (unsigned)(encoder->param_set_id)))
			{
				IMX_VPU_API_WARNING("could not restamp the header parameter set ids; reverting to PPS 0");
				if (VCEncActiveAnotherPPS(encoder->encoder, 0) != VCENC_OK)
					IMX_VPU_API_ERROR("could not revert to PPS 0; the picture and the header disagree");
				encoder->param_set_id = 0;
			}
			else if (encoder->param_set_id != 0)
			{
				IMX_VPU_API_INFO("parameter set id %d stamped into the SPS and the PPS: header %zu -> %zu bytes",
				                 encoder->param_set_id, before, encoder->header_data_size);
			}
		}

		encoder->has_header = TRUE;
	}

	if (encoder->has_header)
	{
		IMX_VPU_API_LOG("header size is %zu byte(s)", encoder->header_data_size);
		*encoded_frame_size += encoder->header_data_size;
	}

	/* Update the bitrate before the actual encoding if a new
	 * bitrate was set through imx_vpu_api_enc_set_bitrate(). */
	if (encoder->new_bitrate > 0)
	{
		VCEncRateCtrl rate_control_config;

		IMX_VPU_API_DEBUG("updating rate control configuration to use new bitrate");

		/* In new-CBR mode the encoder's copy of the bitrate is not what
		 * drives anything - our own controller is - but keep the two in
		 * step so a later VCEncGetRateCtrl still reports the truth. */
		if (encoder->new_cbr_active)
		{
			ext_rate_control_set_bitrate(&encoder->new_cbr, encoder->new_bitrate);
			encoder->cached_rate_ctrl.bitPerSecond = encoder->new_bitrate;
			encoder->new_bitrate = 0;

			enc_ret = VCEncSetRateCtrl(encoder->encoder, &(encoder->cached_rate_ctrl));
			if (enc_ret != VCENC_OK)
			{
				IMX_VPU_API_ERROR("could not set updated rate control configuration: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
				goto error;
			}
		}
		else
		{
			enc_ret = VCEncGetRateCtrl(encoder->encoder, &rate_control_config);
			if (enc_ret != VCENC_OK)
			{
				IMX_VPU_API_ERROR("could not get current rate control configuration: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
				goto error;
			}

			rate_control_config.bitPerSecond = encoder->new_bitrate;
			encoder->new_bitrate = 0;

			enc_ret = VCEncSetRateCtrl(encoder->encoder, &rate_control_config);
			if (enc_ret != VCENC_OK)
			{
				IMX_VPU_API_ERROR("could not set updated rate control configuration: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
				goto error;
			}
		}
	}

	/* Everything the refresh scheduler below is about to advance.
	 *
	 * The measured decision point runs *after* the scheduler, so a picture
	 * discarded there has already had its sweep step taken and, if a
	 * receiver had asked for a region, that request taken off the queue -
	 * and none of it is going anywhere, because the picture is not. Left
	 * alone that silently breaks the guarantee the sweep exists to make:
	 * every CTB row intra coded once per period, and a region a receiver
	 * asked for after losing data actually sent. Restoring these puts the
	 * scheduler back where it was, so the next picture is offered the same
	 * band and the same request.
	 *
	 * Taken here rather than next to the scheduler because the predictive
	 * decision point jumps straight to `skipped` from below, and the
	 * restore there has to be well defined on both paths. For that path it
	 * is a copy of state that has not moved, i.e. a no-op. */
	ImxVpuApiIntraRefreshState const refresh_state_before = encoder->refresh_state;
	int const forced_q_head_before = encoder->forced_intra_q_head;
	int const forced_q_count_before = encoder->forced_intra_q_count;

	/* The rate control's last resort. A picture that cannot be made to fit
	 * in the HRD buffer even at the coarsest quantiser allowed is not coded
	 * at all: past qp_max the re-encode ladder has no rungs left, and every
	 * further picture would go in on top of a buffer that is already full,
	 * so the queue - which is the latency - would grow without bound.
	 *
	 * This has to happen here, ahead of everything that carries per-picture
	 * state: the refresh sweep just below, the SEI that describes its band,
	 * and the GOP position VCEncFindNextPic() advances at the end. A skipped
	 * picture must leave every one of them exactly where it was, so that the
	 * next input frame takes the slot this one would have had. A sweep step
	 * spent on a picture that is never coded would leave those CTB rows
	 * unrefreshed for a whole further period with a receiver told they were
	 * clean.
	 *
	 * Nothing is handed to the hardware, so no reference picture moves
	 * either: the next picture predicts from the same reconstruction the
	 * decoder is holding. */
	{
		int const intra_input = (encoder_input->codingType == VCENC_INTRA_FRAME);

		/* Two reasons to refuse the picture before encoding it. The valve's
		 * forecast, and - for a keyframe - the buffer simply not having room
		 * for one yet. The second leaves the intra request armed and drains
		 * a frame budget, so the keyframe lands a picture or two later at
		 * the quantiser it was predicted at instead of being ground down to
		 * fit a buffer that was briefly full. */
		if (encoder->new_cbr_active && !is_first_picture
		 && (ext_rate_control_should_skip(&(encoder->new_cbr), intra_input)
		  || (intra_input && !ext_rate_control_keyframe_ready(&(encoder->new_cbr)))))
		{
			ext_rate_control_skip(&(encoder->new_cbr), intra_input, 0, NULL);
			goto skipped;
		}
	}

	int sei_refresh = 0, sei_top = 0, sei_bottom = 0;

	/* One refresh scheduler for what used to be three modes. An IDR
	 * refreshes the whole picture by itself, so the sweep sits that picture
	 * out without losing its place. */
	if (encoder->refresh_active && !is_idr)
	{
		ImxVpuApiIntraRefreshBand band;
		int forced_first = 0, forced_num = 0;

		/* Regions asked for through
		 * imx_vpu_api_enc_set_intra_refresh_region(), which is how a
		 * receiver reports what it lost. Offer the head of the queue,
		 * and only take it off if the scheduler used it - it declines on
		 * a picture the sweep itself needs, and a request that keeps
		 * getting declined has to stay queued rather than be dropped. */
		if (encoder->pending_full_refresh)
		{
			/* The whole picture, in one go. The band is expressed in coding
			 * units and spans the picture, so however it is divided into
			 * slices every slice of it comes out intra. */
			imx_vpu_api_intra_refresh_full(&(encoder->refresh_cfg), &band);
			imx_vpu_api_intra_refresh_realign(&(encoder->refresh_state));

			/* A recovery point of 0: this picture is complete by itself, so
			 * a decoder joining here waits for nothing. The sweep's own
			 * recovery_count cannot express that, being a countdown. */
			encoder->recovery_sei_size = imx_vpu_api_build_recovery_point_sei(
				encoder->recovery_sei, 0,
				encoder->open_params.compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H264);
		}
		else
		{
			if (encoder->forced_intra_q_count > 0)
			{
				int const idx = encoder->forced_intra_q_head;

				forced_first = (int)(encoder->forced_intra_q[idx].first);
				forced_num = (int)(encoder->forced_intra_q[idx].num);
			}

			imx_vpu_api_intra_refresh_step(&(encoder->refresh_state), &(encoder->refresh_cfg),
			                               forced_first, forced_num, &band);

			if (band.forced)
			{
				encoder->forced_intra_q_head = (encoder->forced_intra_q_head + 1)
				                             % FORCED_INTRA_QUEUE_SIZE;
				encoder->forced_intra_q_count--;
			}
		}

		if (band.recovery_count > 0)
		{
			encoder->recovery_sei_size = imx_vpu_api_build_recovery_point_sei(
				encoder->recovery_sei, band.recovery_count,
				encoder->open_params.compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H264);
		}

		if (band.apply)
		{
			encoder->cached_coding_ctrl.intraArea.enable = 1;
			encoder->cached_coding_ctrl.intraArea.left   = band.left;
			encoder->cached_coding_ctrl.intraArea.right  = band.right;
			encoder->cached_coding_ctrl.intraArea.top    = band.top;
			encoder->cached_coding_ctrl.intraArea.bottom = band.bottom;

			enc_ret = VCEncSetCodingCtrl(encoder->encoder, &(encoder->cached_coding_ctrl));
			if (enc_ret != VCENC_OK)
			{
				IMX_VPU_API_ERROR("could not set intra refresh region, CTB rows %d..%d "
				                  "cols %d..%d: %s (%d)",
				                  band.top, band.bottom, band.left, band.right,
				                  vcenc_retval_to_string(enc_ret), (int)enc_ret);
				goto error;
			}

			/* The refresh SEI describes a full width band, so a region
			 * narrowed to a column range - the deprecated tiles
			 * mapping, and nothing else - deliberately does not get
			 * one. A receiver told that rows N..M are clean when only
			 * half their width is would treat stale macroblocks as
			 * recovered. */
			if ((band.left == 0) && (band.right == (encoder->refresh_cfg.ctb_cols - 1)))
			{
				sei_refresh = 1;
				sei_top = band.top;
				sei_bottom = band.bottom;
			}

			if (getenv("VR_FORCED_INTRA_DEBUG"))
				fprintf(stderr, "VR intra refresh: CTB rows %d..%d cols %d..%d "
				        "(frame %d, %s, qdepth %d)\n",
				        band.top, band.bottom, band.left, band.right,
				        (int)encoder->num_encoded_pictures,
				        band.forced ? "requested" : "sweep",
				        encoder->forced_intra_q_count);
		}
		else if (encoder->cached_coding_ctrl.intraArea.enable)
		{
			encoder->cached_coding_ctrl.intraArea.enable = 0;
			VCEncSetCodingCtrl(encoder->encoder, &(encoder->cached_coding_ctrl));
		}
	}

	if (encoder->refresh_active &&
	    !(getenv("VR_RECOVERY_SEI") && getenv("VR_RECOVERY_SEI")[0] == '0'))
	{
		if (sei_refresh)
		{
			uint8_t sei[IMX_VPU_API_REFRESH_BAND_SEI_SIZE];
			size_t const sei_size = imx_vpu_api_build_refresh_band_sei(
				sei, sei_top, sei_bottom - sei_top + 1,
				(int)(encoder->num_encoded_pictures));
			VCEncSetSeiUserData(encoder->encoder, sei, (u32)sei_size);
		}
		else
		{
			VCEncSetSeiUserData(encoder->encoder, NULL, 0);
		}
	}

	/* Everything imx_vpu_api_enc_get_encoded_frame_ext() will put on the
	 * link ahead of the picture itself. Both are known by now: has_header
	 * was decided at the top, and recovery_sei_size by the refresh
	 * scheduler just above. The rate control is charged for all of it, so
	 * the ceiling it re-encodes against, the valve's fit test and the bucket
	 * all measure the same thing the link carries. */
	size_t const overhead_bits = ((encoder->has_header ? encoder->header_data_size : 0)
	                            + encoder->recovery_sei_size) * 8;

	/* New CBR: pick this picture's QP before handing it to the encoder. */
	if (encoder->new_cbr_active)
	{
		int qp = ext_rate_control_pre(&encoder->new_cbr, (encoder_input->codingType == VCENC_INTRA_FRAME), overhead_bits);

		encoder->cached_rate_ctrl.qpHdr = qp;
		enc_ret = VCEncSetRateCtrl(encoder->encoder, &(encoder->cached_rate_ctrl));
		if (enc_ret != VCENC_OK)
		{
			IMX_VPU_API_ERROR("could not set QP %d for picture %" PRId32 ": %s (%d)",
			                  qp, encoder->num_encoded_pictures,
			                  vcenc_retval_to_string(enc_ret), (int)enc_ret);
			goto error;
		}
	}

	/* Perform the actual frame encoding. */
	memset(&encoder_output, 0, sizeof(encoder_output));
	enc_ret = encode_one_picture(encoder, encoder_input, &encoder_output);

	if (encoder->drop_inserted_pps)
	{
		if (enc_ret == VCENC_FRAME_READY)
			drop_inserted_pps_nal(encoder, &encoder_output);
		else
		{
			/* The picture it was written into is being thrown away, and the
			 * vendor library only inserts it once. The header carries the
			 * same PPS, so nothing is lost. */
			IMX_VPU_API_DEBUG("the picture carrying the inserted PPS was discarded");
			encoder->drop_inserted_pps = FALSE;
		}
	}

	/* New CBR: a picture past the ceiling is re-encoded coarser. Only the
	 * attempt that is kept is reported to the rate control, so its model
	 * never sees the discarded ones. Re-encoding is deterministic and leaves
	 * the reference chain intact - the encoder has not been told to advance
	 * yet, which is what VCEncFindNextPic() below does. */
	if (encoder->new_cbr_active && (enc_ret == VCENC_FRAME_READY))
	{
		int const is_intra = (encoder_output.codingType == VCENC_INTRA_FRAME);
		int kept_qp = encoder->cached_rate_ctrl.qpHdr;
		int qp;

		while ((qp = ext_rate_control_check(&encoder->new_cbr, (size_t)(encoder_output.streamSize) * 8, overhead_bits, is_intra)) > 0)
		{
			VCEncOut retry_output;

			encoder->cached_rate_ctrl.qpHdr = qp;
			if (VCEncSetRateCtrl(encoder->encoder, &(encoder->cached_rate_ctrl)) != VCENC_OK)
				break;

			memset(&retry_output, 0, sizeof(retry_output));
			if (encode_one_picture(encoder, encoder_input, &retry_output) != VCENC_FRAME_READY)
				break;

			encoder_output = retry_output;
			kept_qp = qp;
		}

		/* An attempt that was proposed but never produced output must not be
		 * what the model learns from - the QP the kept bits were coded at is
		 * the only one that describes them. */
		encoder->new_cbr.current_qp = kept_qp;

		/* The ladder's last rung. The picture is at qp_max and still does
		 * not fit in the buffer, so it is not sent: there is nothing left
		 * to trade quality for, and sending it would put the queue - which
		 * is the latency - past the buffer the caller asked for.
		 *
		 * Discarding it here is safe precisely because none of this
		 * picture's own bookkeeping has advanced yet. VCEncFindNextPic()
		 * below is what moves the GOP on, so skipping to `skipped` leaves
		 * the next input frame to be encoded in this picture's slot,
		 * against the same reference, overwriting the reconstruction that
		 * is being thrown away. Encoder and decoder stay in step. The
		 * first picture of the stream is exempt: it carries the header and
		 * starts the stream, and there is no earlier reference for a later
		 * frame to be coded against. */
		if (!is_first_picture
		 && ext_rate_control_should_drop(&(encoder->new_cbr),
		                                 (size_t)(encoder_output.streamSize) * 8 + overhead_bits,
		                                 is_intra))
		{
			ExtRateControlStats cu_stats;
			unsigned int const total_blocks =
				(unsigned int)((encoder->open_params.frame_width / 8)
				             * (encoder->open_params.frame_height / 8));

			vc8000e_fill_rc_stats(&cu_stats, &encoder_output, total_blocks);
			/* The model learns the picture's own cost; the overhead was
			 * never going to be sent for a picture that is not sent. */
			ext_rate_control_skip(&(encoder->new_cbr), is_intra,
			                      (size_t)(encoder_output.streamSize) * 8, &cu_stats);
			goto skipped;
		}
	}

	if (enc_ret == VCENC_HRD_ERROR)
	{
		/* HRD/CPB overflow: the hardware discarded this picture by design and
		 * restored its internal state - this is the HRD's graceful skip, not
		 * a hang. Do NOT run the lost-IRQ auto-recovery: a restart forces a
		 * new bootstrap I-frame, which under sustained overload (e.g.
		 * max-gain sensor noise in darkness) overflows the CPB again ->
		 * restart storm -> frozen stream. Report a skipped frame and
		 * continue; output resumes as soon as the CPB drains. */
		IMX_VPU_API_INFO("HRD CPB overflow; picture skipped by rate control");

		encoder->skipped_frame_context = encoder->staged_raw_frame.context;
		encoder->skipped_frame_pts = encoder->staged_raw_frame.pts;
		encoder->skipped_frame_dts = encoder->staged_raw_frame.dts;
		encoder->skipped_frame_available = TRUE;

		*output_code = IMX_VPU_API_ENC_OUTPUT_CODE_FRAME_SKIPPED;
		ret = IMX_VPU_API_ENC_RETURN_CODE_OK;
		goto finish;
	}
	/* PR #19's lost-IRQ auto-recovery used to sit here, releasing and
	 * re-initialising the encoder on any non-FRAME_READY return. It is gone for
	 * good.
	 *
	 * It was aimed at the wrong thing. The stall it worked around is the
	 * VC8000E wedging on picture start, which happens only when slice-ready
	 * interrupts are armed - and those are now permanently off, so the failure
	 * it caught should not occur at all. The restart also worked for a reason
	 * its own log message got wrong: not "lost IRQ", but that VCEncRelease()
	 * closes /dev/mxc_hantro_vc8000e and the reopen power-cycles the block.
	 * It cost a dropped frame and a forced IDR every time it fired, and it
	 * fired on genuine hardware errors too, turning them into silent quality
	 * dips instead of reports.
	 *
	 * The full diagnosis is in the comment above vcenc_reject_slice_ready_cb()
	 * in the VC8000E encoder's hevcencapi.c.
	 * https://github.com/Auterion/vpu-imx-recipe/pull/19 */
	if (enc_ret != VCENC_FRAME_READY)
	{
		IMX_VPU_API_ERROR(
			"VCEncStrmEncode returned %s (%d) after %d encoded pictures",
			vcenc_retval_to_string(enc_ret), (int)enc_ret,
			(int)encoder->num_encoded_pictures
		);
		goto error;
	}

	if (encoder_output.streamSize == 0)
	{
		/* With HRD enabled the VC8000E signals a CPB-overflow skip as a
		 * SUCCESSFUL encode with zero output bytes (not VCENC_HRD_ERROR).
		 * Without this check the zero-byte "frame" propagates with a bogus
		 * frame context, the gst layer cannot map it to a queued frame and
		 * discards it, and the queued GstVideoCodecFrame (plus its DMA input
		 * buffer) leaks -> stream stalls/freezes and CMA memory runs out
		 * under sustained overload (e.g. max-gain sensor noise in darkness).
		 * Report it as a proper skipped frame instead; the gst skipped-frame
		 * handler finishes/releases the queued frame and the stream resumes
		 * as soon as the CPB drains. */
		IMX_VPU_API_INFO("HRD CPB overflow: zero-byte encode; reporting picture as skipped");

		encoder->skipped_frame_context = encoder->staged_raw_frame.context;
		encoder->skipped_frame_pts = encoder->staged_raw_frame.pts;
		encoder->skipped_frame_dts = encoder->staged_raw_frame.dts;
		encoder->skipped_frame_available = TRUE;

		*output_code = IMX_VPU_API_ENC_OUTPUT_CODE_FRAME_SKIPPED;
		ret = IMX_VPU_API_ENC_RETURN_CODE_OK;
		goto finish;
	}

	encoder->num_bytes_in_stream_buffer = encoder_output.streamSize;
	*encoded_frame_size += encoder_output.streamSize + encoder->recovery_sei_size;

	/* New CBR: report the attempt that was kept. Only the picture's own bits
	 * are counted, not a header that may be prepended to it - the header is
	 * ~90 bytes against several kilobytes of picture, and charging it would
	 * teach the model that a refresh picture is more expensive than it is. */
	if (encoder->new_cbr_active)
	{
		ExtRateControlStats cu_stats;
		unsigned int total_blocks = (unsigned int)((encoder->open_params.frame_width / 8) * (encoder->open_params.frame_height / 8));

		vc8000e_fill_rc_stats(&cu_stats, &encoder_output, total_blocks);
		ext_rate_control_post(
			&encoder->new_cbr,
			(size_t)(encoder_output.streamSize) * 8,
			overhead_bits,
			(encoder_output.codingType == VCENC_INTRA_FRAME),
			&cu_stats
		);

		IMX_VPU_API_LOG(
			"new CBR: picture %" PRId32 " qp %d target %d bits %u (%+d%%) HRD buffer %.0f/%.0f kbit coded %u/%u",
			encoder->num_encoded_pictures,
			encoder->new_cbr.current_qp,
			ext_rate_control_target(&encoder->new_cbr),
			(unsigned int)(encoder_output.streamSize * 8),
						(ext_rate_control_target(&encoder->new_cbr) > 0)
				? (int)(((double)(encoder_output.streamSize) * 8.0 - ext_rate_control_target(&encoder->new_cbr))
				        * 100.0 / ext_rate_control_target(&encoder->new_cbr))
				: 0,
			encoder->new_cbr.bucket / 1000.0, encoder->new_cbr.bucket_cap / 1000.0,
			total_blocks - cu_stats.skip_blocks, total_blocks
		);
	}

	/* Request the coding type for the next frame. This must be called, even
	 * when encoding h.264 (contrary to the comments in the hevcencapi.h header),
	 * otherwise the driver crashes when trying to encode a predicted frame. */
	encoder->next_coding_type = VCEncFindNextPic(encoder->encoder, encoder_input, 1, encoder_input->gopConfig.gopCfgOffset, FALSE);

	IMX_VPU_API_LOG(
		"encoded frame:  IDR: %d  coding type: %s  size: %" PRIu32 "  next coding type: %s",
		is_idr,
		vcenc_picture_coding_type_to_string(encoder_output.codingType),
		(uint32_t)(encoder_output.streamSize),
		vcenc_picture_coding_type_to_string(encoder->next_coding_type)
	);

	/* Copy over metadata from the raw frame to the encoded frame. Since the
	 * encoder does not perform any kind of delay or reordering, this is
	 * appropriate, because in that case, one input frame always immediately
	 * leads to one output frame. */
	encoder->encoded_frame_context = encoder->staged_raw_frame.context;
	encoder->encoded_frame_pts = encoder->staged_raw_frame.pts;
	encoder->encoded_frame_dts = encoder->staged_raw_frame.dts;
	encoder->encoded_frame_data_size = *encoded_frame_size;
	encoder->encoded_frame_available = TRUE;

	encoder->force_IDR_frame = FALSE;
	encoder->pending_full_refresh = FALSE;
	encoder->postponed_frame_type = IMX_VPU_API_FRAME_TYPE_UNKNOWN;

	encoder->num_encoded_pictures++;

	*output_code = IMX_VPU_API_ENC_OUTPUT_CODE_ENCODED_FRAME_AVAILABLE;

	goto finish;


	/* Shared by the valve's two decision points: the predictive one before
	 * the encode and the measured one after it. Reached only by goto - the
	 * success path above jumps over it. */
skipped:
	encoder->refresh_state = refresh_state_before;
	encoder->forced_intra_q_head = forced_q_head_before;
	encoder->forced_intra_q_count = forced_q_count_before;

	/* An intra picture is deferred, never dropped. The GOP's own periodic
	 * IDR needs nothing: the GOP position does not advance either, so the
	 * next frame is offered the same coding type again. force_IDR_frame
	 * likewise re-arms itself, being cleared only once a picture has been
	 * coded. A type that arrived on the raw frame would otherwise expire
	 * with it, so that one is carried forward explicitly. */
	if ((requested_frame_type == IMX_VPU_API_FRAME_TYPE_I)
	 || (requested_frame_type == IMX_VPU_API_FRAME_TYPE_IDR))
	{
		encoder->postponed_frame_type = requested_frame_type;
		IMX_VPU_API_INFO("the intra picture is deferred to the next frame, not dropped");
	}

	encoder->skipped_frame_context = encoder->staged_raw_frame.context;
	encoder->skipped_frame_pts = encoder->staged_raw_frame.pts;
	encoder->skipped_frame_dts = encoder->staged_raw_frame.dts;
	encoder->skipped_frame_available = TRUE;

	/* has_header is cleared only when a picture is handed out with the
	 * header attached, so it stays pending for the next one; the size
	 * accumulated for it above belongs to that picture and not to this one,
	 * which is not being sent. */
	*encoded_frame_size = 0;
	encoder->num_bytes_in_stream_buffer = 0;

	*output_code = IMX_VPU_API_ENC_OUTPUT_CODE_FRAME_SKIPPED;
	ret = IMX_VPU_API_ENC_RETURN_CODE_OK;

finish:
	if (encoder->staged_raw_frame_set)
	{
		imx_dma_buffer_unmap(encoder->staged_raw_frame.fb_dma_buffer);
		encoder->staged_raw_frame_set = FALSE;
	}

	return ret;

error:
	IMX_VPU_API_ERROR("error while encoding frame: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
	ret = IMX_VPU_API_ENC_RETURN_CODE_ERROR;
	goto finish;
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_get_encoded_frame(ImxVpuApiEncoder *encoder, ImxVpuApiEncodedFrame *encoded_frame)
{
	return imx_vpu_api_enc_get_encoded_frame_ext(encoder, encoded_frame, NULL);
}


ImxVpuApiEncReturnCodes imx_vpu_api_enc_get_encoded_frame_ext(ImxVpuApiEncoder *encoder, ImxVpuApiEncodedFrame *encoded_frame, int *is_sync_point)
{
	uint8_t *encoded_data = encoded_frame->data;

	assert(encoder != NULL);
	assert(encoded_frame != NULL);
	assert(encoded_frame->data != NULL);

	if (!(encoder->encoded_frame_available))
	{
		IMX_VPU_API_ERROR("cannot retrieve encoded frame since there is none");
		return IMX_VPU_API_ENC_RETURN_CODE_INVALID_CALL;
	}

	/* Prepend the header data if it was previously requested. */
	if (encoder->has_header)
	{
		memcpy(encoded_data, encoder->header_data, encoder->header_data_size);
		encoded_data += encoder->header_data_size;
	}

	/* After the parameter sets, before the slice: a prefix SEI has to precede
	 * the first VCL NAL of the picture it applies to. */
	if (encoder->recovery_sei_size > 0)
	{
		memcpy(encoded_data, encoder->recovery_sei, encoder->recovery_sei_size);
		encoded_data += encoder->recovery_sei_size;
	}


	/* Use synced access since we have to copy the encoded
	 * data out of the stream buffer. */
	imx_dma_buffer_start_sync_session(encoder->stream_buffer);
	memcpy(encoded_data, encoder->stream_buffer_virtual_address, encoder->num_bytes_in_stream_buffer);
	imx_dma_buffer_stop_sync_session(encoder->stream_buffer);


	/* Copy encoded frame metadata. */

	encoded_frame->data_size = encoder->encoded_frame_data_size;
	encoded_frame->has_header = encoder->has_header;
	encoded_frame->header_size = encoder->has_header ? encoder->header_data_size : 0;
	encoded_frame->frame_type = encoder->encoded_frame_type;
	encoded_frame->context = encoder->encoded_frame_context;
	encoded_frame->pts = encoder->encoded_frame_pts;
	encoded_frame->dts = encoder->encoded_frame_dts;


	if (is_sync_point)
	{
		/* In h.264 and h.265, IDR frames are sync points. Non-IDR I-frames
		 * are also treated as sync points when they carry inline headers
		 * (has_header=TRUE), which happens at stream start and after a
		 * flush.  This allows refresh-mode streams (which begin with a
		 * non-IDR I-frame instead of an IDR) to preroll correctly in
		 * GStreamer and be used as valid decoder entry points. */

		switch (encoder->encoded_frame_type)
		{
			case IMX_VPU_API_COMPRESSION_FORMAT_H264:
			case IMX_VPU_API_COMPRESSION_FORMAT_H265:
				*is_sync_point = (encoder->encoded_frame_type == IMX_VPU_API_FRAME_TYPE_IDR)
				              || (encoder->encoded_frame_type == IMX_VPU_API_FRAME_TYPE_I && encoder->has_header);
				break;
			default:
				*is_sync_point = (encoder->encoded_frame_type == IMX_VPU_API_FRAME_TYPE_I);
				break;
		}
	}


	/* Reset some flags for the next imx_vpu_api_enc_encode() call,
	 * since we are done with this frame. */
	encoder->encoded_frame_available = FALSE;
	encoder->num_bytes_in_stream_buffer = 0;
	encoder->has_header = FALSE;


	return IMX_VPU_API_ENC_RETURN_CODE_OK;
}

ImxVpuApiEncReturnCodes imx_vpu_api_enc_get_skipped_frame_info(ImxVpuApiEncoder *encoder, void **context, uint64_t *pts, uint64_t *dts)
{
	assert(encoder != NULL);

	if (!encoder->skipped_frame_available)
		return IMX_VPU_API_ENC_RETURN_CODE_INVALID_CALL;

	if (context != NULL) *context = encoder->skipped_frame_context;
	if (pts != NULL)     *pts = encoder->skipped_frame_pts;
	if (dts != NULL)     *dts = encoder->skipped_frame_dts;

	encoder->skipped_frame_available = FALSE;
	return IMX_VPU_API_ENC_RETURN_CODE_OK;
}
