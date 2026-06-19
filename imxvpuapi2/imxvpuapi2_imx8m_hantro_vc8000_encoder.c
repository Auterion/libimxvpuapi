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
#define MAX_ROLLING_SLICES 64

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

	/* Rolling intra slice/tile refresh state (H.265 / VC8000E only).
	 * num_rolling_slices = 0 means disabled; 1 = auto (4 slices); 2..16 = explicit count.
	 * num_rolling_tiles  = 0 means disabled; 1 = auto (2x2 grid); 2..16 = explicit even count.
	 *                      (arranged in 2 fixed columns, ceil(N/2) rows).
	 * cached_coding_ctrl holds the VCEncCodingCtrl captured right after
	 * the initial VCEncSetCodingCtrl call so that roiMapDeltaQpEnable is
	 * correct for all subsequent mid-stream SetCodingCtrl calls. */
	int num_rolling_slices;
	int num_rolling_tiles;
	VCEncCodingCtrl cached_coding_ctrl;

	struct { uint32_t first; uint32_t num; } forced_intra_q[FORCED_INTRA_QUEUE_SIZE];
	int forced_intra_q_head;
	int forced_intra_q_count;

	int roll_size;
	int last_refresh_picture;
	int slice_last_refresh[MAX_ROLLING_SLICES];

	BOOL skipped_frame_available;
	void *skipped_frame_context;
	uint64_t skipped_frame_pts, skipped_frame_dts;
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
	open_params->hrd_buffer_size = 1000;
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

static void init_encoder_input(ImxVpuApiEncoder *encoder,
                               int num_rolling_slices,
                               int num_rolling_tiles)
{
	ImxVpuApiEncOpenParams *open_params = &encoder->open_params;
	VCEncIn *encoder_input = &encoder->encoder_input;
	int i;

	memset(encoder_input, 0, sizeof(VCEncIn));

	encoder_input->gopConfig.pGopPicCfg = &encoder->gop_pic_config[0];
	encoder_input->gopConfig.size = 1;
	encoder_input->gopConfig.special_size = 0;
	encoder_input->gopConfig.pGopPicSpecialCfg = &encoder->gop_pic_special_config[0];
	encoder_input->gopConfig.idr_interval = ((num_rolling_slices > 0) || (num_rolling_tiles > 0)) ? INT32_MAX : open_params->gop_size;
	if (open_params->flags & IMX_VPU_API_ENC_OPEN_PARAMS_FLAG_USE_INTRA_REFRESH)
		encoder_input->gopConfig.gdrDuration = open_params->gop_size;
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
                                                   ImxVpuApiFramebufferMetrics const *fb_metrics,
                                                   int num_rolling_slices,
                                                   int num_rolling_tiles)
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
		if (open_params->flags & IMX_VPU_API_ENC_OPEN_PARAMS_FLAG_USE_INTRA_REFRESH)
			coding_config.gdrDuration = open_params->gop_size;
		else
			coding_config.gdrDuration = 0;

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


		if (num_rolling_slices > 0)
		{
			const int ctu_rows = ((int)(open_params->frame_height) + 63) / 64;
			const int rows_per_slice = (ctu_rows + num_rolling_slices - 1) / num_rolling_slices;
			coding_config.sliceSize = rows_per_slice;
		}
		else if (num_rolling_tiles > 0)
		{
			const int ctu_rows = ((int)(open_params->frame_height) + 63) / 64;
			const int num_tile_rows = (num_rolling_tiles + 1) / 2; /* 2 columns per row */
			const int rows_per_tile_row = (ctu_rows + num_tile_rows - 1) / num_tile_rows;
			coding_config.sliceSize = rows_per_tile_row;
		}

		enc_ret = VCEncSetCodingCtrl(encoder->encoder, &coding_config);
		if (enc_ret != VCENC_OK)
		{
			IMX_VPU_API_ERROR("could not set coding configuration: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
			VCEncRelease(encoder->encoder);
			encoder->encoder = NULL;
			return IMX_VPU_API_ENC_RETURN_CODE_ERROR;
		}

		VCEncGetCodingCtrl(encoder->encoder, &encoder->cached_coding_ctrl);
		encoder->num_rolling_slices = num_rolling_slices;
		encoder->num_rolling_tiles = num_rolling_tiles;

		encoder->roll_size = (int)encoder->open_params.roll_size;
		encoder->last_refresh_picture = -1000000;
		{
			int i;
			for (i = 0; i < MAX_ROLLING_SLICES; i++)
				encoder->slice_last_refresh[i] = -1000000;
		}
		encoder->forced_intra_q_head = 0;
		encoder->forced_intra_q_count = 0;
	}



	/* Set up rate control. */

	{
		VCEncRateCtrl rate_control_config;
		memset(&rate_control_config, 0, sizeof(rate_control_config));
		BOOL use_rate_control = (open_params->bitrate != 0);

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
		rate_control_config.qpMaxPB = rate_control_config.qpMaxI = use_rate_control ? 51 : open_params->quantization;
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
		rate_control_config.hrdCpbSize = open_params->hrd_buffer_size * 1000;
		rate_control_config.bitrateWindow = open_params->gop_size;
		rate_control_config.intraQpDelta = open_params->intra_qp_delta;
		rate_control_config.tolMovingBitRate = 2000;
		rate_control_config.rcQpDeltaRange = 10;
		rate_control_config.rcBaseMBComplexity = 15;
		rate_control_config.picQpDeltaMin = -2;
		rate_control_config.picQpDeltaMax = +3;
		rate_control_config.tolCtbRcIntra = -1;

		enc_ret = VCEncSetRateCtrl(encoder->encoder, &rate_control_config);
		if (enc_ret != VCENC_OK)
		{
			IMX_VPU_API_ERROR("could not set rate control configuration: %s (%d)", vcenc_retval_to_string(enc_ret), (int)enc_ret);
			VCEncRelease(encoder->encoder);
			encoder->encoder = NULL;
			return IMX_VPU_API_ENC_RETURN_CODE_ERROR;
		}
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
		preprocessing_config.rotation = VCENC_ROTATE_0;
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
	int num_rolling_slices = open_params->num_rolling_slices;
	if (num_rolling_slices == 1) num_rolling_slices = 4; /* auto */
	int num_rolling_tiles = open_params->num_rolling_tiles;
	if (num_rolling_tiles == 1) num_rolling_tiles = 4; /* auto → 2×2 grid */

	init_encoder_input(*encoder, num_rolling_slices, num_rolling_tiles);

	ret = init_vcenc_instance(*encoder, encoder_pixel_format, fb_metrics, num_rolling_slices, num_rolling_tiles);
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


static ImxVpuApiEncReturnCodes restart_encoder(ImxVpuApiEncoder *encoder)
{
	ImxVpuApiEncReturnCodes ret;
	int num_rolling_slices;
	int num_rolling_tiles;

	IMX_VPU_API_INFO(
		"VC8000E auto-recovery: releasing and re-initializing the encoder "
		"after %d encoded frames",
		(int)encoder->num_encoded_pictures
	);

	if (encoder->encoder != NULL)
	{
		VCEncRelease(encoder->encoder);
		encoder->encoder = NULL;
		usleep(50 * 1000);
	}

	free(encoder->header_data);
	encoder->header_data = NULL;
	encoder->header_data_size = 0;
	encoder->has_header = FALSE;

	encoder->num_encoded_pictures = 0;
	encoder->next_coding_type = VCENC_NOTCODED_FRAME;
	encoder->force_IDR_frame = FALSE;

	num_rolling_slices = encoder->open_params.num_rolling_slices;
	if (num_rolling_slices == 1) num_rolling_slices = 4;
	num_rolling_tiles = encoder->open_params.num_rolling_tiles;
	if (num_rolling_tiles == 1) num_rolling_tiles = 4;

	init_encoder_input(encoder, num_rolling_slices, num_rolling_tiles);

	ret = init_vcenc_instance(
		encoder,
		convert_to_vc8000e_pixel_format(encoder->open_params.color_format),
		&encoder->stream_info.frame_encoding_framebuffer_metrics,
		num_rolling_slices,
		num_rolling_tiles
	);
	if (ret != IMX_VPU_API_ENC_RETURN_CODE_OK)
	{
		IMX_VPU_API_ERROR("encoder restart failed; subsequent encode calls will fail");
		return ret;
	}

	return IMX_VPU_API_ENC_RETURN_CODE_OK;
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
	if (is_first_picture)
	{
		BOOL use_refresh_mode = (encoder->num_rolling_slices > 0)
		                     || (encoder->num_rolling_tiles  > 0);
		if (use_refresh_mode)
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

	if (!encoder->has_header && (encoder->num_encoded_pictures > 0) &&
	    ((encoder->num_rolling_slices > 0) || (encoder->num_rolling_tiles > 0) ||
	     (encoder->open_params.flags & IMX_VPU_API_ENC_OPEN_PARAMS_FLAG_USE_INTRA_REFRESH)) &&
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

	if (is_first_picture)
	{
		/* Start the stream if we are encoding the very first picture.
		 * This is where the SPS/PPS/VPS header data is generated. */

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

	int sei_refresh = 0, sei_top = 0, sei_bottom = 0;

	if ((encoder->num_rolling_slices > 0) && !is_idr &&
	    (encoder->open_params.compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H265))
	{
		int gop_size    = (int)(encoder->open_params.gop_size);
		int num_slices  = encoder->num_rolling_slices;
		int frame_h     = (int)(encoder->open_params.frame_height);
		int frame_w     = (int)(encoder->open_params.frame_width);
		int ctu_rows    = (frame_h + 63) / 64;
		int ctu_cols    = (frame_w + 63) / 64;
		int rows_per_slice;
		int roll        = (encoder->roll_size > 0) ? encoder->roll_size : gop_size;
		int pic         = (int)(encoder->num_encoded_pictures);
		int min_gap;
		int top = 0, bottom = -1;
		int from_forced = 0;

		if (num_slices > MAX_ROLLING_SLICES) num_slices = MAX_ROLLING_SLICES;
		rows_per_slice = (ctu_rows + num_slices - 1) / num_slices;
		if (roll > gop_size) roll = gop_size;
		min_gap = roll / num_slices;
		if (min_gap < 1) min_gap = 1;

		{
			int s, oldest = 0, oldest_age, emergency;

			for (s = 1; s < num_slices; s++)
			{
				if (encoder->slice_last_refresh[s] < encoder->slice_last_refresh[oldest])
					oldest = s;
			}
			oldest_age = pic - encoder->slice_last_refresh[oldest];
			emergency = (oldest_age >= roll);

			if (encoder->forced_intra_q_count > 0 && !emergency)
			{
				int idx = encoder->forced_intra_q_head;
				int cb;
				top    = (int)encoder->forced_intra_q[idx].first;
				bottom = top + (int)encoder->forced_intra_q[idx].num - 1;
				encoder->forced_intra_q_head = (idx + 1) % FORCED_INTRA_QUEUE_SIZE;
				encoder->forced_intra_q_count--;
				from_forced = 1;

				cb = (bottom < ctu_rows) ? bottom : (ctu_rows - 1);
				for (s = 0; s < num_slices; s++)
				{
					int s_top = s * rows_per_slice;
					int s_bot = s_top + rows_per_slice - 1;
					if (!(s_bot < top || s_top > cb))
						encoder->slice_last_refresh[s] = pic;
				}
			}
			else if (emergency || (pic - encoder->last_refresh_picture >= min_gap))
			{
				top = oldest * rows_per_slice;
				bottom = top + rows_per_slice - 1;
				encoder->slice_last_refresh[oldest] = pic;
				encoder->last_refresh_picture = pic;
			}
		}

		if (bottom >= top)
		{
			if (top < 0) top = 0;
			if (bottom >= ctu_rows) bottom = ctu_rows - 1;
			encoder->cached_coding_ctrl.intraArea.enable = 1;
			encoder->cached_coding_ctrl.intraArea.left   = 0;
			encoder->cached_coding_ctrl.intraArea.right  = ctu_cols - 1;
			encoder->cached_coding_ctrl.intraArea.top    = top;
			encoder->cached_coding_ctrl.intraArea.bottom = bottom;
			VCEncSetCodingCtrl(encoder->encoder, &(encoder->cached_coding_ctrl));
			sei_refresh = 1; sei_top = top; sei_bottom = bottom;
			if (getenv("VR_FORCED_INTRA_DEBUG"))
				fprintf(stderr, "VR refresh applied: CTB rows %d..%d (frame %d, %s, qdepth %d)\n",
				        top, bottom, (int)encoder->num_encoded_pictures,
				        from_forced ? "forced" : "wave", encoder->forced_intra_q_count);
		}
		else if (encoder->cached_coding_ctrl.intraArea.enable)
		{
			encoder->cached_coding_ctrl.intraArea.enable = 0;
			VCEncSetCodingCtrl(encoder->encoder, &(encoder->cached_coding_ctrl));
		}
	}

	if ((encoder->num_rolling_tiles > 0) && !is_idr &&
	    (encoder->open_params.compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H265))
	{
		int gop_size      = (int)(encoder->open_params.gop_size);
		int num_tiles     = encoder->num_rolling_tiles;
		int frame_h       = (int)(encoder->open_params.frame_height);
		int frame_w       = (int)(encoder->open_params.frame_width);
		int ctu_rows      = (frame_h + 63) / 64;
		int ctu_cols      = (frame_w + 63) / 64;
		int num_tile_cols = 2;
		int num_tile_rows = (num_tiles + num_tile_cols - 1) / num_tile_cols;
		int ctu_cols_per_tile = ctu_cols / num_tile_cols;
		int ctu_rows_per_tile = ctu_rows / num_tile_rows;
		int gop_pos       = (int)(encoder->num_encoded_pictures) % gop_size;
		int tile_idx      = -1;
		int i;

		for (i = 0; i < num_tiles; i++)
		{
			if (gop_pos == (i * gop_size) / num_tiles)
			{
				tile_idx = i;
				break;
			}
		}

		if (tile_idx >= 0)
		{
			int tile_col  = tile_idx % num_tile_cols;
			int tile_row  = tile_idx / num_tile_cols;
			int left      = tile_col * ctu_cols_per_tile;
			int right     = left + ctu_cols_per_tile - 1;
			int top       = tile_row * ctu_rows_per_tile;
			int bottom    = top + ctu_rows_per_tile - 1;
			/* Extend last column/row to cover any remainder from integer division. */
			if (tile_col == num_tile_cols - 1) right  = ctu_cols - 1;
			if (tile_row == num_tile_rows - 1) bottom = ctu_rows - 1;
			encoder->cached_coding_ctrl.intraArea.enable = 1;
			encoder->cached_coding_ctrl.intraArea.left   = left;
			encoder->cached_coding_ctrl.intraArea.right  = right;
			encoder->cached_coding_ctrl.intraArea.top    = top;
			encoder->cached_coding_ctrl.intraArea.bottom = bottom;
			VCEncSetCodingCtrl(encoder->encoder, &(encoder->cached_coding_ctrl));
		}
		else if (encoder->cached_coding_ctrl.intraArea.enable)
		{
			encoder->cached_coding_ctrl.intraArea.enable = 0;
			VCEncSetCodingCtrl(encoder->encoder, &(encoder->cached_coding_ctrl));
		}
	}

	if ((encoder->open_params.compression_format == IMX_VPU_API_COMPRESSION_FORMAT_H265) &&
	    !(getenv("VR_RECOVERY_SEI") && getenv("VR_RECOVERY_SEI")[0] == '0'))
	{
		if (sei_refresh)
		{
			static const uint8_t kRecoveryUuid[16] =
				{ 'V','R','-','S','L','I','-','R','E','C','O','V','R','Y','0','1' };
			uint8_t sei[20];
			memcpy(sei, kRecoveryUuid, 16);
			sei[16] = 1;
			sei[17] = (uint8_t)sei_top;
			sei[18] = (uint8_t)(sei_bottom - sei_top + 1);
			sei[19] = (uint8_t)(encoder->num_encoded_pictures & 0xFF);
			VCEncSetSeiUserData(encoder->encoder, sei, sizeof(sei));
		}
		else
		{
			VCEncSetSeiUserData(encoder->encoder, NULL, 0);
		}
	}

	/* Perform the actual frame encoding. */
	memset(&encoder_output, 0, sizeof(encoder_output));
	enc_ret = VCEncStrmEncode(encoder->encoder, encoder_input, &encoder_output, NULL, NULL);
	if (enc_ret != VCENC_FRAME_READY)
	{
		IMX_VPU_API_WARNING(
			"VCEncStrmEncode returned %s (%d); attempting auto-recovery via encoder restart",
			vcenc_retval_to_string(enc_ret), (int)enc_ret
		);

		encoder->skipped_frame_context = encoder->staged_raw_frame.context;
		encoder->skipped_frame_pts = encoder->staged_raw_frame.pts;
		encoder->skipped_frame_dts = encoder->staged_raw_frame.dts;
		encoder->skipped_frame_available = TRUE;

		if (restart_encoder(encoder) != IMX_VPU_API_ENC_RETURN_CODE_OK)
		{
			IMX_VPU_API_ERROR("encoder auto-recovery failed; declaring fatal");
			goto error;
		}

		*output_code = IMX_VPU_API_ENC_OUTPUT_CODE_FRAME_SKIPPED;
		ret = IMX_VPU_API_ENC_RETURN_CODE_OK;
		goto finish;
	}

	encoder->num_bytes_in_stream_buffer = encoder_output.streamSize;
	*encoded_frame_size += encoder_output.streamSize;

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

	encoder->num_encoded_pictures++;

	*output_code = IMX_VPU_API_ENC_OUTPUT_CODE_ENCODED_FRAME_AVAILABLE;


finish:
	if (encoder->staged_raw_frame_set)
	{
		//imx_dma_buffer_unmap(encoder->staged_raw_frame.fb_dma_buffer);
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
