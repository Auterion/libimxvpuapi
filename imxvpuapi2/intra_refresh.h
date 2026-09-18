/* Intra refresh scheduling: which CTB rectangle to code intra in this picture.
 *
 * Author: Andrii Voznytsia <andrew@auterion.com>
 *
 * No encoder types, no vendor headers, no logging, no I/O - this takes a
 * picture index and returns a rectangle, so it can be replayed offline and so
 * that tools/rcprobe can compile it in and drive the vendor API with exactly
 * the schedule the library ships. Three separate refresh schedulers used to
 * live in the encoder backend and a fourth transcription of them in rcprobe;
 * they all computed bands the same way and drifted apart anyway.
 *
 * The schedule is one contiguous band sweeping top to bottom, and it has to
 * stay that way. The video-receiver tracks refresh progress by asserting that
 * each band starts where the previous one ended, and opens its render gate
 * when the accumulated bands cover the picture; a band out of that order makes
 * it fall back to the recovery point or to a timeout.
 */
#ifndef IMXVPUAPI2_INTRA_REFRESH_H
#define IMXVPUAPI2_INTRA_REFRESH_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* What the caller wants. Resolve this with imx_vpu_api_intra_refresh_resolve()
 * before using it, which fills in the defaults and clamps the combinations the
 * hardware cannot express. */
typedef struct
{
	/* Picture size in CTBs. */
	int ctb_rows;
	int ctb_cols;

	/* Pictures between two sweep starts. */
	int period;

	/* Pictures one sweep is spread over. Never more than period: a longer
	 * duration would start the next sweep before this one finished, and the
	 * band order the receiver relies on would break. 0 = period, which is
	 * the default and means a continuous, evenly spaced refresh with no
	 * idle tail. */
	int duration;

	/* Size in pixels of one row and column unit. This is the encoder's own
	 * coding unit, and it is codec dependent: 64 for h.265 and 16 for h.264
	 * (hevcencapi.c:8594). Everything the hardware is told about geometry -
	 * intraArea, sliceSize - is counted in these, so the same row count
	 * covers four times less picture on h.264. 0 = 64.
	 *
	 * It is here so that the default band height can be a fixed number of
	 * picture lines rather than a row count that silently means something
	 * different per codec. Getting this wrong does not fail: the encoder
	 * validates intraArea against its own ctbPerCol, and a band addressed
	 * in 64 pixel rows on an h.264 stream sits well inside 45 macroblock
	 * rows, so it is accepted and quietly refreshes the top of the picture
	 * and nothing else. */
	int unit_pixels;

	/* Target band height in rows of unit_pixels. It is a target, not an
	 * exact height:
	 * the sweep is split into ceil(ctb_rows / rows) bands of as equal a
	 * height as they divide into, so that coverage stays exact.
	 *
	 * 0 selects a band of 128 picture lines, which is a row count that
	 * depends on unit_pixels: 2 rows on h.265, 8 on h.264. The default is a
	 * pixel height rather than a row count so that it covers the same slice
	 * of the picture on either codec; the field itself is always a row
	 * count.
	 *
	 * A one row band measures worse than every coarser height tried, on
	 * both test clips and with either rate control - least intra prediction
	 * context to work with, and twice as many refresh pictures needed.
	 * Above two rows the differences are inside the run to run spread. */
	int rows;

	/* Region width in columns of unit_pixels. 0 = full width, which is what
	 * everything except the legacy rolling tiles mode uses; a narrower
	 * region makes the sweep two dimensional, descending a row of regions
	 * left to right before moving down.
	 *
	 * No property exposes this. A narrower band breaks the receiver's
	 * assumption that a refreshed band spans the picture, and the refresh
	 * SEI has nowhere to put a column - which is why the only caller is the
	 * deprecated rolling tiles alias, and why that alias emits no SEI. */
	int columns;
}
ImxVpuApiIntraRefreshCfg;


typedef struct
{
	/* Regions one sweep is split into, after resolving:
	 * row_steps * col_steps. */
	int num_steps;
	int row_steps;
	int col_steps;

	/* Pictures since this sweep started, in [0, period). */
	int sweep_pic;

	/* Index of the region that was refreshed last, -1 before the first one
	 * of a sweep. The next region fires when the schedule moves past this,
	 * which is what keeps the sweep monotone even when a picture is spent
	 * on something else. */
	int step;
}
ImxVpuApiIntraRefreshState;


typedef struct
{
	/* 0 = code this picture with no forced intra area at all. */
	int apply;

	/* The rectangle, in CTBs, inclusive on all four sides. */
	int top;
	int bottom;
	int left;
	int right;

	/* The band came from imx_vpu_api_enc_set_intra_refresh_region() rather
	 * than from the sweep. */
	int forced;

	/* Pictures from this one until the picture is fully covered, for a
	 * recovery point SEI. Only nonzero on a sweep start. Derived from the
	 * duration and not from the period: a decoder joining here waits this
	 * long, and telling it to wait out the idle tail as well would make it
	 * hold back frames it could already show. */
	int recovery_count;
}
ImxVpuApiIntraRefreshBand;


/* Fill in the defaults and clamp what cannot be expressed, and report the
 * resulting geometry. Region i of a sweep lands on picture i * duration /
 * num_steps, so the last one lands on (num_steps - 1) * duration / num_steps
 * and that is exactly what a recovery point has to count down.
 *
 * row_steps comes out smaller than ceil(ctb_rows / rows) when a sweep has
 * fewer pictures than it would need regions - the bands are then taller than
 * the caller asked for, so the caller should say so rather than let it pass
 * silently. */
void imx_vpu_api_intra_refresh_resolve(ImxVpuApiIntraRefreshCfg *cfg,
                                       int *num_steps, int *row_steps, int *col_steps);

/* Start (or restart, after a flush) at the beginning of a sweep. */
void imx_vpu_api_intra_refresh_init(ImxVpuApiIntraRefreshState *state,
                                    ImxVpuApiIntraRefreshCfg const *cfg);

/* The rectangle for the next coded picture, and advance.
 *
 * forced_num > 0 asks for the band [forced_first, forced_first + forced_num)
 * to be refreshed in this picture instead of the one the sweep would have
 * picked. The sweep is re-anchored below it rather than interrupted, so the
 * bands the receiver sees stay in order.
 *
 * Call this once per coded picture, including the pictures it answers with
 * apply = 0, or the sweep loses track of where it is. */
void imx_vpu_api_intra_refresh_step(ImxVpuApiIntraRefreshState *state,
                                    ImxVpuApiIntraRefreshCfg const *cfg,
                                    int forced_first, int forced_num,
                                    ImxVpuApiIntraRefreshBand *band);

/* An out-of-order refresh: the band covering the whole picture, for a caller
 * that has been asked for a keyframe while sweeping.
 *
 * Sweeping, the answer to "give me a picture a decoder can start from" is not
 * an IDR - that is the once-per-GOP bitrate spike the sweep exists to avoid,
 * and it resets the reference structure as well. It is this: one picture with
 * every coding unit intra. It costs about what an IDR costs, so the caller
 * has to have the buffer room for it, but it leaves the reference structure
 * and the picture order alone.
 *
 * Slicing is untouched - the region is expressed in coding units and spans the
 * picture, so however the picture is divided into slices, every slice of it
 * comes out intra.
 *
 * recovery_count is 0: the picture is complete by itself, so a decoder
 * starting here needs to wait for nothing.
 *
 * Pass the band to the encoder, then call
 * imx_vpu_api_intra_refresh_realign() so the sweep that follows is anchored
 * to it. */
void imx_vpu_api_intra_refresh_full(ImxVpuApiIntraRefreshCfg const *cfg,
                                    ImxVpuApiIntraRefreshBand *band);

/* Re-anchor the sweep onto an out-of-order full-picture refresh that has just
 * been coded.
 *
 * The picture refreshed everything, so the sweep in progress is complete: it
 * is marked as such and the period restarts here. The next sweep therefore
 * begins one period after the out-of-order picture rather than at whatever
 * point in the old schedule came next, which is what keeps the refresh cycles
 * aligned to it and stops a sweep from starting again immediately over a
 * picture that is already clean.
 *
 * Call once, on the picture the full refresh was coded into, and only after
 * the encoder accepted it - a picture that was skipped or discarded must
 * leave the schedule where it was. */
void imx_vpu_api_intra_refresh_realign(ImxVpuApiIntraRefreshState *state);

/* What a caller asked for, before any of it is mapped or clamped. Plain ints,
 * so that this header stays free of encoder and GObject types and the library,
 * rcprobe and rctest can all hand it the same request.
 *
 * The deprecated fields are here rather than translated by each caller,
 * because the three refresh modes they selected were never different
 * mechanisms - all of them forced intraArea over a band of CTB rows - so what
 * they meant is one mapping, and one mapping is what everything should use. */
typedef struct
{
	int ctb_rows;
	int ctb_cols;
	int unit_pixels;    /* 64 for h.265, 16 for h.264; 0 = 64 */
	int gop_size;

	int enable;         /* the unified switch */
	int period;
	int duration;
	int rows;
	int slice_height;
	int slice_count;

	int gdr_refresh_period;   /* deprecated -> period */
	int num_rolling_slices;   /* deprecated -> rows, slice_count */
	int num_rolling_tiles;    /* deprecated -> rows, columns, slice_count */
	int roll_size;            /* deprecated -> period, clamped to gop_size */
}
ImxVpuApiIntraRefreshRequest;


/* What a request resolved to, for the caller to program and to report. */
typedef struct
{
	/* 0 = no intra refresh. The step geometry below then means nothing, but
	 * slice_size and slice_count still do: slicing is independent of the
	 * sweep and is asked for on its own. */
	int active;
	int num_steps;
	int row_steps;
	int col_steps;
	/* Regions the requested band height would have needed. Larger than
	 * num_steps when the sweep had too few pictures to place them all, in
	 * which case the bands are taller than were asked for. */
	int wanted_steps;
	/* sliceSize to program, 0 = one slice per picture, and the slice count
	 * that height actually produces. */
	int slice_size;
	int slice_count;
}
ImxVpuApiIntraRefreshPlan;


/* Map a request onto a config and a plan. The config is resolved, so it can be
 * handed straight to imx_vpu_api_intra_refresh_init(). */
void imx_vpu_api_intra_refresh_plan(ImxVpuApiIntraRefreshRequest const *req,
                                    ImxVpuApiIntraRefreshCfg *cfg,
                                    ImxVpuApiIntraRefreshPlan *plan);

/* Slice height in CTB rows for a requested slice count, and the count that
 * height actually produces. Only the height is programmable - the hardware
 * derives the count as ceil(ctb_rows / height) - so most counts are not
 * achievable: at 720p, 12 CTB rows, only 1, 2, 3, 4, 6 and 12 are. Ask for 5
 * and the answer is 4. */
int imx_vpu_api_slice_height_for_count(int ctb_rows, int slice_count,
                                       int *achieved_count);


/* --- SEI payloads that go with the sweep ---------------------------------
 *
 * Both encoder backends emit these, and both are wire formats a receiver
 * parses, so they are built here rather than once per backend.
 */

/* Bytes imx_vpu_api_build_refresh_band_sei() writes, and the smallest buffer
 * it may be handed. */
#define IMX_VPU_API_REFRESH_BAND_SEI_SIZE 20

/* Bytes imx_vpu_api_build_recovery_point_sei() may write, worst case. */
#define IMX_VPU_API_RECOVERY_POINT_SEI_MAX 24

/* A recovery_point SEI in its own prefix NAL unit, to be sent immediately
 * ahead of the picture that begins a sweep. recovery_count is how many further
 * pictures the decoder has to take before the picture is fully refreshed:
 * recovery_frame_cnt for h.264, recovery_poc_cnt for h.265. Written as a
 * complete NAL, start code included, because neither vendor API will emit one
 * for a caller driven sweep. Returns the number of bytes written. */
size_t imx_vpu_api_build_recovery_point_sei(uint8_t *out, int recovery_count, int is_h264);

/* The 20 byte user-data-unregistered payload naming the rows this picture
 * refreshed - uuid, version, top row, height in rows, picture number. Only the
 * payload: both vendor APIs wrap it in the SEI NAL themselves. Returns the
 * number of bytes written. */
size_t imx_vpu_api_build_refresh_band_sei(uint8_t *out, int top, int height,
                                          int picture_number);


#ifdef __cplusplus
}
#endif

#endif /* IMXVPUAPI2_INTRA_REFRESH_H */
