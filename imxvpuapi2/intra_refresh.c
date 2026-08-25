/* Intra refresh scheduling. See the header for the rationale.
 *
 * Author: Andrii Voznytsia <andrew@auterion.com>
 */
#include <string.h>

#include "intra_refresh.h"


/* Default band height in picture lines, so that the default is the same slice
 * of the picture on both codecs: two 64 pixel CTB rows on h.265, eight 16
 * pixel macroblock rows on h.264. */
#define DEFAULT_BAND_PIXELS 128


/* The CTB rows (or columns) that slice number "step" covers, when "total" of
 * them are split into "num_steps" contiguous slices of as equal a size as they
 * divide into.
 *
 * The slices do not overlap and together they cover the picture exactly, which
 * is the whole point of computing them here instead of letting the encoder do
 * it. The encoder's own sweep (hevcencapi.c:6357-6371) makes the band one CTB
 * row taller than the distance it advances, because bottom_pos is inclusive
 * and it adds a full gdrAverageMBRows to it. Every row is therefore coded
 * intra twice per sweep, and at 720p, where gdrAverageMBRows rounds down to
 * zero and the band is two rows advancing by one, that is 22 row refreshes to
 * cover 12 rows. Measured cost of the duplication: 0.67-0.70 dB of PSNR at a
 * fixed bitrate. */
static void split_range(int total, int num_steps, int step, int *first, int *last)
{
	int const base = total / num_steps;
	int const rem = total % num_steps;
	int const size = base + ((step < rem) ? 1 : 0);

	*first = step * base + ((step < rem) ? step : rem);
	*last = *first + size - 1;
}


/* Which region this picture belongs to.
 *
 * Region i is placed at picture i * duration / num_steps, so this is that
 * mapping inverted: the largest i whose placement is at or before the picture.
 * Placing the regions rather than deriving them from a running quotient is
 * what makes the last one land on a picture the recovery point can name
 * exactly, and it is also the placement the deprecated rolling tiles mode
 * used, so that alias keeps its schedule to the picture.
 *
 * Past the end of the sweep the index saturates, so a duration shorter than
 * the period idles for the remainder instead of wrapping early. The encoder's
 * own GDR has that shape and cannot have any other: its sweep advances exactly
 * once per coded picture and cannot be slowed, so gdr_refresh_period only
 * decided how often a sweep restarted, never how long it took. Spreading the
 * same refreshes over the whole period instead is worth 30-60 ms of p99
 * queueing delay, which is why duration defaults to period. */
static int sweep_step_index(ImxVpuApiIntraRefreshState const *state,
                            ImxVpuApiIntraRefreshCfg const *cfg)
{
	int const step = ((state->sweep_pic + 1) * state->num_steps - 1) / cfg->duration;

	return (step > (state->num_steps - 1)) ? (state->num_steps - 1) : step;
}


void imx_vpu_api_intra_refresh_resolve(ImxVpuApiIntraRefreshCfg *cfg,
                                       int *num_steps, int *row_steps, int *col_steps)
{
	int rows, cols;

	if (cfg->ctb_rows < 1)
		cfg->ctb_rows = 1;
	if (cfg->ctb_cols < 1)
		cfg->ctb_cols = 1;

	if (cfg->period < 1)
		cfg->period = 1;
	/* A duration longer than the period would start the next sweep before
	 * this one finished. Clamping it down is safe; clamping the period up
	 * to meet it would silently refresh less often than asked. */
	if ((cfg->duration < 1) || (cfg->duration > cfg->period))
		cfg->duration = cfg->period;

	if (cfg->unit_pixels < 1)
		cfg->unit_pixels = 64;

	if (cfg->rows < 1)
		cfg->rows = DEFAULT_BAND_PIXELS / cfg->unit_pixels;
	if (cfg->rows < 1)
		cfg->rows = 1;
	if (cfg->rows > cfg->ctb_rows)
		cfg->rows = cfg->ctb_rows;

	if (cfg->columns < 1)
		cfg->columns = cfg->ctb_cols;
	if (cfg->columns > cfg->ctb_cols)
		cfg->columns = cfg->ctb_cols;

	cols = (cfg->ctb_cols + cfg->columns - 1) / cfg->columns;
	rows = (cfg->ctb_rows + cfg->rows - 1) / cfg->rows;

	/* A sweep cannot have more regions than it has pictures to place them
	 * on. The column split is the one that carries meaning when it is set
	 * at all - it is the region width somebody asked for - so the row split
	 * is what gives, and the bands come out taller than requested. */
	if (cols > cfg->duration)
		cols = cfg->duration;
	if (cols < 1)
		cols = 1;
	if ((rows * cols) > cfg->duration)
		rows = cfg->duration / cols;
	if (rows < 1)
		rows = 1;

	if (row_steps != NULL)
		*row_steps = rows;
	if (col_steps != NULL)
		*col_steps = cols;
	if (num_steps != NULL)
		*num_steps = rows * cols;
}


void imx_vpu_api_intra_refresh_init(ImxVpuApiIntraRefreshState *state,
                                    ImxVpuApiIntraRefreshCfg const *cfg)
{
	ImxVpuApiIntraRefreshCfg resolved = *cfg;

	memset(state, 0, sizeof(*state));
	imx_vpu_api_intra_refresh_resolve(&resolved, &(state->num_steps),
	                                  &(state->row_steps), &(state->col_steps));
	state->sweep_pic = 0;
	state->step = -1;
}


void imx_vpu_api_intra_refresh_step(ImxVpuApiIntraRefreshState *state,
                                    ImxVpuApiIntraRefreshCfg const *cfg,
                                    int forced_first, int forced_num,
                                    ImxVpuApiIntraRefreshBand *band)
{
	int const step = sweep_step_index(state, cfg);
	int const sweep_due = (step > state->step);

	memset(band, 0, sizeof(*band));
	band->left = 0;
	band->right = cfg->ctb_cols - 1;

	/* A requested region wins only on a picture the sweep was not going to
	 * use anyway. There is one intraArea, so the two cannot both be served,
	 * and letting requests win unconditionally would let a burst of them
	 * starve the sweep - which is the one thing that guarantees every row
	 * gets refreshed at all. Deferring a request by a picture costs nothing
	 * next to that; deferring the sweep can cost coverage.
	 *
	 * The sweep is deliberately not advanced past a region it did not code:
	 * sweep_pic keeps counting, so a region that loses its picture to a
	 * request simply fires on the next one. Skipping it instead would leave
	 * those rows stale for the rest of the sweep, and re-anchoring the
	 * sweep onto the requested band would do the same to everything
	 * between. */
	if ((forced_num > 0) && !sweep_due)
	{
		int top = forced_first;
		int bottom = forced_first + forced_num - 1;

		if (top < 0)
			top = 0;
		if (bottom > (cfg->ctb_rows - 1))
			bottom = cfg->ctb_rows - 1;

		if (bottom >= top)
		{
			band->apply = 1;
			band->forced = 1;
			band->top = top;
			band->bottom = bottom;
		}
	}
	else if (sweep_due)
	{
		/* Descend a row of regions left to right before moving down, so
		 * that with one region per row - everything but the deprecated
		 * tiles alias - the sweep is a plain top to bottom band sweep. */
		int const row_step = step / state->col_steps;
		int const col_step = step % state->col_steps;

		split_range(cfg->ctb_rows, state->row_steps, row_step,
		            &(band->top), &(band->bottom));
		split_range(cfg->ctb_cols, state->col_steps, col_step,
		            &(band->left), &(band->right));

		if (band->bottom > (cfg->ctb_rows - 1))
			band->bottom = cfg->ctb_rows - 1;
		if (band->right > (cfg->ctb_cols - 1))
			band->right = cfg->ctb_cols - 1;

		state->step = step;
		band->apply = 1;

		if (step == 0)
		{
			/* The picture the last region of this sweep lands on,
			 * which is when the picture is completely refreshed. */
			band->recovery_count =
				((state->num_steps - 1) * cfg->duration) / state->num_steps;
		}
	}

	state->sweep_pic++;
	if (state->sweep_pic >= cfg->period)
	{
		state->sweep_pic = 0;
		state->step = -1;
	}
}


void imx_vpu_api_intra_refresh_plan(ImxVpuApiIntraRefreshRequest const *req,
                                    ImxVpuApiIntraRefreshCfg *cfg,
                                    ImxVpuApiIntraRefreshPlan *plan)
{
	int num_rolling_slices = req->num_rolling_slices;
	int num_rolling_tiles = req->num_rolling_tiles;
	int period, rows, columns, slice_height, slice_count;

	memset(cfg, 0, sizeof(*cfg));
	memset(plan, 0, sizeof(*plan));

	/* Filled in even when refresh is off, because the picture geometry is
	 * the caller's too - it is what sliceSize and any region are counted
	 * in - and a zero here would make a caller divide by it. */
	cfg->ctb_rows = req->ctb_rows;
	cfg->ctb_cols = req->ctb_cols;
	cfg->unit_pixels = (req->unit_pixels > 0) ? req->unit_pixels : 64;

	/* 1 means "pick something sensible" in both deprecated counts. */
	if (num_rolling_slices == 1)
		num_rolling_slices = 4;
	if (num_rolling_tiles == 1)
		num_rolling_tiles = 4;

	plan->active = req->enable || (num_rolling_slices > 0) || (num_rolling_tiles > 0);
	plan->slice_count = 1;
	if (!plan->active)
		return;

	period = req->period;
	if (period == 0)
		period = req->gdr_refresh_period;
	/* The rolling modes took their period from roll_size, and clamped it to
	 * gop_size because the wave was front loaded into each GOP. */
	if ((period == 0) && ((num_rolling_slices > 0) || (num_rolling_tiles > 0)))
	{
		period = req->roll_size;
		if ((period == 0) || (period > req->gop_size))
			period = req->gop_size;
	}
	if (period == 0)
		period = req->gop_size;

	rows = req->rows;
	columns = 0;
	slice_height = req->slice_height;
	slice_count = req->slice_count;

	if (num_rolling_slices > 0)
	{
		/* One band per slice, and the band was the slice. */
		if (rows == 0)
			rows = (req->ctb_rows + num_rolling_slices - 1) / num_rolling_slices;
		if ((slice_height == 0) && (slice_count == 0))
			slice_count = num_rolling_slices;
	}
	else if (num_rolling_tiles > 0)
	{
		/* Two fixed columns, ceil(N/2) rows, one slice per tile row. */
		int const tile_rows = (num_rolling_tiles + 1) / 2;

		if (rows == 0)
			rows = (req->ctb_rows + tile_rows - 1) / tile_rows;
		if (columns == 0)
			columns = req->ctb_cols / 2;
		if ((slice_height == 0) && (slice_count == 0))
			slice_count = tile_rows;
	}

	cfg->period = period;
	cfg->duration = req->duration;
	cfg->rows = rows;
	cfg->columns = columns;

	imx_vpu_api_intra_refresh_resolve(cfg, &(plan->num_steps), &(plan->row_steps),
	                                  &(plan->col_steps));
	plan->wanted_steps = ((cfg->ctb_rows + cfg->rows - 1) / cfg->rows) * plan->col_steps;

	if (slice_height > 0)
	{
		plan->slice_size = slice_height;
		plan->slice_count = (cfg->ctb_rows + slice_height - 1) / slice_height;
	}
	else
	{
		plan->slice_size = imx_vpu_api_slice_height_for_count(cfg->ctb_rows, slice_count,
		                                                      &(plan->slice_count));
	}
}


int imx_vpu_api_slice_height_for_count(int ctb_rows, int slice_count,
                                       int *achieved_count)
{
	int height, count;

	if (ctb_rows < 1)
		ctb_rows = 1;
	if (slice_count < 1)
		slice_count = 1;
	if (slice_count > ctb_rows)
		slice_count = ctb_rows;

	height = (ctb_rows + slice_count - 1) / slice_count;
	count = (ctb_rows + height - 1) / height;

	if (achieved_count != NULL)
		*achieved_count = count;

	/* sliceSize 0 is how the vendor API spells "one slice per picture". */
	return (count <= 1) ? 0 : height;
}
