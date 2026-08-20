/* Vendor-facing view of the VC8000E, kept in one place.
 *
 * The rate control needs a handful of numbers the encoder reports after each
 * picture, and nothing else from the VC8000E. Naming them here rather than in
 * the rate control itself keeps that module free of vendor headers, so it can
 * be compiled and exercised off-target, and gives a single file to point at
 * the real VeriSilicon sources when they are available.
 *
 * Author: Andrii Voznytsia <andrew@auterion.com>
 */
#ifndef VC8000E_SHIM_H
#define VC8000E_SHIM_H

#include <stdint.h>

#include "ext_rate_control.h"


/* VCENC_MAX_REF_FRAMES is defined by hevcencapi.h, so the conversion below
 * compiles only in translation units that actually talk to the encoder.
 *
 * The counts come from VCEncOut::cuStatis and are in 8x8 blocks, so they are
 * comparable across codecs and CTB sizes. The total is not reported by the
 * encoder and is derived from the frame size by the caller. rd_cost is
 * PBFrame4NRdCost, straight from a hardware register - it is the vendor's own
 * complexity signal, and their static-scene test leads with it. */
#ifdef VCENC_MAX_REF_FRAMES

static inline void vc8000e_fill_rc_stats(ExtRateControlStats *stats, VCEncOut const *out, uint32_t total_blocks)
{
	stats->total_blocks = total_blocks;
	stats->intra_blocks = out->cuStatis.intraCu8Num;
	stats->skip_blocks  = out->cuStatis.skipCu8Num;
	stats->rd_cost      = out->cuStatis.PBFrame4NRdCost;
}

#endif


#endif /* VC8000E_SHIM_H */
