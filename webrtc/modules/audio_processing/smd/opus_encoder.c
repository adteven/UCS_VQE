/* Copyright (c) 2010-2011 Xiph.Org Foundation, Skype Limited
   Written by Jean-Marc Valin and Koen Vos */
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdarg.h>
#include "celt.h"
#include "entenc.h"
#include "modes.h"
//#include "API.h"
#include "stack_alloc.h"
//#include "float_cast.h"
#include "opus.h"
#include "arch.h"
#include "opus_private.h"
#include "os_support.h"
//#include "cpu_support.h"
#include "analysis.h"
#include "mathops.h"
//#include "tuning_parameters.h"
#ifdef FIXED_POINT
#include "fixed/structs_FIX.h"
#else
//#include "float/structs_FLP.h"
#endif

#define MAX_ENCODER_BUFFER 480

typedef struct {
   opus_val32 XX, XY, YY;
   opus_val16 smoothed_width;
   opus_val16 max_follower;
} StereoWidthState;

void downmix_int(const void *_x, opus_val32 *sub, int subframe, int offset, int c1, int c2, int C)
{
	const opus_int16 *x;
	opus_val32 scale;
	int j;
	x = (const opus_int16 *)_x;
	for (j = 0; j<subframe; j++)
		sub[j] = x[(j + offset)*C + c1];
	if (c2>-1)
	{
		for (j = 0; j<subframe; j++)
			sub[j] += x[(j + offset)*C + c2];
	}
	else if (c2 == -2)
	{
		int c;
		for (c = 1; c<C; c++)
		{
			for (j = 0; j<subframe; j++)
				sub[j] += x[(j + offset)*C + c];
		}
	}
#ifdef FIXED_POINT
	scale = (1 << SIG_SHIFT);
#else
	scale = 1.f / 32768;
#endif
	if (C == -2)
		scale /= C;
	else
		scale /= 2;
	for (j = 0; j<subframe; j++)
		sub[j] *= scale;
}
