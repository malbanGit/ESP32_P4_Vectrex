/*
 * pokey.h: POKEY chip simulation functions
 *
 * Copyright 1991, 1992, 1993, Hedley Rainnie and Eric Smith
 * Copyright 1996 Eric Smith
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * $Id: pokey.h 2 2003-08-20 01:51:05Z eric $
 */

#include <stdint.h>
#include "memory.h"

#define MAX_POKEY 4  /* POKEYs are numbered 0 .. MAX_POKEY - 1 */

void pokey_init (void);

/* Set how many POKEY chips are active for this game (1..MAX_POKEY).
 * Affects both pokey_read/write routing and audio mix level.
 * Call after pokey_init(), before emulation starts. Default is 1. */
void pokey_set_count (int n);
byte pokey_read (int pokeynum, int reg, int PC, unsigned long cyc);
void pokey_write (int pokeynum, int reg, byte val, int PC, unsigned long cyc);

/* Mix all active POKEY chips into buf as stereo interleaved 16-bit PCM
 * (L, R, L, R, ...). buf must hold n_samples * 2 int16_t values.
 * clocks_per_sample = 1789790 / sample_rate (e.g. 40 for 44100 Hz). */
void pokey_generate_samples (int16_t *buf, int n_samples, int clocks_per_sample);

