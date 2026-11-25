/* Accident Memory, step 1: a pass-through effect.
 * It does nothing to the sound. Its only job is to prove the whole pipeline works:
 * build -> .knl file -> workshop -> OP-1 -> shows up in the effects list.
 * Knobs are declared but unused for now (blue, green, white, orange).
 * Copyright (C) 2026 <you>. GPL-3.0-or-later with the OP-1 firmware permission (see LICENSE and NOTICE). */
#include "op1_plugin.h"

static void am_init(int pos)
{
    (void)pos;                       /* nothing to set up yet */
}

static void am_render(int pos, int16_t *buf, const int16_t *p)
{
    (void)pos; (void)buf; (void)p;   /* audio passes through untouched */
}

const op1_fx_t FX_accident_memory = {
    am_init, am_render,
    { OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE }
};
