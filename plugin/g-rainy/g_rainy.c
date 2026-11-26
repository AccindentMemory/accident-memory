/* g-rainy, step 0.2.0: granular + plate reverb FX for the OP-1.
 *
 * The input is recorded continuously into a circular buffer (in the fast
 * memory lease). A small pool of grains reads back from that buffer:
 * each grain is a short slice (10..300 ms) with its own pitch, a smooth
 * sin^2 envelope that always closes to zero, and a random start position
 * that is always inside audio that has really been recorded. The grains
 * are summed to a mono cloud and fed into the Hundekraft plate reverb
 * (the plate already installed on the OP-1), which supplies the width and
 * the wash. The dry signal stays stereo throughout.
 *
 * Knobs (blue, green, white, orange): Mix, Reverb, Grain, Pitch.
 *   Blue   Mix: dry vs processed (grains + reverb).
 *   Green  Reverb: reverb amount inside the processed signal. 0 = dry
 *          grains only, max = fully washed in plate reverb.
 *   White  Grain: grain size, ~10 ms (glitchy, textural) to ~300 ms
 *          (smooth, pad-like). Density follows: grains overlap ~2x, with
 *          a little random timing so the cloud never ticks like a clock.
 *   Orange Pitch: random pitch spread on the grains. 0 = original pitch,
 *          max = up to +/-1 octave, chromatic.
 * Presses (all LATCH): blue = Freeze (stop recording, grains keep
 * reading the frozen buffer), green = Shimmer (grains an octave up),
 * white = Reverse (grains play backward), orange = Clear (erase the
 * buffer). Page 2 is not used.
 *
 * Fixes in 0.2.0 over 0.1.0 (found by a host test of the 0.1.0 source):
 *  - A grain position was kept as 16.16 fixed point in 32 bits, which can
 *    address only 65,536 samples while the buffer holds 110,000: about a
 *    third of the grains started in the wrong place, and some jumped
 *    mid-grain (a click). Now: integer index + 16-bit fraction.
 *  - The envelope step was an integer 4096/len, which is 1 for every grain
 *    longer than ~93 ms, so those grains ended at 40-70% amplitude (a click
 *    at every grain end). Now: a 16.16 envelope phase that reaches exactly
 *    the end of the sine half-turn on the last sample.
 *  - Shimmer and Reverse could read audio that was not recorded yet (stale
 *    memory, a noise burst after loading the effect) or run past the write
 *    head. Now the grain's whole path is placed inside the recorded window.
 *  - A modulo per grain per sample (no hardware divide on this chip) and 64
 *    bit multiplies are gone from the per-sample loop.
 *
 * Memory: the plate (166 KB) lives in the standard FX scratch. The grain
 * buffer (~2.5 s) lives in the fast memory lease (op1_fx_fast, 224 KiB),
 * falling back to the big lease (op1_fx_big) when the fast lease is not
 * available. A file that imports the fast lease is refused by older cores;
 * this plugin needs a current core. The grain buffer contents are undefined
 * after an FX switch, so the valid length is tracked and grains only read
 * what has been recorded since the FX was loaded.
 *
 * CPU: 8 simultaneous grains plus the plate. If the OP-1 meter runs hot,
 * reduce GP_GRAINS.
 *
 * All math is fixed-point integers; no float, and no division in the
 * per-sample loop. The plate DSP (plate_dsp.c) is Hundekraft's, reused with
 * attribution (GPL-3.0-or-later).
 * Copyright (C) 2026 Accident Memory. GPL-3.0-or-later with the OP-1
 * firmware permission (see LICENSE and NOTICE). */
#include "op1_plugin.h"
#include "plate_dsp.h"

#define GP_BUF_LEN   110000        /* grain buffer, samples (~2.5 s) */
#define GP_GRAINS    8             /* simultaneous grains */
#define GP_GRAIN_MIN 441           /* min grain size, samples (~10 ms) */
#define GP_GRAIN_MAX 13230         /* max grain size, samples (~300 ms) */
#define GP_SPAWN_WIN 44100         /* grains start within the last ~1 s */
#define GP_ENV_TOP   (4096u << 16) /* envelope phase: half a sine turn, 16.16 */

/* Semitone ratios, Q16 (65536 = 1.0x), for -12..+12 semitones. */
static const int32_t gp_semi[25] = {
    32768, 34716, 36781, 38968, 41285, 43741, 46341, 49097, 52015, 55109,
    58386, 61858, 65536, 69433, 73562, 77936, 82570, 87483, 92682, 98193,
    104031, 110218, 116773, 123715, 131072
};

typedef struct {
    uint8_t active;
    uint8_t rev;        /* 1 = this grain plays backward */
    int32_t idx;        /* read position: whole samples into gbuf */
    uint32_t frac;      /* read position: fraction, 0..65535 */
    uint32_t step;      /* playback rate, Q16 (65536 = 1.0x), always positive */
    uint32_t len;       /* grain length, samples */
    uint32_t age;       /* samples played so far */
    uint32_t env;       /* envelope phase, 16.16, 0 .. GP_ENV_TOP over the grain */
    uint32_t env_inc;   /* envelope increment per sample */
    int32_t gain;       /* Q15 */
} gp_grain;

typedef struct {
    int16_t *gbuf;              /* grain buffer (fast lease) */
    uint32_t gbuf_len;          /* length in samples */
    uint32_t whead;             /* write head: the next sample to write */
    uint32_t wlen;              /* valid recorded length (0 after clear/switch) */
    gp_grain grains[GP_GRAINS];
    uint32_t spawn;             /* countdown to next grain spawn, samples */
    uint32_t chaos;             /* xorshift32 state */
    plate_state *plate;         /* plate reverb state (in scratch) */
    int32_t mix, rev, grain_s, pitch_s;   /* smoothed gains, Q15 */
    uint8_t prev_clear;
    uint8_t freeze, reverse, shim;
} gp_state;

static gp_state st[3];

/* xorshift32: cheap deterministic pseudo-random. */
static uint32_t gp_rand(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}

static void gp_init(int pos)
{
    gp_state *s = &st[pos % 3];
    uint32_t size = 0;
    int16_t *buf = (int16_t *)op1_fx_fast(pos % 3, &size);
    if (!buf || size < (uint32_t)(GP_BUF_LEN * 2 + 64)) {
        size = 0;
        buf = (int16_t *)op1_fx_big(pos % 3, &size);
    }
    s->gbuf = buf;
    s->gbuf_len = (buf && size >= 128) ? (size / 2) : 0;
    if (s->gbuf_len > GP_BUF_LEN) s->gbuf_len = GP_BUF_LEN;
    s->whead = 0; s->wlen = 0;
    for (int i = 0; i < GP_GRAINS; i++) s->grains[i].active = 0;
    s->spawn = 0;
    s->chaos = 0x12345678u ^ (0x9E3779B9u * (uint32_t)(pos + 1));
    /* The plate state (ring + fields) lives in the standard scratch. */
    s->plate = (plate_state *)op1_fx_scratch(pos % 3);
    plate_init(s->plate);
    s->mix = 0; s->rev = 0; s->grain_s = 0; s->pitch_s = 0;
    s->prev_clear = 0;
    s->freeze = 0; s->reverse = 0; s->shim = 0;
}

/* Spawn one grain, if a slot is free and enough audio has been recorded. */
static void gp_spawn(gp_state *s, int32_t grain_size, int32_t pitch_spread)
{
    if (s->wlen < 1024 || !s->gbuf || s->gbuf_len == 0) return;
    int slot = -1;
    for (int i = 0; i < GP_GRAINS; i++) {
        if (!s->grains[i].active) { slot = i; break; }
    }
    if (slot < 0) return;   /* all grains busy: skip this spawn */
    gp_grain *g = &s->grains[slot];

    /* Grain length from the knob (10..300 ms). */
    uint32_t len = (uint32_t)GP_GRAIN_MIN +
        (uint32_t)(((GP_GRAIN_MAX - GP_GRAIN_MIN) * grain_size) >> 15);

    /* Pitch: random chromatic offset within the spread, plus the shimmer octave. */
    int32_t semi = 0;
    int32_t spread = (pitch_spread * 12) >> 15;   /* 0..12 semitones */
    if (spread > 0)
        semi = (int32_t)((((gp_rand(&s->chaos) >> 16) * (uint32_t)(2 * spread + 1)) >> 16)) - spread;
    if (s->shim) semi += 12;
    if (semi > 12) semi = 12;
    if (semi < -12) semi = -12;
    const uint32_t rate = (uint32_t)gp_semi[semi + 12];   /* Q16, 0.5x .. 2x */

    /* The grain walks over `span` samples of the buffer (up to 2x its length). Place that whole path inside the
     * audio that is really recorded and behind the write head: it can then never read stale memory or cross the
     * write head, forward or backward, at any pitch. */
    uint32_t win = s->wlen < GP_SPAWN_WIN ? s->wlen : GP_SPAWN_WIN;
    uint32_t span = (len * rate) >> 16;
    if (span + 2 > win) {                      /* not enough recorded yet: make the grain shorter */
        len = ((win - 2) << 16) / rate;
        span = (len * rate) >> 16;
    }
    if (len < 64) return;
    const uint32_t room = win - span - 1;      /* >= 1 */
    const uint32_t back = 1 + (((gp_rand(&s->chaos) >> 16) * room) >> 16);   /* 1 .. room, no division */

    int32_t start;
    if (s->reverse) start = (int32_t)s->whead - (int32_t)back;               /* run from here downward */
    else            start = (int32_t)s->whead - (int32_t)(back + span);      /* run from here upward */
    if (start < 0) start += (int32_t)s->gbuf_len;

    g->idx = start;
    g->frac = 0;
    g->step = rate;
    g->rev = s->reverse ? 1 : 0;
    g->len = len;
    g->age = 0;
    g->env = 0;
    g->env_inc = GP_ENV_TOP / len;             /* once per grain; the last sample lands on the end of the half-turn */
    g->gain = 23170;                           /* ~0.707: keeps the summed cloud from clipping */
    g->active = 1;
}

/* Shared process function: r == 0 is the mono render for the synth/drum
 * slots, r != 0 is the stereo render for the master slot. The grain cloud
 * is mono in both cases; the plate supplies the stereo width. The dry stays
 * stereo per channel. */
static void gp_process(int pos, int16_t *l, int16_t *r, const int16_t *p)
{
    gp_state *s = &st[pos % 3];
    if (!s->gbuf || s->gbuf_len == 0 || !s->plate) return;  /* no lease: pass dry */
    const int32_t mix_t   = op1_fx_param_q15(p, 0);
    const int32_t rev_t   = op1_fx_param_q15(p, 1);
    const int32_t grain_t = op1_fx_param_q15(p, 2);
    const int32_t pitch_t = op1_fx_param_q15(p, 3);
    /* Latches: blue = Freeze, green = Shimmer, white = Reverse,
     * orange = Clear (rising edge). */
    s->freeze  = op1_fx_page[pos % 3][0];
    s->shim    = op1_fx_page[pos % 3][1];
    s->reverse = op1_fx_page[pos % 3][2];
    uint8_t clear = op1_fx_page[pos % 3][3];
    if (clear && !s->prev_clear) {
        s->whead = 0; s->wlen = 0;
        for (int i = 0; i < GP_GRAINS; i++) s->grains[i].active = 0;
    }
    s->prev_clear = clear;

    int16_t grain_mix[OP1_FX_BLOCK];
    int16_t pl[OP1_FX_BLOCK], pr[OP1_FX_BLOCK];
    for (int n = 0; n < OP1_FX_BLOCK; n++) {
        s->mix     += (mix_t   - s->mix)     >> 6;
        s->rev     += (rev_t   - s->rev)     >> 6;
        s->grain_s += (grain_t - s->grain_s) >> 6;
        s->pitch_s += (pitch_t - s->pitch_s) >> 6;
        int32_t xL = l[n];
        int32_t xR = r ? r[n] : xL;
        /* Record the mono mix of the input, unless frozen. */
        if (!s->freeze) {
            int32_t in = (xL + xR) >> 1;
            s->gbuf[s->whead] = (int16_t)op1_sat16(in);
            s->whead++;
            if (s->whead >= s->gbuf_len) s->whead = 0;
            if (s->wlen < s->gbuf_len) s->wlen++;
        }
        /* Spawn a grain when the countdown expires. Interval = about half
         * the grain length (grains overlap ~2x), +/-25% at random so the
         * cloud is organic rather than a steady pulse. */
        if (s->spawn == 0) {
            gp_spawn(s, s->grain_s, s->pitch_s);
            uint32_t glen = (uint32_t)GP_GRAIN_MIN +
                (uint32_t)(((GP_GRAIN_MAX - GP_GRAIN_MIN) * s->grain_s) >> 15);
            uint32_t base = glen >> 1;
            s->spawn = base - (base >> 2) + (((gp_rand(&s->chaos) >> 16) * (base >> 1)) >> 16);
            if (s->spawn < 220) s->spawn = 220;   /* ~5 ms minimum gap */
        } else {
            s->spawn--;
        }
        /* Render the active grains into the mono cloud. */
        int32_t cloud = 0;
        for (int i = 0; i < GP_GRAINS; i++) {
            gp_grain *g = &s->grains[i];
            if (!g->active) continue;
            /* Linear-interpolated read (17-bit difference x 15-bit fraction fits 32 bits). */
            int32_t i0 = g->idx;
            int32_t i1 = i0 + 1;
            if (i1 >= (int32_t)s->gbuf_len) i1 = 0;
            int32_t a = s->gbuf[i0];
            int32_t b = s->gbuf[i1];
            int32_t v = a + (((b - a) * (int32_t)(g->frac >> 1)) >> 15);
            /* Hann envelope: sin^2 over the half-turn, ending exactly at zero. */
            int32_t e = OP1_SINE_TABLE[(g->env >> 16) & 8191];
            int32_t w = (e * e) >> 15;
            cloud += (((v * w) >> 15) * g->gain) >> 15;
            /* Advance: whole samples plus fraction, wrapping by one subtract/add (steps are at most 3). */
            if (!g->rev) {
                uint32_t acc = g->frac + g->step;
                g->frac = acc & 0xFFFF;
                i0 += (int32_t)(acc >> 16);
                if (i0 >= (int32_t)s->gbuf_len) i0 -= (int32_t)s->gbuf_len;
            } else {
                int32_t f = (int32_t)g->frac - (int32_t)(g->step & 0xFFFF);
                int32_t d = (int32_t)(g->step >> 16);
                if (f < 0) { f += 65536; d++; }
                g->frac = (uint32_t)f;
                i0 -= d;
                if (i0 < 0) i0 += (int32_t)s->gbuf_len;
            }
            g->idx = i0;
            g->env += g->env_inc;
            g->age++;
            if (g->age >= g->len) g->active = 0;
        }
        grain_mix[n] = (int16_t)op1_sat16(cloud);
    }
    /* Plate reverb on the grain cloud. The same mono cloud feeds both
     * channels; the plate's different taps per channel supply the width.
     * q0 decay, q1 damping, q2 pre-delay are fixed (lush plate); q3 is
     * the dry/wet mix, driven by the green Reverb knob. */
    for (int n = 0; n < OP1_FX_BLOCK; n++) { pl[n] = grain_mix[n]; pr[n] = grain_mix[n]; }
    int32_t q[4];
    q[0] = 24000;   /* decay, long and lush */
    q[1] = 18000;   /* damping, slightly warm */
    q[2] = 0;       /* pre-delay */
    q[3] = s->rev;  /* dry/wet: the green Reverb knob */
    plate_process(s->plate, pl, pr, q);
    /* Mix the dry input with the processed (grains + plate) signal. */
    for (int n = 0; n < OP1_FX_BLOCK; n++) {
        int32_t wetL = pl[n], wetR = pr[n];
        int32_t dryL = l[n], dryR = r ? r[n] : dryL;
        int32_t m = s->mix;
        l[n] = (int16_t)op1_sat16(((dryL * (32768 - m)) >> 15) + ((wetL * m) >> 15));
        if (r) r[n] = (int16_t)op1_sat16(((dryR * (32768 - m)) >> 15) + ((wetR * m) >> 15));
    }
}

static void gp_render_mono(int pos, int16_t *buf, const int16_t *params)
{
    gp_process(pos, buf, 0, params);
}

static void gp_render_stereo(int pos, int16_t *left, int16_t *right, const int16_t *params)
{
    gp_process(pos, left, right, params);
}

/* press: blue = Freeze latch, green = Shimmer latch, white = Reverse
 * latch, orange = Clear latch. All LATCH: the reverb amount is a main knob
 * (green), not page 2. */
const op1_fx_stereo_t FX_g_rainy = { { gp_init, gp_render_mono, { OP1_PRESS_LATCH, OP1_PRESS_LATCH, OP1_PRESS_LATCH, OP1_PRESS_LATCH } }, gp_render_stereo };
