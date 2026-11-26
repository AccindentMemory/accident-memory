/* Accident Memory, step 4.0: stereo master.
 * Built on the 3.6 base (the keeper version; the 3.7 Blur lowpass and Drift
 * tremolo were rejected as muddy and are NOT in this file). The master
 * slot now renders in true stereo (plugin.json "stereo": true), while
 * the synth and drum slots keep the unchanged mono render. The two
 * delay lines become the stereo pair, the way the Cosmos assigns its
 * L/R inputs to different delay groups: line 1 takes the left input,
 * line 2 takes the right input. Drift is now a true panorama - the left
 * wet is line 1 weighted by (1-bal) plus line 2 weighted by bal, the
 * right wet is the mirror, so the LFO swings the sound between the
 * speakers instead of just shifting a mono crossfade. The dry stays
 * stereo (left dry to left out, right dry to right out). Blur, Drive,
 * Freeze, Reverse and Erase all work across both channels exactly as
 * they did in mono: the cross-feedback still mixes the lines, Freeze
 * holds both, Reverse plays the frozen loop backward in stereo, Erase
 * clears both lines. The buffer is unchanged (110,012 bytes, standard
 * scratch, no lease), so it loads everywhere.
 *
 * The mono render and the stereo render share one process function
 * (the ente pattern): am_process(pos, l, r, p) with r == 0 for mono.
 * In mono the behaviour is bit-for-bit the 3.6 build.
 *
 * Everything else is the 3.6 build unchanged: the 3.1 CPU fix (16-bit
 * MACs, sum/difference cross-feedback), the knob slew, the cubic Drive,
 * the Freeze button (blue), the Drift crossfade with knob-set rate and
 * depth (3.5), the frozen-buffer Reverse with per-pass decay and wrap
 * crossfade (3.4), and the short Reverse-off mute with DC-blocker
 * reset (3.6).
 *
 * Note: a 4.1 build (Freeze follows the green knob instead of forcing
 * 1.0, green cap raised to 1.0) was built and tested but NOT used -
 * the owner chose to keep it as is. The pad comes from
 * Drift and Blur being on, and Freeze already keeps the rhythm when
 * they are off. Do not re-propose the 4.1 Freeze change.
 *
 * The sound: the input feeds two delay lines of different large prime
 * lengths (0.544 s and 0.703 s). Prime lengths never line up the same way
 * twice, so the pattern never exactly repeats. Two more mechanisms keep
 * the sound moving: an asynchronous LFO whose rate is wandered by a slow
 * chaotic signal (the Drift knob), which crossfades the wet signal
 * between the lines, and cross-feedback between the lines (the Blur
 * knob), so the lines smear into each other. The loop is soft-limited
 * like tape instead of hard-clipped, and feedback stops just below 1.0,
 * the way the Cosmos manual recommends.
 *
 * Knobs (blue, green, white, orange): Mix, Feedback (capped 0.99),
 * Drift, Blur. Presses: blue = Freeze latch, green = Drive page 2,
 * white = Reverse latch, orange = Erase latch.
 *
 * Delay lines live in the FX scratch buffer (176460 bytes per position):
 *   line 1: 23993 samples, line 2: 31013 samples
 *   total: 55006 samples = 110012 bytes - fits, with 66448 bytes to spare.
 *   Two instances (synth + master) total ~2.5 s, the short variation.
 * All math is fixed-point integers (Q15); no float, no division in the loop.
 * Copyright (C) 2026 Accident Memory. GPL-3.0-or-later with the OP-1 firmware
 * permission (see LICENSE and NOTICE). */
#include "op1_plugin.h"

/* --- delay lines (44.1 kHz, int16 samples) --- */
#define AM_L1_LEN    23993          /* 0.544 s, prime */
#define AM_L2_LEN    31013          /* 0.703 s, prime */
#define AM_TOTAL_LEN (AM_L1_LEN + AM_L2_LEN)
#if AM_TOTAL_LEN * 2 > OP1_FX_SCRATCH_BYTES
#error "delay lines do not fit in the FX scratch buffer"
#endif

/* Drift LFO rate: the white knob now sets BOTH the rate and the depth,
 * so it responds gradually and immediately. At the knob's maximum the
 * balance completes a full swing in ~2 s; at low settings it is slow
 * and shallow. The chaotic signal wanders the rate by +/-50%. */
#define AM_DRIFT_INC_MAX 48700

#define AM_FB_MAX     32440          /* 0.99 in Q15: "slightly less than 1.0" */
#define AM_DC_R       32604          /* DC blocker, ~0.995 */
#define AM_KNEE       28672          /* soft-limiter knee, 0.875 of full scale */
#define AM_CLEAR_CHUNK 2048          /* samples zeroed per block on erase */
#define AM_MUTE_LEN   256            /* samples the wet is muted on Reverse/Erase */
#define AM_REV_FADE   64             /* samples faded out/in around a reverse wrap */

typedef struct {
    int32_t w1, w2;                  /* write heads, sample indices */
    int32_t rr1, rr2;                /* reverse read heads */
    uint32_t ph1;                    /* drift LFO phase */
    uint32_t chaos;                  /* xorshift state for the slow chaos */
    int32_t wander;                  /* smoothed chaos, -32768..32767 */
    int16_t dc1, dc2;                /* DC blocker states (clamped to int16) */
    int16_t yp1, yp2;                /* previous line outputs (DC blocker) */
    int32_t fb, mix;                 /* smoothed gains, Q15 */
    int32_t drift_s;                 /* smoothed drift depth, 0..16383 */
    int32_t blur_s;                  /* smoothed blur, Q15 */
    int32_t drive_s;                 /* smoothed drive, Q15 */
    int32_t rev_gain1, rev_gain2;    /* reverse repeat gains, Q15 (decay per pass) */
    int32_t mute;                    /* wet-mute countdown, samples */
    uint8_t prev_erase;              /* previous erase latch state */
    uint8_t prev_rev;                /* previous reverse latch state */
    int32_t clear_idx;               /* erase progress, -1 = idle */
} am_state_t;
static am_state_t st[3];             /* one per FX position: 0 synth, 1 drum, 2 master */

/* Soft limiter (tape-like): linear up to the knee, then a quarter slope, then
 * a hard clamp at full scale. No gain change in the linear region. */
static int32_t am_softlimit(int32_t v)
{
    if (v > AM_KNEE) return op1_sat16(AM_KNEE + ((v - AM_KNEE) >> 2));
    if (v < -AM_KNEE) return op1_sat16(-AM_KNEE - ((-AM_KNEE - v) >> 2));
    return v;
}

static uint32_t am_xorshift(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}

/* Reverse read of one line. The buffer is frozen while reversing, so this
 * only reads. It fades out and back in over AM_REV_FADE samples around
 * the loop point (the head near 0 or len-1) so the jump from the oldest
 * sample back to the newest is inaudible, and it decays the gain by the
 * feedback amount each full pass so the reverse repeats fade. */
static int32_t am_rev_read(int16_t *d, int32_t len, int32_t *rr, int32_t *rev_gain, int32_t fb)
{
    int32_t y = d[*rr];
    int32_t dist = *rr < (len - 1 - *rr) ? *rr : (len - 1 - *rr);
    if (dist > AM_REV_FADE) dist = AM_REV_FADE;
    int32_t fade = dist << 9;
    if (fade > 32767) fade = 32767;
    int32_t g = (((int16_t)*rev_gain * (int16_t)fade) >> 15);
    y = ((int16_t)y * (int16_t)g) >> 15;
    if (--(*rr) < 0) {
        *rr += len;
        *rev_gain = (((int16_t)*rev_gain * (int16_t)fb) >> 15);
    }
    return y;
}

static void am_init(int pos)
{
    am_state_t *s = &st[pos % 3];
    s->w1 = 0; s->w2 = 0;
    s->rr1 = 0; s->rr2 = 0;
    s->ph1 = 0;
    /* Different chaos seed per position, so the two slots wander differently. */
    s->chaos = 0x12345678u ^ (0x9E3779B9u * (uint32_t)(pos + 1));
    s->wander = 0;
    s->dc1 = 0; s->dc2 = 0;
    s->yp1 = 0; s->yp2 = 0;
    s->fb = 0; s->mix = 0;
    s->drift_s = 0; s->blur_s = 0; s->drive_s = 0;
    s->rev_gain1 = 32767; s->rev_gain2 = 32767;
    s->mute = 0;
    s->prev_erase = 0; s->prev_rev = 0;
    s->clear_idx = -1;
    int16_t *mem = (int16_t *)op1_fx_scratch(pos);
    if (mem) {
        for (int i = 0; i < AM_TOTAL_LEN; i++) mem[i] = 0;
    }
}

/* Drive: a tube-like cubic soft clip (unity gain at small signals,
 * gentle saturation at large ones), blended in by the knob. It shapes
 * the signal going into the memory, not the dry output. Only computed
 * while the knob is up. */
static int32_t am_drive(int32_t x, int32_t drive_s)
{
    if (drive_s <= 0) return x;
    int32_t u = ((int16_t)x * (int16_t)21845) >> 15; /* x * 2/3 */
    int64_t u3 = (int64_t)u * u * u;
    int32_t clipped = (3 * u >> 1) - (int32_t)(u3 >> 31);
    clipped = op1_sat16(clipped);
    return x + ((((int16_t)((clipped - x) >> 1)) * (int16_t)drive_s) >> 14);
}

/* Shared process function (the ente pattern): r == 0 is the mono render
 * for the synth/drum slots, r != 0 is the stereo render for the master
 * slot. In mono the behaviour is bit-for-bit the 3.6 build. In stereo
 * line 1 takes the left input and line 2 takes the right input. */
static void am_process(int pos, int16_t *l, int16_t *r, const int16_t *p)
{
    am_state_t *s = &st[pos % 3];
    int16_t *mem = (int16_t *)op1_fx_scratch(pos);
    if (!mem) return;                /* no scratch: leave the audio untouched */
    int16_t *d1 = mem;
    int16_t *d2 = mem + AM_L1_LEN;

    /* Knobs, read once per block. Gains are smoothed per sample below so
     * turning a knob never clicks. */
    const int32_t mix_t = op1_fx_param_q15(p, 0);
    const int32_t fb_t  = (op1_fx_param_q15(p, 1) * AM_FB_MAX) >> 15;
    const int32_t drift_t = op1_fx_param_q15(p, 2) >> 1; /* 0..16383 */
    const int32_t blur_t  = op1_fx_param_q15(p, 3);
    const int32_t drive_t = op1_fx_param_q15(p, 6); /* green knob, page 2 */

    /* Latches (op1_fx_page[pos][k] = 1 while the latch is on). Blue is the
     * Freeze latch: while it is on, no new input is written and the
     * feedback is held at 1.0, so the captured loop does not fade. */
    const int pi = pos % 3;
    const uint8_t freeze = op1_fx_page[pi][0];
    const uint8_t rev = op1_fx_page[pi][2];
    const uint8_t erase = op1_fx_page[pi][3];

    /* Erase: the rising edge starts an incremental clear. Wiping the whole
     * buffer in one block would starve the audio callback, so it goes a
     * little per block and is done in ~30 blocks (~40 ms). Erase and
     * Reverse both mute the wet briefly (see below) to hide the click. */
    if (erase && !s->prev_erase) { s->clear_idx = 0; s->mute = AM_MUTE_LEN; }
    s->prev_erase = erase;
    if (rev != s->prev_rev) {
        /* Turning Reverse ON needs the longer mute (the jump to backward
         * playback is big). Turning it OFF gets a short mute so the wet
         * comes back almost immediately - a long mute here is what made
         * the plugin sound dry after stopping Reverse. Turning it off
         * also resets the DC blocker states so the feedback restarts
         * from a clean state instead of a stale one. */
        s->mute = rev ? AM_MUTE_LEN : 64;
        s->prev_rev = rev;
        if (rev) {
            s->rev_gain1 = 32767; s->rev_gain2 = 32767;
        } else {
            s->yp1 = 0; s->yp2 = 0; s->dc1 = 0; s->dc2 = 0;
        }
    }
    if (s->clear_idx >= 0) {
        int32_t end = s->clear_idx + AM_CLEAR_CHUNK;
        if (end > AM_TOTAL_LEN) end = AM_TOTAL_LEN;
        for (int32_t i = s->clear_idx; i < end; i++) mem[i] = 0;
        s->clear_idx = (end >= AM_TOTAL_LEN) ? -1 : end;
    }

    /* Reverse heads track the write heads while reverse is off, so turning
     * reverse on starts reading backward from the most recent sound. */
    if (!rev) { s->rr1 = s->w1; s->rr2 = s->w2; }

    /* Chaotic drift: one slow random walk modulates the LFO rate by
     * +/-50%. The walk is updated once per block; it is slow by design.
     * The rate itself is set by the Drift knob (drift_s): at 0 the LFO
     * stands still, at maximum it runs at AM_DRIFT_INC_MAX. That is what
     * makes the knob feel gradual instead of on/off - turning it up makes
     * the balance move faster as well as wider, so the change is audible
     * right away instead of depending on where a slow LFO happens to be. */
    {
        uint32_t c = am_xorshift(&s->chaos);
        int32_t target = (int32_t)(c >> 16) - 32768;
        s->wander += (target - s->wander) >> 9;
    }
    const int32_t base_inc = (s->drift_s * AM_DRIFT_INC_MAX) >> 14;
    const int32_t inc1 = base_inc + (((base_inc >> 1) * s->wander) >> 15);

    for (int n = 0; n < OP1_FX_BLOCK; n++) {
        const int32_t xL = l[n];
        const int32_t xR = r ? r[n] : xL;

        /* Slew the gains and the knob values. The mute makes the mix
         * target 0 while it counts down, so the wet fades out and back in
         * around a Reverse toggle or an Erase instead of clicking. While
         * Freeze is on the feedback target is 1.0, so the loop is held;
         * it slews back to the knob when Freeze goes off. The drift depth
         * is a gain (the crossfade), so it can slew fast without glitching. */
        const int32_t mix_eff = (s->mute > 0) ? 0 : mix_t;
        const int32_t fb_eff = freeze ? 32767 : fb_t;
        if (s->mute > 0) s->mute--;
        s->fb  += (fb_eff - s->fb)  >> 6;
        s->mix += (mix_eff - s->mix) >> 6;
        s->drift_s += (drift_t - s->drift_s) >> 6;
        s->blur_s  += (blur_t  - s->blur_s)  >> 6;
        s->drive_s += (drive_t - s->drive_s) >> 6;

        /* Drift: the LFO position, crossfading the wet signal between the
         * two lines. bal = 0 is all line 1, 32767 is all line 2, 16384 is
         * even. The knob sets how far the balance swings; at 0 it stays
         * even. This moves the sound without moving any read head, so it
         * cannot glitch. Both operands are int16, so it is a MAC. */
        s->ph1 += (uint32_t)inc1;
        const int32_t bal = 16384 + ((OP1_SINE_TABLE[s->ph1 >> 19] * (int16_t)s->drift_s) >> 15);

        int32_t y1, y2;
        if (rev) {
            /* Reverse: the buffer is frozen (nothing is written, the write
             * head does not move), so the reverse head plays the held
             * memory backward without ever meeting a write head. This is
             * also why Reverse no longer destroys a Freeze. */
            y1 = am_rev_read(d1, AM_L1_LEN, &s->rr1, &s->rev_gain1, s->fb);
            y2 = am_rev_read(d2, AM_L2_LEN, &s->rr2, &s->rev_gain2, s->fb);
        } else {
            /* Drive shapes the signal going into the memory, per
             * channel in stereo (line 1 from the left, line 2 from the
             * right). In mono both lines take the same input, as before. */
            int32_t inL = am_drive(xL, s->drive_s);
            int32_t inR = r ? am_drive(xR, s->drive_s) : inL;
            if (freeze) { inL = 0; inR = 0; }   /* Freeze: play over the loop, do not record */

            /* --- line 1 --- */
            int32_t r1 = s->w1 - AM_L1_LEN;
            if (r1 < 0) r1 += AM_L1_LEN;
            y1 = d1[r1];
            /* --- line 2 --- */
            int32_t r2 = s->w2 - AM_L2_LEN;
            if (r2 < 0) r2 += AM_L2_LEN;
            y2 = d2[r2];

            /* DC blockers on the feedback signals, clamped to int16 so the
             * maths below can never overflow. Both operands are int16, so
             * the multiplies are single-cycle MACs. */
            int32_t f1 = y1 - s->yp1 + (((int16_t)s->dc1 * (int16_t)AM_DC_R) >> 15);
            s->yp1 = (int16_t)y1;
            f1 = op1_sat16(f1);
            s->dc1 = (int16_t)f1;
            int32_t f2 = y2 - s->yp2 + (((int16_t)s->dc2 * (int16_t)AM_DC_R) >> 15);
            s->yp2 = (int16_t)y2;
            f2 = op1_sat16(f2);
            s->dc2 = (int16_t)f2;

            /* Blur: cross-feedback between the lines, done by sum/difference
             * (1 multiply instead of 4). At blur 0 each line feeds itself;
             * at max each line feeds the other, and the lines blur.
             *   m1 = f1*(1-b) + f2*b,  m2 = f1*b + f2*(1-b)
             *   m1+m2 = f1+f2,  m1-m2 = (f1-f2)*(1-2b) */
            const int32_t sum = f1 + f2;
            const int16_t b2 = (int16_t)(32767 - 2 * s->blur_s);
            const int32_t diff = (((int16_t)((f1 - f2) >> 1) * b2) >> 14);
            const int32_t m1 = (sum + diff) >> 1;
            const int32_t m2 = (sum - diff) >> 1;
            const int32_t ft1 = (((int16_t)(m1 >> 1)) * (int16_t)s->fb) >> 14;
            const int32_t ft2 = (((int16_t)(m2 >> 1)) * (int16_t)s->fb) >> 14;

            /* Soft limiter on the loop (tape-like), then write. Line 1
             * takes the left input, line 2 the right (in mono both take
             * the same input, as before). */
            d1[s->w1] = (int16_t)am_softlimit(inL + ft1);
            d2[s->w2] = (int16_t)am_softlimit(inR + ft2);
            if (++s->w1 >= AM_L1_LEN) s->w1 = 0;
            if (++s->w2 >= AM_L2_LEN) s->w2 = 0;
        }

        /* --- mix --- */
        /* The wet signal. In mono it is the Drift crossfade between the
         * two lines. In stereo it is a true panorama: the left wet is
         * line 1 weighted by (1-bal) plus line 2 weighted by bal, and
         * the right wet is the mirror, so the LFO swings the sound
         * between the speakers. At the centre (bal even) both channels
         * carry the same mix; at the extremes line 1 is on the left and
         * line 2 on the right, or the other way round. Halved first so
         * the MACs can never overflow int32. The dry stays stereo. */
        const int32_t wetL = ((((int16_t)(y1 >> 1)) * (int16_t)(32767 - bal)) +
                              (((int16_t)(y2 >> 1)) * (int16_t)bal)) >> 14;
        /* (wet - x) is halved first so (diff * mix) can never overflow int32 */
        l[n] = op1_sat16(xL + ((((int16_t)((wetL - xL) >> 1)) * (int16_t)s->mix) >> 14));
        if (r) {
            const int32_t wetR = ((((int16_t)(y1 >> 1)) * (int16_t)bal) +
                                  (((int16_t)(y2 >> 1)) * (int16_t)(32767 - bal))) >> 14;
            r[n] = op1_sat16(xR + ((((int16_t)((wetR - xR) >> 1)) * (int16_t)s->mix) >> 14));
        }
    }
}

/* Mono render (synth/drum slots): the single buffer in place. */
static void am_render(int pos, int16_t *buf, const int16_t *p) { am_process(pos, buf, 0, p); }
/* Stereo render (master slot): independent L/R buffers. */
static void am_render_stereo(int pos, int16_t *left, int16_t *right, const int16_t *p) { am_process(pos, left, right, p); }

/* press: blue = Freeze latch, green = Drive page 2, white = Reverse latch,
 * orange = Erase latch. The mono prefix is unchanged; the master slot
 * calls render_stereo. */
const op1_fx_stereo_t FX_accident_memory = { { am_init, am_render, { OP1_PRESS_LATCH, OP1_PRESS_PAGE, OP1_PRESS_LATCH, OP1_PRESS_LATCH } }, am_render_stereo };
