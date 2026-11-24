// Pitchly - pitch correction engine (no JUCE dependency, so it can be unit-tested anywhere)
//
// Pipeline:  YIN pitch detection (decimated)  ->  snap to allowed notes  ->  retune-speed smoothing
//            ->  TD-PSOLA style pitch-synchronous grain resynthesis (with optional formant scaling)
//
// All memory is allocated in prepare(); process() is real-time safe.
#pragma once
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

namespace pitchly
{
// ---- key detection ---------------------------------------------------------------------------------------
// Krumhansl-Schmuckler: correlate a pitch-class histogram with typical major / minor key profiles.
// NB: a major key and its relative minor (C major / A minor) contain exactly the same notes, so for
// pitch correction it makes no difference which of the two wins; "setId" identifies that shared note set.
struct KeyGuess
{
    bool  valid = false;
    int   root = 0;          // 0 = C ... 11 = B
    bool  minor = false;
    float corr = 0.0f;       // correlation with the winning profile (-1..1)
    float margin = 0.0f;     // lead over the best key with a DIFFERENT note set
    bool  confident() const  { return valid && margin > 0.04f; }
    int   setId() const      { return minor ? (root + 3) % 12 : root; }
};

inline KeyGuess estimateKey (const float h[12])
{
    static const double major[12] = { 6.35, 2.23, 3.48, 2.33, 4.38, 4.09, 2.52, 5.19, 2.39, 3.66, 2.29, 2.88 };
    static const double minor[12] = { 6.33, 2.68, 3.52, 5.38, 2.60, 3.53, 2.54, 4.75, 3.98, 2.69, 3.34, 3.17 };
    KeyGuess g;
    double mx = 0.0; for (int i = 0; i < 12; ++i) mx += h[i];
    if (mx <= 1e-6) return g;
    mx /= 12.0;
    double vx = 0.0; for (int i = 0; i < 12; ++i) vx += (h[i] - mx) * (h[i] - mx);
    if (vx <= 1e-12) return g;

    double corr[2][12];
    for (int m = 0; m < 2; ++m)
    {
        const double* prof = m == 0 ? major : minor;
        double my = 0.0; for (int i = 0; i < 12; ++i) my += prof[i];
        my /= 12.0;
        double vy = 0.0; for (int i = 0; i < 12; ++i) vy += (prof[i] - my) * (prof[i] - my);
        for (int r = 0; r < 12; ++r)
        {
            double c = 0.0; for (int i = 0; i < 12; ++i) c += (h[(r + i) % 12] - mx) * (prof[i] - my);
            corr[m][r] = c / std::sqrt (vx * vy);
        }
    }
    int bm = 0, br = 0; double best = -2.0;
    for (int m = 0; m < 2; ++m) for (int r = 0; r < 12; ++r) if (corr[m][r] > best) { best = corr[m][r]; bm = m; br = r; }
    g.valid = true; g.root = br; g.minor = bm == 1; g.corr = (float) best;
    double second = -2.0;
    for (int m = 0; m < 2; ++m) for (int r = 0; r < 12; ++r)
    {
        const int id = m == 1 ? (r + 3) % 12 : r;
        if (id != g.setId() && corr[m][r] > second) second = corr[m][r];
    }
    g.margin = (float) (best - second);
    return g;
}

struct EngineParams
{
    float retuneMs     = 12.0f;  // 0 = instant (hard tune) .. 200 ms (gentle)
    float humanize     = 0.4f;   // 0..1  (widens dead-zone + slows retune a little)
    float amount       = 1.0f;   // 0..1  (how much of the correction is applied)
    float formantSemis = 0.0f;   // -6..+6
    float mix          = 1.0f;   // 0..1  Studio: dry/wet. Live: how much of the correction is applied (blending a dry copy at a different delay would phase)
    bool  active       = true;
    float tolerance    = 0.0f;   // cents of "close enough" around a note where nothing is corrected (Flex-Tune)
    float transpose    = 0.0f;   // semitones
    float detuneCents  = 0.0f;   // cents
    float refHz        = 440.0f; // A4 reference
    float gateDb       = -48.0f; // below this input level nothing is tracked
    float outGainDb    = 0.0f;
    int   range        = 0;      // 0 wide, 1 low, 2 mid, 3 high
    bool  collectKey   = false;  // true: tally which notes are sung (for key detection)
    bool  live         = false;  // true: low-latency delay-line shifter (formant control inactive); false: Studio (PSOLA)
    bool  notes[12]    = { true, false, true, false, true, true, false, true, false, true, false, true }; // C major
};

class PitchEngine
{
public:
    static constexpr int kMaxChannels = 2;
    static constexpr int kHistory     = 512;   // telemetry points kept for the UI graph

    // ---- voice types (musical terms) -------------------------------------------------------------------------------
    // Fundamental-frequency windows used by the pitch tracker. They are a little wider than the classical choral
    // ranges so that notes at the edges are not lost, but narrow enough to rule out most octave mistakes.
    static constexpr int kNumRanges = 6;                       // 0 Auto, 1 Soprano, 2 Alto, 3 Tenor, 4 Baritone, 5 Bass
    static double rangeMinHz (int r) { static const double v[kNumRanges] = { 75.0, 230.0, 150.0, 110.0, 85.0, 65.0 };    return v[clampRange (r)]; }
    static double rangeMaxHz (int r) { static const double v[kNumRanges] = { 1100.0, 1250.0, 800.0, 620.0, 450.0, 340.0 }; return v[clampRange (r)]; }
    static constexpr double kLiveCentre = 0.4;      // where in its one-period window the Live delay rests (fraction of a pitch period)
    static int clampRange (int r) { return r < 0 ? 0 : (r >= kNumRanges ? kNumRanges - 1 : r); }

    // Smallest delay the Live shifter can run at (cubic interpolation + crossfade safety)
    static int liveMinDelay (double sampleRate) { return std::max (4, (int) std::lround (sampleRate * 0.00008)); }   // cubic interpolation needs >= 3 samples

    // Latency in samples that will be reported to the host. Live: minimum delay plus half a pitch period of a typical
    // note for the voice type (the real delay wanders between ~0 and one period of the note being sung).
    static int latencyFor (double sampleRate, bool liveMode, int range)
    {
        const double mn = rangeMinHz (range), mx = rangeMaxHz (range);
        if (! liveMode) return (int) std::ceil (3.5 * (sampleRate / mn + 2.0)) + 8;
        return liveMinDelay (sampleRate) + (int) std::ceil (kLiveCentre * sampleRate / std::sqrt (mn * mx));
    }

    void prepare (double sampleRate, int numChannels)
    {
        sr  = sampleRate;
        nch = std::max (1, std::min (numChannels, kMaxChannels));

        maxPeriod = (int) std::ceil (sr / kMinHz) + 2;
        int worstLatency = 0;
        for (int r = 0; r < kNumRanges; ++r) worstLatency = std::max (worstLatency, latencyFor (sr, false, r));

        int ringSize = 1024;
        while (ringSize < worstLatency + 6 * maxPeriod) ringSize <<= 1;
        mask = ringSize - 1;
        for (int c = 0; c < kMaxChannels; ++c)
        {
            inRing[c].assign ((size_t) ringSize, 0.0f);
            outRing[c].assign ((size_t) ringSize, 0.0f);
        }
        detRing.assign ((size_t) ringSize, 0.0f);

        decim    = std::max (1, (int) std::lround (sr / 12000.0));
        srd      = sr / decim;
        tauMaxD  = (int) std::ceil (srd / kMinHz) + 2;
        tauMinD  = std::max (2, (int) std::floor (srd / 1300.0));
        decimTotal = (int) std::ceil (1.25 * tauMaxD) + tauMaxD + 8;      // largest analysis span (lowest voice)
        int dsz = 1024;
        while (dsz < decimTotal + 8) dsz <<= 1;
        decimMask = dsz - 1;
        decimRing.assign ((size_t) dsz, 0.0f);
        linear.assign ((size_t) decimTotal, 0.0f);
        diff.assign ((size_t) tauMaxD + 2, 0.0f);
        cmnd.assign ((size_t) tauMaxD + 2, 0.0f);
        spliceTmp.assign (1024, 0.0f);
        refD.assign ((size_t) (2 * (decim + 2) + 1), 1.0);

        hop = std::max (16, (int) std::lround (sr * 0.0025));            // 2.5 ms control rate: quick response
        configure (false, 0);
    }

    // Switch engine type / voice type. Safe on the audio thread (no allocation); clears the delay lines.
    void configure (bool liveMode, int range)
    {
        curLive = liveMode; curRange = clampRange (range);
        latency = latencyFor (sr, curLive, curRange);
        hop = std::max (16, (int) std::lround (sr * (curLive ? 0.0015 : 0.0025)));   // Live reacts faster
        ratioCoef = 1.0 - std::exp (-1.0 / (0.0007 * sr));
        designFilters();
        reset();
    }

    int    getLatencySamples() const { return latency; }
    bool   isLive() const            { return curLive; }
    float  getAppliedSemis() const { return (float) appliedSemis; }
    double getHopSeconds() const     { return (double) hop / sr; }
    double getLiveDelaySamples() const { return liveD1; }          // instantaneous delay of the Live shifter (for tests / meters)

    void reset()
    {
        for (int c = 0; c < kMaxChannels; ++c)
        {
            std::fill (inRing[c].begin(), inRing[c].end(), 0.0f);
            std::fill (outRing[c].begin(), outRing[c].end(), 0.0f);
        }
        std::fill (decimRing.begin(), decimRing.end(), 0.0f);
        std::fill (detRing.begin(), detRing.end(), 0.0f);
        for (auto& b : hp) b.clear();
        for (auto& b : lp) b.clear();
        lastCmin = 1.0f; startRun = 0;
        n = 0; decimWritten = 0; decimCount = 0; decimAcc = 0.0f; sinceHop = 0;
        s = 0.0; a = 0.0;
        period = sr / 200.0; voiced = false; corrSemis = 0.0; appliedSemis = 0.0;
        ratio = 1.0; ratioS = 1.0; fScale = 1.0; actSmooth = 1.0f; slowMidi = 60.0; voicedMs = 0.0; offSm = 0.0; gainSmooth = 1.0f;
        curNote = kNoNote; selMidi = 60.0; holdMs = 0.0; confSm = 0.0f; activeNote.store (-1);
        tauPrevD = 0.0; octCount = 0; octTauD = 0.0; telemCount = 0;
        liveD1 = liveD2 = (double) latency; liveXf = false; liveXfPos = 0; liveXfLen = 1;
        histWrite.store (0);
        for (int i = 0; i < 48; ++i) keyFine[i].store (0.0f);
        keyVoicedHops.store (0); keyClear.store (false);
        for (int i = 0; i < kHistory; ++i) { histDetected[i].store (NAN); histCorrected[i].store (NAN); }
    }

    // channels: in-place. Only the first kMaxChannels are processed; the host is expected to pass the same count as prepare().
    void process (float* const* ch, int numCh, int numSamples, const EngineParams& p)
    {
        const int C = std::min (numCh, nch);
        if (p.live != curLive || clampRange (p.range) != curRange) configure (p.live, p.range);
        for (int i = 0; i < numSamples; ++i)
        {
            float mono = 0.0f;
            for (int c = 0; c < C; ++c)
            {
                inRing[c][(size_t) (n & mask)] = ch[c][i];
                mono += ch[c][i];
            }
            mono /= (float) C;

            // detection-only signal: band-pass (rumble filter below the voice range, anti-alias filter above)
            const float fd = (float) lp[1].tick (lp[0].tick (hp[2].tick (hp[1].tick (hp[0].tick ((double) mono)))));
            detRing[(size_t) (n & mask)] = fd;
            if (++decimCount >= decim)
            {
                decimRing[(size_t) (decimWritten & decimMask)] = fd;
                ++decimWritten; decimCount = 0;
            }
            if (++sinceHop >= hop) { sinceHop = 0; analyse (p); }

            ++n;
            ratioS += (ratio - ratioS) * ratioCoef;      // the correction glides between control ticks instead of stepping
            const int64_t tOut = n - 1 - latency;

            float liveWet[kMaxChannels] = { 0.0f, 0.0f };
            if (curLive)
                liveStep (C, n - 1, liveWet);
            else
            {
                // synthesise every grain whose left edge has reached the output position
                int guard = 0;
                while (s - 1.5 * period <= (double) tOut && guard++ < 8)
                    synthGrain (C, tOut, p);
            }

            actSmooth += ((p.active ? 1.0f : 0.0f) - actSmooth) * 0.002f;
            const float mixEff = (curLive ? std::min (1.0f, p.mix * 20.0f) : p.mix) * actSmooth;   // Live: wet only (a dry copy at another delay would phase); mix 0 = true bypass
            gainSmooth += (std::pow (10.0f, p.outGainDb * 0.05f) - gainSmooth) * 0.002f;

            for (int c = 0; c < C; ++c)
            {
                float wet = 0.0f, dry = 0.0f;
                if (curLive) wet = liveWet[c];
                if (tOut >= 0)
                {
                    if (! curLive) { float& slot = outRing[c][(size_t) (tOut & mask)]; wet = slot; slot = 0.0f; }
                    dry = inRing[c][(size_t) (tOut & mask)];
                }
                ch[c][i] = (dry + (wet - dry) * mixEff) * gainSmooth;
            }
        }
    }

    // key detection tally (written by the audio thread while EngineParams::collectKey is set)
    // The tally has 25-cent resolution so the singer's overall tuning (e.g. consistently 35 cents flat) can be
    // estimated and compensated for before notes are assigned to semitones.
    std::atomic<float>    keyFine[48];
    std::atomic<uint32_t> keyVoicedHops { 0 };
    std::atomic<int>      activeNote { -1 };   // pitch class (0 = C) the voice is currently being tuned to, -1 = none (drives the lit note buttons)
    void requestKeyReset() { keyClear.store (true); }
    void getKeyHist (float out[12]) const
    {
        float f[48]; double sw = 0.0, sc = 0.0, ss = 0.0;
        const double twoPi = 6.283185307179586;
        for (int b = 0; b < 48; ++b)
        {
            f[b] = keyFine[b].load (std::memory_order_relaxed);
            sw += f[b]; sc += f[b] * std::cos (twoPi * b * 0.25); ss += f[b] * std::sin (twoPi * b * 0.25);
        }
        double offset = 0.0;                                   // circular mean of the sung pitch's distance from the semitone grid
        if (sw > 0.0 && std::sqrt (sc * sc + ss * ss) / sw > 0.3) offset = std::atan2 (ss, sc) / twoPi;
        for (int i = 0; i < 12; ++i) out[i] = 0.0f;
        for (int b = 0; b < 48; ++b) out[(((int) std::floor (b * 0.25 - offset + 0.5) % 12) + 12) % 12] += f[b];
    }

    // UI telemetry (lock-free; may be read from the message thread)
    std::atomic<uint32_t> histWrite { 0 };
    std::atomic<float> histDetected[kHistory];   // detected MIDI note (NaN = unvoiced)
    std::atomic<float> histCorrected[kHistory];  // detected + applied correction
    std::atomic<bool> keyClear { false };

private:
    static constexpr double kMinHz = 65.0;

    // ---- detection front end ---------------------------------------------------------------------------------------
    struct Biquad
    {
        double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
        void set (double fs, double fc, double q, bool high)
        {
            const double w0 = 6.283185307179586 * fc / fs, c = std::cos (w0), al = std::sin (w0) / (2.0 * q), a0 = 1.0 + al;
            b0 = (high ? (1 + c) : (1 - c)) * 0.5 / a0; b1 = (high ? -(1 + c) : (1 - c)) / a0; b2 = b0;
            a1 = -2.0 * c / a0; a2 = (1.0 - al) / a0;
        }
        double tick (double x) { const double y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
        void clear() { z1 = z2 = 0.0; }
    };
    void designFilters()
    {
        // 6th-order Butterworth high-pass at the lowest note of the voice type (-12 dB one voice-range-step below, -26 dB at 0.6x, -40 dB at 0.45x): mic-stand thumps, handling noise, AC hum and
        // room rumble are removed from what the pitch tracker (and its level gate) sees. The audio path itself is never filtered.
        const double fcH = std::max (50.0, std::min (250.0, rangeMinHz (curRange)));
        hp[0].set (sr, fcH, 0.5176, true);  hp[1].set (sr, fcH, 0.7071, true);  hp[2].set (sr, fcH, 1.9319, true);
        const double fcL = 0.42 * srd;                              // anti-alias before decimation
        lp[0].set (sr, fcL, 0.5412, false); lp[1].set (sr, fcL, 1.3066, false);
    }

    // Sub-sample refinement at the full sample rate: the coarse decimated estimate is polished by a normalised squared-difference
    // search (+-decim+2 lags) over ~1.5 periods, with a parabolic fit around the minimum. Typically well under 1 cent of error.
    // `e` = index of the newest sample of the analysis window: Live uses "now"; Studio centres the window on the output position
    // (the synthesis reads audio `latency` samples in the past, so the period must describe THAT part of the signal).
    double refinePeriod (double Pc)
    {
        const int64_t e = curLive ? n : n - std::max<int64_t> (0, (int64_t) latency - (int64_t) (0.75 * Pc));
        const int Lr = decim + 2, L0 = (int) std::lround (Pc), Lmin = L0 - Lr, Lmax = L0 + Lr, nl = Lmax - Lmin + 1;
        if (Lmin < 6 || (int) refD.size() < nl) return Pc;
        const int Wf = std::max (48, (int) std::ceil (1.5 * Pc));
        if (e < (int64_t) (Wf + Lmax + 4) || (n - e) + Wf + Lmax + 4 >= mask) return Pc;
        const float* r = detRing.data();
        int kb = 0;
        for (int k = 0; k < nl; ++k)
        {
            const int L = Lmin + k; double d = 0.0, en = 0.0;
            for (int j = 0; j < Wf; ++j)
            {
                const double x = r[(size_t) ((e - j) & mask)], y = r[(size_t) ((e - j - L) & mask)], df = x - y;
                d += df * df; en += x * x + y * y;
            }
            refD[(size_t) k] = en > 1e-12 ? d / en : 1.0;
            if (refD[(size_t) k] < refD[(size_t) kb]) kb = k;
        }
        if (kb == 0 || kb == nl - 1) return Pc;
        const double y0 = refD[(size_t) (kb - 1)], y1 = refD[(size_t) kb], y2 = refD[(size_t) (kb + 1)], den = y0 - 2.0 * y1 + y2;
        double ref = (double) (Lmin + kb);
        if (den > 1e-12) ref += std::max (-1.0, std::min (1.0, 0.5 * (y0 - y2) / den));
        return std::fabs (ref - Pc) > 1.5 * decim ? Pc : ref;
    }

    // ---- pitch detection (YIN) on a ~12 kHz decimated copy of the input -------------------
    //
    // Octave-error defences (applied on top of plain YIN):
    //  1. the search is limited to the selected voice type's range;
    //  2. every dip in the difference function is a candidate; among near-equal candidates the SHORTEST period wins
    //     (a sub-harmonic dip is always there for a periodic sound, the true fundamental is the first);
    //  3. continuity: a candidate close to the previous pitch is preferred over a slightly better one far away;
    //  4. a sudden jump of about an octave (or more) must persist for a few hops before it is believed.
    bool detect (double& periodDec, float& rms, double minHz, double maxHz)
    {
        lastCmin = 1.0f;
        const int tauMin = std::max (tauMinD, (int) std::floor (srd / maxHz));
        int tauMax = std::min (tauMaxD, (int) std::ceil (srd / minHz) + 1);
        // Once locked on, only look ~2 periods deep: a window sized for the current pitch (not the lowest possible one) reacts much faster.
        // (Live mode only: Studio mode prefers the smoother estimate of a longer window.)
        if (curLive && tauPrevD > 0.0) tauMax = std::min (tauMax, (int) std::ceil (2.2 * tauPrevD) + 2);
        if (tauMax - tauMin < 6) { tauPrevD = 0.0; return false; }
        const int W = std::max (24, (int) std::ceil (1.25 * tauMax));      // short window for high voices = quick response
        const int total = W + tauMax + 1;
        if (decimWritten < total || total > decimTotal) return false;

        for (int i = 0; i < total; ++i)
            linear[(size_t) i] = decimRing[(size_t) ((decimWritten - total + i) & decimMask)];

        double e = 0.0;
        for (int i = total - W; i < total; ++i) e += (double) linear[(size_t) i] * linear[(size_t) i];
        rms = (float) std::sqrt (e / W);

        for (int tau = 1; tau <= tauMax; ++tau)
        {
            double d = 0.0;
            for (int j = 0; j < W; ++j)
            {
                const double df = (double) linear[(size_t) j] - (double) linear[(size_t) (j + tau)];
                d += df * df;
            }
            diff[(size_t) tau] = (float) d;
        }
        cmnd[0] = 1.0f;
        double run = 0.0;
        for (int tau = 1; tau <= tauMax; ++tau)
        {
            run += diff[(size_t) tau];
            cmnd[(size_t) tau] = run > 1e-12 ? (float) (diff[(size_t) tau] * tau / run) : 1.0f;
        }

        // 1. collect candidates = local minima of the normalised difference function
        int candTau[32]; float candV[32]; int nc = 0; float cmin = 1e9f;
        for (int tau = tauMin; tau < tauMax; ++tau)
        {
            const float v = cmnd[(size_t) tau];
            if (v < 0.35f && v < cmnd[(size_t) (tau - 1)] && v <= cmnd[(size_t) (tau + 1)])
            {
                if (nc < 32) { candTau[nc] = tau; candV[nc] = v; ++nc; }
                else { int worst = 0; for (int k = 1; k < 32; ++k) if (candV[k] > candV[worst]) worst = k;
                       if (v < candV[worst]) { candTau[worst] = tau; candV[worst] = v; } }
                cmin = std::min (cmin, v);
            }
        }
        lastCmin = cmin;
        if (nc == 0 || cmin >= 0.25f) return false;

        // 2. shortest period among the near-best candidates
        int chosen = -1;
        for (int k = 0; k < nc; ++k)
            if (candV[k] <= cmin + 0.06f && (chosen < 0 || candTau[k] < candTau[chosen])) chosen = k;
        // high pitches: the true period falls between decimated samples, so its dip can be shallower than the double period's.
        // Prefer a reasonably deep dip at HALF the chosen lag (a genuine sub-octave would not have one).
        for (int k = 0; k < nc; ++k)
            if (std::fabs ((double) candTau[k] * 2.0 / candTau[chosen] - 1.0) < 0.08 && candV[k] < 0.25f && candV[k] <= cmin + 0.20f) { chosen = k; break; }

        // 3. continuity with the previous pitch
        if (tauPrevD > 0.0)
        {
            int near = -1; double nearDist = 1e9;
            for (int k = 0; k < nc; ++k)
            {
                if (candV[k] > cmin + 0.12f) continue;
                const double dist = std::fabs (std::log2 ((double) candTau[k] / tauPrevD));
                if (dist < nearDist) { nearDist = dist; near = k; }
            }
            // Stay with the track only if that does not mean accepting a LOWER pitch than the shortest good candidate:
            // for a periodic sound the half-pitch dip is always present, so following it would lock onto a sub-harmonic forever.
            // A genuine jump to a higher octave is handled by the persistence check below.
            if (near >= 0 && nearDist < 0.12 && candTau[near] <= candTau[chosen] * 1.08) chosen = near;
        }
        int tau = candTau[chosen];

        // 4. octave-sized jumps must persist
        if (tauPrevD > 0.0 && std::fabs (std::log2 ((double) tau / tauPrevD)) > 0.8)
        {
            if (octTauD > 0.0 && std::fabs (std::log2 ((double) tau / octTauD)) < 0.1) ++octCount; else { octTauD = tau; octCount = 1; }
            if (octCount < std::max (kOctaveHold, (int) std::ceil (0.0075 * sr / hop)))   // ~7.5 ms regardless of the control rate
                 { periodDec = tauPrevD; return true; }      // not convinced yet: keep the old pitch
        }
        octCount = 0; octTauD = 0.0;

        double ref = tau;
        if (tau > 1 && tau < tauMax - 1)
        {
            const double s0 = cmnd[(size_t) (tau - 1)], s1 = cmnd[(size_t) tau], s2 = cmnd[(size_t) (tau + 1)];
            const double den = 2.0 * (s0 - 2.0 * s1 + s2);
            if (std::fabs (den) > 1e-9) ref += std::max (-1.0, std::min (1.0, (s0 - s2) / den));
        }
        periodDec = ref; tauPrevD = ref;
        return true;
    }

    int nearestAllowed (const EngineParams& p, double m) const
    {
        const int base = (int) std::floor (m + 0.5);
        double best = 1e9; int bestNote = kNoNote;
        for (int kk = -7; kk <= 7; ++kk)
        {
            const int cand = base + kk;
            if (p.notes[((cand % 12) + 12) % 12]) { const double d = std::fabs ((double) cand - m); if (d < best) { best = d; bestNote = cand; } }
        }
        return bestNote;
    }

    void analyse (const EngineParams& p)
    {
        const int rg = clampRange (p.range);
        const double hopSec = (double) hop / sr;
        if (keyClear.exchange (false)) { for (int i = 0; i < 48; ++i) keyFine[i].store (0.0f, std::memory_order_relaxed); keyVoicedHops.store (0); }

        double perDec = 0.0; float rms = 0.0f;
        const float gate = std::pow (10.0f, p.gateDb * 0.05f);
        const float thr = voiced ? gate * 0.63f : gate;                 // 4 dB hysteresis: no chattering at the threshold
        const bool det = detect (perDec, rms, rangeMinHz (rg), rangeMaxHz (rg)) && rms > thr;   // rms is measured AFTER the rumble filter
        bool found = det;
        if (det && ! voiced)                                           // a new note must be clearly periodic for 2 hops in a row
        {
            startRun = lastCmin <= 0.25f ? startRun + 1 : 0;
            found = startRun >= 2;
        }
        else if (! det) startRun = 0;
        const bool wasVoiced = voiced;

        if (found)
        {
            const double pNew = refinePeriod (perDec * decim);
            period = voiced ? period + (1.0 - std::exp (-hopSec / 0.00546)) * (pNew - period) : pNew;   // ~5.5 ms time constant, independent of the control rate
            period = std::max ((double) (sr / 1350.0), std::min (period, (double) maxPeriod - 2.0));
            voiced = true;
        }
        else { voiced = false; if (! det) { tauPrevD = 0.0; octCount = 0; octTauD = 0.0; } }

        double detMidi = NAN, corr = 0.0;
        float confTarget = 0.0f;
        if (voiced)
        {
            detMidi = 69.0 + 12.0 * std::log2 ((sr / period) / std::max (400.0, (double) p.refHz));
            if (p.collectKey)
            {
                const int bin = (((int) std::floor (detMidi * 4.0 + 0.5) % 48) + 48) % 48;
                keyFine[bin].store (keyFine[bin].load (std::memory_order_relaxed) + 1.0f, std::memory_order_relaxed);
                keyVoicedHops.fetch_add (1, std::memory_order_relaxed);
            }
            if (! wasVoiced || std::fabs (detMidi - slowMidi) > 0.8) slowMidi = detMidi;     // new note: restart the slow tracker
            else slowMidi += (1.0 - std::exp (-hopSec / 0.15)) * (detMidi - slowMidi);
            voicedMs = wasVoiced ? voicedMs + hopSec * 1000.0 : 0.0;
            selMidi = wasVoiced ? selMidi + (1.0 - std::exp (-hopSec / 0.03)) * (detMidi - selMidi) : detMidi;   // jitter-free copy used to CHOOSE the note

            // --- which note are we on?  Sticky: the pitch must clearly cross the halfway point to the neighbouring note (and stay there
            // for a few ms) before the target changes, so a sustained note that sits near the border never flips back and forth.
            const int cand = nearestAllowed (p, selMidi);
            if (! wasVoiced || curNote == kNoNote || ! p.notes[((curNote % 12) + 12) % 12]) { curNote = cand; holdMs = 0.0; }
            else if (cand != kNoNote && cand != curNote)
            {
                const double mid = 0.5 * ((double) curNote + (double) cand);
                const double margin = 0.15 + 0.15 * (double) p.humanize;                       // semitones past the midpoint (humanize = stickier)
                const bool up = cand > curNote;
                const bool beyond = up ? selMidi > mid + margin : selMidi < mid - margin;
                const bool clearJump = up ? detMidi > mid + 0.9 : detMidi < mid - 0.9;           // an obvious new note: don't wait
                if (beyond) { holdMs += hopSec * 1000.0; if (clearJump || holdMs >= 12.0) { curNote = cand; holdMs = 0.0; } }
                else holdMs = 0.0;
            }
            else holdMs = 0.0;

            if (curNote != kNoNote)
            {
                double e1 = (double) curNote - slowMidi;
                const double mag = std::max (0.0, std::fabs (e1) - (double) p.tolerance / 100.0);
                e1 = e1 < 0 ? -mag : mag;
                const double vib = std::max (-0.6, std::min (0.6, detMidi - slowMidi));        // fast wobble: flattened like everything else
                corr = e1 - vib;
            }
            confTarget = std::max (0.0f, std::min (1.0f, (0.28f - lastCmin) / 0.16f));        // how clearly periodic the sound is
        }
        else { voicedMs = 0.0; curNote = kNoNote; holdMs = 0.0; }

        // Correction strength follows the signal: it fades in with the periodicity and drops out FAST on consonants / breaths / noise,
        // whatever the retune speed, so unpitched sounds are never shifted.
        confSm += (confTarget - confSm) * (float) (1.0 - std::exp (-hopSec / (confTarget > confSm ? 0.008 : 0.003)));

        const double tau = std::max (0.0f, p.retuneMs) / 1000.0 * (1.0 + 3.0 * p.humanize);
        const double alpha = tau < 1e-4 ? 1.0 : 1.0 - std::exp (-hopSec / tau);
        corrSemis += alpha * (corr - corrSemis);

        offSm += 0.25 * (((double) p.transpose + (double) p.detuneCents / 100.0) - offSm);
        const double strength = (double) p.amount * (curLive ? (double) std::max (0.0f, std::min (1.0f, p.mix)) : 1.0);
        appliedSemis = std::max (-6.0, std::min (6.0, corrSemis * (double) confSm * strength)) + offSm;
        appliedSemis = std::max (-12.0, std::min (12.0, appliedSemis));
        ratio  = std::pow (2.0, appliedSemis / 12.0);
        fScale = std::pow (2.0, std::max (-6.0f, std::min (6.0f, p.formantSemis)) / 12.0);

        activeNote.store (voiced && confSm > 0.3f && curNote != kNoNote ? ((curNote % 12) + 12) % 12 : -1, std::memory_order_relaxed);
        if ((++telemCount & 1) != 0) return;                         // the graph only needs every other control tick
        const uint32_t w = histWrite.load (std::memory_order_relaxed);
        histDetected[w % kHistory].store ((float) detMidi, std::memory_order_relaxed);
        histCorrected[w % kHistory].store (voiced ? (float) (detMidi + appliedSemis) : (float) NAN, std::memory_order_relaxed);
        histWrite.store (w + 1, std::memory_order_release);
    }

    // ---- Live shifter: causal delay-line pitch shifting ----------------------------------------------------------------
    // The output reads the input through a delay D that changes by (1 - ratio) every sample, which pitch-shifts by `ratio`
    // (formants move with the pitch, as with any resampling shifter). When D would leave [dMin, dMin + period] we jump by a
    // whole number of pitch periods, which lands on an identical-looking part of the waveform; a short crossfade plus a
    // cross-correlation search for the best alignment hides the join. Nothing in the future is ever needed, so the delay is
    // only a fraction of a pitch period instead of the 2-3 periods that grain-based (PSOLA) resynthesis needs.
    float sampleAt (int c, int64_t i, int64_t newest) const
    {
        if (i < 0) return 0.0f;
        if (i > newest) i = newest;
        return inRing[c][(size_t) (i & mask)];
    }
    float monoAt (int64_t i, int64_t newest) const
    {
        float m = 0.0f; for (int c = 0; c < nch; ++c) m += sampleAt (c, i, newest);
        return m / (float) nch;
    }
    float readCubic (int c, double pos, int64_t newest) const
    {
        const int64_t i = (int64_t) std::floor (pos); const float f = (float) (pos - (double) i);
        const float xm1 = sampleAt (c, i - 1, newest), x0 = sampleAt (c, i, newest), x1 = sampleAt (c, i + 1, newest), x2 = sampleAt (c, i + 2, newest);
        const float k3 = -0.5f * xm1 + 1.5f * x0 - 1.5f * x1 + 0.5f * x2, k2 = xm1 - 2.5f * x0 + 2.0f * x1 - 0.5f * x2, k1 = -0.5f * xm1 + 0.5f * x1;
        return ((k3 * f + k2) * f + k1) * f + x0;
    }

    void startSplice (int64_t newest, double P, double dCentre)
    {
        const double dMin = (double) liveMinDelay (sr);
        const double dir = liveD1 < dCentre ? 1.0 : -1.0;
        const double k = std::max (1.0, std::floor (std::fabs (liveD1 - dCentre) / P + 0.5));
        double d2 = liveD1 + dir * k * P;
        d2 = std::max (dMin + 4.0, std::min (d2, (double) (mask + 1) * 0.5 - 1200.0));

        // find the small offset that makes the waveform at the new tap line up best with the old tap
        const int Lc = std::max (32, std::min ((int) P, 700));
        const int dmax = std::max (2, std::min (150, (int) (0.12 * P)));
        const int64_t i1 = newest - (int64_t) std::llround (liveD1);
        double xx = 0.0;
        for (int j = 0; j < Lc; ++j) { spliceTmp[(size_t) j] = monoAt (i1 - j, newest); xx += (double) spliceTmp[(size_t) j] * spliceTmp[(size_t) j]; }

        double delta = 0.0;
        if (xx > 1e-9)
        {
            double corr[2 * 150 + 3]; double best = -2.0; int bestK = 0;
            for (int d = -dmax; d <= dmax; ++d)
            {
                corr[d + dmax] = -2.0;
                const int64_t D = (int64_t) std::llround (d2) + d;
                if (D < (int64_t) dMin + 2) continue;
                const int64_t i2 = newest - D;
                double xy = 0.0, yy = 0.0;
                for (int j = 0; j < Lc; ++j) { const double y = monoAt (i2 - j, newest); xy += (double) spliceTmp[(size_t) j] * y; yy += y * y; }
                const double cv = xy / std::sqrt (xx * yy + 1e-18);
                corr[d + dmax] = cv;
                if (cv > best) { best = cv; bestK = d; }
            }
            if (best > 0.2)
            {
                delta = bestK;
                if (bestK > -dmax && bestK < dmax)
                {
                    const double y0 = corr[bestK - 1 + dmax], y1 = corr[bestK + dmax], y2 = corr[bestK + 1 + dmax], den = y0 - 2.0 * y1 + y2;
                    if (y0 > -1.5 && y2 > -1.5 && std::fabs (den) > 1e-12) delta += std::max (-1.0, std::min (1.0, 0.5 * (y0 - y2) / den));
                }
            }
        }
        liveD2 = std::max (dMin + 2.0, d2 + delta);
        liveXfLen = std::max (24, std::min (400, (int) std::lround (0.75 * P)));
        liveXfPos = 0; liveXf = true;
    }

    void liveStep (int C, int64_t newest, float* out)
    {
        const double P = period, dMin = (double) liveMinDelay (sr);
        // When pitching UP the delay shrinks while the crossfade is still running, so the whole allowed range [dLow, dLow + P]
        // is moved up a little: the old tap then still has valid audio until the crossfade has finished.
        const double xfEst = (double) std::max (24, std::min (400, (int) std::lround (0.75 * P)));
        const double dLow = dMin + std::max (0.0, ratioS - 1.0) * xfEst, dHigh = dLow + P, dCentre = dLow + 0.5 * P;
        if (! liveXf && (liveD1 < dLow || liveD1 > dHigh)) startSplice (newest, P, dCentre);

        // tiny steering term keeps the delay near the middle of its range when no correction is happening (max ~2.6 cents)
        const double servo = std::max (-0.0015, std::min (0.0015, 0.0004 * (liveD1 - (dLow + kLiveCentre * P))));
        const double step = 1.0 - (ratioS + servo);

        const double w = liveXf ? 0.5 - 0.5 * std::cos (3.14159265358979323846 * (liveXfPos + 1) / (liveXfLen + 1)) : 0.0;
        for (int c = 0; c < C; ++c)
        {
            const float tapA = readCubic (c, (double) newest - liveD1, newest);
            if (liveXf) { const float tapB = readCubic (c, (double) newest - liveD2, newest); out[c] = tapA + (tapB - tapA) * (float) w; }
            else out[c] = tapA;
        }
        liveD1 += step;
        if (liveXf)
        {
            liveD2 += step;
            if (++liveXfPos >= liveXfLen) { liveD1 = liveD2; liveXf = false; }
        }
        liveD1 = std::max (4.0, std::min (liveD1, (double) (mask + 1) * 0.5));
    }

    // ---- grain resynthesis -------------------------------------------------------------------
    float readInterp (int c, double pos) const
    {
        const double newest = (double) (n - 1);
        if (pos < 0.0) return 0.0f;
        if (pos > newest - 1.0) pos = newest - 1.0;
        const int64_t i0 = (int64_t) pos;
        const float fr = (float) (pos - (double) i0);
        const float x0 = inRing[c][(size_t) (i0 & mask)];
        const float x1 = inRing[c][(size_t) ((i0 + 1) & mask)];
        return x0 + (x1 - x0) * fr;
    }

    void synthGrain (int C, int64_t tOut, const EngineParams&)
    {
        const double P  = period;
        const double f  = fScale;
        const double Ps = P / ratio;                // synthesis spacing -> output pitch
        double halfL = std::max (P / f, 0.9 * Ps);  // grain half length (wider for big downward shifts so grains still overlap)
        halfL = std::min (halfL, 1.5 * P / std::max (1.0, f));
        const double gain  = Ps / halfL;            // overlap normalisation
        const double pi = 3.14159265358979323846;

        int64_t o0 = (int64_t) std::ceil (s - halfL);
        const int64_t o1 = (int64_t) std::floor (s + halfL);
        o0 = std::max (o0, std::max<int64_t> (tOut, 0));

        for (int c = 0; c < C; ++c)
        {
            for (int64_t o = o0; o <= o1; ++o)
            {
                const double j = (double) o - s;
                const double w = 0.5 * (1.0 + std::cos (pi * j / halfL));
                outRing[c][(size_t) (o & mask)] += (float) (gain * w) * readInterp (c, a + j * f);
            }
        }

        const double sNext = s + Ps;
        const double m = std::max (0.0, std::floor ((sNext - a) / P + 0.5));
        double aNext = a + m * P;
        if (std::fabs (aNext - sNext) > 1.5 * P) aNext = sNext;   // re-sync if we drifted
        s = sNext; a = aNext;
    }

    // state
    static constexpr int kOctaveHold = 3;
    double ratioCoef = 0.03;
    bool curLive = false; int curRange = 0;
    double tauPrevD = 0.0, octTauD = 0.0; int octCount = 0, telemCount = 0;
    double liveD1 = 0.0, liveD2 = 0.0; bool liveXf = false; int liveXfPos = 0, liveXfLen = 1;
    std::vector<float> spliceTmp;
    double sr = 48000.0; int nch = 2;
    int maxPeriod = 600, latency = 1800, mask = 0;
    std::vector<float> inRing[kMaxChannels], outRing[kMaxChannels];
    int64_t n = 0;
    double s = 0.0, a = 0.0, period = 240.0, corrSemis = 0.0, appliedSemis = 0.0, ratio = 1.0, fScale = 1.0;
    double slowMidi = 60.0, voicedMs = 0.0, offSm = 0.0, ratioS = 1.0, selMidi = 60.0, holdMs = 0.0;
    static constexpr int kNoNote = -100000;
    int curNote = kNoNote; float confSm = 0.0f;
    bool voiced = false;
    float actSmooth = 1.0f, gainSmooth = 1.0f;

    int decim = 4, tauMaxD = 152, tauMinD = 13, winD = 304, decimTotal = 457, decimMask = 1023, hop = 240, sinceHop = 0, decimCount = 0;
    double srd = 12000.0; float decimAcc = 0.0f; int64_t decimWritten = 0;
    std::vector<float> decimRing, linear, diff, cmnd, detRing;
    std::vector<double> refD;
    Biquad hp[3], lp[2];
    float lastCmin = 1.0f; int startRun = 0;
};
} // namespace pitchly
