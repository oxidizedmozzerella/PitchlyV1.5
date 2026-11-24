#include "../Source/PitchEngine.h"
#include <cstdio>
#include <cstdlib>
#include <random>

using namespace pitchly;
static const double SR = 48000.0;

static std::vector<float> voice (double hz, double seconds, double vibCents = 0.0)
{
    const int N = (int) (SR * seconds);
    std::vector<float> x ((size_t) N);
    double ph = 0.0;
    for (int i = 0; i < N; ++i)
    {
        const double t = i / SR;
        const double f = hz * std::pow (2.0, (vibCents * std::sin (2 * M_PI * 5.5 * t)) / 1200.0);
        ph += 2 * M_PI * f / SR;
        double v = 0.0;
        for (int h = 1; h <= 10; ++h) v += std::sin (h * ph) / h;        // buzzy vocal-ish source
        x[(size_t) i] = (float) (0.25 * v);
    }
    return x;
}

// robust autocorrelation pitch estimate on x[start, start+len)
static double measureHz (const std::vector<float>& x, int start, int len)
{
    const int minLag = (int) (SR / 800), maxLag = (int) (SR / 80);
    std::vector<double> r ((size_t) maxLag + 2, 0.0);
    double r0 = 0;
    for (int i = 0; i < len; ++i) r0 += (double) x[(size_t) (start + i)] * x[(size_t) (start + i)];
    if (r0 < 1e-9) return 0;
    for (int lag = minLag; lag <= maxLag; ++lag)
    {
        double acc = 0, e2 = 0;
        for (int i = 0; i < len - maxLag; ++i) { acc += (double) x[(size_t) (start + i)] * x[(size_t) (start + i + lag)]; e2 += (double) x[(size_t) (start + i + lag)] * x[(size_t) (start + i + lag)]; }
        r[(size_t) lag] = acc / std::sqrt (r0 * (e2 + 1e-12));
    }
    double bestV = -1; int best = minLag;
    for (int lag = minLag; lag <= maxLag; ++lag) if (r[(size_t) lag] > bestV) { bestV = r[(size_t) lag]; best = lag; }
    // prefer the smallest lag that is nearly as good (avoid sub-harmonic picks)
    for (int lag = minLag; lag < best; ++lag)
        if (r[(size_t) lag] > 0.97 * bestV && r[(size_t) lag] >= r[(size_t) lag - 1] && r[(size_t) lag] >= r[(size_t) lag + 1]) { best = lag; break; }
    double ref = best;
    const double s0 = r[(size_t) best - 1], s1 = r[(size_t) best], s2 = r[(size_t) best + 1];
    const double den = s0 - 2 * s1 + s2;
    if (std::fabs (den) > 1e-12) ref += 0.5 * (s0 - s2) / den;
    return SR / ref;
}

static double cents (double a, double b) { return 1200.0 * std::log2 (a / b); }


// rms deviation (in cents) of the pitch measured in many overlapping 4096-sample windows, relative to refHz
static double rmsDevCents (const std::vector<float>& x, double refHz, double t0, double t1)
{
    double se = 0; int n = 0;
    for (int st = (int) (SR * t0); st + 4096 < (int) (SR * t1) && st + 4096 < (int) x.size(); st += 1000)
    { const double c = cents (measureHz (x, st, 4096), refHz); se += c * c; ++n; }
    return n ? std::sqrt (se / n) : 1e9;
}
// pitch tracker trace: (time in seconds, detected MIDI note) taken from the engine's telemetry after every small block
static std::vector<std::pair<double, double>> detTrace (const std::vector<float>& in, EngineParams p, bool live = false)
{
    p.live = live; PitchEngine e; e.prepare (SR, 1); std::vector<std::pair<double, double>> tr; std::vector<float> blk (128);
    for (size_t pos = 0; pos + 128 <= in.size(); pos += 128)
    {
        for (int i = 0; i < 128; ++i) blk[(size_t) i] = in[pos + (size_t) i];
        float* ch[1] = { blk.data() }; e.process (ch, 1, 128, p);
        const uint32_t w = e.histWrite.load(); if (w == 0) continue;
        tr.push_back ({ (double) (pos + 128) / SR, (double) e.histDetected[(w - 1) % PitchEngine::kHistory].load() });
    }
    return tr;
}
static double midiOf (double hz) { return 69.0 + 12.0 * std::log2 (hz / 440.0); }
// harmonic tone with a custom spectrum: amps[h-1] for harmonic h
static std::vector<float> tone (double hz, double seconds, const std::vector<double>& amps, double noise = 0.0, unsigned seed = 5)
{
    const int N = (int) (SR * seconds); std::vector<float> x ((size_t) N); double ph = 0; std::mt19937 rng (seed); std::normal_distribution<double> nd (0.0, 1.0);
    for (int i = 0; i < N; ++i) { ph += 2 * M_PI * hz / SR; double v = 0; for (size_t h = 0; h < amps.size(); ++h) v += amps[h] * std::sin ((double) (h + 1) * ph); x[(size_t) i] = (float) (0.2 * v + noise * nd (rng)); }
    return x;
}

static std::vector<float> run (std::vector<float> in, EngineParams p, int block = 512, int channels = 1, int* latencyOut = nullptr)
{
    PitchEngine e; e.prepare (SR, channels); e.configure (p.live, p.range);
    if (latencyOut) *latencyOut = e.getLatencySamples();
    std::vector<float> out (in.size());
    std::vector<float> l (block), r (block);
    for (size_t pos = 0; pos < in.size(); pos += (size_t) block)
    {
        const int cnt = (int) std::min ((size_t) block, in.size() - pos);
        for (int i = 0; i < cnt; ++i) l[(size_t) i] = r[(size_t) i] = in[pos + (size_t) i];
        float* chs[2] = { l.data(), r.data() };
        e.process (chs, channels, cnt, p);
        for (int i = 0; i < cnt; ++i) out[pos + (size_t) i] = l[(size_t) i];
    }
    return out;
}

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf ("  FAIL: "); std::printf (__VA_ARGS__); std::printf ("\n"); } else { std::printf ("  ok:   "); std::printf (__VA_ARGS__); std::printf ("\n"); } } while (0)

int main()
{
    const double B3 = 246.9417, A3 = 220.0;
    EngineParams hard; hard.retuneMs = 0; hard.humanize = 0; hard.amount = 1; hard.mix = 1; hard.formantSemis = 0;

    std::printf ("[1] mix=0 is a clean delayed passthrough\n");
    { EngineParams p = hard; p.mix = 0; int lat = 0; auto in = voice (238, 1.0); auto out = run (in, p, 333, 1, &lat);
      double maxErr = 0; for (size_t i = (size_t) lat + 10; i < in.size(); ++i) maxErr = std::max (maxErr, (double) std::fabs (out[i] - in[i - (size_t) lat]));
      CHECK (maxErr < 1e-5, "max error %.2e (latency %d samples = %.1f ms)", maxErr, lat, 1000.0 * lat / SR); }

    std::printf ("[2] 238 Hz (A#3 +36c) in C major -> should snap to B3 (246.94 Hz)\n");
    { auto out = run (voice (238, 2.0), hard); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, B3)) < 8.0, "measured %.2f Hz (%.1f cents from B3)", hz, cents (hz, B3)); }

    std::printf ("[3] 212 Hz (G#3) in C major -> should snap to A3 (220 Hz)\n");
    { auto out = run (voice (212, 2.0), hard); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, A3)) < 8.0, "measured %.2f Hz (%.1f cents from A3)", hz, cents (hz, A3)); }

    std::printf ("[4] voice already in tune stays put (220 Hz)\n");
    { auto out = run (voice (220, 2.0), hard); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, A3)) < 6.0, "measured %.2f Hz", hz); }

    std::printf ("[5] 5.5 Hz vibrato +-40c around 238 Hz is largely flattened by hard tune (rms deviation, many windows)\n");
    { auto in = voice (238, 2.5, 40); auto out = run (in, hard);
      const double inDev = rmsDevCents (in, 238.0, 0.9, 2.45), outDev = rmsDevCents (out, B3, 0.9, 2.45);
      CHECK (outDev < 0.8 * inDev, "output deviates %.1f cents rms from B3 (untouched input: %.1f)", outDev, inDev); }

    std::printf ("[6] slow retune (200 ms) moves gradually, not instantly\n");
    { EngineParams p = hard; p.retuneMs = 200; auto out = run (voice (238, 2.0), p);
      double early = measureHz (out, (int) (SR * 0.25), 4096), late = measureHz (out, (int) (SR * 1.7), 4096);
      CHECK (std::fabs (cents (late, B3)) < 15.0, "settles on B3 (late %.2f Hz)", late);
      CHECK (std::fabs (cents (early, B3)) > std::fabs (cents (late, B3)), "early %.2f Hz is further from target than late", early); }

    std::printf ("[7] formant shift does not change pitch\n");
    { EngineParams p = hard; p.formantSemis = 4; auto out = run (voice (238, 2.0), p); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, B3)) < 12.0, "measured %.2f Hz with +4 st formant", hz); }

    std::printf ("[8] amount=0 leaves pitch alone\n");
    { EngineParams p = hard; p.amount = 0; auto out = run (voice (238, 2.0), p); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, 238.0)) < 6.0, "measured %.2f Hz", hz); }

    std::printf ("[9] noise (unvoiced) passes through at about unity level, no NaN/inf\n");
    { std::mt19937 rng (1); std::normal_distribution<float> nd (0.f, 0.1f); std::vector<float> in (96000); for (auto& v : in) v = nd (rng);
      auto out = run (in, hard); double ei = 0, eo = 0; bool bad = false;
      for (size_t i = 24000; i < in.size() - 4000; ++i) { ei += (double) in[i] * in[i]; eo += (double) out[i] * out[i]; if (!std::isfinite (out[i])) bad = true; }
      CHECK (!bad && eo / ei > 0.7 && eo / ei < 1.4, "energy ratio %.2f", eo / ei); }

    std::printf ("[10] stereo + odd block sizes are stable\n");
    { for (int blk : { 1, 7, 64, 480, 2048 }) { auto out = run (voice (238, 1.5), hard, blk, 2); bool bad = false; double pk = 0;
        for (float v : out) { if (!std::isfinite (v)) bad = true; pk = std::max (pk, (double) std::fabs (v)); }
        CHECK (!bad && pk < 2.0, "block %d: peak %.2f", blk, pk); } }

    std::printf ("[11] no notes enabled = no correction\n");
    { EngineParams p = hard; for (bool& b : p.notes) b = false; auto out = run (voice (238, 2.0), p); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, 238.0)) < 6.0, "measured %.2f Hz", hz); }

    std::printf ("[12] transpose +12 st doubles the pitch (A3 220 -> A4 440)\n");
    { EngineParams p = hard; p.transpose = 12; auto out = run (voice (220, 2.0), p); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, 440.0)) < 20.0, "measured %.2f Hz", hz); }
    std::printf ("[12b] transpose -12 st halves the pitch (A3 220 -> A2 110)\n");
    { EngineParams p = hard; p.transpose = -12; auto out = run (voice (220, 2.0), p); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, 110.0)) < 25.0, "measured %.2f Hz", hz); }

    std::printf ("[13] detune +30 cents shifts the corrected pitch\n");
    { EngineParams p = hard; p.detuneCents = 30; auto out = run (voice (220, 2.0), p); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, A3) - 30.0) < 8.0, "measured %.2f Hz (%.1f cents above A3)", hz, cents (hz, A3)); }

    std::printf ("[14] tolerance: 30c flat of A3 is left alone with 50c tolerance, fixed with 0c\n");
    { const double inHz = 220.0 * std::pow (2.0, -30.0 / 1200.0);
      EngineParams p = hard; p.tolerance = 50; auto o1 = run (voice (inHz, 2.0), p); double h1 = measureHz (o1, (int) (SR * 1.2), 8192);
      p.tolerance = 0; auto o2 = run (voice (inHz, 2.0), p); double h2 = measureHz (o2, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (h1, inHz)) < 6.0, "tol 50c: %.2f Hz (input %.2f)", h1, inHz);
      CHECK (std::fabs (cents (h2, A3)) < 6.0,   "tol 0c:  %.2f Hz", h2); }

    std::printf ("[17] reference pitch 432: a 216 Hz voice is already A and stays put\n");
    { EngineParams p = hard; p.refHz = 432.0f; auto out = run (voice (216, 2.0), p); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, 216.0)) < 8.0, "measured %.2f Hz", hz); }

    std::printf ("[18] voice range: Mid tracks 238 Hz fine; Low (80-300) fine; High (180+) fine for 238 too\n");
    { for (int r = 0; r < 4; ++r) { EngineParams p = hard; p.range = r; auto out = run (voice (238, 2.0), p); double hz = measureHz (out, (int) (SR * 1.2), 8192);
        const bool inRange = (r != 2 || true);
        CHECK (inRange && std::fabs (cents (hz, B3)) < 10.0, "range %d: %.2f Hz", r, hz); } }
    std::printf ("[18b] a voice below the selected range is ignored (High range 180+ Hz, 100 Hz voice)\n");
    { EngineParams p = hard; p.range = 3; double inHz = 100.0 * std::pow (2.0, 0.35 / 12.0); auto out = run (voice (inHz, 2.0), p); double hz = measureHz (out, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz, inHz)) < 8.0, "input %.1f Hz -> %.1f Hz (untouched)", inHz, hz);
      p.range = 0; auto out2 = run (voice (inHz, 2.0), p); double hz2 = measureHz (out2, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (hz2, 261.6256 / 2.0)) < 10.0 || std::fabs (cents (hz2, 130.8128 / 1.0)) < 10.0 || std::fabs (cents (hz2, inHz)) > 15.0, "Wide range does correct it: %.1f Hz", hz2); }

    std::printf ("[19] gate: a quiet voice is ignored at -48 dB but corrected at -70 dB\n");
    { auto in = voice (238, 2.0); for (auto& v : in) v *= 0.012f;
      EngineParams p = hard; p.gateDb = -48; double h1 = measureHz (run (in, p), (int) (SR * 1.2), 8192);
      p.gateDb = -70; double h2 = measureHz (run (in, p), (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (h1, 238.0)) < 8.0, "gate -48: %.2f Hz (untouched)", h1);
      CHECK (std::fabs (cents (h2, B3)) < 10.0,  "gate -70: %.2f Hz (corrected)", h2); }

    std::printf ("[20] output gain +6 dB doubles the level\n");
    { EngineParams p = hard; p.mix = 0; p.outGainDb = 6; auto in = voice (238, 2.0); auto out = run (in, p);
      double ei = 0, eo = 0; for (size_t i = 48000; i < 90000; ++i) { ei += (double) in[i - 2200] * in[i - 2200]; eo += (double) out[i] * out[i]; }
      CHECK (std::fabs (std::sqrt (eo / ei) - 1.995) < 0.06, "gain ratio %.3f (expect 1.995)", std::sqrt (eo / ei)); }

    // ---------------- key detection ----------------
    auto sing = [&] (const std::vector<int>& scaleMidi, int tonicIndex, double detuneCents, unsigned seed, float* hist, double* voicedSec)
    {
        std::mt19937 rng (seed); std::vector<float> audio;
        std::vector<double> w (scaleMidi.size(), 2.0); w[(size_t) tonicIndex] = 5.0; w[(size_t) ((tonicIndex + 4) % (int) scaleMidi.size())] = 3.0;   // tonic and 5th-ish get sung more
        std::discrete_distribution<int> pick (w.begin(), w.end());
        for (int i = 0; i < 34; ++i)
        {
            const double hz = 440.0 * std::pow (2.0, (scaleMidi[(size_t) pick (rng)] - 69 + detuneCents / 100.0) / 12.0);
            auto v = voice (hz, 0.32); audio.insert (audio.end(), v.begin(), v.end());
            audio.insert (audio.end(), (size_t) (SR * 0.04), 0.0f);
        }
        PitchEngine e; e.prepare (SR, 1); EngineParams p; p.mix = 0; p.collectKey = true;
        std::vector<float> blk (512);
        for (size_t pos = 0; pos < audio.size(); pos += 512)
        { const int cnt = (int) std::min ((size_t) 512, audio.size() - pos); for (int i = 0; i < cnt; ++i) blk[(size_t) i] = audio[pos + (size_t) i]; float* ch[1] = { blk.data() }; e.process (ch, 1, cnt, p); }
        e.getKeyHist (hist); *voicedSec = e.keyVoicedHops.load() * e.getHopSeconds();
        return estimateKey (hist);
    };
    static const char* kn[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
    auto nameOf = [&] (const KeyGuess& g) { return std::string (kn[g.root]) + (g.minor ? " minor" : " major"); };

    std::printf ("[21] key detection: G major melody -> G major / E minor\n");
    { float h[12]; double sec; auto g = sing ({ 55, 57, 59, 60, 62, 64, 66, 67 }, 0, 0.0, 1, h, &sec);
      CHECK (g.valid && g.setId() == 7, "guessed %s (corr %.2f, margin %.2f, %.1f s of singing)", nameOf (g).c_str(), g.corr, g.margin, sec); }

    std::printf ("[22] key detection: D minor melody (tonic-heavy) -> D minor / F major\n");
    { float h[12]; double sec; auto g = sing ({ 50, 52, 53, 55, 57, 58, 60, 62 }, 0, 0.0, 2, h, &sec);
      CHECK (g.valid && g.setId() == 5, "guessed %s (corr %.2f, margin %.2f)", nameOf (g).c_str(), g.corr, g.margin); }

    std::printf ("[23] key detection: F# major (six sharps)\n");
    { float h[12]; double sec; auto g = sing ({ 54, 56, 58, 59, 61, 63, 65, 66 }, 0, 0.0, 3, h, &sec);
      CHECK (g.valid && g.setId() == 6, "guessed %s (corr %.2f, margin %.2f)", nameOf (g).c_str(), g.corr, g.margin); }

    std::printf ("[24] key detection survives a singer who is 35 cents flat all the time (Bb major)\n");
    { float h[12]; double sec; auto g = sing ({ 58, 60, 62, 63, 65, 67, 69, 70 }, 0, -35.0, 4, h, &sec);
      CHECK (g.valid && g.setId() == 10, "guessed %s (corr %.2f, margin %.2f)", nameOf (g).c_str(), g.corr, g.margin); }

    std::printf ("[25] no singing -> no guess; reset request clears the tally\n");
    { float h0[12] = { 0 }; CHECK (! estimateKey (h0).valid, "all-zero histogram is not a guess");
      PitchEngine e; e.prepare (SR, 1); EngineParams p; p.mix = 0; p.collectKey = true; auto v = voice (220, 1.0); std::vector<float> blk (512);
      for (size_t pos = 0; pos + 512 <= v.size(); pos += 512) { for (int i = 0; i < 512; ++i) blk[(size_t) i] = v[pos + (size_t) i]; float* ch[1] = { blk.data() }; e.process (ch, 1, 512, p); }
      const uint32_t before = e.keyVoicedHops.load(); e.requestKeyReset();
      float* ch[1] = { blk.data() }; for (int i = 0; i < 30; ++i) { std::fill (blk.begin(), blk.end(), 0.0f); e.process (ch, 1, 512, p); }
      float h[12]; e.getKeyHist (h); double tot = 0; for (float x : h) tot += x;
      CHECK (before > 100 && e.keyVoicedHops.load() <= 16 && tot <= 16.5, "tally %u hops -> %u after reset (a few hops of audio still in the analysis window)", before, e.keyVoicedHops.load()); }

    // ============================ LOW-LATENCY (LIVE) ENGINE ============================
    static const char* vt[6] = { "Auto", "Soprano", "Alto", "Tenor", "Baritone", "Bass" };

    std::printf ("[30] reported latency of the Live engine is far below 10 ms for every voice type and sample rate\n");
    { double worst = 0;
      for (double rate : { 44100.0, 48000.0, 96000.0 })
      { std::printf ("       %6.0f Hz: ", rate);
        for (int r = 0; r < 6; ++r) { const double ms = 1000.0 * PitchEngine::latencyFor (rate, true, r) / rate; worst = std::max (worst, ms); std::printf ("%s %.1f ms  ", vt[r], ms); }
        std::printf ("\n"); }
      CHECK (worst < 10.0, "worst reported Live latency %.2f ms (Studio mode: Auto %.1f ms, Soprano %.1f ms)", worst,
             1000.0 * PitchEngine::latencyFor (SR, false, 0) / SR, 1000.0 * PitchEngine::latencyFor (SR, false, 1) / SR); }

    std::printf ("[31] Live: dry path is an exact delayed pass-through (mix = 0)\n");
    { EngineParams p = hard; p.live = true; p.mix = 0; int lat = 0; auto in = voice (238, 1.0); auto out = run (in, p, 333, 1, &lat);
      double maxErr = 0; for (size_t i = (size_t) lat + 10; i < in.size(); ++i) maxErr = std::max (maxErr, (double) std::fabs (out[i] - in[i - (size_t) lat]));
      CHECK (maxErr < 1e-5 && lat < 480, "max error %.2e, latency %d samples = %.2f ms", maxErr, lat, 1000.0 * lat / SR); }

    std::printf ("[32] Live: 238 Hz snaps to B3 for every voice type whose range contains it\n");
    for (int r = 0; r < 6; ++r)
    {
        EngineParams p = hard; p.live = true; p.range = r; auto out = run (voice (238, 2.0), p); const double hz = measureHz (out, (int) (SR * 1.2), 8192);
        CHECK (std::fabs (cents (hz, B3)) < 10.0, "%-8s %.2f Hz (%.1f cents from B3)", vt[r], hz, cents (hz, B3));
    }

    std::printf ("[33] Live: transpose +-12 st\n");
    { EngineParams p = hard; p.live = true; p.transpose = 12; auto o1 = run (voice (220, 2.0), p); p.transpose = -12; auto o2 = run (voice (220, 2.0), p);
      const double h1 = measureHz (o1, (int) (SR * 1.2), 8192), h2 = measureHz (o2, (int) (SR * 1.2), 8192);
      CHECK (std::fabs (cents (h1, 440.0)) < 25.0, "+12 st: %.2f Hz", h1);
      CHECK (std::fabs (cents (h2, 110.0)) < 25.0, "-12 st: %.2f Hz", h2); }

    std::printf ("[34] Live: waveform quality - output correlates with an ideal tone at the target pitch (1.0 = perfect)\n");
    { const double shifts[] = { -2.0, -1.0, -0.5, 0.5, 1.0, 2.0 }; double worstQ = 1.0;
      for (double st : shifts)
      { EngineParams p = hard; p.live = true; p.detuneCents = (float) (st * 100.0); p.humanize = 0; p.notes[0] = true;
        for (bool& b : p.notes) b = false;
        p.notes[0] = p.notes[2] = p.notes[4] = p.notes[5] = p.notes[7] = p.notes[9] = p.notes[11] = true;
        const double f0 = 220.0; auto out = run (voice (f0, 3.0), p); const double fT = f0 * std::pow (2.0, st / 12.0);
        auto ideal = voice (fT, 3.0);
        // normalised cross-correlation over a window, searched over +-1 period of lag
        const int st0 = (int) (SR * 1.5), len = 9600, maxLag = (int) (SR / fT) + 2; double best = -1;
        for (int lag = -maxLag; lag <= maxLag; ++lag) { double xy = 0, xx = 0, yy = 0; for (int i = 0; i < len; ++i) { const double a = out[(size_t) (st0 + i)], b = ideal[(size_t) (st0 + i + lag)]; xy += a * b; xx += a * a; yy += b * b; } best = std::max (best, xy / std::sqrt (xx * yy + 1e-18)); }
        std::printf ("       %+.1f st: correlation %.3f\n", st, best); worstQ = std::min (worstQ, best); }
      CHECK (worstQ > 0.90, "worst waveform correlation %.3f", worstQ); }

    std::printf ("[35] Live: the actual delay stays within one pitch period (plus the minimum), also while shifting continuously\n");
    { EngineParams p = hard; p.live = true; p.detuneCents = 40; auto in = voice (238, 6.0);   // a steady detune forces regular splices
      PitchEngine e; e.prepare (SR, 1); std::vector<float> blk (64); double mn = 1e9, mx = 0, sum = 0; long cnt = 0;
      for (size_t pos = 0; pos + 64 <= in.size(); pos += 64)
      { for (int i = 0; i < 64; ++i) blk[(size_t) i] = in[pos + (size_t) i]; float* ch[1] = { blk.data() }; e.process (ch, 1, 64, p);
        if (pos > (size_t) (SR * 1.5)) { const double d = e.getLiveDelaySamples(); mn = std::min (mn, d); mx = std::max (mx, d); sum += d; ++cnt; } }
      const double period = SR / 238.0, dMin = PitchEngine::liveMinDelay (SR);
      std::printf ("       delay range %.2f .. %.2f ms, mean %.2f ms (one period = %.2f ms, reported %.2f ms)\n", 1000 * mn / SR, 1000 * mx / SR, 1000 * sum / cnt / SR, 1000 * period / SR, 1000.0 * PitchEngine::latencyFor (SR, true, 0) / SR);
      CHECK (mx < dMin + period + 8.0 && mx / SR < 0.010 && mn >= dMin - 4.0, "max %.2f ms < 10 ms", 1000 * mx / SR); }

    std::printf ("[36] Live: noise passes through at about unity level, no NaN / inf\n");
    { std::mt19937 rng (1); std::normal_distribution<float> nd (0.f, 0.1f); std::vector<float> in (96000); for (auto& v : in) v = nd (rng);
      EngineParams p = hard; p.live = true; auto out = run (in, p); double ei = 0, eo = 0; bool bad = false;
      for (size_t i = 24000; i < in.size() - 4000; ++i) { ei += (double) in[i] * in[i]; eo += (double) out[i] * out[i]; if (! std::isfinite (out[i])) bad = true; }
      CHECK (! bad && eo / ei > 0.7 && eo / ei < 1.4, "energy ratio %.2f", eo / ei); }

    std::printf ("[37] Live: block sizes 1 / 7 / 64 / 480 / 2048 and stereo behave identically\n");
    { EngineParams p = hard; p.live = true; auto in = voice (238, 1.6); std::vector<float> ref = run (in, p, 64, 2);
      for (int blk : { 1, 7, 480, 2048 }) { auto out = run (in, p, blk, 2); double maxDiff = 0; bool bad = false;
        for (size_t i = 0; i < out.size(); ++i) { if (! std::isfinite (out[i])) bad = true; maxDiff = std::max (maxDiff, (double) std::fabs (out[i] - ref[i])); }
        CHECK (! bad && maxDiff < 1e-4, "block %d: max difference to block-64 output %.1e", blk, maxDiff); } }

    std::printf ("[38] switching Live <-> Studio and between voice types while running never produces NaN\n");
    { PitchEngine e; e.prepare (SR, 1); auto in = voice (238, 3.0); EngineParams p = hard; std::vector<float> blk (256); bool bad = false;
      for (size_t pos = 0, b = 0; pos + 256 <= in.size(); pos += 256, ++b)
      { p.live = ((b / 20) % 2) == 0; p.range = (int) ((b / 7) % 6);
        for (int i = 0; i < 256; ++i) blk[(size_t) i] = in[pos + (size_t) i];
        float* ch[1] = { blk.data() }; e.process (ch, 1, 256, p);
        for (float v : blk) if (! std::isfinite (v) || std::fabs (v) > 4.0f) bad = true; }
      CHECK (! bad, "stable across %d mode/voice switches", 3 * 48000 / 256); }

    // ============================ OCTAVE ERRORS ============================
    auto octStats = [&] (const std::vector<float>& in, double expectedMidi, double fromSec, EngineParams p, bool live)
    {   // fraction of tracker reports that are an octave (or more) away from the truth, fraction within 0.5 st, fraction voiced
        auto tr = detTrace (in, p, live); int tot = 0, oct = 0, good = 0, voicedN = 0;
        for (auto& pr : tr) { if (pr.first < fromSec) continue; ++tot; if (! std::isfinite (pr.second)) continue; ++voicedN;
            const double d = std::fabs (pr.second - expectedMidi); if (d > 6.0) ++oct; if (d < 0.5) ++good; }
        struct R { double octFrac, goodFrac, voicedFrac; }; return R { tot ? (double) oct / tot : 1.0, tot ? (double) good / tot : 0.0, tot ? (double) voicedN / tot : 0.0 }; };

    std::printf ("[40] octave errors: weak fundamental (h1 = 0.1, strong h2-h3) at G3 196 Hz, every voice type that contains it\n");
    { auto in = tone (196.0, 2.5, { 0.1, 1.0, 0.9, 0.6, 0.4, 0.25, 0.15 }); const double m = midiOf (196.0);
      for (int r : { 0, 2, 3, 4, 5 }) { EngineParams p = hard; p.range = r; auto a = octStats (in, m, 0.4, p, false); auto b = octStats (in, m, 0.4, p, true);
        CHECK (a.octFrac < 0.02 && a.goodFrac > 0.95 && b.octFrac < 0.02 && b.goodFrac > 0.95, "%-8s Studio: %.1f%% octave errors, %.1f%% correct | Live: %.1f%% / %.1f%%", vt[r], 100 * a.octFrac, 100 * a.goodFrac, 100 * b.octFrac, 100 * b.goodFrac); } }

    std::printf ("[41] octave errors: 'hollow' tone where the 2nd harmonic dominates (h1 = 0.35, h2 = 1.0) at 165 Hz\n");
    { auto in = tone (165.0, 2.5, { 0.35, 1.0, 0.3, 0.7, 0.2, 0.35 }); const double m = midiOf (165.0); EngineParams p = hard;
      auto a = octStats (in, m, 0.4, p, false), b = octStats (in, m, 0.4, p, true);
      CHECK (a.octFrac < 0.03 && b.octFrac < 0.03, "Studio %.1f%%, Live %.1f%% octave errors (%.0f%% / %.0f%% correct)", 100 * a.octFrac, 100 * b.octFrac, 100 * a.goodFrac, 100 * b.goodFrac); }

    std::printf ("[42] octave errors: noisy voice (about 10 dB signal-to-noise) at D3 147 Hz, Baritone / Tenor ranges\n");
    { auto in = tone (147.0, 3.0, { 1.0, 0.7, 0.5, 0.35, 0.25, 0.15, 0.1 }, 0.09); const double m = midiOf (147.0);
      for (int r : { 3, 4 }) { EngineParams p = hard; p.range = r; auto a = octStats (in, m, 0.4, p, false), b = octStats (in, m, 0.4, p, true);
        CHECK (a.octFrac < 0.03 && b.octFrac < 0.03 && a.goodFrac > 0.8 && b.goodFrac > 0.8, "%-8s Studio %.1f%% oct / %.0f%% ok | Live %.1f%% / %.0f%%", vt[r], 100 * a.octFrac, 100 * a.goodFrac, 100 * b.octFrac, 100 * b.goodFrac); } }

    std::printf ("[43] a real octave leap (A3 -> A4) is followed quickly, but a one-hop glitch is not believed\n");
    { auto lo = voice (220.0, 1.0), hi = voice (440.0, 1.0); std::vector<float> in = lo; in.insert (in.end(), hi.begin(), hi.end());
      auto tr = detTrace (in, hard, true); double followed = -1; for (auto& pr : tr) if (pr.first > 1.0 && std::isfinite (pr.second) && std::fabs (pr.second - 69.0) < 0.5) { followed = pr.first - 1.0; break; }
      CHECK (followed >= 0 && followed < 0.080, "tracker locks onto the new octave %.1f ms after the leap", followed * 1000.0);
      // inject a single 5 ms burst of an octave-up tone into a steady note: tracker output must not jump
      auto steady = voice (220.0, 2.0); auto burst = voice (440.0, 0.005); for (size_t i = 0; i < burst.size(); ++i) steady[(size_t) (SR * 1.0) + i] = burst[i];
      auto tr2 = detTrace (steady, hard, true); int jumps = 0; for (auto& pr : tr2) if (pr.first > 0.4 && std::isfinite (pr.second) && std::fabs (pr.second - 57.0) > 5.0) ++jumps;
      CHECK (jumps == 0, "%d tracker reports jumped an octave during the glitch", jumps); }

    std::printf ("[44] octave errors: glissando A3 -> A4 over one second (no spurious jumps)\n");
    { const int N = (int) SR; std::vector<float> in ((size_t) N); double ph = 0; for (int i = 0; i < N; ++i) { const double f = 220.0 * std::pow (2.0, (double) i / N); ph += 2 * M_PI * f / SR; double v = 0; for (int h = 1; h <= 10; ++h) v += std::sin (h * ph) / h; in[(size_t) i] = (float) (0.25 * v); }
      for (bool live : { false, true }) { auto tr = detTrace (in, hard, live); int bad = 0, tot = 0;
        for (auto& pr : tr) { if (pr.first < 0.1 || pr.first > 0.95) continue; ++tot; const double truth = 57.0 + 12.0 * (pr.first - 0.02); if (std::isfinite (pr.second) && std::fabs (pr.second - truth) > 1.5) ++bad; }
        CHECK ((double) bad / tot < 0.05, "%s: %d of %d reports further than 1.5 st from the true glide", live ? "Live  " : "Studio", bad, tot); } }

    // ---- accuracy + rumble robustness (v1.4) ----
    {
        auto track = [] (double hz, bool live, double rumbleHz, double rumbleAmp, double& rmsCents, double& okPct)
        {
            const double sr = 48000.0; PitchEngine e; e.prepare (sr, 1); EngineParams p; p.live = live;
            std::vector<float> b (256); float* ch[1] = { b.data() }; double ph = 0, rp = 0, lp = 0; unsigned rng = 12345u;
            for (int blk = 0; blk < 500; ++blk)
            {
                for (int i = 0; i < 256; ++i)
                {
                    ph += 6.283185307179586 * hz / sr; rp += 6.283185307179586 * rumbleHz / sr;
                    double v = 0; for (int h = 1; h <= 6; ++h) v += std::sin (h * ph) / h;
                    rng = rng * 1664525u + 1013904223u; const double nz = ((rng >> 8) & 0xffff) / 32768.0 - 1.0;
                    lp += 0.004 * (nz * 1.7 - lp);
                    b[(size_t) i] = 0.2f * (float) v + (float) (rumbleAmp * (std::sin (rp) + 3.6 * lp));
                }
                e.process (ch, 1, 256, p);
            }
            const uint32_t w = e.histWrite.load(); const double tm = 69.0 + 12.0 * std::log2 (hz / 440.0);
            double sq = 0; int cnt = 0;
            for (int i = 0; i < 150; ++i) { const float d = e.histDetected[(w - 1 - i) % PitchEngine::kHistory].load(); if (! std::isnan (d) && std::fabs ((d - tm) * 100.0) < 100.0) { sq += (d - tm) * (d - tm) * 1e4; ++cnt; } }
            rmsCents = cnt ? std::sqrt (sq / cnt) : 999.0; okPct = 100.0 * cnt / 150.0;
        };

        std::printf ("[50] pitch accuracy: clean harmonic tones, every pitch within 0.5 cents rms (Studio and Live)\n");
        double worst = 0;
        for (double hz : { 110.0, 165.0, 220.0, 330.0, 440.0, 660.0 })
        { double a, ok, b2, ok2; track (hz, false, 20.0, 0.0, a, ok); track (hz, true, 20.0, 0.0, b2, ok2); worst = std::max ({ worst, a, b2 }); std::printf ("       %4.0f Hz: %.2f / %.2f cents\n", hz, a, b2); }
        CHECK (worst < 0.5, "worst error %.2f cents rms", worst);

        std::printf ("[51] loud low rumble (30 / 45 Hz, ~8 dB above the voice) does not stop or derail the tracker\n");
        for (double hz : { 130.0, 196.0, 262.0, 350.0 })
            for (double rh : { 30.0, 45.0 })
            { double a, ok, b2, ok2; track (hz, false, rh, 0.5, a, ok); track (hz, true, rh, 0.5, b2, ok2);
              CHECK (ok > 90.0 && ok2 > 90.0 && a < 10.0 && b2 < 10.0, "voice %3.0f Hz + rumble %2.0f Hz: %.1f / %.1f cents, tracked %.0f%% / %.0f%%", hz, rh, a, b2, ok, ok2); }

        std::printf ("[52] rumble alone (no voice) is never taken for a pitch\n");
        for (bool live : { false, true })
        {
            PitchEngine e; e.prepare (48000.0, 1); EngineParams p; p.live = live; std::vector<float> b (256); float* ch[1] = { b.data() }; double lp = 0, rp = 0; unsigned rng = 99u;
            for (int blk = 0; blk < 600; ++blk)
            { for (int i = 0; i < 256; ++i) { rp += 6.283185307179586 * 38.0 / 48000.0; rng = rng * 1664525u + 1013904223u; lp += 0.004 * ((((rng >> 8) & 0xffff) / 32768.0 - 1.0) * 1.7 - lp); b[(size_t) i] = (float) (0.3 * std::sin (rp) + 2.0 * lp); } e.process (ch, 1, 256, p); }
            const uint32_t w = e.histWrite.load(); int v = 0;
            for (int i = 0; i < 200; ++i) if (! std::isnan (e.histDetected[(w - 1 - i) % PitchEngine::kHistory].load())) ++v;
            CHECK (v <= 4, "%s: %d of 200 hops reported a pitch", live ? "Live  " : "Studio", v);
        }
    }

    // ---- v1.5: sticky notes, consonants, confidence, mix, lit notes ----
    {
        struct Probe { std::vector<int> note; std::vector<float> applied; };
        auto probe = [] (const std::vector<float>& in, EngineParams p, int block = 64)
        {
            PitchEngine e; e.prepare (SR, 1); Probe o; std::vector<float> b ((size_t) block); float* ch[1] = { b.data() };
            for (size_t i = 0; i + (size_t) block <= in.size(); i += (size_t) block)
            { std::copy (in.begin () + (long) i, in.begin () + (long) i + block, b.begin()); e.process (ch, 1, block, p); o.note.push_back (e.activeNote.load()); o.applied.push_back (e.getAppliedSemis()); }
            return o;
        };
        auto cMajor = [] (EngineParams p) { const bool cm[12] = { true, false, true, false, true, true, false, true, false, true, false, true }; for (int i = 0; i < 12; ++i) p.notes[i] = cm[i]; return p; };

        std::printf ("[60] A sustained note sitting near the border to the next scale note does not flip back and forth\n");
        { const double hz = 261.6256 * std::pow (2.0, 4.0 / 12.0) * std::pow (2.0, 0.55 / 12.0);   // E4 + 55 cents, F is the next note in C major
          auto in = voice (hz, 2.0, 14.0); EngineParams p = cMajor (EngineParams()); p.live = true; p.retuneMs = 12.0f;
          auto o = probe (in, p); int flips = 0; for (size_t i = (size_t) (0.4 * SR / 64); i + 1 < o.note.size(); ++i) if (o.note[i] >= 0 && o.note[i + 1] >= 0 && o.note[i] != o.note[i + 1]) ++flips;
          CHECK (flips <= 1, "E+55c with 14c vibrato: %d note changes in 1.6 s", flips); }

        std::printf ("[61] Consonant-like noise is never shifted, whatever the retune speed; the vowel before it still is\n");
        { std::vector<float> in = voice (220.0 * std::pow (2.0, -0.4 / 12.0), 0.5); unsigned rng = 7u;
          for (int i = 0; i < (int) (0.12 * SR); ++i) { rng = rng * 1664525u + 1013904223u; in.push_back (0.3f * ((((rng >> 8) & 0xffff) / 32768.0f) - 1.0f)); }
          auto tail = voice (220.0 * std::pow (2.0, -0.4 / 12.0), 0.3); in.insert (in.end(), tail.begin(), tail.end());
          EngineParams p = cMajor (EngineParams()); p.live = true; p.retuneMs = 80.0f; p.humanize = 0.0f; p.tolerance = 0.0f;
          auto o = probe (in, p); const int bpp = 64; float vowel = 0.0f, noise = 0.0f;
          for (int k = (int) (0.35 * SR / bpp); k < (int) (0.48 * SR / bpp); ++k) vowel = std::max (vowel, std::fabs (o.applied[(size_t) k]));
          for (int k = (int) ((0.5 + 0.02) * SR / bpp); k < (int) ((0.5 + 0.11) * SR / bpp); ++k) noise = std::max (noise, std::fabs (o.applied[(size_t) k]));
          CHECK (vowel > 0.2f && noise < 0.03f, "applied correction: vowel %.2f st, 20-110 ms into the noise %.3f st", vowel, noise); }

        std::printf ("[62] Live: Mix scales the correction (no dry copy is blended in), and 0 is a true bypass\n");
        { auto in = voice (220.0 * std::pow (2.0, -0.4 / 12.0), 1.5); EngineParams p = cMajor (EngineParams()); p.live = true; p.retuneMs = 0.0f; p.humanize = 0.0f; p.tolerance = 0.0f; p.amount = 1.0f;
          float v[3]; int idx = 0; for (float m : { 1.0f, 0.5f, 0.0f }) { p.mix = m; auto o = probe (in, p); v[idx++] = o.applied[(size_t) (1.2 * SR / 64)]; }
          CHECK (std::fabs (v[0] - 0.4f) < 0.06f && std::fabs (v[1] - 0.2f) < 0.05f && std::fabs (v[2]) < 0.01f, "applied: mix 100%% %.2f st, 50%% %.2f st, 0%% %.2f st (expect 0.40 / 0.20 / 0.00)", v[0], v[1], v[2]); }

        std::printf ("[63] The engine reports which scale note it is tuning to (lights the keys), and nothing during silence / noise\n");
        { auto in = voice (261.6256 * std::pow (2.0, 4.0 / 12.0) * std::pow (2.0, -0.2 / 12.0), 1.0); std::vector<float> sil ((size_t) (0.4 * SR), 0.0f); in.insert (in.end(), sil.begin(), sil.end());
          EngineParams p = cMajor (EngineParams()); p.live = true; auto o = probe (in, p);
          CHECK (o.note[(size_t) (0.8 * SR / 64)] == 4, "E4 slightly flat -> note %d (4 = E)", o.note[(size_t) (0.8 * SR / 64)]);
          CHECK (o.note.back() == -1, "after the voice stops -> %d", o.note.back()); }
    }

    std::printf ("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
