// End-to-end headless check of DETECT KEY: sings a G-major melody through the real processor while the real
// editor is "clicked", then reports which key/scale the plugin chose and renders the UI before / during / after.
#include "../Source/PluginProcessor.h"
#include "../Source/PluginEditor.h"
#include <random>

static void save (juce::AudioProcessorEditor& ed, const juce::String& path)
{
    auto img = ed.createComponentSnapshot (ed.getLocalBounds(), true, 1.0f);
    juce::File out (path); out.deleteFile(); juce::FileOutputStream fos (out); juce::PNGImageFormat().writeImageToStream (img, fos);
    std::printf ("wrote %s\n", out.getFullPathName().toRawUTF8());
}

int main (int argc, char** argv)
{
    juce::ScopedJuceInitialiser_GUI init;
    PitchlyProcessor proc;
    proc.setPlayConfigDetails (2, 2, 48000.0, 512);
    proc.prepareToPlay (48000.0, 512);

    std::unique_ptr<juce::AudioProcessorEditor> ed (proc.createEditor());
    ed->setBounds (0, 0, 900, 580);
    Pill* detect = nullptr;
    for (auto* c : ed->getChildren()) if (auto* p = dynamic_cast<Pill*> (c)) if (p->label && p->label().startsWith ("DETECT")) detect = p;
    if (detect == nullptr) { std::printf ("FAIL: no DETECT KEY button found\n"); return 1; }

    const juce::String base = argc > 1 ? argv[1] : "pitchly_key";
    auto pump = [] (int ms) { juce::MessageManager::getInstance()->runDispatchLoopUntil (ms); };
    auto keyName = [&] { return PitchlyProcessor::keyNames()[(int) (proc.apvts.getRawParameterValue ("key")->load() + 0.5f)]
                                + " / " + PitchlyProcessor::scaleNames()[(int) (proc.apvts.getRawParameterValue ("scale")->load() + 0.5f)]; };

    std::printf ("start: %s\n", keyName().toRawUTF8());          // default C major
    pump (100); save (*ed, base + "_0_idle.png");

    // --- a singer in G major, a little out of tune --------------------------------------------------
    std::mt19937 rng (7);
    const int scale[8] = { 55, 57, 59, 60, 62, 64, 66, 67 };      // G A B C D E F# G
    std::discrete_distribution<int> pick ({ 5, 2, 3, 2, 3, 2, 2, 2 });
    juce::MidiBuffer midi; juce::AudioBuffer<float> buf (2, 512);
    double ph = 0.0; double t = 0.0; int sampleInNote = 0, noteLen = 0; double hz = 0.0;
    auto feed = [&] (double seconds)
    {
        const int blocks = (int) (seconds * 48000.0 / 512.0);
        for (int b = 0; b < blocks; ++b)
        {
            for (int i = 0; i < 512; ++i)
            {
                if (sampleInNote >= noteLen) { hz = 440.0 * std::pow (2.0, (scale[pick (rng)] - 69 + 0.18) / 12.0); noteLen = (int) (0.36 * 48000); sampleInNote = 0; }
                const bool gap = sampleInNote > noteLen - (int) (0.04 * 48000);
                ph += 6.283185307 * hz * std::pow (2.0, 0.2 * std::sin (6.283185 * 5.4 * t) / 12.0) / 48000.0;
                float v = 0.0f; for (int h = 1; h <= 10; ++h) v += (float) std::sin (h * ph) / (float) h;
                buf.setSample (0, i, gap ? 0.0f : 0.25f * v); buf.setSample (1, i, gap ? 0.0f : 0.25f * v);
                ++sampleInNote; t += 1.0 / 48000.0;
            }
            proc.processBlock (buf, midi);
        }
    };

    detect->onClick();                                            // user presses DETECT KEY
    std::printf ("listening: %s\n", proc.keyListening.load() ? "yes" : "NO");
    feed (6.0); pump (60); save (*ed, base + "_1_listening.png");
    feed (12.0);                                                  // ~18 s of singing in total
    detect->onClick();                                            // user presses it again to finish
    pump (100);
    std::printf ("after detecting: %s   (listening=%s, %.1f s of singing heard)\n", keyName().toRawUTF8(), proc.keyListening.load() ? "yes" : "no", proc.engine.keyVoicedHops.load() * proc.engine.getHopSeconds());
    std::printf ("allowed notes:");
    for (int i = 0; i < 12; ++i) if (proc.apvts.getRawParameterValue (PitchlyProcessor::noteParamId (i))->load() > 0.5f) std::printf (" %s", PitchlyProcessor::keyNames()[i].toRawUTF8());
    std::printf ("\n");
    save (*ed, base + "_2_done.png");

    // --- latency plumbing: what the host is told in each mode / voice type --------------------------------------------
    auto reportedMs = [&] { return 1000.0 * proc.getLatencySamples() / 48000.0; };
    pump (150);
    std::printf ("host latency, Live / Auto:        %.2f ms\n", reportedMs());
    Pill* mode = nullptr;
    for (auto* c : ed->getChildren()) if (auto* p = dynamic_cast<Pill*> (c)) if (p->label && p != detect) mode = p;   // the other dynamic-label pill = LIVE/STUDIO
    std::vector<TabButton*> tabs; for (auto* c : ed->getChildren()) if (auto* t = dynamic_cast<TabButton*> (c)) tabs.push_back (t);
    if (mode == nullptr || tabs.size() < 2) { std::printf ("FAIL: mode pill / tabs not found\n"); return 1; }
    for (int voice = 1; voice <= 5; ++voice)
    {
        proc.apvts.getParameter ("range")->setValueNotifyingHost (proc.apvts.getParameter ("range")->convertTo0to1 ((float) voice)); pump (120);
        std::printf ("host latency, Live / %-9s %.2f ms\n", (PitchlyProcessor::rangeNames()[voice] + ":").toRawUTF8(), reportedMs());
    }
    proc.apvts.getParameter ("range")->setValueNotifyingHost (proc.apvts.getParameter ("range")->convertTo0to1 (3.0f)); pump (120);   // Tenor for the screenshots
    tabs[1]->onClick(); pump (60); save (*ed, base + "_3_shift_live.png");
    mode->onClick(); pump (150);                                                  // switch to Studio
    std::printf ("host latency, Studio / Tenor:     %.2f ms\n", reportedMs());
    tabs[0]->onClick(); pump (60); save (*ed, base + "_4_studio.png");
    // --- the voice is being tuned to a note: its key lights up ---
    mode->onClick(); pump (150);                                                  // back to Live
    feed (1.5); pump (80); save (*ed, base + "_5_lit.png");
    std::printf ("engine says it is tuning to pitch class %d\n", proc.engine.activeNote.load());
    ed.reset();
    return 0;
}
