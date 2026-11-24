#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include "PitchEngine.h"

class PitchlyProcessor : public juce::AudioProcessor,
                         private juce::AsyncUpdater,
                         private juce::AudioProcessorValueTreeState::Listener
{
public:
    PitchlyProcessor();
    ~PitchlyProcessor() override;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    bool isBusesLayoutSupported (const BusesLayout&) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return "Pitchly"; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return {}; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock&) override;
    void setStateInformation (const void*, int) override;

    // ---- parameters ----
    static juce::AudioProcessorValueTreeState::ParameterLayout createLayout();
    static const juce::StringArray& keyNames();
    static const juce::StringArray& scaleNames();       // last entry is "Custom"
    static const juce::StringArray& presetNames();
    static const juce::StringArray& rangeNames();
    static juce::String noteParamId (int pitchClass) { return "note" + juce::String (pitchClass); }

    juce::AudioProcessorValueTreeState apvts;
    pitchly::PitchEngine engine;
    std::atomic<float> inPeak { 0.0f }, outPeak { 0.0f };
    std::atomic<bool>  keyListening { false };     // set by the DETECT KEY button

private:
    void handleAsyncUpdate() override;                                       // keeps the host's latency compensation in step with mode / voice type
    void parameterChanged (const juce::String& id, float) override { if (id == "mode" || id == "range") triggerAsyncUpdate(); }
    std::atomic<float>* pRetune; std::atomic<float>* pHumanize; std::atomic<float>* pAmount;
    std::atomic<float>* pFormant; std::atomic<float>* pMix; std::atomic<float>* pActive;
    std::atomic<float>* pTolerance; std::atomic<float>* pTranspose; std::atomic<float>* pDetune; std::atomic<float>* pReference;
    std::atomic<float>* pGate; std::atomic<float>* pOutGain; std::atomic<float>* pRange; std::atomic<float>* pMode;
    std::atomic<float>* pNote[12];

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (PitchlyProcessor)
};
