// 
// Scrungler - True Stereo Reverb
// Created by Elliot Alomoto 4/9/26
//

#include "fmod.hpp"

#include <math.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

// dB to linear conversion helpers
#define DECIBELS_TO_LINEAR(x) powf(10.0f, (x) / 20.0f)
#define LINEAR_TO_DECIBELS(x) (x) <= 0.0f ? -80.0f : 20.0f * log10f(x)

extern "C"
{
    F_EXPORT FMOD_DSP_DESCRIPTION* F_CALL FMODGetDSPDescription();
}

// ==================== //
// CALLBACK DEFINITIONS //
// ==================== //
FMOD_RESULT Create_Callback(FMOD_DSP_STATE* dsp_state);
FMOD_RESULT Release_Callback(FMOD_DSP_STATE* dsp_state);
FMOD_RESULT Reset_Callback(FMOD_DSP_STATE* dsp_state);
FMOD_RESULT Read_Callback(FMOD_DSP_STATE* dsp_state, float* inbuffer, float* outbuffer, unsigned int length, int inchannels, int* outchannels);
FMOD_RESULT Process_Callback(FMOD_DSP_STATE* dsp_state, unsigned int length, const FMOD_DSP_BUFFER_ARRAY* inbufferarray, FMOD_DSP_BUFFER_ARRAY* outbufferarray, FMOD_BOOL inputsidle, FMOD_DSP_PROCESS_OPERATION op);
FMOD_RESULT SetPosition_Callback(FMOD_DSP_STATE* dsp_state, unsigned int pos);
FMOD_RESULT ShouldIProcess_Callback(FMOD_DSP_STATE* dsp_state, FMOD_BOOL inputsidle, unsigned int length, FMOD_CHANNELMASK inmask, int inchannels, FMOD_SPEAKERMODE speakermode);

FMOD_RESULT SetFloat_Callback(FMOD_DSP_STATE* dsp_state, int index, float value);
FMOD_RESULT SetInt_Callback(FMOD_DSP_STATE* dsp_state, int index, int value);
FMOD_RESULT SetBool_Callback(FMOD_DSP_STATE* dsp_state, int index, FMOD_BOOL value);
FMOD_RESULT SetData_Callback(FMOD_DSP_STATE* dsp_state, int index, void* data, unsigned int length);
FMOD_RESULT GetFloat_Callback(FMOD_DSP_STATE* dsp_state, int index, float* value, char* valuestr);
FMOD_RESULT GetInt_Callback(FMOD_DSP_STATE* dsp_state, int index, int* value, char* valuestr);
FMOD_RESULT GetBool_Callback(FMOD_DSP_STATE* dsp_state, int index, FMOD_BOOL* value, char* valuestr);
FMOD_RESULT GetData_Callback(FMOD_DSP_STATE* dsp_state, int index, void** data, unsigned int* length, char* valuestr);

FMOD_RESULT SystemRegister_Callback(FMOD_DSP_STATE* dsp_state);
FMOD_RESULT SystemDeregister_Callback(FMOD_DSP_STATE* dsp_state);
FMOD_RESULT SystemMix_Callback(FMOD_DSP_STATE* dsp_state, int stage);

// ++++++++++++++++++
//    Parameters 
//+++++++++++++++++++

enum
{
    PARAM_INPUT_DIFFUSE_1 = 0,
    PARAM_INPUT_DIFFUSE_2,
    PARAM_DECAY_DIFFUSE_1,
    PARAM_DECAY_DIFFUSE_2,
    PARAM_BANDWIDTH,
    PARAM_DECAY,
    PARAM_DRY,
    PARAM_WET,
    PARAM_PRE_DELAY,
    NUM_PARAMS
};

// Declarations 
static FMOD_DSP_PARAMETER_DESC p_inputDiffuse1, p_inputDiffuse2,
p_decayDiffuse1, p_decayDiffuse2, p_bandwidth,
p_decay, p_dry, p_wet, p_preDelay;

// Pointer Array
static FMOD_DSP_PARAMETER_DESC* PluginsParameters[NUM_PARAMS] =
{
    &p_inputDiffuse1,
    &p_inputDiffuse2,
    &p_decayDiffuse1,
    &p_decayDiffuse2,
    &p_bandwidth,
    &p_decay,
    &p_dry,
    &p_wet,
    &p_preDelay
};

// DSP Description
static FMOD_DSP_DESCRIPTION PluginCallbacks =
{
    FMOD_PLUGIN_SDK_VERSION,    // version
    "Scrungler Reverb",         // name
    0x00010000,                 // plugin version
    1,                          // no. input buffers
    1,                          // no. output buffers
    Create_Callback,            // create
    Release_Callback,           // release
    Reset_Callback,             // reset
    Read_Callback,              // read
    Process_Callback,           // process
    SetPosition_Callback,       // setposition
    NUM_PARAMS,                 // no. parameter 
    PluginsParameters,          // pointer to parameter descriptions
    SetFloat_Callback,          // Set float
    SetInt_Callback,            // Set int
    SetBool_Callback,           // Set bool
    SetData_Callback,           // Set data
    GetFloat_Callback,          // Get float
    GetInt_Callback,            // Get int
    GetBool_Callback,           // Get bool
    GetData_Callback,           // Get data
    ShouldIProcess_Callback,    // Check states before processing
    0,                          // User data
    SystemRegister_Callback,    // System register
    SystemDeregister_Callback,  // System deregister
    SystemMix_Callback          // Mixer thread execute / after execute
};

// ++++++++++++++++++
//    Plugin Class (Processing) 
//+++++++++++++++++++

// Delay Buffer
class DelayBuffer
{
public:
    DelayBuffer(int size) : m_buffer(size, 0.0f) {}
    float& operator[](int index) { return m_buffer[index]; }
    const float& operator[](int index) const { return m_buffer[index]; }
    void Clear()
    {
        for (size_t i = 0; i < m_buffer.size(); i++) m_buffer[i] = 0.0f;
    }

private:
    std::vector<float> m_buffer;
};

// Delay Unit
class DelayUnit {
public:
    DelayUnit();
    ~DelayUnit();

    void Init(FMOD_DSP_STATE* dsp_state, int maxSamples);
    void CreateBuffers(int channels);
    void SetDelayTime(float ms);
    void SetDelaySamples(int samples);
    void SetFeedback(float feedback);
    float GetDelayTime() const;
    void WriteDelay(float value);
    float GetDelayedSample();
    float GetDelayedSampleAt(int sampleOffset);
    void TickChannel();
    void Clear();

private:
    DelayBuffer* m_delayBuffer;
    int m_writePos;
    float m_delayTime;
    int m_delaySamples;
    float m_feedbackAmount;
    int m_sampleRate;
    int m_maxSamples;
    int m_numChannels;
};

// Delay Unit Implementation 
DelayUnit::DelayUnit()
    : m_delayBuffer(nullptr)
    , m_writePos(0)
    , m_delayTime(10.0f)
    , m_delaySamples(0)
    , m_feedbackAmount(0.0f)
    , m_sampleRate(44100)
    , m_maxSamples(0)
    , m_numChannels(-1)
{
}

DelayUnit::~DelayUnit()
{
    delete m_delayBuffer;
}

void DelayUnit::Init(FMOD_DSP_STATE* dsp_state, int maxSamples)
{
    FMOD_DSP_GETSAMPLERATE(dsp_state, &m_sampleRate);
    m_maxSamples = maxSamples;
    m_writePos = 0;
    m_numChannels = -1;
}

void DelayUnit::CreateBuffers(int channels)
{
    if (m_numChannels != channels && m_maxSamples > 0)
    {
        m_numChannels = channels;
        delete m_delayBuffer;
        m_delayBuffer = new DelayBuffer(m_maxSamples * channels);
        m_writePos = 0;
    }
}

void DelayUnit::SetDelayTime(float ms)
{
    m_delayTime = ms;
    m_delaySamples = (int)(ms * 0.001f * m_sampleRate);
}

void DelayUnit::SetDelaySamples(int samples)
{
    m_delaySamples = samples;
    m_delayTime = (float)samples * 1000.0f / (float)m_sampleRate;
}

void DelayUnit::SetFeedback(float feedback)
{
    m_feedbackAmount = feedback;
}

float DelayUnit::GetDelayTime() const
{
    return m_delayTime;
}

void DelayUnit::WriteDelay(float value)
{
    if (m_delayBuffer)
    {
        (*m_delayBuffer)[m_writePos] = value;
    }
}

float DelayUnit::GetDelayedSample()
{
    if (!m_delayBuffer || m_numChannels <= 0) return 0.0f;

    int readPos = m_writePos - (m_delaySamples * m_numChannels);

    while (readPos < 0) readPos += (m_maxSamples * m_numChannels);
    while (readPos >= (m_maxSamples * m_numChannels)) readPos -= (m_maxSamples * m_numChannels);

    float sample = (*m_delayBuffer)[readPos];

    // Safety limiter
    if (sample > 1.0f) sample = 1.0f;
    if (sample < -1.0f) sample = -1.0f;

    return sample;
}

float DelayUnit::GetDelayedSampleAt(int sampleOffset)
{
    if (!m_delayBuffer || m_numChannels <= 0) return 0.0f;

    int readPos = m_writePos - (sampleOffset * m_numChannels);

    while (readPos < 0) readPos += (m_maxSamples * m_numChannels);
    while (readPos >= (m_maxSamples * m_numChannels)) readPos -= (m_maxSamples * m_numChannels);

    return (*m_delayBuffer)[readPos];
}

void DelayUnit::TickChannel()
{
    if (!m_delayBuffer) return;

    m_writePos++;
    int bufferSize = m_maxSamples * m_numChannels;
    if (m_writePos >= bufferSize) m_writePos = 0;
}

void DelayUnit::Clear()
{
    if (m_delayBuffer)
    {
        m_delayBuffer->Clear();
    }
    m_writePos = 0;
}

// ++++++++++++++++++
//    Main Plugin Class - True Stereo Reverb
//+++++++++++++++++++

class Plugin
{
public:
    Plugin();
    ~Plugin();

    void Init(FMOD_DSP_STATE* dsp_state);
    void Release();
    void Process(float* inbuffer, float* outbuffer, unsigned int length, int channels);
    void SetParameterFloat(int index, float value);
    void GetParameterFloat(int index, float* value);

private:
    // Delay lines for reverb
    DelayUnit* m_predelay;
    DelayUnit* m_diffuseDelay11;
    DelayUnit* m_diffuseDelay12;
    DelayUnit* m_diffuseDelay21;
    DelayUnit* m_diffuseDelay22;
    DelayUnit* m_reverbDiffuse1;
    DelayUnit* m_reverbDiffuse2;
    DelayUnit* m_reverbDelay1;
    DelayUnit* m_reverbDelay2;
    DelayUnit* m_reverbDelay3;
    DelayUnit* m_reverbDelay4;

    // Parameters
    float m_inputDiffuse1;
    float m_inputDiffuse2;
    float m_decayDiffuse1;
    float m_decayDiffuse2;
    float m_bandwidth;
    float m_decay;
    float m_dry;
    float m_wet;
    float m_preDelay;

    bool m_initialized;
};

// Plugin Implementation  
Plugin::Plugin()
    : m_predelay(nullptr)
    , m_diffuseDelay11(nullptr)
    , m_diffuseDelay12(nullptr)
    , m_diffuseDelay21(nullptr)
    , m_diffuseDelay22(nullptr)
    , m_reverbDiffuse1(nullptr)
    , m_reverbDiffuse2(nullptr)
    , m_reverbDelay1(nullptr)
    , m_reverbDelay2(nullptr)
    , m_reverbDelay3(nullptr)
    , m_reverbDelay4(nullptr)
    , m_inputDiffuse1(0.5f)
    , m_inputDiffuse2(0.5f)
    , m_decayDiffuse1(0.5f)
    , m_decayDiffuse2(0.5f)
    , m_bandwidth(0.5f)
    , m_decay(0.5f)
    , m_dry(1.0f)
    , m_wet(0.3f)
    , m_preDelay(10.0f)
    , m_initialized(false)
{
}

Plugin::~Plugin()
{
    Release();
}

void Plugin::Init(FMOD_DSP_STATE* dsp_state)
{
    if (m_initialized) return;

    // Create all delay units
    m_predelay = new DelayUnit();
    m_diffuseDelay11 = new DelayUnit();
    m_diffuseDelay12 = new DelayUnit();
    m_diffuseDelay21 = new DelayUnit();
    m_diffuseDelay22 = new DelayUnit();
    m_reverbDiffuse1 = new DelayUnit();
    m_reverbDiffuse2 = new DelayUnit();
    m_reverbDelay1 = new DelayUnit();
    m_reverbDelay2 = new DelayUnit();
    m_reverbDelay3 = new DelayUnit();
    m_reverbDelay4 = new DelayUnit();

    // Initialize with max delays (in samples at 44.1kHz)
    m_predelay->Init(dsp_state, 88200);           // 2 seconds max pre-delay
    m_diffuseDelay11->Init(dsp_state, 2000);      // Diffusion delays (short)
    m_diffuseDelay12->Init(dsp_state, 2000);
    m_diffuseDelay21->Init(dsp_state, 2000);
    m_diffuseDelay22->Init(dsp_state, 2000);
    m_reverbDiffuse1->Init(dsp_state, 2000);
    m_reverbDiffuse2->Init(dsp_state, 2000);
    m_reverbDelay1->Init(dsp_state, 88200);       // Long reverb delays
    m_reverbDelay2->Init(dsp_state, 88200);
    m_reverbDelay3->Init(dsp_state, 88200);
    m_reverbDelay4->Init(dsp_state, 88200);

    // Set fixed delay times (classic reverb values)
    m_diffuseDelay11->SetDelaySamples(143);
    m_diffuseDelay12->SetDelaySamples(108);
    m_diffuseDelay21->SetDelaySamples(380);
    m_diffuseDelay22->SetDelaySamples(278);
    m_reverbDiffuse1->SetDelaySamples(673);
    m_reverbDiffuse2->SetDelaySamples(909);
    m_reverbDelay1->SetDelaySamples(4454);
    m_reverbDelay2->SetDelaySamples(4217);
    m_reverbDelay3->SetDelaySamples(3721);
    m_reverbDelay4->SetDelaySamples(3164);

    // Set feedback amounts
    m_diffuseDelay11->SetFeedback(0.0f);
    m_diffuseDelay12->SetFeedback(0.0f);
    m_diffuseDelay21->SetFeedback(0.0f);
    m_diffuseDelay22->SetFeedback(0.0f);
    m_reverbDiffuse1->SetFeedback(0.0f);
    m_reverbDiffuse2->SetFeedback(0.0f);

    m_initialized = true;
}

void Plugin::Release()
{
    delete m_predelay; m_predelay = nullptr;
    delete m_diffuseDelay11; m_diffuseDelay11 = nullptr;
    delete m_diffuseDelay12; m_diffuseDelay12 = nullptr;
    delete m_diffuseDelay21; m_diffuseDelay21 = nullptr;
    delete m_diffuseDelay22; m_diffuseDelay22 = nullptr;
    delete m_reverbDiffuse1; m_reverbDiffuse1 = nullptr;
    delete m_reverbDiffuse2; m_reverbDiffuse2 = nullptr;
    delete m_reverbDelay1; m_reverbDelay1 = nullptr;
    delete m_reverbDelay2; m_reverbDelay2 = nullptr;
    delete m_reverbDelay3; m_reverbDelay3 = nullptr;
    delete m_reverbDelay4; m_reverbDelay4 = nullptr;

    m_initialized = false;
}

void Plugin::Process(float* inbuffer, float* outbuffer, unsigned int length, int channels)
{
    if (!m_initialized) return;
    if (channels < 1) return;

    // Create buffers for all delay units
    m_predelay->CreateBuffers(channels);
    m_diffuseDelay11->CreateBuffers(channels);
    m_diffuseDelay12->CreateBuffers(channels);
    m_diffuseDelay21->CreateBuffers(channels);
    m_diffuseDelay22->CreateBuffers(channels);
    m_reverbDiffuse1->CreateBuffers(channels);
    m_reverbDiffuse2->CreateBuffers(channels);
    m_reverbDelay1->CreateBuffers(channels);
    m_reverbDelay2->CreateBuffers(channels);
    m_reverbDelay3->CreateBuffers(channels);
    m_reverbDelay4->CreateBuffers(channels);

    // Update pre-delay time
    m_predelay->SetDelayTime(m_preDelay);

    for (unsigned int i = 0; i < length; i++)
    {
        for (int ch = 0; ch < channels; ch++)
        {
            float input = *inbuffer;
            float wetSignal = 0.0f;

            // === PRE-DELAY ===
            float preDelayed = m_predelay->GetDelayedSample();
            m_predelay->WriteDelay(input);
            m_predelay->TickChannel();

            // === INPUT DIFFUSION NETWORK ===
            // Diffuse stage 1
            float diff1In = preDelayed;
            float diff1Out = m_diffuseDelay11->GetDelayedSample();
            float diff1Top = (-diff1Out * m_inputDiffuse1) + diff1In;
            float diff1Bottom = diff1Out + (diff1Top * m_inputDiffuse1);
            m_diffuseDelay11->WriteDelay(diff1Top);
            m_diffuseDelay11->TickChannel();

            // Diffuse stage 2
            float diff2In = diff1Bottom;
            float diff2Out = m_diffuseDelay12->GetDelayedSample();
            float diff2Top = (-diff2Out * m_inputDiffuse1) + diff2In;
            float diff2Bottom = diff2Out + (diff2Top * m_inputDiffuse1);
            m_diffuseDelay12->WriteDelay(diff2Top);
            m_diffuseDelay12->TickChannel();

            // Diffuse stage 3
            float diff3In = diff2Bottom;
            float diff3Out = m_diffuseDelay21->GetDelayedSample();
            float diff3Top = (-diff3Out * m_inputDiffuse2) + diff3In;
            float diff3Bottom = diff3Out + (diff3Top * m_inputDiffuse2);
            m_diffuseDelay21->WriteDelay(diff3Top);
            m_diffuseDelay21->TickChannel();

            // Diffuse stage 4
            float diff4In = diff3Bottom;
            float diff4Out = m_diffuseDelay22->GetDelayedSample();
            float diff4Top = (-diff4Out * m_inputDiffuse2) + diff4In;
            float diff4Bottom = diff4Out + (diff4Top * m_inputDiffuse2);
            m_diffuseDelay22->WriteDelay(diff4Top);
            m_diffuseDelay22->TickChannel();

            // === REVERB NETWORK ===
            float reverbIn = diff4Bottom;

            // Left channel reverb path
            float revDiff1In = reverbIn + (m_reverbDelay4->GetDelayedSample() * m_decay);
            float revDiff1Out = m_reverbDiffuse1->GetDelayedSample();
            float revDiff1Top = (revDiff1Out * m_decayDiffuse1) + revDiff1In;
            float revDiff1Bottom = (-revDiff1Top * m_decayDiffuse1) + revDiff1Out;
            m_reverbDiffuse1->WriteDelay(revDiff1Top);
            m_reverbDiffuse1->TickChannel();

            float revDelay1Out = m_reverbDelay1->GetDelayedSample();
            float revDelay1Filtered = revDelay1Out * m_bandwidth;
            m_reverbDelay1->WriteDelay(revDiff1Bottom);
            m_reverbDelay1->TickChannel();

            // Right channel reverb path
            float revDiff2In = reverbIn + (revDelay1Filtered * m_decay);
            float revDiff2Out = m_reverbDiffuse2->GetDelayedSample();
            float revDiff2Top = (revDiff2Out * m_decayDiffuse2) + revDiff2In;
            float revDiff2Bottom = (-revDiff2Top * m_decayDiffuse2) + revDiff2Out;
            m_reverbDiffuse2->WriteDelay(revDiff2Top);
            m_reverbDiffuse2->TickChannel();

            float revDelay2Out = m_reverbDelay2->GetDelayedSample();
            float revDelay2Filtered = revDelay2Out * m_bandwidth;
            m_reverbDelay2->WriteDelay(revDiff2Bottom);
            m_reverbDelay2->TickChannel();

            // Second diffusion stage
            float revDiff3In = revDelay2Filtered;
            float revDiff3Out = m_reverbDelay3->GetDelayedSample();
            float revDelay3Filtered = revDiff3Out * m_bandwidth;
            m_reverbDelay3->WriteDelay(revDiff3In);
            m_reverbDelay3->TickChannel();

            float revDiff4In = revDelay3Filtered;
            float revDiff4Out = m_reverbDelay4->GetDelayedSample();
            float revDelay4Filtered = revDiff4Out * m_bandwidth;
            m_reverbDelay4->WriteDelay(revDiff4In);
            m_reverbDelay4->TickChannel();

            wetSignal = revDelay4Filtered;

            // === OUTPUT MIX ===
            float output = (input * m_dry) + (wetSignal * m_wet);

            // Soft limit to prevent clipping
            if (output > 0.95f) output = 0.95f;
            if (output < -0.95f) output = -0.95f;

            *outbuffer = output;

            inbuffer++;
            outbuffer++;
        }
    }
}

void Plugin::SetParameterFloat(int index, float value)
{
    switch (index)
    {
    case PARAM_DRY:
        m_dry = DECIBELS_TO_LINEAR(value);
        m_dry = (m_dry < 0.0f) ? 0.0f : (m_dry > 1.0f) ? 1.0f : m_dry;
        break;

    case PARAM_WET:
        m_wet = DECIBELS_TO_LINEAR(value);
        m_wet = (m_wet < 0.0f) ? 0.0f : (m_wet > 1.0f) ? 1.0f : m_wet;
        break;

    case PARAM_DECAY:
        m_decay = (value < 0.0f) ? 0.0f : (value > 0.95f) ? 0.95f : value;
        break;

    case PARAM_PRE_DELAY:
        m_preDelay = (value < 0.0f) ? 0.0f : (value > 2000.0f) ? 2000.0f : value;
        break;

    case PARAM_BANDWIDTH:
        m_bandwidth = (value < 0.0f) ? 0.0f : (value > 1.0f) ? 1.0f : value;
        break;

    case PARAM_INPUT_DIFFUSE_1:
        m_inputDiffuse1 = (value < 0.0f) ? 0.0f : (value > 1.0f) ? 1.0f : value;
        break;

    case PARAM_INPUT_DIFFUSE_2:
        m_inputDiffuse2 = (value < 0.0f) ? 0.0f : (value > 1.0f) ? 1.0f : value;
        break;

    case PARAM_DECAY_DIFFUSE_1:
        m_decayDiffuse1 = (value < 0.0f) ? 0.0f : (value > 1.0f) ? 1.0f : value;
        break;

    case PARAM_DECAY_DIFFUSE_2:
        m_decayDiffuse2 = (value < 0.0f) ? 0.0f : (value > 1.0f) ? 1.0f : value;
        break;

    default: break;
    }
}

void Plugin::GetParameterFloat(int index, float* value)
{
    switch (index)
    {
    case PARAM_DRY:
        *value = LINEAR_TO_DECIBELS(m_dry);
        break;

    case PARAM_WET:
        *value = LINEAR_TO_DECIBELS(m_wet);
        break;

    case PARAM_DECAY:
        *value = m_decay;
        break;

    case PARAM_PRE_DELAY:
        *value = m_preDelay;
        break;

    case PARAM_BANDWIDTH:
        *value = m_bandwidth;
        break;

    case PARAM_INPUT_DIFFUSE_1:
        *value = m_inputDiffuse1;
        break;

    case PARAM_INPUT_DIFFUSE_2:
        *value = m_inputDiffuse2;
        break;

    case PARAM_DECAY_DIFFUSE_1:
        *value = m_decayDiffuse1;
        break;

    case PARAM_DECAY_DIFFUSE_2:
        *value = m_decayDiffuse2;
        break;

    default:
        *value = 0.0f;
        break;
    }
}

// ++++++++++++++++++
//    Callback Implementations 
//+++++++++++++++++++

FMOD_RESULT Create_Callback(FMOD_DSP_STATE* dsp_state)
{
    Plugin* plugin = new Plugin();
    plugin->Init(dsp_state);
    dsp_state->plugindata = plugin;
    return FMOD_OK;
}

FMOD_RESULT Release_Callback(FMOD_DSP_STATE* dsp_state)
{
    Plugin* plugin = (Plugin*)dsp_state->plugindata;
    delete plugin;
    return FMOD_OK;
}

FMOD_RESULT Reset_Callback(FMOD_DSP_STATE* dsp_state)
{
    return FMOD_OK;
}

FMOD_RESULT Read_Callback(FMOD_DSP_STATE* dsp_state, float* inbuffer, float* outbuffer, unsigned int length, int inchannels, int* outchannels)
{
    return FMOD_OK;
}

FMOD_RESULT Process_Callback(FMOD_DSP_STATE* dsp_state, unsigned int length, const FMOD_DSP_BUFFER_ARRAY* inbufferarray, FMOD_DSP_BUFFER_ARRAY* outbufferarray,
    FMOD_BOOL inputsidle, FMOD_DSP_PROCESS_OPERATION op)
{
    if (op == FMOD_DSP_PROCESS_PERFORM)
    {
        Plugin* plugin = (Plugin*)dsp_state->plugindata;
        int channels = *outbufferarray[0].buffernumchannels;
        plugin->Process(inbufferarray[0].buffers[0], outbufferarray[0].buffers[0], length, channels);
    }
    return FMOD_OK;
}

FMOD_RESULT SetPosition_Callback(FMOD_DSP_STATE* dsp_state, unsigned int pos)
{
    return FMOD_OK;
}

FMOD_RESULT ShouldIProcess_Callback(FMOD_DSP_STATE* dsp_state, FMOD_BOOL inputsidle, unsigned int length, FMOD_CHANNELMASK inmask, int inchannels,
    FMOD_SPEAKERMODE speakermode)
{
    return FMOD_OK;
}

FMOD_RESULT SetFloat_Callback(FMOD_DSP_STATE* dsp_state, int index, float value)
{
    Plugin* plugin = (Plugin*)dsp_state->plugindata;
    plugin->SetParameterFloat(index, value);
    return FMOD_OK;
}

FMOD_RESULT SetInt_Callback(FMOD_DSP_STATE* dsp_state, int index, int value)
{
    return FMOD_OK;
}

FMOD_RESULT SetBool_Callback(FMOD_DSP_STATE* dsp_state, int index, FMOD_BOOL value)
{
    return FMOD_OK;
}

FMOD_RESULT SetData_Callback(FMOD_DSP_STATE* dsp_state, int index, void* data, unsigned int length)
{
    return FMOD_OK;
}

FMOD_RESULT GetFloat_Callback(FMOD_DSP_STATE* dsp_state, int index, float* value, char* valuestr)
{
    Plugin* plugin = (Plugin*)dsp_state->plugindata;
    plugin->GetParameterFloat(index, value);
    return FMOD_OK;
}

FMOD_RESULT GetInt_Callback(FMOD_DSP_STATE* dsp_state, int index, int* value, char* valuestr)
{
    return FMOD_OK;
}

FMOD_RESULT GetBool_Callback(FMOD_DSP_STATE* dsp_state, int index, FMOD_BOOL* value, char* valuestr)
{
    return FMOD_OK;
}

FMOD_RESULT GetData_Callback(FMOD_DSP_STATE* dsp_state, int index, void** data, unsigned int* length, char* valuestr)
{
    return FMOD_OK;
}

FMOD_RESULT SystemRegister_Callback(FMOD_DSP_STATE* dsp_state)
{
    return FMOD_OK;
}

FMOD_RESULT SystemDeregister_Callback(FMOD_DSP_STATE* dsp_state)
{
    return FMOD_OK;
}

FMOD_RESULT SystemMix_Callback(FMOD_DSP_STATE* dsp_state, int stage)
{
    return FMOD_OK;
}

// ++++++++++++++++++
//    FMODGetDSPDescription
//+++++++++++++++++++

extern "C"
{
    F_EXPORT FMOD_DSP_DESCRIPTION* F_CALL FMODGetDSPDescription()
    {
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_inputDiffuse1, "Input Diffuse 1", "", "Input diffusion amount - creates density", 0.0f, 1.0f, 0.5f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_inputDiffuse2, "Input Diffuse 2", "", "Input diffusion amount - creates density", 0.0f, 1.0f, 0.5f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_decayDiffuse1, "Decay Diffuse 1", "", "Decay diffusion - smears the reverb tail", 0.0f, 1.0f, 0.5f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_decayDiffuse2, "Decay Diffuse 2", "", "Decay diffusion - smears the reverb tail", 0.0f, 1.0f, 0.5f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_bandwidth, "Bandwidth", "", "Tone control - lower = darker reverb", 0.0f, 1.0f, 0.5f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_decay, "Decay", "", "Reverb decay time / feedback", 0.0f, 0.95f, 0.5f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_dry, "Dry", "dB", "Dry volume", -80.0f, 10.0f, 0.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_wet, "Wet", "dB", "Wet volume", -80.0f, 10.0f, -6.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_preDelay, "Pre Delay", "ms", "Time before reverb starts", 0.0f, 500.0f, 10.0f);

        return &PluginCallbacks;
    }
}