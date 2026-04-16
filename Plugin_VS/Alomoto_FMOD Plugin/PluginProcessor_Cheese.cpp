// 
// Scrungler - Multi-Tap Delay Plugin (No Feedback Loop)
// Created by Elliot Alomoto 4/9/26
//

#include "fmod.hpp"
#include <math.h>
#include <vector>

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
FMOD_RESULT Process_Callback(FMOD_DSP_STATE* dsp_state, unsigned int length, const FMOD_DSP_BUFFER_ARRAY* inbufferarray, FMOD_DSP_BUFFER_ARRAY* outbufferarray, FMOD_BOOL inputsidle, FMOD_DSP_PROCESS_OPERATION op);
FMOD_RESULT SetFloat_Callback(FMOD_DSP_STATE* dsp_state, int index, float value);
FMOD_RESULT GetFloat_Callback(FMOD_DSP_STATE* dsp_state, int index, float* value, char* valuestr);
FMOD_RESULT SetPosition_Callback(FMOD_DSP_STATE* dsp_state, unsigned int pos);
FMOD_RESULT ShouldIProcess_Callback(FMOD_DSP_STATE* dsp_state, FMOD_BOOL inputsidle, unsigned int length, FMOD_CHANNELMASK inmask, int inchannels, FMOD_SPEAKERMODE speakermode);
FMOD_RESULT SetInt_Callback(FMOD_DSP_STATE* dsp_state, int index, int value) { return FMOD_OK; }
FMOD_RESULT SetBool_Callback(FMOD_DSP_STATE* dsp_state, int index, FMOD_BOOL value) { return FMOD_OK; }
FMOD_RESULT SetData_Callback(FMOD_DSP_STATE* dsp_state, int index, void* data, unsigned int length) { return FMOD_OK; }
FMOD_RESULT GetInt_Callback(FMOD_DSP_STATE* dsp_state, int index, int* value, char* valuestr) { return FMOD_OK; }
FMOD_RESULT GetBool_Callback(FMOD_DSP_STATE* dsp_state, int index, FMOD_BOOL* value, char* valuestr) { return FMOD_OK; }
FMOD_RESULT GetData_Callback(FMOD_DSP_STATE* dsp_state, int index, void** data, unsigned int* length, char* valuestr) { return FMOD_OK; }
FMOD_RESULT SystemRegister_Callback(FMOD_DSP_STATE* dsp_state) { return FMOD_OK; }
FMOD_RESULT SystemDeregister_Callback(FMOD_DSP_STATE* dsp_state) { return FMOD_OK; }
FMOD_RESULT SystemMix_Callback(FMOD_DSP_STATE* dsp_state, int stage) { return FMOD_OK; }
FMOD_RESULT Read_Callback(FMOD_DSP_STATE* dsp_state, float* inbuffer, float* outbuffer, unsigned int length, int inchannels, int* outchannels) { return FMOD_OK; }

// ++++++++++++++++++
//    Parameters 
//+++++++++++++++++++

enum
{
    PARAM_DELAY_TIME = 0,
    PARAM_REPEATS,
    PARAM_BANDWIDTH,
    PARAM_Q,
    PARAM_DRY,
    PARAM_WET,
    NUM_PARAMS
};

// Declarations 
static FMOD_DSP_PARAMETER_DESC p_delayTime, p_repeats, p_bandwidth, p_q, p_dry, p_wet;

// Pointer Array
static FMOD_DSP_PARAMETER_DESC* PluginsParameters[NUM_PARAMS] =
{
    &p_delayTime,
    &p_repeats,
    &p_bandwidth,
    &p_q,
    &p_dry,
    &p_wet
};

// DSP Description
static FMOD_DSP_DESCRIPTION PluginCallbacks =
{
    FMOD_PLUGIN_SDK_VERSION,
    "Scrungler Delay",
    0x00010000,
    1,
    1,
    Create_Callback,
    Release_Callback,
    Reset_Callback,
    Read_Callback,
    Process_Callback,
    SetPosition_Callback,
    NUM_PARAMS,
    PluginsParameters,
    SetFloat_Callback,
    SetInt_Callback,
    SetBool_Callback,
    SetData_Callback,
    GetFloat_Callback,
    GetInt_Callback,
    GetBool_Callback,
    GetData_Callback,
    ShouldIProcess_Callback,
    0,
    SystemRegister_Callback,
    SystemDeregister_Callback,
    SystemMix_Callback
};

// ++++++++++++++++++
//    Delay Buffer
//+++++++++++++++++++

class DelayBuffer
{
public:
    DelayBuffer(int size) : m_buffer(size, 0.0f) {}
    float& operator[](int index) { return m_buffer[index]; }
    const float& operator[](int index) const { return m_buffer[index]; }
    void Clear()
    {
        for (size_t i = 0; i < m_buffer.size(); i++)
            m_buffer[i] = 0.0f;
    }
private:
    std::vector<float> m_buffer;
};

// ++++++++++++++++++
//    Resonant Low-Pass Filter
//+++++++++++++++++++

class ResonantLowPassFilter
{
public:
    ResonantLowPassFilter()
        : m_x1(0.0f), m_x2(0.0f), m_y1(0.0f), m_y2(0.0f)
        , m_cutoff(1.0f), m_q(0.707f), m_a0(1.0f), m_a1(0.0f), m_a2(0.0f)
        , m_b1(0.0f), m_b2(0.0f), m_sampleRate(44100.0f)
    {
    }

    void SetSampleRate(float sampleRate)
    {
        m_sampleRate = sampleRate;
        UpdateCoefficients();
    }

    void SetCutoff(float cutoff)
    {
        m_cutoff = cutoff;
        UpdateCoefficients();
    }

    void SetQ(float q)
    {
        m_q = q;
        UpdateCoefficients();
    }

    float Process(float input)
    {
        float output = m_a0 * input + m_a1 * m_x1 + m_a2 * m_x2 - m_b1 * m_y1 - m_b2 * m_y2;

        m_x2 = m_x1;
        m_x1 = input;
        m_y2 = m_y1;
        m_y1 = output;

        return output;
    }

    void Reset()
    {
        m_x1 = m_x2 = m_y1 = m_y2 = 0.0f;
    }

private:
    void UpdateCoefficients()
    {
        float nyquist = m_sampleRate * 0.5f;
        float minFreq = 20.0f;
        float maxFreq = nyquist;

        float freq = minFreq * powf(maxFreq / minFreq, m_cutoff);

        if (freq < minFreq) freq = minFreq;
        if (freq > nyquist) freq = nyquist;

        float omega = 2.0f * 3.14159265359f * freq / m_sampleRate;
        float sinOmega = sinf(omega);
        float cosOmega = cosf(omega);

        float alpha = sinOmega / (2.0f * m_q);

        float b0 = (1.0f - cosOmega) * 0.5f;
        float b1 = 1.0f - cosOmega;
        float b2 = (1.0f - cosOmega) * 0.5f;
        float a0 = 1.0f + alpha;
        float a1 = -2.0f * cosOmega;
        float a2 = 1.0f - alpha;

        m_a0 = b0 / a0;
        m_a1 = b1 / a0;
        m_a2 = b2 / a0;
        m_b1 = a1 / a0;
        m_b2 = a2 / a0;
    }

    float m_x1, m_x2, m_y1, m_y2;
    float m_cutoff, m_q;
    float m_a0, m_a1, m_a2, m_b1, m_b2;
    float m_sampleRate;
};

// ++++++++++++++++++
//    Multi-Tap Delay (No Feedback)
//+++++++++++++++++++

class MultiTapDelay {
public:
    MultiTapDelay();
    ~MultiTapDelay();
    void Init(FMOD_DSP_STATE* dsp_state, int maxSamples);
    void CreateBuffers(int channels);
    void SetDelayTime(float ms);
    void SetRepeats(int numRepeats);
    void WriteInput(float value);
    float GetTap(int tapNumber, int channel);
    void Tick();
    void Clear();

private:
    DelayBuffer* m_delayBuffer;
    int m_writePos;
    float m_delayTime;
    int m_delaySamples;
    int m_sampleRate;
    int m_maxSamples;
    int m_numChannels;
    int m_repeats;
};

MultiTapDelay::MultiTapDelay()
    : m_delayBuffer(nullptr)
    , m_writePos(0)
    , m_delayTime(500.0f)
    , m_delaySamples(0)
    , m_sampleRate(44100)
    , m_maxSamples(0)
    , m_numChannels(-1)
    , m_repeats(4)
{
}

MultiTapDelay::~MultiTapDelay()
{
    delete m_delayBuffer;
}

void MultiTapDelay::Init(FMOD_DSP_STATE* dsp_state, int maxSamples)
{
    int sampleRate;
    FMOD_DSP_GETSAMPLERATE(dsp_state, &sampleRate);
    m_sampleRate = sampleRate;
    m_maxSamples = maxSamples;
    m_writePos = 0;
    m_numChannels = -1;
}

void MultiTapDelay::CreateBuffers(int channels)
{
    if (m_numChannels != channels && m_maxSamples > 0)
    {
        m_numChannels = channels;
        delete m_delayBuffer;
        m_delayBuffer = new DelayBuffer(m_maxSamples * channels);
        m_writePos = 0;
    }
}

void MultiTapDelay::SetDelayTime(float ms)
{
    m_delayTime = ms;
    m_delaySamples = (int)(ms * 0.001f * m_sampleRate);
    if (m_delaySamples < 1) m_delaySamples = 1;
    if (m_delaySamples > m_maxSamples) m_delaySamples = m_maxSamples;
}

void MultiTapDelay::SetRepeats(int numRepeats)
{
    m_repeats = numRepeats;
    if (m_repeats < 0) m_repeats = 0;
    if (m_repeats > 16) m_repeats = 16;
}

void MultiTapDelay::WriteInput(float value)
{
    if (m_delayBuffer && m_numChannels > 0)
    {
        (*m_delayBuffer)[m_writePos] = value;
    }
}

float MultiTapDelay::GetTap(int tapNumber, int channel)
{
    if (!m_delayBuffer || m_numChannels <= 0) return 0.0f;
    if (tapNumber <= 0) return 0.0f;

    int bufferSize = m_maxSamples * m_numChannels;

    // Each tap is a multiple of the delay time
    int tapSamples = m_delaySamples * tapNumber * m_numChannels;
    int readPos = m_writePos - tapSamples;

    while (readPos < 0) readPos += bufferSize;
    while (readPos >= bufferSize) readPos -= bufferSize;

    return (*m_delayBuffer)[readPos];
}

void MultiTapDelay::Tick()
{
    if (!m_delayBuffer) return;

    m_writePos++;
    int bufferSize = m_maxSamples * m_numChannels;
    if (m_writePos >= bufferSize) m_writePos = 0;
}

void MultiTapDelay::Clear()
{
    if (m_delayBuffer)
    {
        m_delayBuffer->Clear();
    }
    m_writePos = 0;
}

// ++++++++++++++++++
//    Main Plugin Class
//+++++++++++++++++++

class Plugin
{
public:
    Plugin();
    ~Plugin();
    void Init(FMOD_DSP_STATE* dsp_state);
    void Process(float* inbuffer, float* outbuffer, unsigned int length, int channels);
    void SetParameterFloat(int index, float value);
    void GetParameterFloat(int index, float* value);
    void ClearDelay();

private:
    MultiTapDelay* m_delay;
    ResonantLowPassFilter* m_filters;
    float m_dry;
    float m_wet;
    int m_repeats;
    float m_delayTime;
    float m_bandwidth;
    float m_q;
    int m_numChannels;
    float m_sampleRate;
};

Plugin::Plugin()
    : m_delay(nullptr)
    , m_filters(nullptr)
    , m_dry(1.0f)
    , m_wet(0.3f)
    , m_repeats(4)
    , m_delayTime(500.0f)
    , m_bandwidth(1.0f)
    , m_q(0.707f)
    , m_numChannels(0)
    , m_sampleRate(44100.0f)
{
}

Plugin::~Plugin()
{
    delete m_delay;
    delete[] m_filters;
}

void Plugin::Init(FMOD_DSP_STATE* dsp_state)
{
    int sampleRate;
    FMOD_DSP_GETSAMPLERATE(dsp_state, &sampleRate);
    m_sampleRate = (float)sampleRate;
    m_delay = new MultiTapDelay();
    m_delay->Init(dsp_state, 88200);
}

void Plugin::ClearDelay()
{
    if (m_delay)
    {
        m_delay->Clear();
    }
}

void Plugin::Process(float* inbuffer, float* outbuffer, unsigned int length, int channels)
{
    if (!m_delay) return;

    if (m_numChannels != channels)
    {
        m_numChannels = channels;
        delete[] m_filters;
        m_filters = new ResonantLowPassFilter[channels];

        for (int i = 0; i < channels; i++)
        {
            m_filters[i].SetSampleRate(m_sampleRate);
            m_filters[i].SetCutoff(m_bandwidth);
            m_filters[i].SetQ(m_q);
        }
    }

    m_delay->SetDelayTime(m_delayTime);
    m_delay->SetRepeats(m_repeats);
    m_delay->CreateBuffers(channels);

    for (unsigned int i = 0; i < length; i++)
    {
        for (int ch = 0; ch < channels; ch++)
        {
            float input = *inbuffer;

            // Write current input to delay buffer
            m_delay->WriteInput(input);

            // Sum all repeats (dry + multiple taps)
            float wetSum = 0.0f;

            for (int tap = 1; tap <= m_repeats; tap++)
            {
                // Each repeat's gain decreases naturally
                float tapGain = powf(0.6f, (float)tap);

                float tapValue = m_delay->GetTap(tap, ch);

                // Apply bandwidth filter to each tap
                if (m_bandwidth < 0.99f)
                {
                    tapValue = m_filters[ch].Process(tapValue);
                }

                wetSum += tapValue * tapGain;
            }

            // Output = dry + wet
            float output = (input * m_dry) + (wetSum * m_wet);

            // Soft clip output
            if (output > 1.0f) output = 1.0f;
            if (output < -1.0f) output = -1.0f;

            *outbuffer = output;

            inbuffer++;
            outbuffer++;
            m_delay->Tick();
        }
    }
}

void Plugin::SetParameterFloat(int index, float value)
{
    switch (index)
    {
    case PARAM_DRY:
        m_dry = DECIBELS_TO_LINEAR(value);
        if (m_dry < 0.0f) m_dry = 0.0f;
        if (m_dry > 1.0f) m_dry = 1.0f;
        break;

    case PARAM_WET:
        m_wet = DECIBELS_TO_LINEAR(value);
        if (m_wet < 0.0f) m_wet = 0.0f;
        if (m_wet > 1.0f) m_wet = 1.0f;
        break;

    case PARAM_REPEATS:
        m_repeats = (int)value;
        if (m_repeats < 0) m_repeats = 0;
        if (m_repeats > 16) m_repeats = 16;
        break;

    case PARAM_DELAY_TIME:
        m_delayTime = (value < 0.0f) ? 0.0f : (value > 2000.0f) ? 2000.0f : value;
        break;

    case PARAM_BANDWIDTH:
        m_bandwidth = (value < 0.0f) ? 0.0f : (value > 1.0f) ? 1.0f : value;
        if (m_filters)
        {
            for (int i = 0; i < m_numChannels; i++)
                m_filters[i].SetCutoff(m_bandwidth);
        }
        break;

    case PARAM_Q:
        m_q = (value < 0.5f) ? 0.5f : (value > 10.0f) ? 10.0f : value;
        if (m_filters)
        {
            for (int i = 0; i < m_numChannels; i++)
                m_filters[i].SetQ(m_q);
        }
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
    case PARAM_REPEATS:
        *value = (float)m_repeats;
        break;
    case PARAM_DELAY_TIME:
        *value = m_delayTime;
        break;
    case PARAM_BANDWIDTH:
        *value = m_bandwidth;
        break;
    case PARAM_Q:
        *value = m_q;
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
    Plugin* plugin = (Plugin*)dsp_state->plugindata;
    if (plugin)
    {
        plugin->ClearDelay();
    }
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

FMOD_RESULT GetFloat_Callback(FMOD_DSP_STATE* dsp_state, int index, float* value, char* valuestr)
{
    Plugin* plugin = (Plugin*)dsp_state->plugindata;
    plugin->GetParameterFloat(index, value);
    return FMOD_OK;
}

// ++++++++++++++++++
//    FMODGetDSPDescription
//+++++++++++++++++++

extern "C"
{
    F_EXPORT FMOD_DSP_DESCRIPTION* F_CALL FMODGetDSPDescription()
    {
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_delayTime, "Delay Time", "ms", "Delay time in milliseconds", 0.0f, 2000.0f, 500.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_repeats, "Repeats", "", "Number of repeats (0-16)", 0.0f, 16.0f, 4.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_bandwidth, "Bandwidth", "", "Low-pass filter - lower = darker repeats", 0.0f, 1.0f, 1.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_q, "Q", "", "Filter resonance", 0.5f, 10.0f, 0.707f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_dry, "Dry", "dB", "Original signal volume", -80.0f, 10.0f, 0.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_wet, "Wet", "dB", "Delayed signal volume", -80.0f, 10.0f, -6.0f);

        return &PluginCallbacks;
    }
}