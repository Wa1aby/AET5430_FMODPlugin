// 
// Scrungler - Simple Delay Plugin 
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
    PARAM_DECAY,
    PARAM_BANDWIDTH,
    PARAM_Q,
    PARAM_DRY,
    PARAM_WET,
    NUM_PARAMS
};

// Declarations 
static FMOD_DSP_PARAMETER_DESC p_delayTime, p_decay, p_bandwidth, p_q, p_dry, p_wet;

// Pointer Array
static FMOD_DSP_PARAMETER_DESC* PluginsParameters[NUM_PARAMS] =
{
    &p_delayTime,
    &p_decay,
    &p_bandwidth,
    &p_q,
    &p_dry,
    &p_wet
}; 

// DSP Description
static FMOD_DSP_DESCRIPTION PluginCallbacks =
{
    FMOD_PLUGIN_SDK_VERSION,
    "Scrungler Delay w_Pepperoni",
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
//    Resonant Low-Pass Filter (2-pole with Q)
//+++++++++++++++++++

class ResonantLowPassFilter
{
public:
    ResonantLowPassFilter() 
        : m_x1(0.0f), m_x2(0.0f), m_y1(0.0f), m_y2(0.0f)
        , m_cutoff(1.0f), m_q(0.707f), m_a0(1.0f), m_a1(0.0f), m_a2(0.0f)
        , m_b1(0.0f), m_b2(0.0f), m_sampleRate(44100.0f)
    {}
    
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
//    Delay Unit
//+++++++++++++++++++

class DelayUnit {
public:
    DelayUnit();
    ~DelayUnit();
    void Init(FMOD_DSP_STATE* dsp_state, int maxSamples);
    void CreateBuffers(int channels);
    void SetDelayTime(float ms);
    void SetFeedback(float feedback);
    void WriteDelay(float value);
    float GetDelayedSample();
    void TickChannel();
    void Clear();

private:
    DelayBuffer* m_delayBuffer;
    int m_writePos;
    float m_delayTime;
    float m_feedbackAmount;
    int m_sampleRate;
    int m_maxSamples;
    int m_numChannels;
    int m_currentDelaySamples;
};

DelayUnit::DelayUnit()
    : m_delayBuffer(nullptr)
    , m_writePos(0)
    , m_delayTime(500.0f)
    , m_feedbackAmount(0.0f)
    , m_sampleRate(44100)
    , m_maxSamples(0)
    , m_numChannels(-1)
    , m_currentDelaySamples(0)
{
}

DelayUnit::~DelayUnit()
{
    delete m_delayBuffer; 
}

void DelayUnit::Init(FMOD_DSP_STATE* dsp_state, int maxSamples)
{
    int sampleRate;
    FMOD_DSP_GETSAMPLERATE(dsp_state, &sampleRate);
    m_sampleRate = sampleRate;
    m_maxSamples = maxSamples; 
    m_writePos = 0; 
    m_numChannels = -1; 
    m_currentDelaySamples = 0;
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
    
    int desiredSamples = (int)(ms * 0.001f * m_sampleRate);
    
    if (desiredSamples > m_maxSamples)
        desiredSamples = m_maxSamples;
    if (desiredSamples < 0)
        desiredSamples = 0;
        
    m_currentDelaySamples = desiredSamples;
}

void DelayUnit::SetFeedback(float feedback)
{
    if (feedback < 0.0f) feedback = 0.0f;
    if (feedback > 0.85f) feedback = 0.85f;
    m_feedbackAmount = feedback; 
}

void DelayUnit::WriteDelay(float value)
{
    if (m_delayBuffer)
    {
        if (value > 1.0f) value = 1.0f;
        if (value < -1.0f) value = -1.0f;
        (*m_delayBuffer)[m_writePos] = value;
    }
}

float DelayUnit::GetDelayedSample()
{
    if (!m_delayBuffer || m_numChannels <= 0) return 0.0f;
    
    if (m_currentDelaySamples <= 0) return 0.0f;

    int readPos = m_writePos - (m_currentDelaySamples * m_numChannels);
    
    int bufferSize = m_maxSamples * m_numChannels;
    while (readPos < 0) readPos += bufferSize;
    while (readPos >= bufferSize) readPos -= bufferSize;
    
    if (readPos < 0 || readPos >= bufferSize) return 0.0f;
    
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
    DelayUnit* m_delay; 
    ResonantLowPassFilter* m_filters;
    float m_dry; 
    float m_wet; 
    float m_feedback; 
    float m_delayTime;
    float m_bandwidth;
    float m_q;
    int m_numChannels;
    float m_sampleRate;
    
    // Feedback protection
    float m_feedbackEnergy;
    float m_feedbackReduction;
};

Plugin::Plugin()
    : m_delay(nullptr)
    , m_filters(nullptr)
    , m_dry(1.0f)
    , m_wet(0.3f)
    , m_feedback(0.0f)
    , m_delayTime(500.0f)
    , m_bandwidth(1.0f)
    , m_q(0.707f)
    , m_numChannels(0)
    , m_sampleRate(44100.0f)
    , m_feedbackEnergy(0.0f)
    , m_feedbackReduction(1.0f)
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
    m_delay = new DelayUnit();
    m_delay->Init(dsp_state, 88200);
}

void Plugin::ClearDelay()
{
    if (m_delay)
    {
        m_delay->Clear();
    }
    m_feedbackEnergy = 0.0f;
    m_feedbackReduction = 1.0f;
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
    m_delay->SetFeedback(m_feedback * m_feedbackReduction); 
    m_delay->CreateBuffers(channels); 
   
    for (unsigned int i = 0; i < length; i++)
    {
        for (int ch = 0; ch < channels; ch++)
        {
            float input = *inbuffer; 
            float delayed = m_delay->GetDelayedSample();
            
            // Apply resonant low-pass filter to delayed signal only
            float filteredDelayed = delayed;
            if (m_bandwidth < 0.99f)
            {
                filteredDelayed = m_filters[ch].Process(delayed);
            }
            
            // Track feedback energy for protection
            float absDelayed = (filteredDelayed < 0.0f) ? -filteredDelayed : filteredDelayed;
            m_feedbackEnergy = m_feedbackEnergy * 0.999f + absDelayed * 0.001f;
            
            // Dynamic reduction - if energy is too high, reduce feedback
            if (m_feedbackEnergy > 0.4f)
            {
                m_feedbackReduction = 0.4f / m_feedbackEnergy;
                if (m_feedbackReduction < 0.1f) m_feedbackReduction = 0.1f;
            }
            else
            {
                m_feedbackReduction = m_feedbackReduction * 0.999f + 1.0f * 0.001f;
                if (m_feedbackReduction > 1.0f) m_feedbackReduction = 1.0f;
            }
            
            // Output mix
            float output = (input * m_dry) + (filteredDelayed * m_wet);
            
            // Write to delay with dynamic feedback reduction
            float feedbackAmount = m_feedback * m_feedbackReduction;
            float feedbackSignal = filteredDelayed * feedbackAmount;
            
            // Soft clip feedback signal
            if (feedbackSignal > 0.9f) feedbackSignal = 0.9f;
            if (feedbackSignal < -0.9f) feedbackSignal = -0.9f;
            
            m_delay->WriteDelay(input + feedbackSignal); 
            
            // Soft clip output
            if (output > 1.0f) output = 1.0f;
            if (output < -1.0f) output = -1.0f;
            
            *outbuffer = output;

            inbuffer++; 
            outbuffer++; 
            m_delay->TickChannel(); 
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
            
        case PARAM_DECAY: 
            m_feedback = (value < 0.0f) ? 0.0f : (value > 0.85f) ? 0.85f : value;
            
            // When decay changes, re-check Q limit
            {
                float maxSafeQ = 5.0f - (m_feedback * 4.5f);
                if (maxSafeQ < 1.0f) maxSafeQ = 1.0f;
                if (m_q > maxSafeQ)
                {
                    m_q = maxSafeQ;
                    if (m_filters)
                    {
                        for (int i = 0; i < m_numChannels; i++)
                            m_filters[i].SetQ(m_q);
                    }
                }
            }
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
            {
                float requestedQ = (value < 0.5f) ? 0.5f : (value > 10.0f) ? 10.0f : value;
                
                // Limit Q based on decay to prevent feedback
                float maxSafeQ = 5.0f - (m_feedback * 4.5f);
                if (maxSafeQ < 1.0f) maxSafeQ = 1.0f;
                
                m_q = (requestedQ > maxSafeQ) ? maxSafeQ : requestedQ;
                
                if (m_filters)
                {
                    for (int i = 0; i < m_numChannels; i++)
                        m_filters[i].SetQ(m_q);
                }
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
        case PARAM_DECAY: 
            *value = m_feedback;
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
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_delayTime, "Delay Time", "ms", "Delay time in milliseconds (0-2000ms)", 0.0f, 2000.0f, 500.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_decay, "Decay", "", "Feedback amount - how many repeats (0-0.85)", 0.0f, 0.85f, 0.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_bandwidth, "Bandwidth", "", "Low-pass filter cutoff - lower = darker repeats", 0.0f, 1.0f, 1.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_q, "Q", "", "Filter resonance - automatically limited by decay", 0.5f, 10.0f, 0.707f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_dry, "Dry", "dB", "Original signal volume", -80.0f, 10.0f, 0.0f);
        FMOD_DSP_INIT_PARAMDESC_FLOAT(p_wet, "Wet", "dB", "Delayed signal volume", -80.0f, 10.0f, -6.0f);

        return &PluginCallbacks;
    }
}