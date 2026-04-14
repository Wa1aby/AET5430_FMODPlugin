// 
// Scrungler
//Created by Elliot Alomoto 4/9/26

#include "fmod.hpp"

#include <math.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

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

//Num_Parameters 
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

//Declarations 
static FMOD_DSP_PARAMETER_DESC p_inputDiffuse1, p_inputDiffuse2, 
p_decayDiffuse1, p_decayDiffuse2, p_bandwidth,
p_decay, p_dry, p_wet, p_preDelay;

//Pointer Array
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

// ++++++++++++++++++
//    CallBacks
//+++++++++++++++++++

//FMOD_RESULT Create_Callback(FMOD_DSP_STATE* dsp_state);
//FMOD_RESULT Release_Callback(FMOD_DSP_STATE* dsp_state);

static FMOD_DSP_DESCRIPTION PluginCallbacks =
{
    FMOD_PLUGIN_SDK_VERSION,    // version
    "Advanced FMOD Reverb",          // name
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
    SystemMix_Callback          // Mixer thread exucute / after execute
};

// ++++++++++++++++++
//    Plugin Class (Processing) 
//+++++++++++++++++++

//Delay Buffer
class DelayBuffer
{
public:
    DelayBuffer(int size) : m_buffer(size, 0.0f) {}
    float& operator[](int index) { return m_buffer[index]; }
    const float& operator[](int index)const { return m_buffer[index]; }

private:
    std::vector<float>m_buffer;
};

//Delay Unit
class DelayUnit {
public:
    DelayUnit();
    ~DelayUnit();

    //int. max delay in samples 

    void Init(FMOD_DSP_STATE* dsp_state, int maxSamples);  //max delay length 

    //create buffer based on channel count 

    void CreateBuffers(int channels); //allocates mem. for channels 

    //controls 
    void SetDelayTime(float ms); //change how long delay is 
    void SetFeedback(float feedback);
    float GetDelayTime()const;

    //core 
    void WriteDelay(float value); //store sample into buffer
    float GetDelaySample(); //read a delayed sample 
    void TickChannel(); //write position forward

private:
    DelayBuffer* m_delayBuffer;
    int m_writePos;
    float m_delayTime;
    float m_feedbackAmount;
    int m_sampleRate;
    int m_maxSamples; // changed from float to int 
    int m_numChannels;
};

//Delay Unit Implementation 
DelayUnit::DelayUnit()
    : m_delayBuffer(nullptr)
    , m_writePos(0)
    , m_delayTime(10.0f)
    , m_feedbackAmount(0.0f)
    , m_sampleRate(44100.0f)// I'm sure this will be fine? 
    , m_maxSamples(0)
    , m_numChannels(-1)
{
}
//Deconstructor (Mem Cleaner 9000) 
DelayUnit::~DelayUnit()
{
    delete m_delayBuffer; 
}

// Init Bruv 
void DelayUnit::Init(FMOD_DSP_STATE* dsp_state, int maxSamples)
{
    FMOD_RESULT result = FMOD_DSP_GETSAMPLERATE(dsp_state, &m_sampleRate); 
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

//control medthods 
void DelayUnit::SetDelayTime(float ms)
{
    m_delayTime = ms; 
}

void DelayUnit::SetFeedback(float feedback)
{
    m_feedbackAmount = feedback; 
}

float DelayUnit::GetDelayTime()const
{
    return m_delayTime; 
}

float DelayUnit::GetDelaySample()
{
    if (!m_delayBuffer || m_numChannels <= 0) return 0.0f;

        int delaySamples = (int)(m_delayTime * 0.001f * m_sampleRate) * m_numChannels; 
    int readPos = m_writePos - delaySamples; 

    while (readPos < 0)readPos += (m_maxSamples * m_numChannels); 

    return(*m_delayBuffer)[readPos]; 
}

void DelayUnit::TickChannel()
{
    if (!m_delayBuffer)return; 

    m_writePos++; 
    int bufferSize = m_maxSamples * m_numChannels; 
    if (m_writePos >= bufferSize) m_writePos = 0; 
}
// ++++++++++++++++++
//    Main Verb Pross (Dealing with Output) 
//+++++++++++++++++++

class Plugin
{
public:
        Plugin(); 
        ~Plugin(); 

        void Init(FMOD_DSP_STATE* dsp_state); 
        void Release(); 
        void Process(float* inbuffer, float* outbugger, unsigned int length, int channels); 
        void SetParameterFloat(int index, float value); 
        void GetParameterFloat(int index, float* value); 

private:  
    DelayUnit* m_delay; 
    float m_dry; 
    float m_wet; 
    float m_feedback; 
    float m_delayTime; 
};

//Implementation  
Plugin::Plugin(); 
: m_delay(nullptr) 
, m_dry(1.0f)
, m_wet(0.5f)
, m_feedback(0.3f)
, m_delayTime(500.0f)
, m_dry(1.0f)
{
}

Plugin::~Plugin()
{
    delete m_delay; 
}

void Plugin::Init(FMOD_DSP_STATE* dsp_state)
{
    m_delay = newDelayUnit();
    m_delay->Init(dsp_state, 44100);  // 1 second max delay
}

void Plugin::Release()
{
    delete m_delay
        m_delay = nullprt; 
}

void Plugin::Process(float* inbuffer, float* outbuffer, unsigned int length, int channels)
{
    if (!m_delay)return; 

    m_delay->SetdelayTime(m_delayTime); 
    m_delay->SetFeedback(m_feedback); 
    m_delay->CreateBuggers(channels); 
   
    for (unsigned int i = 0; i < length; i++)
    {
        for (int chh = 0; ch < channels; ch++)
        {
            float input = *inbuffer; 
            float delayed = m_delay->GetDelaySample(); 
            float output = (input * m_dry) + (delayed * m_wet); 
                m_delay->WriteDelay(input + (delayed * m_feedback)); 
                outbuffer = output;

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
    case PARAM_DRY: m_dry = value; break;
    case PARAM_WET: m_wet = value; break;
    case PARAM_DECAY: m_feedback = value; break;
    case PARAM_DRY: m_delayTime = value; break;
    default:break; 
    }
}

void Plugin::GetParameterFloat(int index, float* value)
{
    switch (index)
    {
    case PARAM_DRY: m_dry = value; break;
    case PARAM_WET: m_wet = value; break;
    case PARAM_DECAY: m_feedback = value; break;
    case PARAM_DRY: m_delayTime = value; break;
    default:*value = 0.0f; break; 
    }
}
// ++++++++++++++++++
//    Callback 2 (Dealing with Output) 
//+++++++++++++++++++

