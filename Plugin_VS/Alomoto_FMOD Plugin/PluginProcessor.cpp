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

FMOD_RESULT Create_Callback(FMOD_DSP_STATE* dsp_state);
FMOD_RESULT Release_Callback(FMOD_DSP_STATE* dsp_state);

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
    DelayBuffer(int size) : m_buffer(size,0.0f){}
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
    float m_sampleRate; 
    int m_maxSamples; 
    int m_numChannels; 
};

//Delay Unit Implementation 
DelayUnit::DelayUnit()
    : m_delayBuffer(nullptr)
    , m_writePos(0)
    , m_delayTime(10.0f)
    , m_feedbackAmount(0.0f)
    , m_sampleRate(44100)
    , m_maxSamples(0)
  //  , m_numOfChannels(-1)

//Main Verb

// ++++++++++++++++++
//    Callback 2 (Dealing with Output) 
//+++++++++++++++++++

