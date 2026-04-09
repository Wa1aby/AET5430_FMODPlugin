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


// ++++++++++++++++++
//    CallBacks
//+++++++++++++++++++


FMOD_DSP_DESCRIPTION PluginCallbacks =
{
    FMOD_PLUGIN_SDK_VERSION,    // version
    "Kelly Hard Clip",          // name
    0x00010000,                 // plugin version
    1,                          // no. input buffers
    1,                          // no. output buffers
    Create_Callback,            // create
    Release_Callback,           // release
    Reset_Callback,             // reset
    Read_Callback,              // read
    Process_Callback,           // process
    SetPosition_Callback,       // setposition
    0,                          // no. parameter
   NULL,                       // clears error since no parameters yet
   // PluginsParameters,          // pointer to parameter descriptions
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

// ++++++++++++++++++
//    Callback 2 (Dealing with Output) 
//+++++++++++++++++++

