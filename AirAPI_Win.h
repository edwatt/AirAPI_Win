#pragma once

#ifdef AIRAPI_EXPORTS
#define AIRAPI_API __declspec(dllexport)
#else
#define AIRAPI_API __declspec(dllimport)
#endif

//Function to start connection to Air
extern "C" AIRAPI_API int StartConnection();

//Function to stop connection to Air
extern "C" AIRAPI_API int StopConnection();

//Function to get quaternion
extern "C" AIRAPI_API float* GetQuaternion();

//Function to get euler
extern "C" AIRAPI_API float* GetEuler();