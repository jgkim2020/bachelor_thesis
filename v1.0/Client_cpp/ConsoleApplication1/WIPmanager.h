#ifndef WIPmanager_h
#define WIPmanager_h

#include "WIP.h"
#include "csv.h"
#include "csv.cpp"

class WIPmanager
{
public:
	WIPmanager(float samplingRate)
		: left(samplingRate), right(samplingRate), sampleFrequency(samplingRate),
		leftLPF(1, coeff1, coeff2, 0.0), rightLPF(1, coeff1, coeff2, 0.0), locomotionLPF(1, coeff1, coeff2, 0.0),
		leftQuat0(1, 0, 0, 0), rightQuat0(1, 0, 0, 0), leftQuat(1, 0, 0, 0), rightQuat(1, 0, 0, 0)
	{}
	void setquietMode(bool arg);
	void setrecordMode(bool arg);
	void setplaybackMode(bool arg);
	void setscaleCoeff(float scaling);
	void setscaleCoeff2(float scaling);
	float getVirtualSpeed();
	float getVirtualSpeed2();
	void syncVirtualHeading();
	float getVirtualHeading();
	float getZPositionSum();
	float getLLCM();
	int exportCSV(const char* foutName);
	void updateLeft(float gx, float gy, float gz, float ax, float ay, float az, float qw, float qx, float qy, float qz, float timestamp);
	void updateRight(float gx, float gy, float gz, float ax, float ay, float az, float qw, float qx, float qy, float qz, float timestamp);
	WIP left;
	WIP right;

private:
	// user configurable variables
	bool quietMode = false;
	bool recordMode = true;
	bool playbackMode = true;
	
	// internal variables
	float leftLatestTimeStamp = -1.0f;
	float rightLatestTimeStamp = -1.0f;
	float sampleFrequency = 100.0f;
	float leftZpos_ = 0.0f; // LLCM
	float leftZpos = 0.0f;
	float rightZpos_ = 0.0f;
	float rightZpos = 0.0f;
	float scaleCoeff = 1.0f; // virtual speed
	int leftState_vv[2] = { 0, 0 };
	float leftPosPadding = 0.0f;
	int rightState_vv[2] = { 0, 0 };
	float rightPosPadding = 0.0f;
	float padding = 0.0f;
	bool paddingFlag = true;
	int paddingCount = 0;
	float scaleCoeff2 = 1.0f; // virtual speed v2
	int leftState_vv2[2] = { 0, 0 };
	int rightState_vv2[2] = { 0, 0 };
	float FO_leftTime = -1.0f; 
	bool FO_leftTrigger = false;
	float FC_leftTime = -1.0f;
	bool FC_leftTrigger = false;
	float FO_rightTime = -1.0f;
	bool FO_rightTrigger = false;
	float FC_rightTime = -1.0f;
	bool FC_rightTrigger = false;
	float WIPPeriod = 0.0f;
	int stationaryCount = 1;
	float maxPos = 0.0f;
	float returnSpeed2 = 0.0f;
	Quaternionf leftQuat0; // virtual heading
	Quaternionf leftQuat;
	Quaternionf rightQuat0;
	Quaternionf rightQuat;
	float virtualHeading;
	// coefficients obtained from MATLAB butter function
	// digital low-pass filter (cutoff frequency : 5Hz, sampling rate : 100Hz)
	float coeff1[2] = { 1.000000000000000f, -0.726542528005361f };
	float coeff2[2] = { 0.136728735997319f,  0.136728735997319f };
	LowPassFilter<float> leftLPF;
	LowPassFilter<float> rightLPF;
	LowPassFilter<float> locomotionLPF;

	// export to csv
	vector<float> recordZPosSum;
	vector<float> recordLLCM;
	vector<float> recordLeftZPos;
	vector<float> recordRightZPos;
	vector<float> recordLeftSynthVel;
	vector<float> recordRightSynthVel;
	vector<float> recordVirtualSpeed;
	vector<float> recordVirtualSpeed2;
	vector<float> recordVirtualHeading;
	vector<int> recordLeftState;
	vector<int> recordRightState;
};

#endif