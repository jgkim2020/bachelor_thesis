#ifndef WIP_h
#define WIP_h

#include "LowPassFilter.h"
#include "LowPassFilter.cpp"
#include "Eigen/Geometry"
#include "MadgwickAHRSclass.h"
#include <chrono>
#include <vector>

using namespace std;
using namespace std::chrono;

class WIP
{
public:
	WIP(float samplingRate)
		: sampleFrequency(samplingRate), dt(1.0f/samplingRate), aLPF(1, coeff1, coeff2, 0.0), gLPF(1, coeff1, coeff2, 0.0), 
		AHRSalgorithm(samplingRate, 0.1f), Quat(1, 0, 0, 0), Quat0(1, 0, 0, 0), Acc_g(0, 0, 1), LinAcc_g(0, 0, 0), 
		LinAcc_g_(0, 0, 0), Vel_g(0, 0, 0), Vel_g_(0, 0, 0), VelDriftRate(0, 0, 0), Pos_g(0, 0, 0), PosDriftRate(0, 0, 0)
	{}
	void setAccMagThreshold(float accMag);
	void setGyroMagThreshold(float gyroMag);
	void setLPFmode(bool arg);
	void setrecordMode(bool arg);
	void setLPMSquatMode(bool arg);
	bool isStationary();
	int getstate_vv();
	Vector3f getVelocity();
	Vector3f getPosition();
	Quaternionf getQuaternion();
	void update(float gx, float gy, float gz, float ax, float ay, float az, float qw, float qx, float qy, float qz, float timestamp);
	int exportCSV(const char* foutName);

private:
	// user configurable variables
	float sampleFrequency = 100.0f;
	float dt = 1.0f/sampleFrequency;
	float accMagthreshold = 0.04f; // in g
	float gyroMagthreshold = 30.0f; // in dps
	bool LPFmode = true;
	bool recordMode = false;
	bool LPMSquatMode = false;
	bool timeStampmode = true;

	// internal variables
	bool firstTimeMadgwick = true;
	bool stationary = true;
	bool VelDriftCorrflag = false;
	bool PosDriftCorrflag = false;
	steady_clock::time_point FC_time;
	steady_clock::time_point FO_time;
	float FC_timeStamp;
	float FO_timeStamp;
	int state_vv = 0;
	float vv_val = 0.0f;
	int vv_count = 0;
	const int vv_countMax = 0;
	// coefficients obtained from MATLAB butter function
	// digital low-pass filter (cutoff frequency : 5Hz, sampling rate : 100Hz)
	float coeff1[2] = { 1.000000000000000f, -0.726542528005361f };
	float coeff2[2] = { 0.136728735997319f,  0.136728735997319f };
	// digital low-pass filter (cutoff frequency : 5Hz, sampling rate : 200Hz)
	/*float coeff1[2] = { 1.000000000000000f, -0.854080685463467f };
	float coeff2[2] = { 0.072959657268267f,  0.072959657268267f };*/
	LowPassFilter<float> aLPF;
	LowPassFilter<float> gLPF;
	MadgwickAHRS AHRSalgorithm;
	Quaternionf Quat;
	Quaternionf Quat0;
	Vector3f Acc_g;
	Vector3f LinAcc_g;
	Vector3f LinAcc_g_;
	Vector3f Vel_g;
	Vector3f Vel_g_;
	Vector3f VelDriftRate;
	Vector3f Pos_g;
	Vector3f PosDriftRate;

	// export to csv
	vector<bool> recordstationary;
	vector<float> recordFC_timeStamp;
	vector<float> recordFO_timeStamp;
	vector<float> recordtimeStamp;
	vector<int> recordstate_vv;
	vector<Vector3f> recordAcc;
	vector<Vector3f> recordGyro;
	vector<Quaternionf> recordQuat;
	vector<Vector3f> recordLinAcc_g;
	vector<Vector3f> recordVel_g;
	vector<Vector3f> recordPos_g;
};

#endif