#ifndef Kitagawa2016_h
#define Kitagawa2016_h

#include <vector>
#include "LpmsSensorI.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "MadgwickAHRSclass.h"

using namespace std;
using namespace Eigen;

class Kitagawa2016
{
public:
	Kitagawa2016(vector<ImuData>& data, float samplingRate)
		: lpms_data(data), sampleFrequency(samplingRate), dt(1.0f / samplingRate), flagThreshold((int)(0.1/dt)),
		AHRSalgorithm(samplingRate, 0.1f)
	{}
	void detectEvent_FSM(); // by jgkim, 2017
	void chooseQuaternion();
	void detectEvent();
	void gaitTracking();
	void gaitAnalysis();
	float getscaleCoeff();
	void exportCSV(const char* foutName);
	void setquietMode(bool arg);
	void setLPMSquatMode(bool arg);
	vector<vector<Vector3f>> getLinAcc_G();
	vector<vector<Vector3f>> getLinVel_G();
	vector<vector<Vector3f>> getLinPos_G();

private:
	bool quietMode = true;
	bool LPMSquatMode = false;
	float sampleFrequency = 100.0f;
	float dt = 1.0f / sampleFrequency;
	int flagCount = 0;
	int flagThreshold = (int)(0.1/dt);
	MadgwickAHRS AHRSalgorithm;
	vector<bool> stationary;
	vector<float> FF_timeStamp;
	vector<float> FO_timeStamp;
	vector<float> T0_timeStamp;
	vector<float> Tf_timeStamp;
	vector<int> T0_index;
	vector<int> Tf_index;
	vector<vector<Vector3f>> saveLinAcc_G;
	vector<vector<Vector3f>> saveLinVel_G;
	vector<vector<Vector3f>> saveLinPos_G;
	vector<ImuData>& lpms_data;
	vector<float> saveStrideTime;
	vector<float> saveVzAmplitude;
	vector<float> saveVrAmplitude;
};

#endif
