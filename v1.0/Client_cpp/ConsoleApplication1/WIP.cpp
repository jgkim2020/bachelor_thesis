#include "WIP.h"
#include "LowPassFilter.h"
#include "MadgwickAHRSclass.h"
#include "csv.h"
#include "csv.cpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <cmath>
#include <chrono>
#include <iostream>

using namespace Eigen;
using namespace std::chrono;
const float PI = 4 * atan(1);

void WIP::setAccMagThreshold(float accMag)
{
	accMagthreshold = accMag;
}

void WIP::setGyroMagThreshold(float gyroMag)
{
	gyroMagthreshold = gyroMag;
}

void WIP::setLPFmode(bool arg)
{
	LPFmode = arg;
}

void WIP::setrecordMode(bool arg)
{
	recordMode = arg;
}

void WIP::setLPMSquatMode(bool arg)
{
	LPMSquatMode = arg;
}

bool WIP::isStationary()
{
	return stationary;
}

int WIP::getstate_vv()
{
	return state_vv;
}

Vector3f WIP::getVelocity()
{
	return Vel_g;
}

Vector3f WIP::getPosition()
{
	return Pos_g;
}

Quaternionf WIP::getQuaternion()
{
	return Quat;
}

void WIP::update(float gx, float gy, float gz, float ax, float ay, float az, float qw, float qx, float qy, float qz, float timestamp)
{
	// Compute magnitude
	float g = sqrtf(gx*gx + gy*gy + gz*gz);
	float a = sqrtf(ax*ax + ay*ay + az*az);
	
	// HPF
	a = fabs(a - 1.0f);
	// LPF
	if (LPFmode)
	{
		g = gLPF.update(g);
		//std::cout << a << " ";
		a = aLPF.update(a);
		//std::cout << a << std::endl;
	}

	// Thresholding for stationary detection
	if (a < accMagthreshold && g < gyroMagthreshold) // below threshold
	{
		if (!stationary)
		{
			stationary = true;
			if (timeStampmode) 
			{
				FC_timeStamp = timestamp;
				if (recordMode) recordFC_timeStamp.push_back(FC_timeStamp);
			}
			else FC_time = steady_clock::now();
		}
	}
	else // above threshold
	{
		if (stationary)
		{
			stationary = false;
			if (timeStampmode)
			{
				FO_timeStamp = timestamp;
				if (recordMode) recordFO_timeStamp.push_back(FO_timeStamp);
			}
			else FO_time = steady_clock::now();
		}
	}

	// Quaternion & coordinate transformation
	if (LPMSquatMode)
	{
		Quat = Quaternionf(qw, qx, qy, qz);
		Acc_g = Quat.conjugate()._transformVector(Vector3f(ax, ay, az));
		LinAcc_g_ = LinAcc_g;
		LinAcc_g = Acc_g - Vector3f(0, 0, 1);
	}
	else
	{
		if (firstTimeMadgwick)
		{
			for (int i = 0; i < 10000; i++) AHRSalgorithm.updateIMU(0.0f, 0.0f, 0.0f, ax, ay, az);
			firstTimeMadgwick = false;
		}
		else AHRSalgorithm.updateIMU(gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, ax, ay, az);
		Quat = AHRSalgorithm.getQuaternion(); // q_SE
		Acc_g = Quat._transformVector(Vector3f(ax, ay, az)); // Acc_g = q_ES*Acc_s*q_SE
		LinAcc_g_ = LinAcc_g;
		LinAcc_g = Acc_g - Vector3f(0, 0, 1); // gravity compensation
	}

	// Track velocity (with velocity drift rate compensation)
	Vel_g_ = Vel_g;
	Vel_g = Vel_g + 0.5f*(LinAcc_g_ + LinAcc_g)*9.81f*dt - VelDriftRate*dt;
	if (stationary)
	{
		if (VelDriftCorrflag)
		{
			VelDriftCorrflag = false;
			float deltat = dt;
			if (timeStampmode) deltat = timestamp - FO_timeStamp;
			else deltat = duration_cast<nanoseconds>(steady_clock::now() - FO_time).count()*1e-9f;
			VelDriftRate = VelDriftRate + Vel_g/deltat; // update drift rate (feedback loop)
		}
		Vel_g = Vector3f(0, 0, 0);
	}
	else
	{
		VelDriftCorrflag = true;
	}

	// Track position (bounded z)
	Pos_g = Pos_g + 0.5f*(Vel_g_ + Vel_g)*dt;
	if (Pos_g[2] < 0) Pos_g = Vector3f(0, 0, 0);
	if (stationary) Pos_g = Vector3f(0, 0, 0);

	/*// Track position (with position drift rate compensation in z axis)
	Pos_g = Pos_g + 0.5f*(Vel_g_ + Vel_g)*dt - PosDriftRate*dt;
	if (stationary)
	{
		if (PosDriftCorrflag)
		{
			PosDriftCorrflag = false;
			float deltat = dt;
			if (timeStampmode) deltat = timestamp - FO_timeStamp;
			else deltat = duration_cast<nanoseconds>(steady_clock::now() - FO_time).count()*1e-9f;
			PosDriftRate = PosDriftRate + Pos_g/deltat;
			PosDriftRate(0) = 0;
			PosDriftRate(1) = 0;
		}
		Pos_g(2) = 0;
	}
	else
	{
		PosDriftCorrflag = true;
	}*/

	// FSM for vertical velocity state
	if (stationary) state_vv = 0;
	else
	{
		switch (state_vv)
		{
		case 0:
			state_vv = 1;
			vv_val = -1.0f;
			vv_count = 0;
			break;
		case 1:
			// Find local max
			if (vv_val < Vel_g[2])
			{
				vv_val = Vel_g[2];
				vv_count = 0;
			}
			else vv_count++;
			if (vv_count > vv_countMax)
			{
				state_vv = 2;
				vv_val = 0.0f;
				vv_count = 0;
			}
			break;
		case 2:
			// Find zero-crossing
			if (Vel_g[2] < vv_val) vv_count++;
			if (vv_count > vv_countMax)
			{
				state_vv = 3;
				vv_val = 1.0f;
				vv_count = 0;
			}
			break;
		case 3:
			// Find local min
			if (vv_val > Vel_g[2])
			{
				vv_val = Vel_g[2];
				vv_count = 0;
			}
			else vv_count++;
			if (vv_count > vv_countMax)
			{
				state_vv = 4;
				vv_val = 0.0f;
				vv_count = 0;
			}
			break;
		default:
			state_vv = 4;
		}
	}

	// Record session (recordMode)
	if (recordMode)
	{
		recordtimeStamp.push_back(timestamp);
		recordAcc.push_back(Vector3f(ax, ay, az));
		recordGyro.push_back(Vector3f(gx, gy, gz));
		recordQuat.push_back(Quat);
		recordLinAcc_g.push_back(LinAcc_g);
		recordVel_g.push_back(Vel_g);
		recordPos_g.push_back(Pos_g);
		recordstationary.push_back(stationary);
		recordstate_vv.push_back(state_vv);
	}
}

int WIP::exportCSV(const char* foutName)
{
	if (!recordMode)
	{
		cout << " << recordMode false! Ignoring exportCSV command..." << endl;
		return 0;
	}

	csv<float> csvout(foutName);

	// Write header
	vector<string> header;
	header.push_back("timeStamp"); 
	header.push_back("ax"); header.push_back("ay"); header.push_back("az");
	header.push_back("gx"); header.push_back("gy"); header.push_back("gz");
	header.push_back("qw"); header.push_back("qx"); header.push_back("qy"); header.push_back("qz");
	header.push_back("px"); header.push_back("py"); header.push_back("pz");
	header.push_back("vx"); header.push_back("vy"); header.push_back("vz");
	header.push_back("lin_ax"); header.push_back("lin_ay"); header.push_back("lin_az");
	header.push_back("stationary"); header.push_back("foot-contact"); header.push_back("foot-off");
	header.push_back("state_vv");

	// Write data
	vector<vector<float>> fields;
	unsigned int FC_iter = 0; unsigned int FO_iter = 0;
	for (unsigned int i = 0; i < recordAcc.size(); i++)
	{
		fields.push_back(vector<float>());
		fields.back().push_back(recordtimeStamp[i]);
		for (int j = 0; j < 3; j++) fields.back().push_back(recordAcc[i][j]);
		for (int j = 0; j < 3; j++) fields.back().push_back(recordGyro[i][j]);
		fields.back().push_back(recordQuat[i].w()); fields.back().push_back(recordQuat[i].x());
		fields.back().push_back(recordQuat[i].y()); fields.back().push_back(recordQuat[i].z());
		for (int j = 0; j < 3; j++) fields.back().push_back(recordPos_g[i][j]);
		for (int j = 0; j < 3; j++) fields.back().push_back(recordVel_g[i][j]);
		for (int j = 0; j < 3; j++) fields.back().push_back(recordLinAcc_g[i][j]);
		if (recordstationary[i]) fields.back().push_back(1.0f); else fields.back().push_back(0.0f);
		if (FC_iter < recordFC_timeStamp.size())
		{
			if (recordtimeStamp[i] != recordFC_timeStamp[FC_iter]) fields.back().push_back(0.0f);
			else
			{
				fields.back().push_back(1.0f);
				FC_iter++;
			}
		}
		if (FO_iter < recordFO_timeStamp.size())
		{
			if (recordtimeStamp[i] != recordFO_timeStamp[FO_iter]) fields.back().push_back(0.0f);
			else
			{
				fields.back().push_back(1.0f);
				FO_iter++;
			}
		}
		fields.back().push_back((float)recordstate_vv[i]);
	}

	csvout.write(header, fields);
	return 0;
}