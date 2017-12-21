#include "Kitagawa2016.h"
#include "csv.h"
#include "csv.cpp"
#include <cmath>
#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace std;
using namespace Eigen;
const float PI = 4 * atanf(1);

void Kitagawa2016::detectEvent_FSM()
{

}

void Kitagawa2016::chooseQuaternion()
{
	if (!LPMSquatMode)
	{
		// Initialize Madgwick filter
		for (int i = 0; i < 10000; i++)
		{
			AHRSalgorithm.updateIMU(0.0f, 0.0f, 0.0f, -lpms_data[0].a[0], -lpms_data[0].a[1], -lpms_data[0].a[2]);
		}
		for (auto it = lpms_data.begin(); it != lpms_data.end(); it++)
		{
			AHRSalgorithm.updateIMU(it->g[0]*PI/180.0f, it->g[1]*PI/180.0f, it->g[2]*PI/180.0f, -it->a[0], -it->a[1], -it->a[2]);
			Quaternionf Quat = AHRSalgorithm.getQuaternion().conjugate();
			// Replace LPMS quaternion with Madgwick quaternion
			it->q[0] = Quat.w();
			it->q[1] = Quat.x();
			it->q[2] = Quat.y();
			it->q[3] = Quat.z();
		}
	}
}

void Kitagawa2016::detectEvent()
{
	// Find FF (Foot-Flat) and FO (Foot-Off) timeStamps
	for (auto it = lpms_data.begin(); it != lpms_data.end(); it++)
	{
		float g = sqrtf(it->g[0] * it->g[0] + it->g[1] * it->g[1] + it->g[2] * it->g[2]);
		if (g < 25.0f)
		{
			flagCount++;
			if (flagCount == flagThreshold) FF_timeStamp.push_back((float)it->timeStamp);
			if (flagCount >= flagThreshold) stationary.push_back(true);
			else stationary.push_back(false);
		}
		else
		{
			if (flagCount >= flagThreshold) FO_timeStamp.push_back((float)it->timeStamp);
			flagCount = 0;
			stationary.push_back(false);
		}
	}
	
	// Find T0 and Tf timeStamps (as defined in Kitagawa2016 paper)
	auto it_FO = FO_timeStamp.begin();
	for (; it_FO != FO_timeStamp.end() && *it_FO < FF_timeStamp.front(); it_FO++);
	for (auto it_FF = FF_timeStamp.begin(); (it_FF + 1) != FF_timeStamp.end() && it_FO != FO_timeStamp.end(); it_FO++, it_FF++)
	{
		T0_timeStamp.push_back(*it_FF*0.25f + *it_FO*0.75f);
		Tf_timeStamp.push_back(*(it_FF + 1));
	}

	// Find T0 and Tf indices
	auto it_T0 = T0_timeStamp.begin();
	auto it_Tf = Tf_timeStamp.begin();
	for (unsigned int i = 0; i < lpms_data.size(); i++)
	{
		if (it_T0 != T0_timeStamp.end())
		{
			if (*it_T0 - lpms_data[i].timeStamp < 0.5f*dt)
			{
				T0_index.push_back(i);
				it_T0++;
			}
		}
		if (it_Tf != Tf_timeStamp.end())
		{
			if (*it_Tf - lpms_data[i].timeStamp < 0.5f*dt)
			{
				Tf_index.push_back(i);
				it_Tf++;
			}
		}
	}

	// For debugging purposes
	if (!quietMode)
	{
		cout << "FF_timeStamp (s):" << endl;
		for (auto it = FF_timeStamp.begin(); it != FF_timeStamp.end(); it++) cout << *it << " ";
		cout << endl;
		cout << "FO_timeStamp (s):" << endl;
		for (auto it = FO_timeStamp.begin(); it != FO_timeStamp.end(); it++) cout << *it << " ";
		cout << endl;
		cout << "T0_timeStamp (s):" << endl;
		for (auto it = T0_timeStamp.begin(); it != T0_timeStamp.end(); it++) cout << *it << " ";
		cout << endl;
		cout << "T0_index corresponding timeStamp (s):" << endl;
		for (auto it = T0_index.begin(); it != T0_index.end(); it++) cout << lpms_data[*it].timeStamp << " ";
		cout << endl;
		cout << "Tf_timeStamp (s):" << endl;
		for (auto it = Tf_timeStamp.begin(); it != Tf_timeStamp.end(); it++) cout << *it << " ";
		cout << endl;
		cout << "Tf_index corresponding timeStamp (s):" << endl;
		for (auto it = Tf_index.begin(); it != Tf_index.end(); it++) cout << lpms_data[*it].timeStamp << " ";
		cout << endl;
	}
}

void Kitagawa2016::gaitTracking()
{
	// Initialize variables
	vector<vector<Vector3f>>().swap(saveLinAcc_G);
	vector<vector<Vector3f>>().swap(saveLinVel_G);
	vector<vector<Vector3f>>().swap(saveLinPos_G);
	vector<float>().swap(saveStrideTime);
	vector<float>().swap(saveVzAmplitude);
	vector<float>().swap(saveVrAmplitude);
	vector<float> strideLength; // for debugging

	for (unsigned int i = 0; i < T0_timeStamp.size(); i++)
	{
		// Find quaternion and acceleration at T0
		const float* q_t0 = lpms_data[T0_index[i]].q; // base coordinate orientation (q_GF)
		const float* a_t0 = lpms_data[T0_index[i]].a; // gravity from F frame

		// Initialize variables
		vector<Vector3f> LinAcc_G;
		vector<Vector3f> LinVel_G;
		LinVel_G.push_back(Vector3f(0, 0, 0));
		vector<Vector3f> LinPos_G;
		LinPos_G.push_back(Vector3f(0, 0, 0));
	
		// Find linear acceleration in G frame
		for (int t = T0_index[i]; t < Tf_index[i]; t++)
		{
			Quaternionf Quat_t = Quaternionf(lpms_data[t].q[0], lpms_data[t].q[1], lpms_data[t].q[2], lpms_data[t].q[3]); // q_GS
			Quaternionf Quat_t0 = Quaternionf(q_t0[0], q_t0[1], q_t0[2], q_t0[3]); // q_GF
			Quaternionf Quat_t_ = Quat_t*(Quat_t0.conjugate()); // q_FS = q_GS*qFG
			Vector3f Acc_S = Vector3f(-lpms_data[t].a[0], -lpms_data[t].a[1], -lpms_data[t].a[2]);
			Vector3f Acc_F = Quat_t_.conjugate()._transformVector(Acc_S); // Acc_F = q_FS*Acc_S*q_SF
			Vector3f LinAcc_F = Acc_F - Vector3f(-a_t0[0], -a_t0[1], -a_t0[2]);
			LinAcc_G.push_back(Quat_t0.conjugate()._transformVector(LinAcc_F)); // LinAcc_G = q_GF*LinAcc_F*q_FG
		}

		// Zero vertical velocity correction
		Vector3f VelDriftRate = Vector3f(0, 0, 0);
		for (unsigned int t = 1; t < LinAcc_G.size(); t++)
		{
			VelDriftRate += 0.5f*(LinAcc_G[t - 1] + LinAcc_G[t])*9.81f*dt;
		}
		VelDriftRate /= Tf_timeStamp[i] - T0_timeStamp[i];
		//VelDriftRate[0] = 0; VelDriftRate[1] = 0; // only correct vertical component
		for (unsigned int t = 1; t < LinAcc_G.size(); t++)
		{
			LinVel_G.push_back(LinVel_G.back() + 0.5f*(LinAcc_G[t - 1] + LinAcc_G[t])*9.81f*dt - VelDriftRate*dt);
		}

		// Zero vertical displacement correction
		Vector3f PosDriftRate = Vector3f(0, 0, 0);
		for (unsigned int t = 1; t < LinVel_G.size(); t++)
		{
			PosDriftRate += 0.5f*(LinVel_G[t - 1] + LinVel_G[t])*dt;
		}
		PosDriftRate /= Tf_timeStamp[i] - T0_timeStamp[i];
		strideLength.push_back(sqrtf(PosDriftRate[0]*PosDriftRate[0] + PosDriftRate[1]*PosDriftRate[1]));
		PosDriftRate[0] = 0; PosDriftRate[1] = 0; // only correct vertical component
		for (unsigned int t = 1; t < LinVel_G.size(); t++)
		{
			LinPos_G.push_back(LinPos_G.back() + 0.5f*(LinVel_G[t - 1] + LinVel_G[t])*dt - PosDriftRate*dt);
		}

		// Save data
		saveLinAcc_G.push_back(LinAcc_G);
		saveLinVel_G.push_back(LinVel_G);
		saveLinPos_G.push_back(LinPos_G);

		// Data for scaleCoeff
		saveStrideTime.push_back(Tf_timeStamp[i] - T0_timeStamp[i]);
		float Vmax = -100.0f;
		float Vmin = 100.0f;
		for (auto it = LinVel_G.begin(); it != LinVel_G.end(); it++)
		{
			if ((*it)[2] > Vmax) Vmax = (*it)[2];
			if ((*it)[2] < Vmin) Vmin = (*it)[2];
		}
		saveVzAmplitude.push_back((Vmax > fabsf(Vmin)) ? Vmax : fabsf(Vmin));
		Vmax = -100.0f;
		for (auto it = LinVel_G.begin(); it != LinVel_G.end(); it++)
		{
			if (((*it)[0]*(*it)[0] + (*it)[1]*(*it)[1]) > Vmax) Vmax = (*it)[0]*(*it)[0] + (*it)[1]*(*it)[1];
		}
		saveVrAmplitude.push_back(sqrtf(Vmax));
	}

	// For debugging purposes
	if (!quietMode)
	{
		cout << "strideLength (m):" << endl;
		float sum = 0;
		for (auto it = strideLength.begin(); it != strideLength.end(); it++)
		{
			cout << *it << " ";
			sum += *it;
		}
		cout << endl;
		cout << "distance traveled (m): " << sum << endl;
	}
}

float Kitagawa2016::getscaleCoeff()
{
	sort(saveStrideTime.begin(), saveStrideTime.end());
	sort(saveVzAmplitude.begin(), saveVzAmplitude.end());
	sort(saveVrAmplitude.begin(), saveVrAmplitude.end());
	
	int length = T0_timeStamp.size();
	float meanStrideTime = 0.0f;
	float meanVzAmplitude = 0.0f;
	float meanVrAmplitude = 0.0f;
	int count = 0;
	for (auto it = saveStrideTime.begin() + length/4; it != saveStrideTime.end() - length/4; it++)
	{
		meanStrideTime = (count*meanStrideTime + *it)/(count + 1);
		count++;
	}
	count = 0;
	for (auto it = saveVzAmplitude.begin() + length/4; it != saveVzAmplitude.end() - length/4; it++)
	{
		meanVzAmplitude = (count*meanVzAmplitude + *it)/(count + 1);
		count++;
	}
	count = 0;
	for (auto it = saveVrAmplitude.begin() + length/4; it != saveVrAmplitude.end() - length/4; it++)
	{
		meanVrAmplitude = (count*meanVrAmplitude + *it)/(count + 1);
		count++;
	}
	return 2*PI*meanVrAmplitude/meanStrideTime/meanVzAmplitude;
}

void Kitagawa2016::gaitAnalysis()
{
	chooseQuaternion();
	detectEvent();
	gaitTracking();
}

void Kitagawa2016::exportCSV(const char* foutName)
{
	csv<float> csvout(foutName);

	// Write header
	vector<string> header;
	header.push_back("gait number");
	header.push_back("px"); header.push_back("py"); header.push_back("pz");
	header.push_back("vx"); header.push_back("vy"); header.push_back("vz");
	header.push_back("ax"); header.push_back("ay"); header.push_back("az");
	
	// Write data
	vector<vector<float>> fields;
	for (unsigned int i = 0; i < saveLinPos_G.size(); i++)
	{
		for (unsigned int j = 0; j < saveLinPos_G[i].size(); j++)
		{
			fields.push_back(vector<float>());
			fields.back().push_back((float)(i + 1));
			for (int k = 0; k < 3; k++) fields.back().push_back(saveLinPos_G[i][j][k]);
			for (int k = 0; k < 3; k++) fields.back().push_back(saveLinVel_G[i][j][k]);
			for (int k = 0; k < 3; k++) fields.back().push_back(saveLinAcc_G[i][j][k]);
		}
	}

	csvout.write(header, fields);
}

void Kitagawa2016::setquietMode(bool arg)
{
	quietMode = arg;
}

void Kitagawa2016::setLPMSquatMode(bool arg)
{
	LPMSquatMode = arg;
}

vector<vector<Vector3f>> Kitagawa2016::getLinAcc_G()
{
	return saveLinAcc_G;
}

vector<vector<Vector3f>> Kitagawa2016::getLinVel_G()
{
	return saveLinVel_G;
}

vector<vector<Vector3f>> Kitagawa2016::getLinPos_G()
{
	return saveLinPos_G;
}
