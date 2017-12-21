#include "Stdio.h"
#include "stdafx.h"
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
#include "MadgwickAHRS.h"
#include "MadgwickAHRSclass.h"
#include "MahonyAHRS.h"
#include "LowPassFilter.h"
#include "WIPmanager.h"
#include "csv.h"
#include "csv.cpp"
#include "Kitagawa2016.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <iostream>
#include <thread>
#include <cmath>
#include <vector>
#include <string>
#include <chrono>
#include <Windows.h>

using namespace std;
using namespace this_thread;
using namespace chrono;

// Define constants
const float PI = 4 * atanf(1);
const int number_of_sensor = 3;
const int bluetooth_timeout = 75; // *0.1s
// const char* ip_address = "192.168.1.34"; // cmd > ipconfig (WLAN IPV4 address) - KIST
// const char* ip_address = "192.168.0.9"; // cmd > ipconfig (WLAN IPV4 address) - home
const char* ip_address = "192.168.43.51"; // cmd > ipconfig (WLAN IPV4 address) - AndroidAP
// const char* ip_address = "192.168.0.2"; // cmd > ipconfig (LAN IPV4 address) - dorm
const string VELOCITY_KEY = "19775824";
const string QUATERNION_KEY = "38241913";
const string GAZE_KEY = "22353818";
const char* finName = "../../../lpms_datasets/0728/wip_neutral.csv";
const char* foutName = "fout.csv";

// Define global variables
SOCKET client;
ImuData lpms_data[number_of_sensor]; // ImuData instance
bool lpms_connect[number_of_sensor] = { true, true, false };
LpmsSensorManagerI* manager = LpmsSensorManagerFactory(); // LpmsSensorManager instance
LpmsSensorI* lpms[number_of_sensor]; // LpmsSensor instance
vector<const char*> SensorId;
double timeStamp0[number_of_sensor] = { 0.0 };
int runCount = 0;
string cmdString = "standby";
bool runLoop = true;
bool toggle = false;
WIP WIPtracker[2] = {WIP(100.0f), WIP(100.0f)};

float test_speed = 0.0f;
float test_angle = 0.0f;

float speed2_history = 0.0f;

// Demos
void runKitagawa(const char* fin, const char* fout, int number_of_sensor, int whichfoot);
void WIPmanager_realtime(bool gazeMode, bool recordMode, bool unityMode, int version);
void WIPmanager_offline(const char* fin, int number_of_sensor, bool unityMode, int version);
void WIPtracking_realtime(void);
void WIPtracking_offline(const char* fin, int num);
void WIPlocomotion_simple(void);
void simple_demo(void);
void LPMSBasicExample(void);

// Utils
void sendVelocity(double v, double vx, double vy, double vz, bool debugmode);
void sendQuaternion(double qw, double qx, double qy, double qz, bool debugmode);
void sendSpeed(double speed, bool debugmode);
void synctimeStamp(double time0);
void initializeLPMS(void);
void terminateLPMS(void);
void openClient(void);
void closeClient(void);
void showError(const char* msg);

int main(int argc, char *argv[])
{
	while (runLoop)
	{
		// Read command
		if (GetAsyncKeyState(VK_RETURN) & 0x8000)
		{
			cout << " >> ";
			cin >> cmdString;
			cout << " << Command is " << cmdString << "." << endl;
			this_thread::sleep_for(chrono::milliseconds(250));
		}

		if (cmdString == "begin")
		{
			openClient();
			initializeLPMS();
			//VirtualLocomotion.setquietMode(true);
			//VirtualLocomotion.setrecordMode(true);
			//VirtualLocomotion.setscaleCoeff(10.0f);
			cmdString = "standby";
		}
		else if (cmdString == "LPF") // test LowPassFilter class
		{
			// @100Hz sampling rate, @5Hz cutoff frequency
			double a[2] = { 1.000000000000000, -0.726542528005361 };
			double b[2] = { 0.136728735997319,  0.136728735997319 };
			LowPassFilter<double> lpf(1, a, b, 0.0);
			double data;
			ifstream fin;
			ofstream fout;
			fin.open("sample.txt");
			fout.open("filter.txt");
			while (fin >> data)
			{
				cout << data << endl;
				fout << lpf.update(data) << endl;
				// cout << " >> " << lpf.get_filteredData(data) << endl;
			}
			fin.close();
			fout.close();
			return 0;
		}
		else if (cmdString == "Madgwick") // test Madgwick filter
		{
			MadgwickAHRS AHRS(100.0f, 0.1f);
			float g[3] = { 0.0f, 0.0f, 0.0f };
			float a[3] = { 0.707f, 0.0f, 0.707f };
			for (int i = 0; i < 10000; i++)
			{
				MadgwickAHRSupdateIMU(g[0], g[1], g[2], a[0], a[1], a[2]);
				AHRS.updateIMU(g[0], g[1], g[2], a[0], a[1], a[2]);
			}
			cout << "C   " << q0 << " " << q1 << " " << q2 << " " << q3 << endl;
			Quaternionf q = AHRS.getQuaternion();
			cout << "C++ " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;
			return 0;
		}
		else if (cmdString == "w") // test sendVelocity / sendSpeed
		{
			test_speed += 0.1f;
			//sendVelocity(test_speed, 0, 0, 1, false);
			sendSpeed(test_speed, false);
			cout << "current speed : " << test_speed << endl;
			cmdString = "standby";
		}
		else if (cmdString == "s") // test sendVelocity / sendSpeed
		{
			test_speed -= 0.1f;
			//sendVelocity(test_speed, 0, 0, 1, false);
			sendSpeed(test_speed, false);
			cout << "current speed : " << test_speed << endl;
			cmdString = "standby";
		}
		else if (cmdString == "a") // test sendQuaternion
		{
			test_angle += PI/3;
			sendQuaternion(cosf(test_angle/2), 0, -sinf(test_angle/2), 0, false);
			cout << "current angle : " << test_angle << endl;
			cmdString = "standby";
		}
		else if (cmdString == "d") // test sendQuaternion
		{
			test_angle -= PI/3;
			sendQuaternion(cosf(test_angle/2), 0, -sinf(test_angle/2), 0, false);
			cout << "current angle : " << test_angle << endl;
			cmdString = "standby";
		}
		else if (cmdString == "sync")
		{
			synctimeStamp(0.0);
			cout << " << All timeStamps have been reset to zero." << endl;
			cmdString = "standby";
		}
		else if (cmdString == "record")
		{
			manager->saveSensorData("data.csv");
			cout << " << Type 'save' to stop recording..." << endl;
			cmdString = "standby";
		}
		else if (cmdString == "save")
		{
			manager->stopSaveSensorData();
			cmdString = "standby";
		}
		else if (cmdString == "calibrate")
		{
			cout << " << Start calibration? (Y/n)" << endl;
			while (cmdString != "Y" && cmdString != "n") cin >> cmdString;
			if (cmdString == "n") cmdString = "standby";
			else
			{
				steady_clock::time_point begin = steady_clock::now();
				manager->saveSensorData("calibration_normal_walking.csv");
				cout << " << Stop calibration? (Y/n)" << endl;
				cmdString = "standby";
				while (cmdString != "Y") cin >> cmdString;
				manager->stopSaveSensorData();
				steady_clock::time_point end = steady_clock::now();
				cout << " << Calibration done! (elapsed time : " << duration_cast<seconds>(end - begin).count() << "s)" << endl;
			}

			runKitagawa("calibration_normal_walking.csv", "Kitagawa_result_left.csv", 2, 0);
			runKitagawa("calibration_normal_walking.csv", "Kitagawa_result_right.csv", 2, 1);
			cmdString = "standby";
		}
		else if (cmdString == "kita") // offline version of calibrate
		{
			//runKitagawa("../../../lpms_datasets/1010/toe_100Hz_2.csv", "Kitagawa_result_left.csv", 2, 0);
			//runKitagawa("../../../lpms_datasets/1010/toe_100Hz_2.csv", "Kitagawa_result_right.csv", 2, 1);
			runKitagawa("../../../lpms_datasets/0728/w_normal.csv", "Kitagawa_result_left.csv", 3, 0);
			runKitagawa("../../../lpms_datasets/0728/w_normal.csv", "Kitagawa_result_right.csv", 3, 1);
			cmdString = "standby";
		}
		else if (cmdString == "run")
		{
			//WIPtracking_realtime();
			WIPmanager_realtime(true, true, true, 2); // gazeMode, recordMode, unityMode
			//runCount++;
			cmdString = "standby";
		}
		else if (cmdString == "playback")
		{
			//WIPtracking_offline("../../../lpms_datasets/0728/wip_toebias.csv", 3);
			//WIPtracking_offline("../../../lpms_datasets/1006/dorsal_100Hz_1.csv", 2);
			//WIPmanager_offline("../../../lpms_datasets/0728/wip_neutral.csv", 3, false, 2);
			WIPmanager_offline("../../../lpms_datasets/1006/dorsal_100Hz_4.csv", 2, false, 2);
			cmdString = "standby";
		}
		else if (cmdString == "pause")
		{
			//VirtualLocomotion.exportCSV("WIPmanager_realtime.csv");
			//cout << " << WIP tracking complete." << endl;
			cout << " << runCount = " << runCount << endl;
			runCount = 0;
			cmdString = "standby";
		}
		else if (cmdString == "standby")
		{
			// do nothing (idle state)
		}
		else if (cmdString == "end")
		{
			//closeClient();
			terminateLPMS();
			runLoop = false;
		}
		else if (cmdString == "help")
		{
			cout << " << begin : Open client and connect to LPMS-B2" << endl;
			cout << " << sync : Sync time stamp for all sensors to zero" << endl;
			cout << " << record : Log data for all connected sensors" << endl;
			cout << " << save : Save data for all connected sensors" << endl;
			cout << " << calibrate : Log normal walking for user calibration" << endl;
			cout << " << kita : Offline version of 'calibrate'" << endl;
			cout << " << run : Run program" << endl;
			cout << " << playback : Offline version of 'run'" << endl;
			cout << " << pause : Pause current task" << endl;
			cout << " << end : Close client and disconnect LPMS-B2" << endl;
			cmdString = "standby";
		}
		else
		{
			cout << " << [WARNING] Invalid command! Type help to view available commands..." << endl;
			cmdString = "standby";
		}
		this_thread::sleep_for(chrono::milliseconds(1));
	}

	return 0;
}

// **************************************************************************************************************** //

void runKitagawa(const char* fin, const char* fout, int number_of_sensor, int whichfoot)
{
	// Import csv file
	csv<float> csv_kita(fin);
	csv_kita.setquietMode(true);
	vector<vector<ImuData>> lpms_kita = csv_kita.importLPMS(number_of_sensor);
	cout << " << Imported csv file." << endl;

	// Track gait motion (Kitagawa's methodology)
	Kitagawa2016 test(lpms_kita[whichfoot], 100.0f);
	test.setquietMode(true);
	test.setLPMSquatMode(false);
	test.gaitAnalysis();
	cout << test.getscaleCoeff() << endl;
	cout << " << Gait analysis complete." << endl;

	// Export csv file
	test.exportCSV(fout);
	cout << " << Tracking result exported." << endl;
}

void WIPmanager_realtime(bool gazeMode, bool recordMode, bool unityMode, int version) // previously demo0909
{
	WIPmanager VirtualLocomotion(100.0f);
	VirtualLocomotion.setquietMode(true);
	VirtualLocomotion.setrecordMode(recordMode);
	VirtualLocomotion.setscaleCoeff(26.0f*0.1f); // times 0.1f when running on Android
	VirtualLocomotion.setscaleCoeff2(6.0f*0.1f); // times 0.1f when running on Android
	int updateCount = 0;

	cout << " << Press any key to stop." << endl;
	while (!(GetAsyncKeyState(VK_RETURN) & 0x8000))
	{
		// left foot
		if (lpms_connect[0] == true && lpms[0]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[0]->hasImuData())
		{
			ImuData data = lpms[0]->getCurrentData();
			VirtualLocomotion.updateLeft(data.g[0], data.g[1], data.g[2],
				-data.a[0], -data.a[1], -data.a[2], 0.0f, 0.0f, 0.0f, 0.0f, (float)data.timeStamp);
			updateCount++;
		}
		if (updateCount > 1)
		{
			VirtualLocomotion.getZPositionSum();
			VirtualLocomotion.getLLCM();
			float speed = VirtualLocomotion.getVirtualSpeed();
			float speed2 = VirtualLocomotion.getVirtualSpeed2();
			float heading = VirtualLocomotion.getVirtualHeading();

			if (unityMode)
			{
				if (gazeMode)
				{
					if (version == 1) sendSpeed(speed, false);
					else if (version == 2) if (speed2 != speed2_history) sendSpeed(speed2, false);
				}
				else
				{
					if (version == 1) sendVelocity(speed, -sinf(heading), 0, cosf(heading), false);
					else if (version == 2) sendVelocity(speed2, -sinf(heading), 0, cosf(heading), false);
				}
			}
			else
			{
				if (version == 1) cout << "L" << speed << " " << heading << endl;
				else if (version == 2) cout << "L" << speed2 << " " << heading << endl;
			}
			speed2_history = speed2;
			updateCount = 0;
		}

		// right foot
		if (lpms_connect[1] == true && lpms[1]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[1]->hasImuData())
		{
			ImuData data = lpms[1]->getCurrentData();
			VirtualLocomotion.updateRight(data.g[0], data.g[1], data.g[2],
				-data.a[0], -data.a[1], -data.a[2], 0.0f, 0.0f, 0.0f, 0.0f, (float)data.timeStamp);
			updateCount++;
		}
		if (updateCount > 1)
		{
			VirtualLocomotion.getZPositionSum();
			VirtualLocomotion.getLLCM();
			float speed = VirtualLocomotion.getVirtualSpeed();
			float speed2 = VirtualLocomotion.getVirtualSpeed2();
			float heading = VirtualLocomotion.getVirtualHeading();

			if (unityMode)
			{
				if (gazeMode)
				{
					if (version == 1) sendSpeed(speed, false);
					else if (version == 2) if (speed2 != speed2_history) sendSpeed(speed2, false);
				}
				else
				{
					if (version == 1) sendVelocity(speed, -sinf(heading), 0, cosf(heading), false);
					else if (version == 2) sendVelocity(speed2, -sinf(heading), 0, cosf(heading), false);
				}
			}
			else
			{
				if (version == 1) cout << "R" << speed << " " << heading << endl;
				else if (version == 2) cout << "R" << speed2 << " " << heading << endl;
			}
			speed2_history = speed2;
			updateCount = 0;
		}
	}

	// Export csv file
	if (recordMode)
	{
		VirtualLocomotion.exportCSV("WIPmanager_realtime.csv");
		cout << " << Session saved." << endl;
	}
}

void WIPmanager_offline(const char* fin, int number_of_sensor, bool unityMode, int version)
{
	// Import csv file
	csv<float> csv_WIP(fin);
	csv_WIP.setquietMode(true);
	vector<vector<ImuData>> lpms_WIP = csv_WIP.importLPMS(number_of_sensor); // number_of_sensor
	cout << " << Imported csv file." << endl;

	// Track WIP motion
	WIPmanager manager(100.0f);
	manager.setquietMode(true);
	manager.setscaleCoeff(25.0f);
	manager.setscaleCoeff2(10.0f);
	for (unsigned int j = 0; j < min(lpms_WIP[0].size(), lpms_WIP[1].size()); j++)
	{
		manager.updateLeft(lpms_WIP[0][j].g[0], lpms_WIP[0][j].g[1], lpms_WIP[0][j].g[2],
			-lpms_WIP[0][j].a[0], -lpms_WIP[0][j].a[1], -lpms_WIP[0][j].a[2],
			lpms_WIP[0][j].q[0], lpms_WIP[0][j].q[1], lpms_WIP[0][j].q[2], lpms_WIP[0][j].q[3], (float)lpms_WIP[0][j].timeStamp);
		manager.updateRight(lpms_WIP[1][j].g[0], lpms_WIP[1][j].g[1], lpms_WIP[1][j].g[2],
			-lpms_WIP[1][j].a[0], -lpms_WIP[1][j].a[1], -lpms_WIP[1][j].a[2],
			lpms_WIP[1][j].q[0], lpms_WIP[1][j].q[1], lpms_WIP[1][j].q[2], lpms_WIP[1][j].q[3], (float)lpms_WIP[1][j].timeStamp);

		manager.getZPositionSum();
		manager.getLLCM();
		float speed = manager.getVirtualSpeed();
		float speed2 = manager.getVirtualSpeed2();
		float heading = manager.getVirtualHeading();

		if (unityMode)
		{
			if (version == 1) sendVelocity(speed, -sinf(heading), 0, cosf(heading), false);
			else if (version == 2) sendVelocity(speed2, -sinf(heading), 0, cosf(heading), false);
			//sendQuaternion(cosf(0.5f*heading), 0, -sinf(0.5f*heading), 0, false); // Unity uses left-hand system
			sleep_for(milliseconds(10)); // 100Hz sampling rate
		}
	}
	cout << " << WIP tracking complete." << endl;

	// Export csv file
	manager.exportCSV("WIPmanager_offline.csv");
	cout << " << testWIPmanager exported." << endl;
}

void WIPtracking_realtime()
{
	int i = 1;
	if (lpms_connect[i] == true && lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[i]->hasImuData())
	{
		// Read data
		lpms_data[i] = lpms[i]->getCurrentData();
		WIPtracker[i].update(lpms_data[i].g[0], lpms_data[i].g[1], lpms_data[i].g[2], -lpms_data[i].a[0], -lpms_data[i].a[1], -lpms_data[i].a[2],
			lpms_data[i].q[0], lpms_data[i].q[1], lpms_data[i].q[2], lpms_data[i].q[3], (float)lpms_data[i].timeStamp);
		if (WIPtracker[i].isStationary() != toggle)
		{
			toggle = WIPtracker[i].isStationary();
			if (toggle) cout << "stationary" << endl;
			else cout << "moving" << endl;
		}
		//cout << lpms_data[i].timeStamp << "s, " << WIPtracker[i].isStationary() << endl;
		//cout << lpms_data[i].timeStamp << "s, " << WIPtracker[i].getQuaternion().coeffs().transpose() << endl;
		//cout << lpms_data[i].timeStamp << "s, " << WIPtracker[i].getVelocity().transpose()(2) << endl;
		cout << lpms_data[i].timeStamp << "s, " << WIPtracker[i].getPosition().transpose() << endl;
	}
}

void WIPtracking_offline(const char* fin, int number_of_sensor) // previously demo0911
{
	// Import csv file
	csv<float> csv_WIP(fin);
	csv_WIP.setquietMode(true);
	vector<vector<ImuData>> lpms_WIP = csv_WIP.importLPMS(number_of_sensor);
	cout << " << Imported csv file." << endl;

	// Track WIP motion
	WIP WIPtracker_offline[2] = { WIP(100.0f), WIP(100.0f) };
	WIPtracker_offline[0].setrecordMode(true);
	WIPtracker_offline[0].setLPMSquatMode(false);
	WIPtracker_offline[1].setrecordMode(true);
	WIPtracker_offline[1].setLPMSquatMode(false);
	for (int i = 0; i < 2; i++)
	{
		for (unsigned int j = 0; j < lpms_WIP[i].size(); j++)
		{
			WIPtracker_offline[i].update(lpms_WIP[i][j].g[0], lpms_WIP[i][j].g[1], lpms_WIP[i][j].g[2], 
				-lpms_WIP[i][j].a[0], -lpms_WIP[i][j].a[1], -lpms_WIP[i][j].a[2], 
				lpms_WIP[i][j].q[0], lpms_WIP[i][j].q[1], lpms_WIP[i][j].q[2], lpms_WIP[i][j].q[3], (float)lpms_WIP[i][j].timeStamp);
		}
	}
	cout << " << WIP tracking complete." << endl;

	// Export csv file
	WIPtracker_offline[0].exportCSV("WIPtracking_left.csv");
	WIPtracker_offline[1].exportCSV("WIPtracking_right.csv");
	cout << " << Tracking result exported." << endl;
}

void WIPlocomotion_simple(void) // real-time WIP detection based VR locomotion previously demo0829
{
	for (int i = 0; i < 2; i++)
	{
		if (lpms_connect[i] == true && lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[i]->hasImuData())
		{
			// Read data
			lpms_data[i] = lpms[i]->getCurrentData();
			WIPtracker[i].update(lpms_data[i].g[0], lpms_data[i].g[1], lpms_data[i].g[2], lpms_data[i].a[0], lpms_data[i].a[1], lpms_data[i].a[2], 
				lpms_data[i].q[0], lpms_data[i].q[1], lpms_data[i].q[2], lpms_data[i].q[3], (float)lpms_data[i].timeStamp);
		}
	}
	if ((!WIPtracker[0].isStationary() || !WIPtracker[1].isStationary()) != toggle)
	{
		toggle = !WIPtracker[0].isStationary() || !WIPtracker[1].isStationary();
		if (toggle)
		{
			cout << "moving" << endl;
			sendVelocity(10, 0, 0, 1, false);
		}
		else
		{
			cout << "stationary" << endl;
			sendVelocity(0, 0, 0, 0, false);
		}
	}
}

void simple_demo(void) // previously demo0822
{
	for (int i = 2; i < 3; i++)
	{
		if (lpms_connect[i] == true && lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[i]->hasImuData())
		{
			// Read data
			lpms_data[i] = lpms[i]->getCurrentData();
			// Send quaternion
			string sendQuat = "38241913 ";
			// sendQuat += to_string(lpms_data[i].timeStamp);
			sendQuat += to_string(lpms_data[i].q[0]);
			sendQuat += " 0 ";
			sendQuat += to_string(lpms_data[i].q[3]);
			sendQuat += " 0";
			const char* sendQuat_c = sendQuat.c_str();
			cout << sendQuat_c << ", " << strlen(sendQuat_c) << endl;
			if (runCount % 2) send(client, sendQuat_c, strlen(sendQuat_c), 0);
		}
	}
}

void LPMSBasicExample(void) // display LPMS data
{
	for (int i = 0; i < 3; i++)
	{
		if (lpms_connect[i] == true && lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[i]->hasImuData())
		{
			// Read data
			lpms_data[i] = lpms[i]->getCurrentData();
			// Show data
			printf(" << Id=%d, Timestamp=%.3f, qW=%.3f, qX=%.3f, qY=%.3f, qZ=%.3f\n", lpms_data[i].openMatId,
				lpms_data[i].timeStamp - timeStamp0[i], lpms_data[i].q[0], lpms_data[i].q[1], lpms_data[i].q[2], lpms_data[i].q[3]);
		}
	}
}

// **************************************************************************************************************** //

void sendVelocity(double v, double vx, double vy, double vz, bool debugmode)
{
	string msg = VELOCITY_KEY;
	msg += " ";
	msg += to_string(v);
	msg += " ";
	msg += to_string(vx);
	msg += " ";
	msg += to_string(vy);
	msg += " ";
	msg += to_string(vz);
	const char* msg_c = msg.c_str();
	send(client, msg_c, strlen(msg_c), 0);
	if (debugmode) cout << msg_c << ", " << strlen(msg_c) << endl;
}

void sendQuaternion(double qw, double qx, double qy, double qz, bool debugmode)
{
	string msg = QUATERNION_KEY;
	msg += " ";
	msg += to_string(qw);
	msg += " ";
	msg += to_string(qx);
	msg += " ";
	msg += to_string(qy);
	msg += " ";
	msg += to_string(qz);
	const char* msg_c = msg.c_str();
	send(client, msg_c, strlen(msg_c), 0);
	if (debugmode) cout << msg_c << ", " << strlen(msg_c) << endl;
}

void sendSpeed(double speed, bool debugmode)
{
	string msg = GAZE_KEY;
	msg += " ";
	msg += to_string(speed);
	const char* msg_c = msg.c_str();
	send(client, msg_c, strlen(msg_c), 0);
	if (debugmode) cout << msg_c << ", " << strlen(msg_c) << endl;
}

void synctimeStamp(double time0)
{
	bool exitcond = false;
	bool flag[number_of_sensor] = { false, false, false };

	while (!exitcond)
	{
		for (int i = 0; i < number_of_sensor; i++)
		{
			if (lpms_connect[i] == true && flag[i] == 0 &&
				lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[i]->hasImuData())
			{
				timeStamp0[i] = lpms[i]->getCurrentData().timeStamp - time0;
				flag[i] = true;
			}
		}
		exitcond = true;
		for (int i = 0; i < number_of_sensor; i++) if (flag[i] != lpms_connect[i]) exitcond = false;
	}
}

void initializeLPMS(void)
{
	// List of SensorId
	SensorId.push_back("00:04:3E:30:34:3A"); // left foot
	SensorId.push_back("00:04:3E:30:34:43"); // right foot
	SensorId.push_back("00:04:3E:30:34:62"); // waist

	// Connect to sensor
	for (int i = 0; i < number_of_sensor; i++)
	{
		if (lpms_connect[i] == true)
		{
			lpms[i] = manager->addSensor(DEVICE_LPMS_B2, SensorId[i]);
			int tick = 0;
			while (lpms[i]->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED && tick != bluetooth_timeout)
			{
				this_thread::sleep_for(chrono::milliseconds(100));
				tick++;
			}
			if (tick == bluetooth_timeout)
			{
				manager->removeSensor(lpms[i]);
				lpms_connect[i] = false;
				cout << " << [WARNING] Bluetooth timeout. Unable to connect to sensor " << SensorId[i] << "." << endl;
			}
			else
			{
				lpms_connect[i] = true;
				cout << " << Connected to sensor " << SensorId[i] << "." << endl;
			}
			this_thread::sleep_for(chrono::milliseconds(100));
		}
	}
}

void terminateLPMS(void)
{
	// Remove initialized sensor
	for (int i = 0; i < number_of_sensor; i++) if (lpms_connect[i]) manager->removeSensor(lpms[i]);
	// Delete LpmsSensorManager object 
	delete manager;
}

void openClient(void)
{
	WSADATA data;
	::WSAStartup(MAKEWORD(2, 2), &data);

	client = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (client == INVALID_SOCKET) showError("Failed to generate client!");

	sockaddr_in addr = { 0 };

	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(ip_address);
	addr.sin_port = htons(22999);

	if (connect(client, (sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR) showError("Failed to connect to server!");
	else
	{
		send(client, "ConnectionOK", 12, 0);
		cout << "openClient complete" << endl;
	}
}

void closeClient(void)
{
	closesocket(client);
	::WSACleanup();
}

void showError(const char * msg)
{
	cout << " << [ERROR] " << msg << endl;
	//exit(-1);
}
