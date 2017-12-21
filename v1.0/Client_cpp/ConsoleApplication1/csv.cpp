#ifndef csv_cpp
#define csv_cpp

#include "csv.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "ImuData.h"
using namespace std;

template <class T>
void csv<T>::read()
{
	ifstream in(filename);
	if (in)
	{
		string line;
		bool isheader = true;
		while (getline(in, line))
		{
			stringstream sep(line);
			string field;
			if (!isheader)
			{
				fields.push_back(vector<T>());
				while (getline(sep, field, ','))
				{
					fields.back().push_back((T)stod(field));
				}
			}
			else
			{
				while (getline(sep, field, ','))
				{
					header.push_back(field);
				}
				isheader = false;
			}
		}
	}
}

template <class T>
vector<vector<ImuData>> csv<T>::importLPMS(int number_of_sensor)
{
	read();
	int colSize = header.size();
	int rowSize = fields.size();
	if (!quietMode)
	{
		cout << "Column size : " << colSize << endl << "Row size : " << rowSize << endl;
	}
	vector<vector<ImuData>> lpms_data;
	for (int i = 0; i < number_of_sensor; i++) lpms_data.push_back(vector<ImuData>());

	/* column headers
	00 SensorId, TimeStamp, FrameNumber, AccX (g), AccY (g), AccZ (g), GyroX (deg/s),
	07 GyroY (deg/s), GyroZ (deg/s), MagX (uT), MagY (uT), MagZ (uT), EulerX (deg),
	13 EulerY (deg), EulerZ (deg), QuatW, QuatX, QuatY, QuatZ, LinAccX (g), LinAccY (g),
	21 LinAccZ (g), Pressure (kPa), Altitude (m), Temperature (degC), HeaveMotion (m) */
	for (int i = 0; i < rowSize; i++)
	{
		ImuData temp;
		temp.openMatId = (int)fields[i][0];
		int idx = temp.openMatId - 1;
		temp.timeStamp = (double)fields[i][1];
		temp.frameCount = (int)fields[i][2];
		for (int j = 0; j < 3; j++)
		{
			temp.a[j] = fields[i][3 + j];
			temp.g[j] = fields[i][6 + j];
			temp.q[j] = fields[i][15 + j];
			temp.linAcc[j] = fields[i][19 + j];
		}
		temp.q[3] = fields[i][18];
		lpms_data[idx].push_back(temp);
	}

	// check data integrity
	for (int i = 0; i < number_of_sensor; i++)
	{
		if (lpms_data[i].back().frameCount + 1 != lpms_data[i].size() && !quietMode)
		{
			cout << " << [WARNING] Sensor " << i + 1 << " packet loss!" << endl;
		}
	}

	return lpms_data;
}

template <class T>
void csv<T>::write(vector<string>& wheader, vector<vector<T>>& wfields)
{
	ofstream out(filename);
	for (auto it = wheader.begin(); it != wheader.end(); it++)
	{
		out << *it << ",";
	}
	out << endl;
	for (auto it = wfields.begin(); it != wfields.end(); it++)
	{
		for (auto it2 = it->begin(); it2 != it->end(); it2++)
		{
			out << *it2 << ",";
		}
		out << endl;
	}
}

template <class T>
vector<string> csv<T>::getheader()
{
	if (!quietMode)
	{
		for (auto it = header.begin(); it != header.end(); ++it)
		{
			cout << *it;
		}
	}
	return header;
}

template <class T>
vector<vector<T>> csv<T>::getfields()
{
	if (!quietMode)
	{
		for (auto row : fields)
		{
			for (auto field : row)
			{
				cout << field << ' ';
			}
			cout << '\n';
		}
	}
	return fields;
}

template <class T>
void csv<T>::setquietMode(bool arg)
{
	quietMode = arg;
}

#endif