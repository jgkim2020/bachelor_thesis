#include "WIPmanager.h"

void WIPmanager::updateLeft(float gx, float gy, float gz, float ax, float ay, float az, float qw, float qx, float qy, float qz, float timestamp)
{
	left.update(gx, gy, gz, ax, ay, az, qw, qx, qy, qz, timestamp);
	leftLatestTimeStamp = timestamp;
}

void WIPmanager::updateRight(float gx, float gy, float gz, float ax, float ay, float az, float qw, float qx, float qy, float qz, float timestamp)
{
	right.update(gx, gy, gz, ax, ay, az, qw, qx, qy, qz, timestamp);
	rightLatestTimeStamp = timestamp;
}

void WIPmanager::setquietMode(bool arg)
{
	quietMode = arg;
}

void WIPmanager::setrecordMode(bool arg)
{
	recordMode = arg;
}

void WIPmanager::setplaybackMode(bool arg)
{
	playbackMode = arg;
}

void WIPmanager::setscaleCoeff(float scaling)
{
	scaleCoeff = scaling;
}

void WIPmanager::setscaleCoeff2(float scaling)
{
	scaleCoeff2 = scaling;
}

float WIPmanager::getVirtualSpeed() // previously getLocomotion1030
{
	// Recover left foot horizontal velocity (with padding)
	leftState_vv[0] = leftState_vv[1];
	leftState_vv[1] = left.getstate_vv();
	float leftVel = 0.0f;
	float leftPos = left.getPosition()[2];
	if (leftState_vv[1] == 4)
	{
		if (leftState_vv[0] != leftState_vv[1]) leftPosPadding = leftPos;
		leftVel = 0.5f*scaleCoeff*(leftPos + leftPosPadding);
	}
	else if (leftState_vv[1] > 0) leftVel = scaleCoeff*leftPos;
	else leftVel = 0.0f;
	if (recordMode)
	{
		recordLeftState.push_back(leftState_vv[1]);
		recordLeftSynthVel.push_back(leftVel);
		recordLeftZPos.push_back(leftPos);
	}

	// Recover right foot horizontal velocity
	rightState_vv[0] = rightState_vv[1];
	rightState_vv[1] = right.getstate_vv();
	float rightVel = 0.0f;
	float rightPos = right.getPosition()[2];
	if (rightState_vv[1] == 4)
	{
		if (rightState_vv[0] != rightState_vv[1]) rightPosPadding = rightPos;
		rightVel = 0.5f*scaleCoeff*(rightPos + rightPosPadding);
	}
	else if (rightState_vv[1] > 0) rightVel = scaleCoeff*rightPos;
	else rightVel = 0.0f;
	if (recordMode)
	{
		recordRightState.push_back(rightState_vv[1]);
		recordRightSynthVel.push_back(rightVel);
		recordRightZPos.push_back(rightPos);
	}

	// Set padding
	if (leftState_vv[1] + rightState_vv[1] < 5) paddingFlag = true;
	else if (paddingFlag)
	{
		paddingFlag = false;
		padding = 0.5f*(0.5f*(rightVel + leftVel) + locomotionLPF.getFilteredData());
		paddingCount++;
		// For debugging purposes
		if(!quietMode) cout << paddingCount << ", " << leftLatestTimeStamp << "s, " << padding << endl;
	}

	// Set locomotion speed
	float returnSpeed = 0.0f;
	if (leftState_vv[1] + rightState_vv[1] == 0)
	{
		locomotionLPF.update(0.0f);
		returnSpeed = 0.0f;
		if (recordMode) recordVirtualSpeed.push_back(0.0f);
	}
	else
	{
		float rawSpeed = 0.5f*(rightVel + leftVel);
		if (rawSpeed < padding) rawSpeed = padding;
		returnSpeed = locomotionLPF.update(rawSpeed);
		if (recordMode) recordVirtualSpeed.push_back(returnSpeed);
	}
	return returnSpeed;
}

float WIPmanager::getVirtualSpeed2()
{
	leftState_vv2[0] = leftState_vv2[1];
	leftState_vv2[1] = left.getstate_vv();
	rightState_vv2[0] = rightState_vv2[1];
	rightState_vv2[1] = right.getstate_vv();
	float leftPos = left.getPosition()[2];
	float rightPos = right.getPosition()[2];

	// find WIP period
	if (leftState_vv2[0] == 0 && leftState_vv2[1] == 1)
	{
		FO_leftTime = leftLatestTimeStamp;
		FO_leftTrigger = true;
	}
	if (leftState_vv2[0] == 2 && leftState_vv2[1] == 3)
	{
		FC_leftTime = leftLatestTimeStamp;
		FC_leftTrigger = true;
	}
	if (FO_leftTrigger && FC_leftTrigger)
	{
		FO_leftTrigger = false;
		FC_leftTrigger = false;
		if (FC_leftTime > FO_leftTime) WIPPeriod = 2*(FC_leftTime - FO_leftTime);
	}
	if (rightState_vv2[0] == 0 && rightState_vv2[1] == 1)
	{
		FO_rightTime = rightLatestTimeStamp;
		FO_rightTrigger = true;
	}
	if (rightState_vv2[0] == 2 && rightState_vv2[1] == 3)
	{
		FC_rightTime = rightLatestTimeStamp;
		FC_rightTrigger = true;
	}
	if (FO_rightTrigger && FC_rightTrigger)
	{
		FO_rightTrigger = false;
		FC_rightTrigger = false;
		if (FC_rightTime > FO_rightTime) WIPPeriod = 2*(FC_rightTime - FO_rightTime);
	}

	// generate speed
	if (leftState_vv2[1] + rightState_vv2[1] != 0) // in motion
	{
		stationaryCount = 0;
		if (leftState_vv2[0] == 2 && leftState_vv2[1] == 3) maxPos = leftPos;
		if (rightState_vv2[0] == 2 && rightState_vv2[1] == 3) maxPos = rightPos;
		if (maxPos == 0.0f) returnSpeed2 = scaleCoeff2*(leftPos + rightPos); // startup
		else 
		{
			if (maxPos < 0.0f) returnSpeed2 = 0.0f;
			else returnSpeed2 = scaleCoeff2*maxPos*2/(WIPPeriod + 1);
		}
	}
	else // stationary
	{
		if (stationaryCount > 6) // timeout (1/samplingFreq seconds per count)
		{
			returnSpeed2 = 0.0f;
			maxPos = 0.0f;
			WIPPeriod = 0.0f;
		}
		else stationaryCount++;
	}
	if (recordMode) recordVirtualSpeed2.push_back(returnSpeed2);
	return returnSpeed2;
}

void WIPmanager::syncVirtualHeading()
{
	// Get initial quaternion
	leftQuat0 = left.getQuaternion();
	rightQuat0 = right.getQuaternion();
}

float WIPmanager::getVirtualHeading()
{
	// Compute relative quaternion to initial quaternion
	leftQuat = left.getQuaternion()*(leftQuat0.conjugate());
	rightQuat = right.getQuaternion()*(leftQuat0.conjugate());
	
	// Find yaw angle (Psi) of range -pi ~ pi
	if (leftQuat.w() < 0)
	{
		leftQuat.w() = -leftQuat.w();
		leftQuat.x() = -leftQuat.x();
		leftQuat.y() = -leftQuat.y();
		leftQuat.z() = -leftQuat.z();
	}
	float leftPsi = 2.0f*atan2f(leftQuat.z(), leftQuat.w());
	if (rightQuat.w() < 0)
	{
		rightQuat.w() = -rightQuat.w();
		rightQuat.x() = -rightQuat.x();
		rightQuat.y() = -rightQuat.y();
		rightQuat.z() = -rightQuat.z();
	}
	float rightPsi = 2.0f*atan2f(rightQuat.z(), rightQuat.w());

	// Get virtual heading
	float Y = sinf(leftPsi) + sinf(rightPsi);
	float X = cosf(leftPsi) + cosf(rightPsi);
	if (X*X + Y*Y < 1e-32) virtualHeading = 100.0f; // Domain error handling
	else virtualHeading = atan2f(Y, X); // range -pi ~ pi
	if (recordMode) recordVirtualHeading.push_back(virtualHeading);

	return virtualHeading;
}

float WIPmanager::getZPositionSum() // Simple implementation
{
	if (recordMode)
	{
		recordZPosSum.push_back(left.getPosition()[2] + right.getPosition()[2]);
		return recordZPosSum.back();
	}
	else return left.getPosition()[2] + right.getPosition()[2];
}

float WIPmanager::getLLCM() // LLCM-WIP as benchmark
{
	// LLCM
	leftZpos_ = leftZpos;
	leftZpos = left.getPosition()[2];
	rightZpos_ = rightZpos;
	rightZpos = right.getPosition()[2];
	float leftZvel = leftLPF.update(fabsf((leftZpos - leftZpos_)*sampleFrequency));
	float rightZvel = rightLPF.update(fabsf((rightZpos - rightZpos_)*sampleFrequency));
	if (recordMode)
	{
		recordLLCM.push_back(leftZvel + rightZvel);
		return recordLLCM.back();
	}
	else return leftZvel + rightZvel;
}

int WIPmanager::exportCSV(const char* foutName)
{
	if (!recordMode)
	{
		cout << " << recordMode false! Ignoring exportCSV command..." << endl;
		return 0;
	}

	csv<float> csvout(foutName);

	// Write header
	vector<string> header;
	header.push_back("timeStamp"); header.push_back("leftZPos"); header.push_back("rightZPos"); header.push_back("leftSynthVel"); header.push_back("rightSynthVel");
	header.push_back("virtualSpeed"); header.push_back("virtualSpeed2"); header.push_back("virtualHeading");
	header.push_back("virtualXPos"); header.push_back("virtualYPos"); header.push_back("virtualDistanceTraveled");
	header.push_back("virtualXPos2"); header.push_back("virtualYPos2"); header.push_back("virtualDistanceTraveled2");
	header.push_back("LeftState"); header.push_back("RightState"); header.push_back("StateSum");

	// Write data
	vector<vector<float>> fields;
	float virtualXPos = 0.0f;
	float virtualYPos = 0.0f;
	float virtualDist = 0.0f;
	float virtualXPos2 = 0.0f;
	float virtualYPos2 = 0.0f;
	float virtualDist2 = 0.0f;
	for (unsigned int i = 0; i < recordZPosSum.size(); i++)
	{
		fields.push_back(vector<float>());
		fields.back().push_back(i/sampleFrequency);
		fields.back().push_back(recordLeftZPos[i]);
		fields.back().push_back(recordRightZPos[i]);
		fields.back().push_back(recordLeftSynthVel[i]);
		fields.back().push_back(recordRightSynthVel[i]);
		fields.back().push_back(recordVirtualSpeed[i]);
		fields.back().push_back(recordVirtualSpeed2[i]);
		fields.back().push_back(recordVirtualHeading[i]);
		virtualXPos -= recordVirtualSpeed[i]*sinf(recordVirtualHeading[i])/sampleFrequency;
		virtualYPos += recordVirtualSpeed[i]*cosf(recordVirtualHeading[i])/sampleFrequency;
		virtualDist += recordVirtualSpeed[i]/sampleFrequency;
		fields.back().push_back(virtualXPos);
		fields.back().push_back(virtualYPos);
		fields.back().push_back(virtualDist);
		virtualXPos2 -= recordVirtualSpeed2[i] * sinf(recordVirtualHeading[i]) / sampleFrequency;
		virtualYPos2 += recordVirtualSpeed2[i] * cosf(recordVirtualHeading[i]) / sampleFrequency;
		virtualDist2 += recordVirtualSpeed2[i] / sampleFrequency;
		fields.back().push_back(virtualXPos2);
		fields.back().push_back(virtualYPos2);
		fields.back().push_back(virtualDist2);
		fields.back().push_back((float)recordLeftState[i]);
		fields.back().push_back((float)recordRightState[i]);
		fields.back().push_back((float)recordLeftState[i] + (float)recordRightState[i]);
	}
	
	csvout.write(header, fields);
	return 0;
}