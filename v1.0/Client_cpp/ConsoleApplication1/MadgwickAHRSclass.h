//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRSclass_h
#define MadgwickAHRSclass_h
#include <math.h>
#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace Eigen;

//--------------------------------------------------------------------------------------------
// Variable declaration
class MadgwickAHRS {
private:
	float beta;				// algorithm gain
	float q0;
	float q1;
	float q2;
	float q3;	// quaternion of sensor frame relative to auxiliary frame
	float sampleFreq;
	float invSampleFreq;
	float invSqrt(float x);

	//-------------------------------------------------------------------------------------------
	// Function declarations
public:
	MadgwickAHRS(void);
	MadgwickAHRS(float sampleFrequency, float beta);
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	Quaternionf getQuaternion();
};
#endif