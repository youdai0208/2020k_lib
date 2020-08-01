/*
 * IMUFilter.hpp
 *
 *  Created on: 2019/06/20
 *      Author: youda
 */

#ifndef MADGWICK_HPP_
#define MADGWICK_HPP_

#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
	static constexpr float sampleFreqDef = 512.0f;          // sample frequency in Hz
	static constexpr float betaDef = 0.1f;            // 2 * proportional gain
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick();
    ~Madgwick();
    inline void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    //float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    //float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    //float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    inline float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    inline float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    inline float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f/* + 180.0f*/;
    }
    inline float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    inline float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    inline float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
};

#endif /* MADGWICK_HPP_ */
