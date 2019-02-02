/*
 * PIDGyroSource.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: Team 6101
 */

#include <PIDGyroSource.h>

PIDGyroSource::PIDGyroSource(AHRS* gyro) {
	pGyro = gyro;
}

PIDGyroSource::~PIDGyroSource() {
	// TODO Auto-generated destructor stub
}

double PIDGyroSource::PIDGet()
{
	return pGyro->GetYaw();
}

//frc::PIDSourceType PIDGyroSource::getPIDSourceType() {
//	return frc::PIDSourceType.kDisplacement;
//}