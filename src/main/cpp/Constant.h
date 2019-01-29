/*
 * Constant.h
 *
 *  Created on: Jan 20, 2018
 *      Author: Team 6101
 */

#ifndef SRC_CONSTANT_H_
#define SRC_CONSTANT_H_


class Constant
{
public:
	static const int pidChannel = 0;

	static const int LeftLeaderID = 21;
	static const int LeftFollowerID = 22;


	static const int RightLeaderID = 11;
	static const int RightFollowerID = 12;

	static constexpr double pulsesPerRotationQuad = 2048 * 4;
	static constexpr double circumference = 6 * 3.14;

	static constexpr double tankDriveDeadbandVal = .15;
	static constexpr double arcadeDriveDeadbandVal = .2;

	// Target Calculation = (inches / Constant::circumference) * Constant::pulsesPerRotationQuad
	//		Deadband = 130 = .3" / circumference * PPR quad
	static const int autonPositionDeadbandVal = 0;

	static constexpr double leftMotionVel = 6000;
	static constexpr double leftMotionAcc = 2000;

	static constexpr double rightMotionVel = 6000;
	static constexpr double rightMotionAcc = 2000;
};


#endif /* SRC_CONSTANT_H_ */
