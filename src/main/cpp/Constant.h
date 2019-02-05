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

	static constexpr double pulsesPerRotationQuad = 8192;
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

	//Robot propeties
	static constexpr float DIST_FROM_RIGHT_WEEL_TO_LEFT_WEEL = 2.0;  //TODO set this to correct value
	static constexpr float DIST_CENTER_AXEL_TO_CAM = 1.0;  //TODO set this to correct value
	static constexpr float DIST_CENTER_AXEL_TO_HATCH_PANEL = 1.0;  //TODO set this to correct value
	static constexpr float HORIZONTAL_FOV = 60.0;  //TODO set this to correct value
	static constexpr float DIST_TO_MANTAIN_CAM_ANGLE = 1.0;  //TODO set this to correct value
	static constexpr float DIST_TO_MANTAIN_FINAL_ANGLE = 0.5;  //TODO set this to correct value
	static constexpr float ANGLE_MAX_ERROR = 5.0;  //TODO set this to correct value
};


#endif /* SRC_CONSTANT_H_ */
