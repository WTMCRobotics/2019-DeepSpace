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
	static const int autonPositionDeadbandVal = 130;

	static constexpr double leftMotionVel = 0;
	static constexpr double leftMotionAcc = 0;

	static constexpr double rightMotionVel = 0;
	static constexpr double rightMotionAcc = 0;

	//Robot propeties
	static const float DIST_FROM_RIGHT_WEEL_TO_LEFT_WEEL = 2.0f;  //TODO set this to correct value
	static const float DIST_CENTER_AXEL_TO_CAM = 1.0f;  //TODO set this to correct value
	static const float DIST_CENTER_AXEL_TO_HATCH_PANEL = 1.0f;  //TODO set this to correct value
	static const float HORIZONTAL_FOV = 60.0f;  //TODO set this to correct value
	static const float DIST_TO_MANTAIN_CAM_ANGLE = 1.0f;  //TODO set this to correct value
	static const float DIST_TO_MANTAIN_FINAL_ANGLE = 0.5f;  //TODO set this to correct value
	static const float ANGLE_MAX_ERROR = 5.0f;  //TODO set this to correct value
};


#endif /* SRC_CONSTANT_H_ */
