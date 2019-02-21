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

	static const int IntakeLeaderID = 31;
	static const int IntakeFollowerID = 41;

	static const int ArmLeaderID = 32;

	static constexpr double pulsesPerRotationQuad = 8192;
	static constexpr double circumference = 8 * 3.14;

	static constexpr double tankDriveDeadbandVal = .15;
	static constexpr double arcadeDriveDeadbandVal = .2;

	static constexpr double moveSpeed = 1;

	// Target Calculation = (inches / Constant::circumference) * Constant::pulsesPerRotationQuad
	//		Deadband = 130 = .3" / circumference * PPR quad
	static const int autonPositionDeadbandVal = 125;

	static constexpr double leftMotionVel = 6000;
	static constexpr double leftMotionAcc = 2000;

	static constexpr double rightMotionVel = 6000;
	static constexpr double rightMotionAcc = 2000;

	static constexpr int MAX_AUTON_INSTRUCTIONS = 100;

	//Robot propeties
	static constexpr float DIST_FROM_RIGHT_WEEL_TO_LEFT_WEEL = 2.0;  //TODO set this to correct value
	static constexpr float DIST_CENTER_AXEL_TO_CAM = 1.0;  //TODO set this to correct value
	static constexpr float DIST_CENTER_AXEL_TO_HATCH_PANEL = 1.0;  //TODO set this to correct value
	static constexpr float HORIZONTAL_FOV = 45.0;  //TODO set this to correct value
	static constexpr float DIST_TO_MANTAIN_CAM_ANGLE = 1.0;  //TODO set this to correct value
	static constexpr float DIST_TO_MANTAIN_FINAL_ANGLE = 0.5;  //TODO set this to correct value
	static constexpr float ANGLE_MAX_ERROR = 5.0;  //TODO set this to correct value
	static constexpr float VERTICAL_FOV = 33.75;
	static constexpr float CAMERA_HEIGHT = 16.0;
	static constexpr float CAMERA_CENTER_LINE_DISTANCE = 24.0;
};


#endif /* SRC_CONSTANT_H_ */
