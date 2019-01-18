/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>


#include <frc/WPILib.h>

#include <frc/liveWindow/LiveWindow.h>
#include <frc/smartDashboard/SendableChooser.h>
#include <frc/smartDashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>

#include <frc/Spark.h>
#include "ctre/Phoenix.h"
#include "Constant.h"

#include "AHRS.h"
#include <frc/Joystick.h>
#include <frc/XboxController.h>

using namespace frc;

class Robot : public frc::TimedRobot
{

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

	XboxController xboxController{0};
	//Joystick joystick2, joystick2;
	double leftjoyY;
	double rightjoyY;
	double rightjoyX;
	double leftTarget;
	double rightTarget;

	//wether input comes from joystick or auton
	bool isAuton = false;

	AHRS *gyro = new AHRS(SPI::Port::kMXP);

	TalonSRX leftLeader {Constant::LeftLeaderID};
	TalonSRX leftFollower {Constant::LeftFollowerID};
	TalonSRX rightLeader {Constant::RightLeaderID};
	TalonSRX rightFollower {Constant::RightFollowerID};

public:
	void RobotInit()
	{
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select between different autonomous modes using the dashboard. The
	 * sendable chooser code works with the Java SmartDashboard. If you
	 * prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the GetString line to get the auto name from the text box
	 * below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override
	{
		m_autoSelected = m_chooser.GetSelected();

		// m_autoSelected = SmartDashboard::GetString("Auto Selector",
		//		 kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom)
		{
			// Custom Auto goes here
		}
		else
		{
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if (m_autoSelected == kAutoNameCustom)
		{
			// Custom Auto goes here
		}
		else
		{
			// Default Auto goes here
		}
	}

	void TeleopInit()
	{
		SetupMoters();
	}

	void TeleopPeriodic()
	{

		Drive();
		std::cout << gyro->GetAngle() << std::endl;
		//Colin said to remove this line
		//std::cout << rightLeader<< std::endl;
	}

	void TestPeriodic() {}

	void Drive() {
		UpdateJoystickTank();

		if(isAuton){

		}
		else
		{
			//Right motor move, negative value = forward
			leftTarget = leftjoyY / 3;
			rightTarget = rightjoyY / 3;
		}


		leftLeader.Set(ControlMode::PercentOutput, leftTarget);
		//leftFollower.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, leftLeader.GetMotorOutputPercent());
		//leftFollower.Set(ControlMode::PercentOutput, leftTarget);

		//Right motor move
		rightLeader.Set(ControlMode::PercentOutput, -rightTarget);

		std::cout << "LeftMotionVel: " << (Constant::leftMotionVel == 0 ? "true" : "false") << "\n";
		std::cout << "LeftMotionAcc: " << (Constant::leftMotionAcc == 0 ? "true" : "false") << "\n";
		std::cout << "RightMotionVel: " << (Constant::rightMotionVel == 0 ? "true" : "false") << "\n";
		std::cout << "RightMotionAcc: " << (Constant::rightMotionAcc == 0 ? "true" : "false") << "\n";
	}

	void SetupMoters()
	{
		//Right motor setup
		leftLeader.ClearStickyFaults(0);
		//leftLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		leftLeader.ConfigNominalOutputForward(0, 0);
		leftLeader.ConfigNominalOutputReverse(0, 0);
		leftLeader.ConfigPeakOutputForward(1, 0);
		leftLeader.ConfigPeakOutputReverse(-1, 0);
		leftLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		leftLeader.ConfigMotionCruiseVelocity(0, 0);
		leftLeader.ConfigMotionAcceleration(0, 0);
		leftLeader.SetSensorPhase(false);
		leftLeader.SetInverted(true);

		leftFollower.ConfigNominalOutputForward(0, 0);
		leftFollower.ConfigNominalOutputReverse(0, 0);
		leftFollower.ConfigPeakOutputForward(1, 0);
		leftFollower.ConfigPeakOutputReverse(-1, 0);
		leftFollower.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		leftFollower.ConfigMotionCruiseVelocity(0, 0);
		leftFollower.ConfigMotionAcceleration(0, 0);
		leftFollower.SetSensorPhase(false);
		leftFollower.SetInverted(true);


		//Right motor setup
		rightLeader.ClearStickyFaults(0);
		//rightLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		rightLeader.ConfigNominalOutputForward(0, 0);
		rightLeader.ConfigNominalOutputReverse(0, 0);
		rightLeader.ConfigPeakOutputForward(1, 0);
		rightLeader.ConfigPeakOutputReverse(-1, 0);
		rightLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightLeader.ConfigMotionCruiseVelocity(0, 0);
		rightLeader.ConfigMotionAcceleration(0, 0);
		rightLeader.SetSensorPhase(false);
		rightLeader.SetInverted(true);

		rightFollower.ConfigNominalOutputForward(0, 0);
		rightFollower.ConfigNominalOutputReverse(0, 0);
		rightFollower.ConfigPeakOutputForward(1, 0);
		rightFollower.ConfigPeakOutputReverse(-1, 0);
		rightFollower.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightFollower.ConfigMotionCruiseVelocity(0, 0);
		rightFollower.ConfigMotionAcceleration(0, 0);
		rightFollower.SetSensorPhase(false);
		rightFollower.SetInverted(true);

		// PID Setup
		// leftLeader.Config_kP(Constant::pidChannel, .09, 0);
		// leftLeader.Config_kI(Constant::pidChannel, 0, 0);
		// leftLeader.Config_kD(Constant::pidChannel, 0, 0);
		// leftLeader.Config_IntegralZone(Constant::pidChannel, 0, 0);

		// rightLeader.Config_kP(Constant::pidChannel, .09, 0);
		// rightLeader.Config_kI(Constant::pidChannel, 0, 0);
		// rightLeader.Config_kD(Constant::pidChannel, 0, 0);
		// rightLeader.Config_IntegralZone(Constant::pidChannel, 0, 0);

		rightFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, Constant::RightLeaderID);
		leftFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, Constant::LeftLeaderID);

	}


	void UpdateJoystickTank()
	{
		//Tank drive both stick
		leftjoyY = xboxController.GetY(frc::GenericHID::JoystickHand::kLeftHand);
		rightjoyY = xboxController.GetY(frc::GenericHID::JoystickHand::kRightHand);

#ifdef FINEMOTIONCONTROL
		//If right trigger pressed
		if((xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) >0.1)  && !(xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand))) {
			//Kinda slow down
			leftjoyY = leftjoyY / 1.5;
			rightjoyY = rightjoyY / 1.5;
		}
		//If both triggers pressed
		if((xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand)) >0.1 && (xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) >0.1)) {
			//Really slow down
			leftjoyY = leftjoyY / 3;
			rightjoyY = rightjoyY / 3;
		}
#endif
	}


};

START_ROBOT_CLASS(Robot);
