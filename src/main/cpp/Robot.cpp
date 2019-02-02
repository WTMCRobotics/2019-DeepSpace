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

#include "PIDMotorOutput.h"
#include "PIDGyroSource.h"
#include <frc/PIDController.h>



#include "Vision.h"

using namespace frc;
using namespace std;

class Robot : public frc::TimedRobot
{

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<string> m_chooser;
	const string kAutoNameDefault = "Default";
	const string kAutoNameCustom = "My Auto";
	string m_autoSelected;

	

	XboxController xboxController{0};
	//Joystick joystick2, joystick2;
	double leftjoyY;
	double rightjoyY;
	double rightjoyX;
	double leftTarget;
	double rightTarget;

	//triggers
	double rightTrigger;
	double leftTrigger;
	bool rightShoulder;
	bool leftShoulder;

	//PID for turning
	PIDMotorOutput pidMotorOutput { &leftLeader, &rightLeader };
	PIDGyroSource pidGyroSource { &gyro };
	PIDController pidAngle { .0125, 0, 0.01, &pidGyroSource, &pidMotorOutput, 0.02 };

	//wether input comes from joystick or auton
	bool isAuton = false;
	float autonInstructions [100] = {0};  //Odd positions like autonInstructions[1] are distance and Even ones are angles they are exicuted in order from greates to least

	//raspi input
	vision_frame_t frame;

	AHRS *gyro = new AHRS(SPI::Port::kMXP);

	TalonSRX leftLeader {Constant::LeftLeaderID};
	TalonSRX leftFollower {Constant::LeftFollowerID};
	TalonSRX rightLeader {Constant::RightLeaderID};
	TalonSRX rightFollower {Constant::RightFollowerID};
	
	uint8_t vision_threshold;

public:
	SerialPort serial_port = SerialPort(9600);
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
		cout << "Auto selected: " << m_autoSelected << endl;

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
		Drive();
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
		

		cout << rightLeader.GetSelectedSensorPosition() << endl;
		
		//cout << works << endl;
	}

	void ResetEncoders()
	{
		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 0);
	}

	void TeleopPeriodic()
	{
		//cout << DriveDistance(6 * 3.14) << endl;
		//DriveDistance(1);
		Drive();
		//cout << gyro->GetAngle() << endl;
		
		//cout << rightLeader.GetSelectedSensorPosition() << endl;
		//cout << leftLeader.GetSelectedSensorPosition() << endl;
	}

	void Drive() {
		UpdateControllerInputs();
		//UpdateRaspiInput();
		isAuton = leftShoulder;

		if(isAuton) {
			if(DriveDistance(10)){
				ResetEncoders();
			}
		}
		else {
			leftTarget = leftjoyY;
      		//cout << "leftTarget: " << leftTarget << endl;
			rightTarget = rightjoyY;
      		//cout << "rightTarget: " << rightTarget << endl;

			leftLeader.Set(ControlMode::PercentOutput, leftTarget);
			rightLeader.Set(ControlMode::PercentOutput, -rightTarget);

			//Right motor move
			
		}


		

		//cout << "LeftMotionVel: " << (Constant::leftMotionVel == 0 ? "true" : "false") << "\n";
		//cout << "LeftMotionAcc: " << (Constant::leftMotionAcc == 0 ? "true" : "false") << "\n";
		//cout << "RightMotionVel: " << (Constant::rightMotionVel == 0 ? "true" : "false") << "\n";
		//cout << "RightMotionAcc: " << (Constant::rightMotionAcc == 0 ? "true" : "false") << "\n";
	}

	bool AutonPositionDeadband(double value, int target) {
		if (fabs(target - value) < Constant::autonPositionDeadbandVal) {
			return true; 
		} else {
			return false;
		}
	}


	bool DriveDistance(double inches) {
		// inches / circumference = number of rotations
		// * pulsesPerRotationQuad = number of pulses in one rotation
		// targetEncPos = position encoder should read
		//int targetEncPos = (inches / Constant::circumference) * Constant::pulsesPerRotationQuad;
		
		int targetEncPos = (inches / Constant::circumference) * Constant::pulsesPerRotationQuad;

		if (AutonPositionDeadband(leftLeader.GetSelectedSensorPosition(Constant::pidChannel), targetEncPos)) {
			leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			return true;
		}
		cout << "rightLeader.GetSelectedSensorPosition(): " << rightLeader.GetSelectedSensorPosition() << endl;

		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetEncPos);
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetEncPos);

		return false;
	}


	bool turnAngle(float angle, bool wideAngle) {
		// positive angle is clockwise
		// negative angle is counterclockwise
		
		if (!wideAngle) {
			
		
		} else if(wideAngle) {
			
			if (angle > 0 ) {
		
		

			} else if(angle <= 0) {

		

			}	

		}


		return false;
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
		leftLeader.ConfigMotionCruiseVelocity(Constant::leftMotionVel, 0);
		leftLeader.ConfigMotionAcceleration(Constant::leftMotionAcc, 0);
		leftLeader.SetSensorPhase(false);
		leftLeader.SetInverted(false);

		leftFollower.ConfigNominalOutputForward(0, 0);
		leftFollower.ConfigNominalOutputReverse(0, 0);
		leftFollower.ConfigPeakOutputForward(1, 0);
		leftFollower.ConfigPeakOutputReverse(-1, 0);
		leftFollower.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		leftFollower.ConfigMotionCruiseVelocity(0, 0);
		leftFollower.ConfigMotionAcceleration(0, 0);
		leftFollower.SetSensorPhase(false);
		leftFollower.SetInverted(false);


		//Right motor setup
		rightLeader.ClearStickyFaults(0);
		//rightLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		rightLeader.ConfigNominalOutputForward(0, 0);
		rightLeader.ConfigNominalOutputReverse(0, 0);
		rightLeader.ConfigPeakOutputForward(1, 0);
		rightLeader.ConfigPeakOutputReverse(-1, 0);
		rightLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightLeader.ConfigMotionCruiseVelocity(Constant::rightMotionVel, 0);
		rightLeader.ConfigMotionAcceleration(Constant::rightMotionAcc, 0);
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
		leftLeader.Config_kP(Constant::pidChannel, .69, 0);
		leftLeader.Config_kI(Constant::pidChannel, 0.0000, 0);
		leftLeader.Config_kD(Constant::pidChannel, 0.0484, 0);
		leftLeader.Config_IntegralZone(Constant::pidChannel, 0, 0);

		rightLeader.Config_kP(Constant::pidChannel, .69, 0);
		rightLeader.Config_kI(Constant::pidChannel, 0.0000, 0);
		rightLeader.Config_kD(Constant::pidChannel, 0.0484, 0);
		rightLeader.Config_IntegralZone(Constant::pidChannel, 0, 0);

		rightFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, Constant::RightLeaderID);
		leftFollower.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, Constant::LeftLeaderID);

	}

	//This Method will get inputs from raspi
	void UpdateRaspiInput()
	{
		frame = getFrame(&serial_port);
		if (frameIsInfo(frame)) {
			vision_info_t vinfo = getInfo(frame);
			vision_threshold = (uint8_t)vinfo.threshold;
		}
		// TODO: Make the robot actually do stuff
	}


	void UpdateControllerInputs()
	{
		//Tank drive both stick
		leftjoyY = xboxController.GetY(frc::GenericHID::JoystickHand::kLeftHand);
		rightjoyY = xboxController.GetY(frc::GenericHID::JoystickHand::kRightHand);
		
		//Auton overide
		leftShoulder = xboxController.GetBumper(frc::GenericHID::JoystickHand::kLeftHand);

		

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
/*
	void setArmAngle(float angle){

	}

	void setPistonExtended(int pistonID){

	}

	bool getPistonExtended(int pistonID){

	}

	void placeHatch(){

	}

	void placeCargo(){

	}

	void retrieveHatch(){

	}

	void ejectCargo(){

	}*/
	//stuff Elliot needs to do
	//setArmAngle(float angle)
	//setPistonExtended(int pistonID)
	//getPistonExtended(int pistonID)
	//placeHatch()
	//placeCargo(float rocket or ship angle)
	//retrieveHatch()
	//ejectCargo()


};

START_ROBOT_CLASS(Robot);
