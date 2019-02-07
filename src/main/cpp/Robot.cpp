#include <iostream>
#include <string>
#include <chrono>

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

#include <math.h>
#define PI 3.14159265

#include "Vision.h"

using namespace frc;
using namespace std;

class Robot : public frc::TimedRobot {

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
	double rightTrigger;//not used yet
	double leftTrigger;//not used yet
	bool rightShoulder;//not used yet
	bool leftShoulder;

	//Auton
	bool isAuton = false;
	bool isOffHab = false;
	
	//raspi input
	vision_frame_t frame;

	AHRS* pGyro = new AHRS(SPI::Port::kMXP);

	TalonSRX leftLeader {Constant::LeftLeaderID};
	TalonSRX leftFollower {Constant::LeftFollowerID};
	TalonSRX rightLeader {Constant::RightLeaderID};
	TalonSRX rightFollower {Constant::RightFollowerID};
	
	uint8_t vision_threshold;
	chrono::steady_clock time_clock;
	chrono::steady_clock::time_point last_frame_time;
	unsigned int missed_frames = 0;

	//PID for turning
	PIDMotorOutput pidMotorOutput { &leftLeader, &rightLeader };
	PIDGyroSource pidGyroSource { pGyro };
	PIDController pidAngle { .0073, 0, 0, &pidGyroSource, &pidMotorOutput, 0.02 };

public:
	float autonInstructions [100] = {};  //Even positions like autonInstructions[2] are distance and Odd ones are angles they are exicuted in order from greates to least


	SerialPort serial_port = SerialPort(9600, SerialPort::Port::kUSB);

	//this gets called once at the beggining of the match
	void RobotInit() {
		SetupMoters();
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		serial_port.SetTimeout(.05);
	}

	//configures the motors should be called at begining of match
	void SetupMoters() {
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

		// PID setup for driving a distance
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

	//this does nothing in 2019
	void AutonomousInit() override {
		m_autoSelected = m_chooser.GetSelected();

		// m_autoSelected = SmartDashboard::GetString("Auto Selector",
		//		 kAutoNameDefault);
		cout << "Auto selected: " << m_autoSelected << endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	//this code does nothing in 2019
	void AutonomousPeriodic() {
		Drive();
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	//is called once at the begining of the mathc in 2019
	void TeleopInit() {
		ResetGyro();
		ResetEncoders();
	}

	//this code gets call once every tick or about 50 times per second
	void TeleopPeriodic() {
		Drive();
		//cout << DriveDistance(6 * 3.14, 1) << endl;
		//DriveDistance(1, 1);
		//cout << gyro->GetAngle() << endl;
		
		//cout << rightLeader.GetSelectedSensorPosition() << endl;
		//cout << leftLeader.GetSelectedSensorPosition() << endl;
	}

	//this gets called every tick and contans all the drivetrain related code
	void Drive() {
		UpdateControllerInputs();
		UpdateRaspiInput();
		isAuton = leftShoulder;

		if(isAuton) {
			if(!isOffHab) {
				GetOffHab();
			}		
		}
		else {
			leftTarget = leftjoyY;
			rightTarget = rightjoyY;
			
			//Set the motors to the motor targets
			leftLeader.Set(ControlMode::PercentOutput, leftTarget);
			rightLeader.Set(ControlMode::PercentOutput, -rightTarget);

			
		}

	}

	// polls the xbox for input
	void UpdateControllerInputs() {
		//Tank drive both stick
		leftjoyY = -xboxController.GetY(frc::GenericHID::JoystickHand::kLeftHand);
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

	//gets inputs from raspi
	void UpdateRaspiInput() {
		frame = getFrame(serial_port);
		if (frame.error) {
			if (frame.error & VISION_ERROR_NO_DATA) missed_frames++;
			return;
		}
		if (frameIsInfo(frame)) {
			vision_info_t vinfo = getInfo(frame);
			vision_threshold = (uint8_t)vinfo.threshold;
			//cout << "Threshold now " << vision_threshold << "\n";
		} else {
			//cout << "Angle: " << frame.angle << ", offset: " << frame.line_offset << ", distance: " << frame.reserved_distance << "\n";
		}
		last_frame_time = time_clock.now();
		missed_frames = 0;
	}

	//call this every tick to get off the hab
	bool GetOffHab() {
		//returns true when done
		if(DriveDistance(36,1)) {
			ResetEncoders();
			isOffHab = true; 
			return true;
		} else {
			return false;
		}
	}

	//drives "inches" inches at "speed" the cruise velocity
	bool DriveDistance(double inches, float speed) {
		//speed is a percentage from 0.0 to 1.0

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
		//cout << "rightLeader.GetSelectedSensorPosition(): " << rightLeader.GetSelectedSensorPosition() << endl;

		if(speed > 1)
		speed = 1;
		if(speed < 0)
		speed = 0;
		leftLeader.ConfigMotionCruiseVelocity(speed * Constant::leftMotionVel);
		rightLeader.ConfigMotionCruiseVelocity(speed * Constant::rightMotionVel);
		leftLeader.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetEncPos);
		rightLeader.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetEncPos);

		return false;
	}

	//checks margen of error for DriveDistance()
	bool AutonPositionDeadband(double value, int target) {
		//returns true if "value" is within the margen of error of "target"
		if (fabs(target - value) < Constant::autonPositionDeadbandVal) {
			return true; 
		} else {
			return false;
		}
	}

	//sets encoder position to zero
	void ResetEncoders() {
		//this will reset the progress of DriveDistance()
		leftLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 50);  // 50 is the number of ms before it times out
		rightLeader.SetSelectedSensorPosition(0, Constant::pidChannel, 50);  // 50 is the number of ms before it times out
	}

	//call this once per tick to dive "targetDistance" inches away from the end of the line
	//TODO
	bool ApproachLine(float targetDistance) {
		//returns true when done
		CalculateAutonInstructions();
		return false; // unimplemented
	}

	//call this to update autonInstructions[] make sure to call UpdateRaspiInput() first
	//TODO
	bool CalculateAutonInstructions() {
		//returns true if AutonInstructions have bean generated successfully

		float outPut [100] = {}; //autonInstructions will be set to this if successful

		float x;
		float y;
		float targetAngle = (float)atan(y / x) * 180 / PI;
		if(fabs(targetAngle - frame.angle) < Constant::ANGLE_MAX_ERROR) {
			outPut[1] = (float)atan(y / x) * 180 / PI; //TODO fix this its not exact
			outPut[0] = (float)sqrt( x*x + y*y ); //TODO fix this its not exact
			for(int i = 0; i < 100; i++){
				autonInstructions[i] = outPut[i];
			}
			return true;
		}
		else{
			outPut[0] = Constant::DIST_TO_MANTAIN_FINAL_ANGLE;
			float alinedAtX;
			float alinedAtY;
			if(fabs(targetAngle - frame.angle) < (Constant::HORIZONTAL_FOV / 2)){
			}
			else{
				outPut[2] = Constant::DIST_TO_MANTAIN_CAM_ANGLE;
				outPut[1] = Constant::HORIZONTAL_FOV / 2;
			}
		}
		return false;
	}

	//TODO make a comment that explains what this does
	bool TurnDegrees(double degrees) {
		cout << pGyro->GetYaw() << endl;

		if(pidAngle.GetSetpoint() != degrees) {
			pidAngle.SetSetpoint(degrees);
		}

		if(pidAngle.IsEnabled() && pidAngle.OnTarget()) {
			pidAngle.Disable();
			return true;
		} else {
			return false;
		}
	}

	//TODO add a coment here explaning what ResetGyro() does
	void ResetGyro() {
		pGyro->ZeroYaw();
		while(!(pGyro->GetYaw() < 0.01 && pGyro->GetYaw() > -.01)) {}
	}

	//call every tick to climb hab
	bool Climb() {
		//returns true when done
		return false; // unimplemented
	}

	//sets "pistonID" to extended if true and retracted if false
	//TODO
	void SetPistonExtended(int pistonID, bool value){

	}

	//checks whether the piston is extended
	//TODO
	bool GetPistonExtended(int pistonID){
		//returns true if "pistonID" is extended false if retracted
		return false; // unimplemented
	}

	// Checks if vision is active and available
	bool IsVisionAvailable() {
		return missed_frames < 50;
	}

	//checks if auton can ApproachLine()
	bool IsLineApproachable() {
		if (chrono::duration_cast<chrono::milliseconds>(time_clock.now() - last_frame_time).count() > 35) // old frame is outdated
			UpdateRaspiInput();
		return (abs(frame.angle) < 1 && abs(frame.line_offset) < 2.5 &&      // thresholds
		        frame.angle != 0 && frame.line_offset != 0 && !frame.error && IsVisionAvailable()); // check for problems
	}

	//this is a duplicate of ApproachLine()
	//TODO is this better than ApproachLine()?
	// returns 0 if needs to be run again, 1 if done, -1 if impossible
	uint8_t dock_state = 0;
	int DockRobot() {
		if (!IsVisionAvailable()) return -1;
		if (IsLineApproachable()) {
			if (frame.wall_distance > 3) {
				if (DriveDistance(frame.wall_distance / (4 * 2.54), 0.5)) ResetEncoders();
				else return 0;
			}
			return 1;
		} else if (abs(frame.line_offset) >= 2.5 || (dock_state > 0 && dock_state < 3)) {
			if (dock_state == 0) {
				if (TurnDegrees((90 - frame.angle) * (frame.line_offset < 0 ? 1 : -1))) {
					ResetEncoders();
					dock_state = 1;
				}
				return 0;
			} else if (dock_state == 1) {
				if (DriveDistance(frame.line_offset / 2.54, 0.5)) {
					ResetEncoders();
					dock_state = 2;
				}
				return 0;
			} else if (dock_state == 2) {
				if (TurnDegrees(90 * (frame.line_offset < 0 ? 1 : -1))) {
					ResetEncoders();
					dock_state = 0;
				}
				return 0;
			}
		} else if (frame.error) {
			return 0;
		} else {
			if (TurnDegrees(-frame.angle)) {
				ResetEncoders();
				dock_state = 0;
			}
			return 0;
		}
	}

	//call every tick to take a hatch pannel from the brushes
	//TODO
	bool RetrieveHatch(){
		//returns true when done
		return false; // unimplemented
	}

	//call every tick to place the hatch pannel on a rocket or cargoship
	//TODO
	bool PlaceHatch(){
		//returns true when done

		//SetArmAngle();
		//DriveDistance();
		//EjectHatch();
		return false; // unimplemented
	}

	//call every tick to set the arm angle to "angle"
	//TODO
	bool SetArmAngle(float angle){
		//returns true when done
		return false; // unimplemented
	}

	//fires pistons to remove hatch from robot velcrow
	//TODO
	void EjectHatch(){

	}

	// call every tick to put the orange ball in a cargoship/rocket
	bool PlaceCargo(){
		//returns true when done
		return false; // unimplemented
	}

	//spins weels to shot out cargo
	//TODO
	void EjectCargo(){

	}
};

START_ROBOT_CLASS(Robot);