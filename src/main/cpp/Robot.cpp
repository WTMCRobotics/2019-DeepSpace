#include <iostream>
#include <istream>
#include <string>
#include <chrono>

#include <frc/WPILib.h>

#include <frc/DigitalInput.h>

//#include <frc/liveWindow/LiveWindow.h>
//#include <frc/smartDashboard/SendableChooser.h>
//#include <frc/smartDashboard/SmartDashboard.h>
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

#include <stdio.h> 
#include <signal.h>
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <sstream>
#define PORT    3805
#define MAXLINE 1024 

#include <math.h>
#define PI 3.14159265
#define rad(d) (d * (M_PI/180.0))

#include "Vision.h"

using namespace frc;
using namespace std;

void sighandler(int signal) {
	std::cout << "Caught signal " << signal << "! Exiting.\n";
	exit(signal);
}

class Robot : public frc::TimedRobot {

private:
	//frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
	frc::SendableChooser<string> m_chooser;
	const string kAutoNameDefault = "Default";
	const string kAutoNameCustom = "My Auto";
	string m_autoSelected;
	

	frc::XboxController xboxController{0};
	frc::XboxController guitar{1};
	frc::Joystick joystick{2};

	//Joystick joystick2, joystick2;
	double leftjoyY;
	double rightjoyY;
	double rightjoyX;
	double leftTarget;
	double rightTarget;

	double codriverY;
	double armTarget;
	double intake;

	//xbox triggers
	double rightTrigger;//not used yet
	double leftTrigger;//not used yet
	bool rightShoulder;//not used yet
	bool leftShoulder;

	//xbox dpad
	bool dpadUp;
	bool dpadDown;
	bool nextCameraButton;

	//xbox buttons

	bool AButton;
	bool BButton;
	bool XButton;
	bool YButton;

	//joystick buttons

	bool latchButton;
	bool unLatchButton;
	
	//gHero controler buttons
	bool gbuttonGreen;
	bool gbuttonRed;
	bool gbuttonBlue;
	bool gbuttonYellow;
	bool gbuttonOrange;
	bool gbuttonAltGreen;
	bool gbuttonAltRed;
	bool gbuttonAltBlue;
	bool gbuttonAltYellow;
	bool gbuttonAltOrange;
	bool gbuttonDown;
	bool gbuttonUp;

	bool extended = false;
	bool retracted = true;

	bool rextended = false;
	bool rretracted = true;

	bool latchIsOut;
	bool latchWasOut;

	static const int GREEN = 1;
	static const int RED = 2;
	static const int YELLOW = 4;
	static const int BLUE = 3;
	static const int ORANGE = 5;
	static const int AltColor = 9;


	//Auton
	bool isAuton = false;
	bool isOffHab = false;
	bool runDockRobot = false;
	bool canRunDockRobot = true;
	bool FollowingInstructions = false;
	bool ShouldFollowInstructions = false;

	//  customGyroOfset + GetGyroYaw() = pGyro->GetYaw()
	float customGyroOfset = 0;
	float gyroTarget;

	bool waiting = false;

	float AllowedAngleError = 2.0;
	int ejectTrigger;

	//Demo
	bool slowModeEnabled = true;
	
	//raspi input
	vision_frame_t frame;

	AHRS* pGyro = new AHRS(frc::SPI::Port::kMXP);

	TalonSRX leftMaster {Constant::LeftMasterID};
	TalonSRX leftSlave {Constant::LeftSlaveID};
	TalonSRX rightMaster {Constant::RightMasterID};
	TalonSRX rightSlave {Constant::RightSlaveID};

	TalonSRX intakeMaster {Constant::IntakeMasterID};
	TalonSRX intakeSlave {Constant::IntakeSlaveID};

	TalonSRX armMaster {Constant::ArmMasterID};

	DigitalInput* lowerLimitSwitch;
	DigitalInput* uperLimitSwitch;
	DigitalInput* latchLimitSwitch;
	DigitalInput* isPractice;
	
	uint8_t vision_threshold;
	chrono::steady_clock time_clock;
	chrono::steady_clock::time_point last_frame_time;
	unsigned int missed_frames = 0;

	static bool cameraLoopStatus;


	frc::Compressor compressor {Constant::PCM_ID};

	//frc::DoubleSolenoid frontLeftSol {Constant::PCM_CHANNEL_FRONT_LEFT_IN, Constant::PCM_CHANNEL_FRONT_LEFT_OUT};
	
	frc::Solenoid frontSol {1, Constant::PCM_CHANNEL_FRONT};

	frc::Solenoid rearSol {1, Constant::PCM_CHANNEL_REAR};

	frc::DoubleSolenoid ejectSol {1, Constant::PCM_EJECT_IN, Constant::PCM_EJECT_OUT};

	frc::DoubleSolenoid latchSol {1, Constant::PCM_CHANNEL_LATCH_OUT, Constant::PCM_CHANNEL_LATCH_IN};

	// frc::DoubleSolenoid rearRightSol {1, Constant::PCM_CHANNEL_REAR_RIGHT_IN, Constant::PCM_CHANNEL_REAR_RIGHT_OUT};

	// frc::DoubleSolenoid ejectLeftSol {1, Constant::PCM_CHANNEL_EJECT_LEFT_IN, Constant::PCM_CHANNEL_EJECT_LEFT_OUT};

	// frc::DoubleSolenoid ejectRightSol {1, Constant::PCM_CHANNEL_EJECT_RIGHT_IN, Constant::PCM_CHANNEL_EJECT_RIGHT_OUT};

	// frc::DoubleSolenoid latchSol {1, Constant::PCM_CHANNEL_LATCH_IN, Constant::PCM_CHANNEL_LATCH_OUT};

	int currArmPos;
	double wheelsTarget;
	//PID for turning
	PIDMotorOutput pidMotorOutput { &leftMaster, &rightMaster };
    PIDGyroSource pidGyroSource { pGyro };
	//practice
	PIDController pidAngle { 0.045, 0.0000001, 0.1, &pidGyroSource, &pidMotorOutput, 0.02 };
	//compition
	//PIDController pidAngle { 0.035, 0.0000001, 0.1, &pidGyroSource, &pidMotorOutput, 0.02 };

public:
	float autonInstructions [Constant::MAX_AUTON_INSTRUCTIONS];  //Even positions like autonInstructions[2] are distance and Odd ones are angles they are exicuted in order from greates to least


	frc::SerialPort serial_port = frc::SerialPort(9600, frc::SerialPort::Port::kUSB);

	// Adds two numbers together and overflows to a minimum if it goes over a maximum.
	// Ex: addWithMax(2, 5, 3) = 1
	double addWithMax(double add1, double add2, double max, double min = 0.0) {
		if (add1 + add2 >= max) return ((add1 + add2) - max * floor((add1 + add2) / max)) + min;
		else return add1 + add2;
	}

	static void CameraError(std::string err) {
		std::cout << "Error initializing camera: " << err << "\n";
		cameraLoopStatus = false;
	}

	static void CameraServerLoop() {
		try {
			CameraServer * serv = CameraServer::GetInstance();
			cs::CvSource outputStreamStd = serv->PutVideo("Vision Camera", 640, 480);
			outputStreamStd.SetFPS(10);
			outputStreamStd.SetConnected(true);
			int sockfd, newsockfd, clilen;
			char * buffer = (char*)malloc(640*480*3 + 512);
			struct sockaddr_in serv_addr, cli_addr;
			int n;
			sockfd = socket(AF_INET, SOCK_STREAM, 0);
			if (sockfd < 0) 
				return CameraError("Could not open socket");
			std::cout << "Created socket";
			std::cout.flush();
			memset((void*)&serv_addr, sizeof(serv_addr), 0);
			//bzero((char *) &serv_addr, sizeof(serv_addr));
			int tru = 1;
			if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &tru, sizeof(int)) < 0) 
				return CameraError("Could not set socket options");
			std::cout << "Set socket options";
			std::cout.flush();
			serv_addr.sin_family = AF_INET;
			serv_addr.sin_addr.s_addr = INADDR_ANY;
			serv_addr.sin_port = htons(3805);
			if (bind(sockfd, (struct sockaddr *) &serv_addr,
					sizeof(serv_addr)) < 0) 
					return CameraError("Could not bind socket to port");
			std::cout << "Bound socket";
			std::cout.flush();
			listen(sockfd,5);
			std::cout << "Done listening";
			std::cout.flush();
			clilen = sizeof(cli_addr);
			newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (unsigned int*)&clilen);
			if (newsockfd < 0) 
				return CameraError("Could not accept connection");
			std::cout << "Accepted connection";
			std::cout.flush();
			bzero(buffer,256);
			cameraLoopStatus = true;
			std::cout << "Connected to raspi.\n";
			long long count = 0;
			while (true) {
				n = read(newsockfd, buffer + count, 256);
				if (n < 0) return CameraError("Could not read camera data!");
				count += n;
				/*if (strlen(buffer) == 0) {
					int cont = 1;
					for (int i = 0; i < 256; i++) {if (buffer[i] != 0) {cont = 0; break;}}
					if (cont) {
						shutdown(sockfd, SHUT_RDWR);
						fprintf(stderr, "Disconnected.\n");
						goto Restart;
					}
				}*/
				//std::cout << count << "\n";
				if (count >= 640*480*3) {
					std::cout << "Got frame from pi\n";
					cv::Mat image(cv::Size(640, 480), CV_8UC3, buffer, cv::Mat::AUTO_STEP);
					cv::Mat newimage;
					cv::resize(image, newimage, cv::Size(320, 240));
					outputStreamStd.PutFrame(newimage);
					count -= 640*480*3;
					std::cout << count << "\n";
					char tmp[512];
					memcpy(tmp, &buffer[640*480*3], count);
					bzero(buffer, 640*480*3 + 512);
					memcpy(buffer, tmp, count);
				}
				//this line of code makes things crash--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			}
		} catch (std::exception e) {
			std::cout << "Got exception: " << e.what() << "\n";
			cameraLoopStatus = false;
			return;
		}
		//while (true) ;
	}

	std::thread cameraThread;

	//this gets called once at the beggining of the match
	void RobotInit() {
		for (int s = 0; s < 32; s++) signal(s, sighandler);
		SetupMoters();
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		serial_port.SetTimeout(.05);
		serial_port.Reset();
		/*try {
	    	cameraThread = std::thread(CameraServerLoop);
			cameraThread.detach();
		} catch (std::exception e) {
			std::cout << "Could not start camera thread\n";
			cameraLoopStatus = false;
		}*/
		//CameraServer::GetInstance()->StartAutomaticCapture();
		lowerLimitSwitch = new DigitalInput(0);
		uperLimitSwitch = new DigitalInput(1);
		latchLimitSwitch = new DigitalInput(2);
		isPractice = new DigitalInput(3);

		currArmPos = armMaster.GetSelectedSensorPosition();
	}

	//configures the motors should be called at begining of match
	void SetupMoters() {
		//Right motor setup
		leftMaster.ClearStickyFaults(0);
		//leftMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		leftMaster.ConfigNominalOutputForward(0, 0);
		leftMaster.ConfigNominalOutputReverse(0, 0);
		leftMaster.ConfigPeakOutputForward(1, 0);
		leftMaster.ConfigPeakOutputReverse(-1, 0);
		leftMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		leftMaster.ConfigMotionCruiseVelocity(Constant::leftMotionVel, 0);
		leftMaster.ConfigMotionAcceleration(Constant::leftMotionAcc, 0);
		leftMaster.SetSensorPhase(false);
		leftMaster.SetInverted(false);

		leftSlave.ConfigNominalOutputForward(0, 0);
		leftSlave.ConfigNominalOutputReverse(0, 0);
		leftSlave.ConfigPeakOutputForward(1, 0);
		leftSlave.ConfigPeakOutputReverse(-1, 0);
		leftSlave.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		leftSlave.ConfigMotionCruiseVelocity(0, 0);
		leftSlave.ConfigMotionAcceleration(0, 0);
		leftSlave.SetSensorPhase(false);
		leftSlave.SetInverted(false);


		//Right motor setup
		rightMaster.ClearStickyFaults(0);
		//rightMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		rightMaster.ConfigNominalOutputForward(0, 0);
		rightMaster.ConfigNominalOutputReverse(0, 0);
		rightMaster.ConfigPeakOutputForward(1, 0);
		rightMaster.ConfigPeakOutputReverse(-1, 0);
		rightMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightMaster.ConfigMotionCruiseVelocity(Constant::rightMotionVel, 0);
		rightMaster.ConfigMotionAcceleration(Constant::rightMotionAcc, 0);
		rightMaster.SetSensorPhase(false);
		rightMaster.SetInverted(true);

		rightSlave.ConfigNominalOutputForward(0, 0);
		rightSlave.ConfigNominalOutputReverse(0, 0);
		rightSlave.ConfigPeakOutputForward(1, 0);
		rightSlave.ConfigPeakOutputReverse(-1, 0);
		rightSlave.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		rightSlave.ConfigMotionCruiseVelocity(0, 0);
		rightSlave.ConfigMotionAcceleration(0, 0);
		rightSlave.SetSensorPhase(false);
		rightSlave.SetInverted(true);
		
		double p = 0.65;
		double i = 0.0000;
		double d = 0.484;
		// PID setup for driving a distance
		leftMaster.Config_kP(Constant::pidChannel, p, 0);
		leftMaster.Config_kI(Constant::pidChannel, i, 0);
		leftMaster.Config_kD(Constant::pidChannel, d, 0);
		leftMaster.Config_IntegralZone(Constant::pidChannel, 0, 0);

		rightMaster.Config_kP(Constant::pidChannel, p, 0);
		rightMaster.Config_kI(Constant::pidChannel, i, 0);
		rightMaster.Config_kD(Constant::pidChannel, d, 0);
		rightMaster.Config_IntegralZone(Constant::pidChannel, 0, 0);

		rightSlave.Set(ctre::phoenix::motorcontrol::ControlMode::Slave, Constant::RightMasterID);
		leftSlave.Set(ctre::phoenix::motorcontrol::ControlMode::Slave, Constant::LeftMasterID);

		intakeMaster.ClearStickyFaults(0);
		//leftMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		intakeMaster.ConfigNominalOutputForward(0, 0);
		intakeMaster.ConfigNominalOutputReverse(0, 0);
		intakeMaster.ConfigPeakOutputForward(1, 0);
		intakeMaster.ConfigPeakOutputReverse(-1, 0);
		intakeMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		intakeMaster.ConfigMotionCruiseVelocity(Constant::leftMotionVel, 0);
		intakeMaster.ConfigMotionAcceleration(Constant::leftMotionAcc, 0);
		intakeMaster.SetSensorPhase(false);
		intakeMaster.SetInverted(false);

		intakeSlave.ConfigNominalOutputForward(0, 0);
		intakeSlave.ConfigNominalOutputReverse(0, 0);
		intakeSlave.ConfigPeakOutputForward(1, 0);
		intakeSlave.ConfigPeakOutputReverse(-1, 0);
		intakeSlave.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		intakeSlave.ConfigMotionCruiseVelocity(0, 0);
		intakeSlave.ConfigMotionAcceleration(0, 0);
		intakeSlave.SetSensorPhase(false);
		intakeSlave.SetInverted(false);

		//intakeSlave.Set(ctre::phoenix::motorcontrol::ControlMode::Slave, Constant::IntakeMasterID);

		armMaster.ClearStickyFaults(0);
		//leftMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, Constant::pidChannel, 0);
		armMaster.ConfigNominalOutputForward(0, 0);
		armMaster.ConfigNominalOutputReverse(0, 0);
		armMaster.ConfigPeakOutputForward(1, 0);
		armMaster.ConfigPeakOutputReverse(-1, 0);
		armMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
		armMaster.ConfigMotionCruiseVelocity(Constant::armMotionVel, 0);
		armMaster.ConfigMotionAcceleration(Constant::armMotionAcc, 0);
		armMaster.SetSensorPhase(false);
		armMaster.SetInverted(false);

		double ap = 0.65;
		double ai = 0;
		double ad = 0.484;

		armMaster.Config_kP(Constant::pidChannel, ap, 0);
		armMaster.Config_kI(Constant::pidChannel, ai, 0);
		armMaster.Config_kD(Constant::pidChannel, ad, 0);
		armMaster.Config_IntegralZone(Constant::pidChannel, 0, 0);
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
		currArmPos = armMaster.GetSelectedSensorPosition();
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
		TeleopPeriodic();
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	//is called once at the begining of the match in 2019
	void TeleopInit() {
		CalibrateGyro();
		ResetEncoders();
		ResetGyro();
		currArmPos = armMaster.GetSelectedSensorPosition();
	}

	//this code gets call once every tick or about 50 times per second
	void TeleopPeriodic() {
		UpdateCompressor();
		if(!waiting){
			Drive();
			//TestStuff();
			//cout << DriveDistance(6 * 3.14, 1) << endl;
			//DriveDistance(1, 1);
			//cout << gyro->GetAngle() << endl;
			//cout << rightMaster.GetSelectedSensorPosition() << endl;
			//cout << leftMaster.GetSelectedSensorPosition() << endl;
		}
	}

	//this gets called every tick and contans all the drivetrain related code

	void TestStuff() {
		cout << "TestStuffCalled" << endl;
		UpdateCompressor();
		SetPistonExtended(4, true);
	}



	void Drive() {
		UpdateCompressor();
		// cout << "arm encoder" << armMaster.GetSelectedSensorPosition() << endl;
		// cout << "dio lower: " << lowerLimitSwitch->Get() << endl;
		// cout << "dio upper: " << uperLimitSwitch->Get() << endl;
		if(!waiting){
			UpdateControllerInputs();
			//UpdateRaspiInput();
			//cout << pGyro->GetYaw() << endl;

			// if(AButton) {
			// 	SetPistonExtended(Constant::PCM_CHANNEL_EJECT_LEFT, true);
			// 	SetPistonExtended(Constant::PCM_CHANNEL_EJECT_RIGHT, true);
			// } else {
			// 	SetPistonExtended(Constant::PCM_CHANNEL_EJECT_LEFT, false);
			// 	SetPistonExtended(Constant::PCM_CHANNEL_EJECT_RIGHT, false);
			// }
			bool guitarArmControl = false;

			// if (gbuttonAltOrange) {
			// 	if(!lowerLimitSwitch->Get()){
			// 		setArmPosition(Constant::ARMDOWN);
			// 		cout << "516\n";
			// 		guitarArmControl = true;
			// 	} else {
			// 		cout << "518\n";
			// 	}
			// } else if (gbuttonAltBlue) {
			// 	setArmPosition(Constant::ARM90DEGREES);
			// 	currArmPos = armMaster.GetSelectedSensorPosition();
			// 	cout << "523\n";
			// 	guitarArmControl = true;
			// } else if (gbuttonAltYellow) {
			// 	if(uperLimitSwitch->Get()){
			// 		setArmPosition(Constant::ARMSTORAGE);
			// 		cout << "528\n";
			// 		guitarArmControl = true;
			// 	} else {
			// 		cout << "530\n";
			// 	}
			// }
			// cout << "GuitarArmControl: " << guitarArmControl << endl;

			if (ejectTrigger) {
				ejectSol.Set(frc::DoubleSolenoid::Value::kReverse);
			} else {
				ejectSol.Set(frc::DoubleSolenoid::Value::kForward);
			}

			if((codriverY < -0.1 || gbuttonOrange) /*&& (!latchIsOut)*/) {
				if(!lowerLimitSwitch->Get()) {
					if (gbuttonAltOrange) {
						if (currArmPos < -1500) {
							armMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
						} else {
							armMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
						}
					} else {
						armMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -codriverY * 0.5);
					}
					currArmPos = armMaster.GetSelectedSensorPosition();
					//cout << "518" << endl;
				} else {
					if (!guitarArmControl) {
						armMaster.SetSelectedSensorPosition(0, Constant::pidChannel, 50);
						armMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					}
					//cout << "522" << endl;
				}
			} else if((codriverY > 0.1 || gbuttonAltYellow) /*&& (!latchIsOut)*/) {
				if(uperLimitSwitch->Get()){
					if (gbuttonAltYellow) {
						if (currArmPos < Constant::ARM90DEGREES) {
							armMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.1);
						} else {
							armMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
						}
					} else {
						armMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -codriverY * 0.5);
					}
					currArmPos = armMaster.GetSelectedSensorPosition();
					//cout << "528" << endl;
				} else {
					armMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				//cout << "531" << endl;
				}
			} else {
				if (!guitarArmControl) {
					armMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, currArmPos);
				//cout << "533" << endl;
				}
			}
			cout << armMaster.GetSelectedSensorPosition() << endl;
			if(leftShoulder) {
				isAuton=true;
			} else {
				isAuton=false;
			}
			if (runDockRobot == false) {
				dock_state = 0;
				dockRobot2Running = false;
			}

			if (XButton) {
				cout << "Extending 4" << endl;
				SetPistonExtended(Constant::PCM_CHANNEL_FRONT, true);
			} else {
				cout << "Retracting 4" << endl;
				SetPistonExtended(Constant::PCM_CHANNEL_FRONT, false);
			}

			if (BButton) {
				SetPistonExtended(Constant::PCM_CHANNEL_REAR, true);
			} else {
				SetPistonExtended(Constant::PCM_CHANNEL_REAR, false);
			}

			//if (!cameraLoopStatus) std::cout << "Camera loop stopped!\n";
			if(isAuton) {
				
				//gets of hab if on hab
				if(!isOffHab) {
					GetOffHab();
				} else {
					//this code will run once off hab
					if (ShouldFollowInstructions) {
						if(FollowAutonInstructions()) {
							ShouldFollowInstructions = false;
						}
					}
				}
				
				
				
			
			} else if (runDockRobot && canRunDockRobot) {
				int dockRobotRetval = DockRobot2();
				if (dockRobotRetval == -1) {
					cout << "Error: Vision not responding!\n";
					canRunDockRobot = false;
					dock_state = 0;
				} else if (dockRobotRetval == 1) {
					cout << "Finished docking.\n";
					canRunDockRobot = false;
					dock_state = 0;
				}

			} else {	
				//this will be a number between 0.25 and 1.0
				pidAngle.Disable();
				double amountToSlowBy = (1- (rightTrigger * 0.5)) * (1- (leftTrigger * 0.5));

				if(slowModeEnabled){
					amountToSlowBy /= 4;
					std::cout << "Slow enabled\n";
				}
				leftTarget = leftjoyY * amountToSlowBy;
				rightTarget = rightjoyY * amountToSlowBy;

				
				
				//Set the motors to the motor targets
				leftMaster.Set(ControlMode::PercentOutput, leftTarget);
				rightMaster.Set(ControlMode::PercentOutput, -rightTarget);
				//cout << "Intake: " << intake << endl;
				if(intake != -1) {
					//cout << "Intake setting" << endl;
					intakeMaster.Set(ControlMode::PercentOutput, -wheelsTarget);
					intakeSlave.Set(ControlMode::PercentOutput, -wheelsTarget);
				} else {
					//cout << "STOP INTAKE" << endl;
					intakeMaster.Set(ControlMode::PercentOutput, 0);
					intakeSlave.Set(ControlMode::PercentOutput, 0);
				
				}
				//intakeMaster.Set(ControlMode::PercentOutput, intake);
				//cout << "intake: " << intake << endl;
				//cout << "coDriverY: " << codriverY << endl;

				if(joystick.GetRawButtonPressed(2)){
					latchSol.Set(frc::DoubleSolenoid::Value::kReverse);
					latchIsOut = true;
				} else if(joystick.GetRawButtonReleased(2)){
					latchSol.Set(frc::DoubleSolenoid::Value::kForward);
					latchIsOut = false;
					latchWasOut = true;  //not curently doning anything
				}

				//gets things ready for when auton is enabled
				ResetEncoders();	
				ResetGyro();

				
				// autonInstructions[0] = 0;
				//autonInstructions[1] = -90;
				// autonInstructions[2] = 48;
				// autonInstructions[3] = 90;
				// ShouldFollowInstructions = true;
				
				if(gbuttonUp) {
					//Cargo Hatch Close-Front
					//cout << "UP" << endl;
					if(gbuttonOrange){
						//cout << "Orange" << endl;
						autonInstructions[2] = 18 * 12;
						autonInstructions[3] = -149;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}

					//Cargo Hatch Close-Side
					if(gbuttonYellow){
						//cout << "Yellow" << endl;
						autonInstructions[1] = 54;
						autonInstructions[2] = 20.85 * 12;
						autonInstructions[3] = -154;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}

					//Cargo Hatch Middle-Side
					// if(gbuttonRed){
					// 	//cout << "Red" << endl;
					// 	autonInstructions[1] = 66;
					// 	autonInstructions[2] = 22.37 * 12;
					// 	autonInstructions[3] = -156;
					// 	autonInstructions[4] = -4 * 12;
					// 	ShouldFollowInstructions = true;
					// }

					//Cargo Hatch Far-Side
					// if(gbuttonGreen){
					// 	//cout << "Green" << endl;
					// 	autonInstructions[1] = 68;
					// 	autonInstructions[2] = 24 * 12;
					// 	autonInstructions[3] = -158;
					// 	autonInstructions[4] = -4 * 12;
					// 	ShouldFollowInstructions = true;
					// }

					//Rocket Hatch Far
					if(gbuttonAltGreen){
						//cout << "Alt Green" << endl;
						autonInstructions[1] = 76;
						autonInstructions[2] = 19.34 * 12;
						autonInstructions[3] = -166;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}

					//Rocket Hatch Close
					if(gbuttonAltRed){
						//cout << "Alt Red" << endl;
						autonInstructions[2] = 13 * 12;
						autonInstructions[3] = 180;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}
				} else if(gbuttonDown) {
					//Cargo Hatch Close-Front
					//cout << "DOWN" << endl;
					if(gbuttonOrange){
						//cout << "Orange" << endl;
						autonInstructions[2] = 18 * 12;
						autonInstructions[3] = 149;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}

					//Cargo Hatch Close-Side
					if(gbuttonYellow){
						//cout << "Yellow" << endl;
						autonInstructions[1] = -54;
						autonInstructions[2] = 20.85 * 12;
						autonInstructions[3] = 154;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}

					//Cargo Hatch Middle-Side
					if(gbuttonRed){
						//cout << "Red" << endl;
						autonInstructions[1] = -66;
						autonInstructions[2] = 22.37 * 12;
						autonInstructions[3] = 156;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}

					//Cargo Hatch Far-Side
					if(gbuttonGreen){
						//cout << "Green" << endl;
						autonInstructions[1] = -68;
						autonInstructions[2] = 24 * 12;
						autonInstructions[3] = 158;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}

					//Rocket Hatch Far
					if(gbuttonAltGreen){
						//cout << "Alt Green" << endl;
						autonInstructions[1] = -76;
						autonInstructions[2] = 19.34 * 12;
						autonInstructions[3] = 166;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}

					//Rocket Hatch Close
					if(gbuttonAltRed){
						//cout << "Alt Red" << endl;
						autonInstructions[2] = 13 * 12;
						autonInstructions[3] = -180;
						autonInstructions[4] = -4 * 12;
						ShouldFollowInstructions = true;
					}
				} else {
					if(gbuttonRed){
						slowModeEnabled = false;
					}
					if(gbuttonGreen){
						slowModeEnabled = true;
					}
				}
			}
		}
	}

	// polls the xbox for input
	void UpdateControllerInputs() {
		//Tank drive both stick
		leftjoyY = -xboxController.GetY(frc::GenericHID::JoystickHand::kLeftHand);
		rightjoyY = xboxController.GetY(frc::GenericHID::JoystickHand::kRightHand);
		leftTrigger = xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
		rightTrigger = xboxController.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand);
		
		//CoDriver inputs
		codriverY = joystick.GetY();
		intake = joystick.GetPOV();

		ejectTrigger = joystick.GetTrigger();

		if (intake >= 315 || intake <= 45) {
			//slow for outtake
			wheelsTarget = 1;
		} else {
			//fast for intake
			wheelsTarget = -1;
		}

		//Auton overide
		leftShoulder = xboxController.GetBumper(frc::GenericHID::JoystickHand::kLeftHand);
		runDockRobot = xboxController.GetBumper(frc::GenericHID::JoystickHand::kRightHand);
		if (!runDockRobot) canRunDockRobot = true;
		//if (!XButton && xboxController.GetXButton()) calibrateVision(serial_port);
		//if (xboxController.GetAButton()) incrementThreshold(serial_port);
		//if (xboxController.GetBButton()) decrementThreshold(serial_port);
		//if (!nextCameraButton && xboxController.GetYButton()) nextCamera(serial_port);
		AButton = xboxController.GetAButton();
		BButton = xboxController.GetBButton();
		XButton = xboxController.GetXButton();
		YButton = xboxController.GetYButton();
		//dpadUp = xboxController.GetAButton();
		//dpadDown = xboxController.GetBButton();
		dpadUp = (xboxController.GetPOV() == 0);
		dpadDown = (xboxController.GetPOV() == 180);
		//nextCameraButton = xboxController.GetYButton();
		//if (xboxController.GetYButton()) serial_port.Reset();

		latchButton = guitar.GetRawButton(3);
		unLatchButton = guitar.GetRawButton(4);

		gbuttonGreen = guitar.GetRawButton(GREEN) && !guitar.GetRawButton(AltColor);
		gbuttonRed = guitar.GetRawButton(RED) && !guitar.GetRawButton(AltColor);
		gbuttonBlue = guitar.GetRawButton(BLUE) && !guitar.GetRawButton(AltColor);
		gbuttonYellow = guitar.GetRawButton(YELLOW) && !guitar.GetRawButton(AltColor);
		gbuttonOrange = guitar.GetRawButton(ORANGE) && !guitar.GetRawButton(AltColor);
		gbuttonAltGreen = guitar.GetRawButton(GREEN) && guitar.GetRawButton(AltColor);
		gbuttonAltRed = guitar.GetRawButton(RED) && guitar.GetRawButton(AltColor);
		gbuttonAltBlue = guitar.GetRawButton(BLUE) && guitar.GetRawButton(AltColor);
		gbuttonAltYellow = guitar.GetRawButton(YELLOW) && guitar.GetRawButton(AltColor);
		gbuttonAltOrange = guitar.GetRawButton(ORANGE) && guitar.GetRawButton(AltColor);
		gbuttonUp = guitar.GetPOV() == 0;
		gbuttonDown = guitar.GetPOV() == 180;

		if (gbuttonGreen && gbuttonUp) {
			intake = 1;
			wheelsTarget = -1;
			cout << "Intake in" << endl;
		} else if (gbuttonGreen && gbuttonDown) {
			intake = 1;
			wheelsTarget = 1;
			cout << "Intake out" << endl;
		}
		//cout << guitar.GetPOV() << endl;
		//wamy bar guitar.GetX(frc::GenericHID::JoystickHand::kRightHand)
		


		/*#ifdef FINEMOTIONCONTROL
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
		#endif*/
	}

	//gets inputs from raspi
	void UpdateRaspiInput() {
		if (serial_port.StatusIsFatal()) {
			cout << "Cannot use serial port!\n";
			missed_frames = 255;
			return;
		}
		frame = getFrame(serial_port);
		if (frame.error) {
			//cout << "Error getting frame: " << getVisionError(frame) << "\n";
			missed_frames++;
			return;
		}
		if (frameIsInfo(frame)) {
			vision_info_t vinfo = getInfo(frame);
			vision_threshold = (uint8_t)vinfo.threshold;
			cout << "Threshold now " << vision_threshold << "\n";
		} else {
			if (abs(frame.angle) < 90) cout << "Angle: " << frame.angle << ", offset x: " << frame.line_offset << ", offset y: " << frame.line_offset_y << ", distance: " << frame.wall_distance_left << "\n";
		}
		last_frame_time = time_clock.now();
		missed_frames = 0;
	}

	//call this every tick to get off the hab
	bool GetOffHab() {
		//returns true when done
		if(DriveDistance(36,0.1)) {
			ResetEncoders();
			isOffHab = true; 
			return true;
		} else {
			return false;
		}
	}

	//folows instructions in autonInstructions[]
	//TODO
	bool FollowAutonInstructions() {
		//returns true if done
		//cout << autonInstructions[1] << " , " << autonInstructions[2] << " , " << autonInstructions[3] << " , " << autonInstructions[4] << endl;
		//cout << "following auton" << " , ";
		int currentInstrusction = Constant::MAX_AUTON_INSTRUCTIONS -1;
		while(currentInstrusction > 0 && autonInstructions[currentInstrusction] == 0){
			currentInstrusction--;
		}

		if(!FollowingInstructions) {
			
			//Even positions like autonInstructions[2] are distance and Odd ones are angles they are exicuted in order from greates to least
			if((currentInstrusction % 2) == 0){
				//distance
				cout << "driving distance" << currentInstrusction << endl;
				if(DriveDistance(autonInstructions[currentInstrusction], Constant::moveSpeed)) {
					cout << "distance driven " << currentInstrusction << endl;
					autonInstructions[currentInstrusction] = 0;
					ResetEncoders();
					ResetGyro();
				}
			} else {
				//angle
				cout << "turning angle" << currentInstrusction << endl;
				if(abs(autonInstructions[currentInstrusction]) < 3 || TurnDegrees(autonInstructions[currentInstrusction])) {
					cout << "angle turned " << currentInstrusction << endl;
					autonInstructions[currentInstrusction] = 0;
					ResetEncoders();
					ResetGyro();
				}
			
			}
		}
		//returns true if the last step is completed
		if(autonInstructions[currentInstrusction] == 0 && currentInstrusction <= 0) {
			FollowingInstructions = true;
			return true;
		} else {
			FollowingInstructions = false;
			return false;
		}
	

		//if(autonInstructions[currentInstrusction] == 0 && currentInstrusction == 0){
		//	DockRobot();
		//}
	}


	void UpdateCompressor()
	{
		// if not enough pressure
		if(!compressor.GetPressureSwitchValue()) {
			// Start compressor
			compressor.Start();
		}
		// if enough pressure
		else {
			// Stop compressor
			compressor.Stop();
		}
	} // END of UpdateCompressor() function


	//clears all instructions in autonInstructions[]
	void ClearAutonInstructions() {
		for (int i = 0; i < Constant::MAX_AUTON_INSTRUCTIONS; i++) autonInstructions[i] = 0;
		ResetEncoders();
		ResetGyro();
	}

	//drives "inches" inches at "speed" the cruise velocity
	bool DriveDistance(double inches, float speed) {
		/*
		 *returns true when done
		 *
		 *speed is a percentage from 0.0 to 1.0
		 *
		 * inches / circumference = number of rotations
		 * pulsesPerRotationQuad = number of pulses in one rotation
		 * targetEncPos = position encoder should read
		 *int targetEncPos = (inches / Constant::circumference) * Constant::pulsesPerRotationQuad;
		 */

		int targetEncPos = (inches / Constant::circumference) * Constant::pulsesPerRotationQuad;

		if (AutonPositionDeadband(leftMaster.GetSelectedSensorPosition(Constant::pidChannel), targetEncPos)) {
			leftMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			rightMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			return true;
		}
		//cout << "rightMaster.GetSelectedSensorPosition(): " << rightMaster.GetSelectedSensorPosition() << endl;

		if(speed > 1){
			speed = 1;
		}
		if(speed <= 0.01) {
			speed = 0.01;
		}
		leftMaster.ConfigMotionCruiseVelocity(speed * Constant::leftMotionVel);
		rightMaster.ConfigMotionCruiseVelocity(speed * Constant::rightMotionVel);
		leftMaster.ConfigMotionAcceleration((1 / speed) * Constant::leftMotionAcc);
		cout << (0.5 / speed) * Constant::leftMotionAcc << endl;
		rightMaster.ConfigMotionAcceleration((1 / speed) * Constant::rightMotionAcc);
		leftMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetEncPos);
		rightMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetEncPos);

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
		leftMaster.SetSelectedSensorPosition(0, Constant::pidChannel, 50);  // 50 is the number of ms before it times out
		rightMaster.SetSelectedSensorPosition(0, Constant::pidChannel, 50);  // 50 is the number of ms before it times out
	}

	//call this once per tick to dive "targetDistance" inches away from the end of the line
	//TODO
	bool 
	ApproachLine(float targetDistance) {
		//returns true when done
		cout << "ApproachLine" << endl;
		CalculateAutonInstructions();
		if(FollowAutonInstructions()) {
			return true;
		}
		return false; // unimplemented
	}

	//call this to update autonInstructions[] make sure to call UpdateRaspiInput() first
	//TODO
	bool CalculateAutonInstructions() {
		cout << "CalculateAutonInstructions" << endl;
		//returns true if AutonInstructions have bean generated successfully

		float outPut [Constant::MAX_AUTON_INSTRUCTIONS] = {}; //autonInstructions will be set to this if successful

		float x = 1/*frame.wall_distance*/;
		float y = 10/*frame.line_offset*/;
		float targetAngle = (float)atan(y / x) * 180 / PI;
		if(fabs(targetAngle - frame.angle) < Constant::ANGLE_MAX_ERROR || true) {
			outPut[1] = 90 - ((float)atan(y / x) * 180 / PI); //TODO fix this its not exact
			cout << "turn " << outPut[1] << endl;
			outPut[0] = (float)sqrt( x*x + y*y ); //TODO fix this its not exact
			cout << "streght " << (float)sqrt( x*x + y*y ) << endl;
			for(int i = 0; i < Constant::MAX_AUTON_INSTRUCTIONS; i++){
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

	//turns right if positve left if negitive || turns right for less than 180 and left for greater than 180
	bool TurnDegrees(double degrees) {
		cout << "Turning degrees: " << degrees << endl;
		//returns true when done
		if(degrees == 0){
			return true;
		}
		
		if(!pidAngle.IsEnabled()) {
			pidAngle.Enable();
		}
		
		if(degrees > 180){
			degrees -= 360;
		}

		if(degrees < -180){
			degrees += 360;
		}

		if(pidAngle.GetSetpoint() != degrees) {
			pidAngle.SetSetpoint(degrees);
			//pidAngle.SetSetpoint(300);
		}

		cout << "GetSetpoint: " << pidAngle.GetSetpoint() << endl;
		cout << "customGyroOfset: " << customGyroOfset << endl;

		if(pidAngle.IsEnabled() && abs(pidAngle.GetError()) < AllowedAngleError ) {
			pidAngle.Disable();
			return true;
		} else {
			return false;
		}
	}

	//set the current gyro angle to 0 simalar to ResetEncoders()
	void ResetGyro() {
		//  customGyroOfset + GetGyroYaw() = pGyro->GetYaw()
		//customGyroOfset = pGyro->GetYaw();
		CalibrateGyro();
	}

	// use this instead of pGyro->GetYaw()
	float GetGyroYaw() {
		return pGyro->GetYaw() + customGyroOfset;
	}

	// only call at the begginning of match
	// may make code hang for a little bit
	void CalibrateGyro() {
		pGyro->ZeroYaw();
		// while(!(pGyro->GetYaw() < 1 && pGyro->GetYaw() > -1)) {

		// 	cout << pGyro->GetYaw() << endl;
		// 	waiting = true;
		// }
		// if(pGyro->GetYaw() < 1 && pGyro->GetYaw() > -1) {
		// 	cout << "gyro caibrated" << endl;
		// 	waiting = false;
		// 	ResetGyro();
		// }
	}

	//call every tick to climb hab
	bool Climb() {
		

		//returns true when done
		return false; // unimplemented
	}

	void setArmPosition(float targetPos) {
		armMaster.Set(ControlMode::MotionMagic, targetPos);
		currArmPos = armMaster.GetSelectedSensorPosition();
	}
	//sets "pistonID" to extended if true and retracted if false
	//TODO
	void SetPistonExtended(int pistonID, bool value){
		switch(pistonID) {
			// case Constant::PCM_CHANNEL_FRONT_LEFT:
			// 	if(value) {
			// 		frontLeftSol.Set(frc::DoubleSolenoid::Value::kForward);
			// 	} else {
			// 		frontLeftSol.Set(frc::DoubleSolenoid::Value::kReverse);
			// 	}
			// 	break;
			case Constant::PCM_CHANNEL_FRONT:
				cout << "FRONT EXTENDED" << endl;
				cout << "VALUE: " << value << endl;
				if(value) {
					if (retracted) {
						frontSol.Set(true);
						extended = true;
						retracted = false;
						cout << "Extending" << endl;
					}
				} else {
					if (extended) {
						frontSol.Set(false);
						extended = false;
						retracted = true;

						cout << "Retracting" << endl;
					}
					
				}
				break;
			// case Constant::PCM_CHANNEL_REAR_LEFT:
			// 	if(value) {
			// 		rearLeftSol.Set(frc::DoubleSolenoid::Value::kForward);
			// 	} else {
			// 		rearLeftSol.Set(frc::DoubleSolenoid::Value::kReverse);
			// 	}
			// 	break;
			case Constant::PCM_CHANNEL_REAR:
				if(value) {
					if (rretracted) {
						rearSol.Set(true);
						rextended = true;
						rretracted = false;
						cout << "Extending" << endl;
					}
				} else {
					if (rextended) {
						rearSol.Set(false);
						rextended = false;
						rretracted = true;

						cout << "Retracting" << endl;
					}
					
				}
				break;
			case Constant::PCM_EJECT_IN:
				if(value) {
					ejectSol.Set(frc::DoubleSolenoid::Value::kForward);
				} else {
					ejectSol.Set(frc::DoubleSolenoid::Value::kReverse);
				}
				break;
			// case Constant::PCM_CHANNEL_EJECT_RIGHT:
			// 	if(value) {
			// 		ejectRightSol.Set(frc::DoubleSolenoid::Value::kForward);
			// 	} else {
			// 		ejectRightSol.Set(frc::DoubleSolenoid::Value::kReverse);
			// 	}
			// 	break;
			// case Constant::PCM_CHANNEL_LATCH:
			// 	if(value) {
			// 		latchSol.Set(frc::DoubleSolenoid::Value::kForward);
			// 	} else {
			// 		latchSol.Set(frc::DoubleSolenoid::Value::kReverse);
			// 	}
			// 	break;
		}
	}

	// Checks if vision is active and available
	bool IsVisionAvailable() {
		return missed_frames < 50;
	}

	//checks if auton can ApproachLine()
	bool IsLineApproachable() {
		if (chrono::duration_cast<chrono::milliseconds>(time_clock.now() - last_frame_time).count() > 35) // old frame is outdated
			UpdateRaspiInput();
		return (abs(frame.angle) < 10 && abs(frame.line_offset) < 2.5 &&      // thresholds
		        frame.angle != 0 && frame.line_offset != 0 && !frame.error && IsVisionAvailable()); // check for problems
	}

	//this is a duplicate of ApproachLine()
	//TODO is this better than ApproachLine()?
	// returns 0 if needs to be run again, 1 if done, -1 if impossible
	uint8_t dock_state = 0;
	int DockRobot() {
		if (!IsVisionAvailable()) return -1;
		if (dock_state == 1) {
			if (FollowAutonInstructions()) dock_state = 0;
			return 0;
		}
		if (IsLineApproachable()) {
			cout << "Line is approachable.\n";
			if (frame.wall_distance_left > 3 && !(frame.error & VISION_ERROR_BAD_DISTANCE)) {
				ClearAutonInstructions();
				autonInstructions[0] = (frame.wall_distance_left / 2.54) - 2.0;
				cout << "Instructions:\n";
				cout << "\tMoving forward " << autonInstructions[0] << " inches.\n";
				dock_state = 1;
				return 0;
			}
			return 1;
		} else if (abs(frame.line_offset) >= 2.5) {
			ClearAutonInstructions();
			autonInstructions[3] = (90 - frame.angle) * (frame.line_offset < 0 ? 1 : -1);
			autonInstructions[2] = frame.line_offset / 2.54;
			autonInstructions[1] = -90 * (frame.line_offset < 0 ? 1 : -1);
			autonInstructions[0] = 0;
			cout << "Instructions:\n";
			cout << "\tRotating " << autonInstructions[3] << " degrees.\n";
			cout << "\tMoving forward " << autonInstructions[2] << " inches.\n";
			cout << "\tRotating " << autonInstructions[1] << " degrees.\n";
			dock_state = 1;
			return 0;
		} else if (frame.error) {
			return 0;
		} else {
			ClearAutonInstructions();
			autonInstructions[1] = -frame.angle;
			cout << "Instructions:\n";
			cout << "\tRotating " << autonInstructions[1] << " degrees.\n";
			dock_state = 1;
			return 0;
		}
	}

	// Halts all motion.
	void StopRobot() {
		leftMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		rightMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		ResetEncoders();
		ResetGyro();
	}

	// BREAKING NEWS: Jack announces Dock Robot 2 //
	//    Said to be better than Dock Robot 1     //
	bool dockRobot2Running = false;
	int DockRobot2() {
		IsLineApproachable();
		if (frame.wall_distance_left < 15 && !(frame.error & VISION_ERROR_BAD_DISTANCE) && frame.wall_distance_left != 0) {
			cout << "=== Too close, stopping. === " << frame.wall_distance_left << "\n";
			StopRobot();
			return 1;
		}
		if (frame.wall_distance_left < 40 && frame.angle < 5.0 && !(frame.error & VISION_ERROR_BAD_DISTANCE) && frame.wall_distance_left != 0) {
			cout << "=== Using ultrasonic. ===\n";
			if (dockRobot2Running) {
				ResetEncoders();
				ResetGyro();
				float lastRotation = autonInstructions[1];
				ClearAutonInstructions();
				autonInstructions[1] = lastRotation;
				autonInstructions[0] = frame.wall_distance_left / 2.54;
				dockRobot2Running = false;
			}
			return FollowAutonInstructions();
		}
		if (dockRobot2Running) {
			FollowAutonInstructions();
			return 0;
		}
		if (frame.error) return -1;
		float theta = (frame.line_offset * Constant::HORIZONTAL_FOV) / 640.0;
		float r = Constant::CAMERA_HEIGHT * tan(((-frame.line_offset_y * rad(Constant::VERTICAL_FOV)) / 480.0) + atan(Constant::CAMERA_CENTER_LINE_DISTANCE / Constant::CAMERA_HEIGHT));
		// In theory, this should be valid (though my track record with theories is not 100%)
		ClearAutonInstructions();
		autonInstructions[3] = theta;
		autonInstructions[2] = abs(r);
		autonInstructions[1] = -theta - frame.angle;
		cout << "Instructions:\n";
		cout << "\tRotating " << autonInstructions[3] << " degrees.\n";
		cout << "\tMoving forward " << autonInstructions[2] << " inches.\n";
		cout << "\tRotating " << autonInstructions[1] << " degrees.\n";
		cout << "Theta: " << theta << ", r: " << r << "\n";
		dockRobot2Running = true;
		FollowAutonInstructions();
		return 0;
	}

	//call every tick to take a hatch pannel from the brushes
	//TODO
	bool RetrieveHatch(){
		//returns true when done
		return false; // unimplemented
	}

	//call every tick to place the hatch pannel on a rocket or cargoship
	//TODO
	void PlaceHatch(){
		if(latchLimitSwitch->Get()) {
			SetPistonExtended(Constant::PCM_CHANNEL_LATCH, true);
			//SetArmAngle(90);
		} else {
			leftMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.2);
			rightMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.2);
		}
	}

	// call every tick to put the orange ball in a cargoship/rocket
	bool PlaceCargo(){
		//returns true when done
		return false; // unimplemented
	}

	//spins weels to shot out cargo
	//TODO
	void EjectCargo(){
		// for (int i = 0; i < 1000; i++) {
		// 	armMaster.Set(ControlMode::PercentOutput, 1);
		// }
		
		// armMaster.Set(ControlMode::PercentOutput, 0);
		
	}
};

bool Robot::cameraLoopStatus = false;

//START_ROBOT_CLASS(Robot)
int main() {return frc::StartRobot<Robot>();}