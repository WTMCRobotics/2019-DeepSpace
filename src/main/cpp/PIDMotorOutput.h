/*
 * PIDMotorOutput.h
 *
 *  Created on: Jan 31, 2018
 *      Author: Team 6101
 */

#ifndef SRC_PIDMOTOROUTPUT_H_
#define SRC_PIDMOTOROUTPUT_H_

#include <LiveWindow/LiveWindow.h>
#include <ctre/Phoenix.h>
#include <PIDOutput.h>

class PIDMotorOutput : public frc::PIDOutput
{
private:
	TalonSRX* pLeftMotor;
	TalonSRX* pRightMotor;

public:

	PIDMotorOutput();
	PIDMotorOutput(TalonSRX* leftMotor, TalonSRX* rightMotor);
	virtual ~PIDMotorOutput();

	void PIDWrite(double value);

};

#endif /* SRC_PIDMOTOROUTPUT_H_ */