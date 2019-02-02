/*
 * PIDGyroSource.h
 *
 *  Created on: Feb 1, 2018
 *      Author: Team 6101
 */

#ifndef SRC_PIDGYROSOURCE_H_
#define SRC_PIDGYROSOURCE_H_
#include <frc/AHRS.h>

#include <PIDSource.h>

class PIDGyroSource: public frc::PIDSource {

private:
	AHRS* pGyro;
	 frc::PIDSourceType m_pidSourceType;

public:

	PIDGyroSource(AHRS* gyro);
	virtual ~PIDGyroSource();

	double PIDGet();
	void SetPDISourceType( frc::PIDSourceType pidSource) {m_pidSourceType = pidSource; }
	frc::PIDSourceType GetPIDSourceType() const {return m_pidSourceType;}

//	frc::PIDSourceType getPIDSourceType();
};

#endif /* SRC_PIDGYROSOURCE_H_ */