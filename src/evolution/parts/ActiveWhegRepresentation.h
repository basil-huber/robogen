/*
 * ActiveWhegRepresentation.h
 *
 *  Created on: Aug 28, 2013
 *      Author: lis
 */

#ifndef ACTIVEWHEGREPRESENTATION_H_
#define ACTIVEWHEGREPRESENTATION_H_

#include "evolution/PartRepresentation.h"

namespace robogen {

class ActiveWhegRepresentation : public PartRepresentation {
public:
	ActiveWhegRepresentation(std::string id, int orientation, double radius);
	virtual ~ActiveWhegRepresentation();
	virtual std::vector<std::string> getMotors();
	virtual std::vector<std::string> getSensors();
private:
	/**
	 * Radius of the wheg
	 */
	double radius_;
};

} /* namespace robogen */
#endif /* ACTIVEWHEGREPRESENTATION_H_ */
