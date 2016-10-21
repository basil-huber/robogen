/*
 * @(#) TestSphereModel.h   1.0   Feb 8, 2013
 *
 * Basil Huber (basil.huber@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_TEST_SPHEREMODEL_H_
#define ROBOGEN_TEST_SPHEREMODEL_H_

#include "test/TestComponentModel.h"

namespace robogen {

class TestSphereModel: public TestComponentModel {

public:

	/**
	 * Initializes a CoreComponentModel
	 * @param odeWorld
	 * @param odeSpace
	 * @param id
	 * @param isCore should be true for the core/root node and will therefore
	 * 			include mass of electronics.  If isCore is true but hasSensors
	 * 			is false then will not consider IMU
	 * @param hasSensors if true the core component will contain gyro and
	 * 			accelerometer sensors, if false it won't provide any sensor
	 */
	TestSphereModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id);

	virtual ~TestSphereModel();

	virtual bool initModel();
};

}

#endif /* ROBOGEN_TEST_SPHEREMODEL_H_ */
