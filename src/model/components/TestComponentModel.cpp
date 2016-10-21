/*
 * @(#) TestComponentModel.cpp   1.0   Feb 8, 2013
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
#include "model/components/TestComponentModel.h"

namespace robogen {


TestComponentModel::TestComponentModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id) :
		Model(odeWorld, odeSpace, id)
{}

TestComponentModel::~TestComponentModel() {

}

bool TestComponentModel::initModel() {

	rootBody_ = this->addCylinder(
			inGrams(100),
			osg::Vec3(), 1,
			inMm(10), inMm(200), 0);

	return true;
}

boost::shared_ptr<SimpleBody> TestComponentModel::getRoot() {
	return rootBody_;
}

boost::shared_ptr<SimpleBody> TestComponentModel::getSlot(unsigned int /*i*/) {
	return rootBody_;
}

osg::Vec3 TestComponentModel::getSlotPosition(unsigned int i) {
	return osg::Vec3();
}

osg::Vec3 TestComponentModel::getSlotAxis(unsigned int i) {
	return osg::Vec3();
}

osg::Vec3 TestComponentModel::getSlotOrientation(unsigned int i) {
	return osg::Vec3();
}

}
