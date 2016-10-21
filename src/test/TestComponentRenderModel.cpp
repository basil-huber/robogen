/*
 * @(#) CoreComponentRenderModel.cpp   1.0   Feb 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>

#include "render/callback/BodyCallback.h"

#include "test/TestComponentRenderModel.h"
#include "test/TestComponentModel.h"

#include "utils/RobogenUtils.h"

namespace robogen {

TestComponentRenderModel::TestComponentRenderModel(
		boost::shared_ptr<Model> model) :
		RenderModel(model) {
}

TestComponentRenderModel::~TestComponentRenderModel() {
}

bool TestComponentRenderModel::initRenderModel() {

	// Get color from model
	std::vector<osg::Vec4> colors;
	boost::shared_ptr<robogen::TestComponentModel> testComponent = boost::dynamic_pointer_cast<TestComponentModel>(this->getModel());
	if (testComponent) {
		colors.push_back(testComponent->getColor());
	}else
	{
		colors.push_back(osg::Vec4(1,1,1,0.5f));
	}

	pats_ = this->attachGeoms(colors);
	if (isDebugActive()) {
		attachAxis(pats_[0]);
	}

	return true;
}


void TestComponentRenderModel::setColor(osg::Vec4 color) {
	
	for(std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> >::iterator it = pats_.begin(); it != pats_.end(); it++)
	{
		osg::ref_ptr<osg::PositionAttitudeTransform> pat = (*it);
		for(int i = 0; i < pat->getNumChildren(); i++)
		{
			osg::Geode* geode = dynamic_cast<osg::Geode*>(pat->getChild(i));
			if(!geode)
				continue;
			for(int j=0; j < geode->getNumDrawables(); j++)
			{
				osg::Drawable* drawable = geode->getDrawable(j);

				osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(drawable);
				if(geometry)
				{
					osg::Vec4Array* colors = new osg::Vec4Array;
					colors->push_back(color);
    				geometry->setColorArray(colors);
					geometry->setColorBinding(osg::Geometry::BIND_OVERALL );
				}else{
					osg::ShapeDrawable* shape = dynamic_cast<osg::ShapeDrawable*>(drawable);
					if(shape)
					{
						shape->setColor(color);
					}
				}
			}
		}
	}
}

}
