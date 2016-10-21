//#include "render/components/CoreComponentRenderModel.h"
#include "model/components/TestComponentModel.h"
#include "render/components/TestComponentRenderModel.h"
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>

#include "scenario/Terrain.h"
#include "render/objects/TerrainRender.h"

using namespace robogen;

dWorldID odeWorld;
dJointGroupID odeContactGroup;

int main( int argc, const char* argv[] )
{

	dInitODE();

	// Create ODE world
	odeWorld = dWorldCreate();

	// Set gravity
	//dWorldSetGravity(odeWorld, 0, 0, -9.81);

	dWorldSetERP(odeWorld, 0.4);
	dWorldSetCFM(odeWorld, 0.2);

	// Create contact group
	odeContactGroup = dJointGroupCreate(0);

	// Create collision world
	dSpaceID odeSpace = dSimpleSpaceCreate(0);

	// ---------------------------------------
	// OSG Initialization
	// ---------------------------------------

	//Creating the viewer
	osgViewer::Viewer viewer;

	viewer.setUpViewInWindow(300, 300, 400, 400);

	// Camera
	osg::ref_ptr<osg::Camera> camera = viewer.getCamera();

	//Creating the root node
	osg::ref_ptr<osg::Group> root(new osg::Group);


	// boost::shared_ptr<Terrain> terrain(new Terrain(odeWorld, odeSpace));
	// //terrain->initRough("../terrain.gif", 1000, 1000, 500);
	// terrain->initFlat(1, 1);
	// boost::shared_ptr<TerrainRender> terrainRender(new TerrainRender(terrain));
	// root->addChild(terrainRender->getRootNode());

	boost::shared_ptr<TestComponentModel> testComponent(new TestComponentModel(odeWorld, odeSpace, std::string("my_test_component")));
	testComponent->initModel();
	//testComponent->setRootPosition(osg::Vec3(0, 0, 0));
	dBodyID body = testComponent->getRoot()->getBody();
	const dReal* ang_vel = dBodyGetAngularVel(body);
	const dReal* quat = dBodyGetQuaternion(body);

	boost::shared_ptr<TestComponentRenderModel> testRender(new TestComponentRenderModel(testComponent));
	testRender->setDebugActive(true);
	testRender->initRenderModel();
	root->addChild(testRender->getRootNode());
	

	viewer.setSceneData(root.get());

	osg::Timer_t prevTime = osg::Timer::instance()->tick();
	viewer.realize();

	if (!viewer.getCameraManipulator()
			&& viewer.getCamera()->getAllowEventFocus()) {
		viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	}

	viewer.setReleaseContextAtEndOfFrameHint(false);

	double t = 0;
	double deltaSecs = 0;
	int i = 0;
	char torqAxis = '0';
	double torq = 0.01;
	while (!viewer.done())
	{
		viewer.frame();

		// Update physics

		const double MAX_STEP = 0.01;
		const osg::Timer_t now = osg::Timer::instance()->tick();
		deltaSecs += osg::Timer::instance()->delta_s(prevTime, now);
		prevTime = now;

		while (deltaSecs > MAX_STEP) {

			//setupCollision(touchSensors);

			//dSpaceCollide(odeSpace, 0, odeCollisionCallback);

			const double step = std::min(MAX_STEP, deltaSecs);
			deltaSecs -= MAX_STEP;

			dWorldStep(odeWorld, step);

			//std::vector<boost::shared_ptr<Motor> > motors;
			//activeHingeA->getMotors(motors);
			//boost::dynamic_pointer_cast<ServoMotor>(motors[0])->setPosition(0.5*sin(t)+0.5);
			t += step/3;


			dJointGroupEmpty(odeContactGroup);
		}
		


		if(i % 600 == 0)
		{
			// Reset orientation and angular velocity
			dBodySetAngularVel(body, 0,0,0);
			dQuaternion quat0 = {1,0,0,0};
			dBodySetQuaternion(body, quat0);

			int ii = i/600;
			switch(ii%3)
			{
				case 0:
					torqAxis = 'z';
					dBodyAddTorque(body, 0,0,torq);
					break;

				case 1:
					torqAxis = 'y';
					dBodyAddTorque(body, 0,torq,0);
					break;

				case 2:
					torqAxis = 'x';
					dBodyAddTorque(body, torq,0,0);
					break;
			}
			

		}else if(i % 600 == 1)
					std::cout << "torque axis: " << torqAxis << "  angular velocity: x: " << ang_vel[0] << "  y: " << ang_vel[1] << "  z: " << ang_vel[2] << std::endl;
		i++;
	}

	// ---------------------------------------
	// Finalize
	// ---------------------------------------

	// Destroy ODE space
	dSpaceDestroy(odeSpace);

	// Destroy ODE world
	dWorldDestroy(odeWorld);

	dCloseODE();

}