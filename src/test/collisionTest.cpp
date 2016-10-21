//#include "render/components/CoreComponentRenderModel.h"
#include "test/TestSphereModel.h"
#include "test/TestComponentRenderModel.h"
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>

#include "scenario/Terrain.h"
#include "render/objects/TerrainRender.h"

using namespace robogen;

#define MAX_CONTACTS 50

dWorldID odeWorld;
dJointGroupID odeContactGroup;


std::map<dGeomID, boost::shared_ptr<TestComponentRenderModel> > spheresMap;
std::vector<boost::shared_ptr<TestSphereModel> > spheres;
dGeomID testComponentGeomID;
boost::shared_ptr<TestComponentRenderModel> testRender;

void odeCollisionCallback(void*, dGeomID o1, dGeomID o2);

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


	/*---------------------------------------
	 *       Create TestComponent
	 *---------------------------------------*/
	boost::shared_ptr<TestComponentModel> testComponent(new TestComponentModel(odeWorld, odeSpace, std::string("my_test_component")));
	testComponent->initModel();
	testComponent->setRootPosition(osg::Vec3(0, 0, 0));
	dBodyID body = testComponent->getRoot()->getBody();
	testComponentGeomID = testComponent->getRoot()->getGeom();
	const dReal* ang_vel = dBodyGetAngularVel(body);
	const dReal* quat = dBodyGetQuaternion(body);

	testRender = boost::shared_ptr<TestComponentRenderModel>(new TestComponentRenderModel(testComponent));
	testRender->setDebugActive(true);
	testRender->initRenderModel();
	root->addChild(testRender->getRootNode());
	
	/*---------------------------------------
	 *       Create small sphere's
	 *---------------------------------------*/
	
	std::vector<boost::shared_ptr<TestComponentRenderModel> > sphereRenders;
	int i_sphere = 0;
	std::cout << "Creating spheres" << std::endl;
	for(float x = -inMm(70); x < inMm(70); x += inMm(5))
	{
		for(float y = -inMm(70); y < inMm(70); y += inMm(5))
		{
			std::string sphereID("sphere");
			sphereID.append(std::to_string(i_sphere++));
			boost::shared_ptr<TestSphereModel> testSphere(new TestSphereModel(odeWorld, odeSpace, sphereID));
			testSphere->setColor(osg::Vec4(0,1,0,1));
			testSphere->initModel();
			testSphere->setRootPosition(osg::Vec3(x, y, 0));
			boost::shared_ptr<TestComponentRenderModel> testSphereRender(new TestComponentRenderModel(testSphere));
			testSphereRender->initRenderModel();
			root->addChild(testSphereRender->getRootNode());

			spheres.push_back(testSphere);
			sphereRenders.push_back(testSphereRender);
			dGeomID geomID = testSphere->getRoot()->getGeom(); 
			spheresMap[geomID] = testSphereRender;
		}
	}
	std::cout << "Finished Creating spheres" << std::endl;

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
			
			// reset color of all spheres to green
			for(std::map<dGeomID, boost::shared_ptr<TestComponentRenderModel> >::iterator it = spheresMap.begin(); it != spheresMap.end(); it++)
			{
				it->second->setColor(osg::Vec4(0,1,0,1));
			}
			// calculate collisions
			dSpaceCollide(odeSpace, 0, odeCollisionCallback);

			
			const double step = std::min(MAX_STEP, deltaSecs);
			deltaSecs -= MAX_STEP;

			dWorldStep(odeWorld, step);

			//std::vector<boost::shared_ptr<Motor> > motors;
			//activeHingeA->getMotors(motors);
			//boost::dynamic_pointer_cast<ServoMotor>(motors[0])->setPosition(0.5*sin(t)+0.5);
			t += step/3;


			dJointGroupEmpty(odeContactGroup);
		}
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


void odeCollisionCallback(void*, dGeomID o1, dGeomID o2)
{
	boost::shared_ptr<TestComponentRenderModel> sphereRender = NULL;

	// check if o1 is the testComponent
	if(o1 == testComponentGeomID)
	{
		// look for o2 in spheresMap
		if(spheresMap.find(o2) != spheresMap.end())
		{
			sphereRender = spheresMap[o2];
		}
	} else if(o2 == testComponentGeomID)
	{
		// look for o1 in spheresMap
		if(spheresMap.find(o1) != spheresMap.end())
		{
			sphereRender = spheresMap[o1];
		}
	}

	if(sphereRender == NULL){
		return;
	}

	dContact contact[MAX_CONTACTS];
	// for (int i = 0; i < MAX_CONTACTS; i++) {

	// 	contact[i].surface.mode = dContactBounce | dContactSoftCFM;
	// 	contact[i].surface.mu = dInfinity;
	// 	contact[i].surface.mu2 = 0;
	// 	contact[i].surface.bounce = 0.01;
	// 	contact[i].surface.bounce_vel = 0.1;
	// 	contact[i].surface.soft_cfm = 0.0001;

	// }

	int collisionCounts = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
			sizeof(dContact));

	if(collisionCounts > 0)
		sphereRender->setColor(osg::Vec4(1,0,0,1));
	else
		sphereRender->setColor(osg::Vec4(0,0,1,1));
	//testRender->setColor(osg::Vec4(0,1,0,1));
}