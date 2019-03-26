
#include "b3RobotSimulatorClientAPI_NoGUI.h"
#include "b3RobotSimulatorClientAPI_InternalData.h"

#include "SharedMemory/physx/PhysXC_API.h"
#include "SharedMemory/PhysicsDirect.h"
#include "SharedMemory/physx/PhysXServerCommandProcessor.h"

int main(int argc, char* argv[])
{
	b3RobotSimulatorClientAPI_NoGUI* sim = new b3RobotSimulatorClientAPI_NoGUI();

	PhysXServerCommandProcessor* sdk = new PhysXServerCommandProcessor(argc, argv);
	PhysicsDirect* direct = new PhysicsDirect(sdk, true);
	bool connected;
	connected = direct->connect();
		
	b3RobotSimulatorClientAPI_InternalData data;
	data.m_guiHelper = 0;
	data.m_physicsClientHandle = (b3PhysicsClientHandle)direct;

	//we bypass 'sim->connect' since we have a custom connection to the PhysX sdk
	sim->setInternalData(&data);
	
	if (!sim->isConnected())
	{
		printf("Error connecting to PhysX\n");
		exit(0);
	}

	int pluginUid = sim->loadPlugin("eglRendererPlugin");

	//remove all existing objects (if any)
	//sim->resetSimulation();
	sim->setGravity(btVector3(0, 0, -9.8));
	sim->setNumSolverIterations(1);
	b3RobotSimulatorSetPhysicsEngineParameters args1;
	sim->getPhysicsEngineParameters(args1);
	btScalar dt = 1. / 240.;
	dt /= 25;// 25;// 128.;
	sim->setTimeStep(dt);

	b3RobotSimulatorLoadUrdfFileArgs args2;
	args2.m_useMultiBody = false;
	
	int planeUid = sim->loadURDF("plane.urdf", args2);
	printf("planeUid = %d\n", planeUid);

	int r2d2Uid = sim->loadURDF("r2d2.urdf");
	printf("r2d2 #joints = %d\n", sim->getNumJoints(r2d2Uid));

	btVector3 basePosition = btVector3(3, 0, 1.);
	btQuaternion baseOrientation = btQuaternion(0, 0, 0, 1);

	sim->resetBasePositionAndOrientation(r2d2Uid, basePosition, baseOrientation);

	{
		b3RobotSimulatorLoadUrdfFileArgs args;
		b3RobotSimulatorChangeDynamicsArgs dynamicsArgs;
		int massRatio = 2;
		int mass = 1;
		btScalar boxSize = 0.05;
		for (int i = 0; i < 8; i++)
		{
			args.m_startPosition.setValue(0, 0, boxSize*0.5+i * boxSize);
			args.m_useMultiBody = false;
			int boxIdx = sim->loadURDF("cube_small.urdf", args);
			if (i == 7)
				mass = 10000;
			dynamicsArgs.m_mass = mass;
			sim->changeDynamics(boxIdx, -1, dynamicsArgs);
			
		}
	}

	while (sim->isConnected())
	{
		btVector3 basePos;
		btQuaternion baseOrn;
		sim->getBasePositionAndOrientation(r2d2Uid, basePos, baseOrn);
		printf("r2d2 basePosition = [%f,%f,%f]\n", basePos[0], basePos[1], basePos[2]);

		sim->stepSimulation();
	}
	delete sim;
}
