
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
	sim->setNumSolverIterations(100);
	b3RobotSimulatorSetPhysicsEngineParameters args;
	sim->getPhysicsEngineParameters(args);

	int planeUid = sim->loadURDF("plane.urdf");
	printf("planeUid = %d\n", planeUid);

	int r2d2Uid = sim->loadURDF("r2d2.urdf");
	printf("r2d2 #joints = %d\n", sim->getNumJoints(r2d2Uid));

	btVector3 basePosition = btVector3(0, 0, 1.);
	btQuaternion baseOrientation = btQuaternion(0, 0, 0, 1);

	sim->resetBasePositionAndOrientation(r2d2Uid, basePosition, baseOrientation);

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
