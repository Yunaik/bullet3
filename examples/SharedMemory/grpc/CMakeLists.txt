
find_package(GRPC REQUIRED)
find_package(Protobuf REQUIRED)
if(GRPC_FOUND)
	MESSAGE("Grpc FOUND")
	else()
	MESSAGE("Grpc NOT FOUND")
endif()


INCLUDE_DIRECTORIES(
    ${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/examples
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory

)

SET(App_PhysicsServerSharedMemoryBridgeGRPC_SRC
    	../PhysicsClient.cpp
		../PhysicsClient.h
		../PhysicsDirect.cpp
		../PhysicsDirect.h
		../PhysicsCommandProcessorInterface.h
		../SharedMemoryCommandProcessor.cpp
		../SharedMemoryCommandProcessor.h
		../PhysicsClientC_API.cpp
		../PhysicsClientC_API.h
		../Win32SharedMemory.cpp
		../Win32SharedMemory.h
		../PosixSharedMemory.cpp
		../PosixSharedMemory.h
		../../Utils/b3ResourcePath.cpp
		../../Utils/b3ResourcePath.h
		../../Utils/b3Clock.cpp
		../../Utils/b3Clock.h
    )

LINK_LIBRARIES(  pthread dl )

ADD_EXECUTABLE(App_PhysicsServerSharedMemoryBridgeGRPC
	main.cpp
	ConvertGRPCBullet.h
	ConvertGRPCBullet.cpp
	proto/pybullet.grpc.pb.h
	proto/pybullet.grpc.pb.cpp
	proto/pybullet.pb.h
	proto/pybullet.pb.cpp
    ${App_PhysicsServerSharedMemoryBridgeGRPC_SRC}
)

TARGET_LINK_LIBRARIES(App_PhysicsServerSharedMemoryBridgeGRPC
    BulletFileLoader Bullet3Common LinearMath gRPC::grpc gRPC::grpc++ gRPC::grpc++_reflection protobuf::libprotobuf
)

target_compile_options(App_PhysicsServerSharedMemoryBridgeGRPC PRIVATE
        -std=c++11
)


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)





#################
# Standalone BasicExampleGui using OpenGL (but not the example browser)


INCLUDE_DIRECTORIES(
        ${BULLET_PHYSICS_SOURCE_DIR}/src
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
)


SET(App_PhysicsServerGRPC_SRC
	../IKTrajectoryHelper.cpp
	../IKTrajectoryHelper.h
	../SharedMemoryCommands.h
	../SharedMemoryPublic.h
	../PhysicsServerCommandProcessor.cpp
	../PhysicsServerCommandProcessor.h
	../b3PluginManager.cpp
	../PhysicsDirect.cpp
	../PhysicsClientC_API.cpp
	../PhysicsClient.cpp
	../plugins/collisionFilterPlugin/collisionFilterPlugin.cpp
	../plugins/pdControlPlugin/pdControlPlugin.cpp
	../plugins/pdControlPlugin/pdControlPlugin.h
	../b3RobotSimulatorClientAPI_NoDirect.cpp
	../b3RobotSimulatorClientAPI_NoDirect.h
    ../plugins/tinyRendererPlugin/tinyRendererPlugin.cpp
	../plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp
	../../TinyRenderer/geometry.cpp
	../../TinyRenderer/model.cpp
	../../TinyRenderer/tgaimage.cpp
	../../TinyRenderer/our_gl.cpp
	../../TinyRenderer/TinyRenderer.cpp
	../../OpenGLWindow/SimpleCamera.cpp
	../../OpenGLWindow/SimpleCamera.h
	../../Importers/ImportURDFDemo/ConvertRigidBodies2MultiBody.h
	../../Importers/ImportURDFDemo/MultiBodyCreationInterface.h
	../../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
	../../Importers/ImportURDFDemo/MyMultiBodyCreator.h
	../../Importers/ImportMJCFDemo/BulletMJCFImporter.cpp
	../../Importers/ImportMJCFDemo/BulletMJCFImporter.h
	../../Importers/ImportURDFDemo/BulletUrdfImporter.cpp
	../../Importers/ImportURDFDemo/BulletUrdfImporter.h
	../../Importers/ImportURDFDemo/UrdfParser.cpp
 	../../Importers/ImportURDFDemo/urdfStringSplit.cpp
	../../Importers/ImportURDFDemo/UrdfParser.cpp
	../../Importers/ImportURDFDemo/UrdfParser.h
	../../Importers/ImportURDFDemo/URDF2Bullet.cpp
	../../Importers/ImportURDFDemo/URDF2Bullet.h
	../../Utils/b3ResourcePath.cpp
	../../Utils/b3Clock.cpp
	../../Utils/ChromeTraceUtil.cpp
	../../Utils/ChromeTraceUtil.h
	../../Utils/RobotLoggingUtil.cpp
	../../Utils/RobotLoggingUtil.h
#	../../../Extras/Serialize/BulletWorldImporter/*
#	../../../Extras/Serialize/BulletFileLoader/*
	../../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h
	../../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp
	../../../Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.h
	../../../Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.cpp
	../../../Extras/Serialize/BulletWorldImporter/btWorldImporter.h
	../../../Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp
	../../../Extras/Serialize/BulletFileLoader/bFile.h
	../../../Extras/Serialize/BulletFileLoader/bFile.cpp
	../../../Extras/Serialize/BulletFileLoader/bDNA.h
	../../../Extras/Serialize/BulletFileLoader/bDNA.cpp
	../../../Extras/Serialize/BulletFileLoader/bDefines.h
	../../../Extras/Serialize/BulletFileLoader/bCommon.h
	../../../Extras/Serialize/BulletFileLoader/bChunk.h
	../../../Extras/Serialize/BulletFileLoader/bChunk.cpp
	../../../Extras/Serialize/BulletFileLoader/btBulletFile.h
	../../../Extras/Serialize/BulletFileLoader/btBulletFile.cpp
	../../Importers/ImportURDFDemo/URDFImporterInterface.h
	../../Importers/ImportURDFDemo/URDFJointTypes.h
	../../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
	../../Importers/ImportObjDemo/LoadMeshFromObj.cpp
	../../Importers/ImportSTLDemo/ImportSTLSetup.h
	../../Importers/ImportSTLDemo/LoadMeshFromSTL.h
	../../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
	../../Importers/ImportColladaDemo/ColladaGraphicsInstance.h
	../../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
	../../ThirdPartyLibs/tinyxml2/tinyxml2.cpp
	../../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp
	../../ThirdPartyLibs/stb_image/stb_image.cpp
)

#this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
#ADD_DEFINITIONS(-DNO_SHARED_MEMORY)

ADD_EXECUTABLE(App_PhysicsServerGRPC
	main.cpp
	ConvertGRPCBullet.h
	ConvertGRPCBullet.cpp
	proto/pybullet.grpc.pb.h
	proto/pybullet.grpc.pb.cpp
	proto/pybullet.pb.h
	proto/pybullet.pb.cpp
    ${App_PhysicsServerGRPC_SRC}
)


TARGET_LINK_LIBRARIES(App_PhysicsServerGRPC
    clsocket Bullet3Common BulletInverseDynamicsUtils  BulletInverseDynamics 	BulletSoftBody  BulletDynamics BulletCollision  LinearMath  BussIK ${GRPC_LIBRARY} ${GRPC_GRPC++_LIBRARY} ${GRPC_GRPC++_REFLECTION_LIBRARY} ${PROTOBUF_LIBRARIES}
)

target_compile_definitions(App_PhysicsServerGRPC PRIVATE NO_SHARED_MEMORY)

target_compile_options(App_PhysicsServerGRPC PRIVATE
        -std=c++11
)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)


