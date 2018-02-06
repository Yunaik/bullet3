#include "ImportSTLSetup.h"
#include <vector>
#include "../../Utils/b3ResourcePath.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "LoadMeshFromSTL.h"
#include "btBulletDynamicsCommon.h"

class ImportSTLSetup : public CommonRigidBodyBase {
  const char* m_fileName;
  btVector3 m_scaling;

 public:
  ImportSTLSetup(struct GUIHelperInterface* helper, const char* fileName);
  virtual ~ImportSTLSetup();

  virtual void initPhysics();
  virtual void resetCamera() {
    float dist = 3.5;
    float pitch = -28;
    float yaw = -136;
    float targetPos[3] = {0.47, 0, -0.64};
    m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1],
                             targetPos[2]);
  }
};

ImportSTLSetup::ImportSTLSetup(struct GUIHelperInterface* helper,
                               const char* fileName)
    : CommonRigidBodyBase(helper), m_scaling(btVector3(10, 10, 10)) {
  if (fileName) {
    m_fileName = fileName;
    m_scaling = btVector3(0.01, 0.01, 0.01);
  } else {
    m_fileName = "l_finger_tip.stl";
  }
}

ImportSTLSetup::~ImportSTLSetup() {}

void ImportSTLSetup::initPhysics() {
  m_guiHelper->setUpAxis(2);
  this->createEmptyDynamicsWorld();
  m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
  m_dynamicsWorld->getDebugDrawer()->setDebugMode(
      btIDebugDraw::DBG_DrawWireframe);

  char relativeFileName[1024];
  if (!b3ResourcePath::findResourcePath(m_fileName, relativeFileName, 1024)) {
    b3Warning("Cannot find file %s\n", m_fileName);
    return;
  }

  btVector3 shift(0, 0, 0);
  //	int index=10;

  {
    GLInstanceGraphicsShape* gfxShape = LoadMeshFromSTL(relativeFileName);

    btTransform trans;
    trans.setIdentity();
    trans.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI));

    btVector3 position = trans.getOrigin();
    btQuaternion orn = trans.getRotation();

    btVector3 color(0, 0, 1);

    int shapeId = m_guiHelper->getRenderInterface()->registerShape(
        &gfxShape->m_vertices->at(0).xyzw[0], gfxShape->m_numvertices,
        &gfxShape->m_indices->at(0), gfxShape->m_numIndices);

    m_guiHelper->getRenderInterface()->registerGraphicsInstance(
        shapeId, position, orn, color, m_scaling);
  }
}

class CommonExampleInterface* ImportSTLCreateFunc(
    struct CommonExampleOptions& options) {
  return new ImportSTLSetup(options.m_guiHelper, options.m_fileName);
}
