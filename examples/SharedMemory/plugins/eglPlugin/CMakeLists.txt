
find_package(OpenGL REQUIRED)
find_library(COCOA Cocoa)

INCLUDE_DIRECTORIES(
        ${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/glad
		/opt/X11/include
)


SET(pybullet_eglRendererPlugin_SRC
			eglRendererPlugin.cpp
			eglRendererPlugin.h
			eglRendererVisualShapeConverter.cpp
			eglRendererVisualShapeConverter.h
			../../../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
			../../../Importers/ImportColladaDemo/LoadMeshFromCollada.h
			../../../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp
			../../../Importers/ImportMeshUtility/b3ImportMeshUtility.h
			../../../Importers/ImportObjDemo/LoadMeshFromObj.cpp
			../../../Importers/ImportObjDemo/LoadMeshFromObj.h
			../../../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
			../../../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h
			../../../TinyRenderer/geometry.cpp
			../../../TinyRenderer/model.cpp
			../../../TinyRenderer/our_gl.cpp
			../../../TinyRenderer/tgaimage.cpp
			../../../TinyRenderer/TinyRenderer.cpp
			../../../ThirdPartyLibs/glad/gl.c
			../../../ThirdPartyLibs/glad/egl.c
			../../../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
			../../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h
			../../../ThirdPartyLibs/stb_image/stb_image.cpp
			../../../ThirdPartyLibs/stb_image/stb_image.h
			../../../ThirdPartyLibs/stb_image/stb_image_write.cpp
			../../../ThirdPartyLibs/tinyxml2/tinyxml2.cpp
			../../../ThirdPartyLibs/tinyxml2/tinyxml2.h
			../../../OpenGLWindow/SimpleCamera.cpp
			../../../OpenGLWindow/SimpleCamera.h
			../../../OpenGLWindow/GLInstancingRenderer.cpp
			../../../OpenGLWindow/GLInstancingRenderer.h
			../../../OpenGLWindow/LoadShader.cpp
			../../../OpenGLWindow/LoadShader.h
			../../../OpenGLWindow/GLRenderToTexture.cpp
			../../../OpenGLWindow/GLRenderToTexture.h
#			../../../OpenGLWindow/MacOpenGLWindow.h
			../../../OpenGLWindow/MacOpenGLWindow.cpp
			../../../Utils/b3Clock.cpp
			../../../Utils/b3Clock.h
			../../../Utils/b3ResourcePath.cpp
			../../../Utils/b3ResourcePath.h
)

#this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
#ADD_DEFINITIONS(-DNO_SHARED_MEMORY)

ADD_LIBRARY(pybullet_eglRendererPlugin SHARED
    ${pybullet_eglRendererPlugin_SRC}
)


TARGET_LINK_LIBRARIES(pybullet_eglRendererPlugin
        BulletCollision Bullet3Common LinearMath ${COCOA_LIBRARY}
)

target_compile_definitions(pybullet_eglRendererPlugin PRIVATE PHYSICS_IN_PROCESS_EXAMPLE_BROWSER STB_AGAIN BT_USE_EGL)

#target_compile_options(pybullet_eglRendererPlugin PRIVATE
#        -std=c++11
#)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(pybullet_eglRendererPlugin PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(pybullet_eglRendererPlugin PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(pybullet_eglRendererPlugin PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)


