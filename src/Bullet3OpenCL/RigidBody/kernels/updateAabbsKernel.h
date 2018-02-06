// this file is autogenerated using stringify.bat (premake --stringify) in the
// build folder of this project
static const char* updateAabbsKernelCL =
    "#ifndef B3_UPDATE_AABBS_H\n"
    "#define B3_UPDATE_AABBS_H\n"
    "#ifndef B3_AABB_H\n"
    "#define B3_AABB_H\n"
    "#ifndef B3_FLOAT4_H\n"
    "#define B3_FLOAT4_H\n"
    "#ifndef B3_PLATFORM_DEFINITIONS_H\n"
    "#define B3_PLATFORM_DEFINITIONS_H\n"
    "struct MyTest\n"
    "{\n"
    "	int bla;\n"
    "};\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "//keep B3_LARGE_FLOAT*B3_LARGE_FLOAT < FLT_MAX\n"
    "#define B3_LARGE_FLOAT 1e18f\n"
    "#define B3_INFINITY 1e18f\n"
    "#define b3Assert(a)\n"
    "#define b3ConstArray(a) __global const a*\n"
    "#define b3AtomicInc atomic_inc\n"
    "#define b3AtomicAdd atomic_add\n"
    "#define b3Fabs fabs\n"
    "#define b3Sqrt native_sqrt\n"
    "#define b3Sin native_sin\n"
    "#define b3Cos native_cos\n"
    "#define B3_STATIC\n"
    "#endif\n"
    "#endif\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "	typedef float4	b3Float4;\n"
    "	#define b3Float4ConstArg const b3Float4\n"
    "	#define b3MakeFloat4 (float4)\n"
    "	float b3Dot3F4(b3Float4ConstArg v0,b3Float4ConstArg v1)\n"
    "	{\n"
    "		float4 a1 = b3MakeFloat4(v0.xyz,0.f);\n"
    "		float4 b1 = b3MakeFloat4(v1.xyz,0.f);\n"
    "		return dot(a1, b1);\n"
    "	}\n"
    "	b3Float4 b3Cross3(b3Float4ConstArg v0,b3Float4ConstArg v1)\n"
    "	{\n"
    "		float4 a1 = b3MakeFloat4(v0.xyz,0.f);\n"
    "		float4 b1 = b3MakeFloat4(v1.xyz,0.f);\n"
    "		return cross(a1, b1);\n"
    "	}\n"
    "	#define b3MinFloat4 min\n"
    "	#define b3MaxFloat4 max\n"
    "	#define b3Normalized(a) normalize(a)\n"
    "#endif \n"
    "		\n"
    "inline bool b3IsAlmostZero(b3Float4ConstArg v)\n"
    "{\n"
    "	if(b3Fabs(v.x)>1e-6 || b3Fabs(v.y)>1e-6 || b3Fabs(v.z)>1e-6)	\n"
    "		return false;\n"
    "	return true;\n"
    "}\n"
    "inline int    b3MaxDot( b3Float4ConstArg vec, __global const b3Float4* "
    "vecArray, int vecLen, float* dotOut )\n"
    "{\n"
    "    float maxDot = -B3_INFINITY;\n"
    "    int i = 0;\n"
    "    int ptIndex = -1;\n"
    "    for( i = 0; i < vecLen; i++ )\n"
    "    {\n"
    "        float dot = b3Dot3F4(vecArray[i],vec);\n"
    "            \n"
    "        if( dot > maxDot )\n"
    "        {\n"
    "            maxDot = dot;\n"
    "            ptIndex = i;\n"
    "        }\n"
    "    }\n"
    "	b3Assert(ptIndex>=0);\n"
    "    if (ptIndex<0)\n"
    "	{\n"
    "		ptIndex = 0;\n"
    "	}\n"
    "    *dotOut = maxDot;\n"
    "    return ptIndex;\n"
    "}\n"
    "#endif //B3_FLOAT4_H\n"
    "#ifndef B3_MAT3x3_H\n"
    "#define B3_MAT3x3_H\n"
    "#ifndef B3_QUAT_H\n"
    "#define B3_QUAT_H\n"
    "#ifndef B3_PLATFORM_DEFINITIONS_H\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "#endif\n"
    "#endif\n"
    "#ifndef B3_FLOAT4_H\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "#endif \n"
    "#endif //B3_FLOAT4_H\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "	typedef float4	b3Quat;\n"
    "	#define b3QuatConstArg const b3Quat\n"
    "	\n"
    "	\n"
    "inline float4 b3FastNormalize4(float4 v)\n"
    "{\n"
    "	v = (float4)(v.xyz,0.f);\n"
    "	return fast_normalize(v);\n"
    "}\n"
    "	\n"
    "inline b3Quat b3QuatMul(b3Quat a, b3Quat b);\n"
    "inline b3Quat b3QuatNormalized(b3QuatConstArg in);\n"
    "inline b3Quat b3QuatRotate(b3QuatConstArg q, b3QuatConstArg vec);\n"
    "inline b3Quat b3QuatInvert(b3QuatConstArg q);\n"
    "inline b3Quat b3QuatInverse(b3QuatConstArg q);\n"
    "inline b3Quat b3QuatMul(b3QuatConstArg a, b3QuatConstArg b)\n"
    "{\n"
    "	b3Quat ans;\n"
    "	ans = b3Cross3( a, b );\n"
    "	ans += a.w*b+b.w*a;\n"
    "//	ans.w = a.w*b.w - (a.x*b.x+a.y*b.y+a.z*b.z);\n"
    "	ans.w = a.w*b.w - b3Dot3F4(a, b);\n"
    "	return ans;\n"
    "}\n"
    "inline b3Quat b3QuatNormalized(b3QuatConstArg in)\n"
    "{\n"
    "	b3Quat q;\n"
    "	q=in;\n"
    "	//return b3FastNormalize4(in);\n"
    "	float len = native_sqrt(dot(q, q));\n"
    "	if(len > 0.f)\n"
    "	{\n"
    "		q *= 1.f / len;\n"
    "	}\n"
    "	else\n"
    "	{\n"
    "		q.x = q.y = q.z = 0.f;\n"
    "		q.w = 1.f;\n"
    "	}\n"
    "	return q;\n"
    "}\n"
    "inline float4 b3QuatRotate(b3QuatConstArg q, b3QuatConstArg vec)\n"
    "{\n"
    "	b3Quat qInv = b3QuatInvert( q );\n"
    "	float4 vcpy = vec;\n"
    "	vcpy.w = 0.f;\n"
    "	float4 out = b3QuatMul(b3QuatMul(q,vcpy),qInv);\n"
    "	return out;\n"
    "}\n"
    "inline b3Quat b3QuatInverse(b3QuatConstArg q)\n"
    "{\n"
    "	return (b3Quat)(-q.xyz, q.w);\n"
    "}\n"
    "inline b3Quat b3QuatInvert(b3QuatConstArg q)\n"
    "{\n"
    "	return (b3Quat)(-q.xyz, q.w);\n"
    "}\n"
    "inline float4 b3QuatInvRotate(b3QuatConstArg q, b3QuatConstArg vec)\n"
    "{\n"
    "	return b3QuatRotate( b3QuatInvert( q ), vec );\n"
    "}\n"
    "inline b3Float4 b3TransformPoint(b3Float4ConstArg point, b3Float4ConstArg "
    "translation, b3QuatConstArg  orientation)\n"
    "{\n"
    "	return b3QuatRotate( orientation, point ) + (translation);\n"
    "}\n"
    "	\n"
    "#endif \n"
    "#endif //B3_QUAT_H\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "typedef struct\n"
    "{\n"
    "	b3Float4 m_row[3];\n"
    "}b3Mat3x3;\n"
    "#define b3Mat3x3ConstArg const b3Mat3x3\n"
    "#define b3GetRow(m,row) (m.m_row[row])\n"
    "inline b3Mat3x3 b3QuatGetRotationMatrix(b3Quat quat)\n"
    "{\n"
    "	b3Float4 quat2 = (b3Float4)(quat.x*quat.x, quat.y*quat.y, "
    "quat.z*quat.z, 0.f);\n"
    "	b3Mat3x3 out;\n"
    "	out.m_row[0].x=1-2*quat2.y-2*quat2.z;\n"
    "	out.m_row[0].y=2*quat.x*quat.y-2*quat.w*quat.z;\n"
    "	out.m_row[0].z=2*quat.x*quat.z+2*quat.w*quat.y;\n"
    "	out.m_row[0].w = 0.f;\n"
    "	out.m_row[1].x=2*quat.x*quat.y+2*quat.w*quat.z;\n"
    "	out.m_row[1].y=1-2*quat2.x-2*quat2.z;\n"
    "	out.m_row[1].z=2*quat.y*quat.z-2*quat.w*quat.x;\n"
    "	out.m_row[1].w = 0.f;\n"
    "	out.m_row[2].x=2*quat.x*quat.z-2*quat.w*quat.y;\n"
    "	out.m_row[2].y=2*quat.y*quat.z+2*quat.w*quat.x;\n"
    "	out.m_row[2].z=1-2*quat2.x-2*quat2.y;\n"
    "	out.m_row[2].w = 0.f;\n"
    "	return out;\n"
    "}\n"
    "inline b3Mat3x3 b3AbsoluteMat3x3(b3Mat3x3ConstArg matIn)\n"
    "{\n"
    "	b3Mat3x3 out;\n"
    "	out.m_row[0] = fabs(matIn.m_row[0]);\n"
    "	out.m_row[1] = fabs(matIn.m_row[1]);\n"
    "	out.m_row[2] = fabs(matIn.m_row[2]);\n"
    "	return out;\n"
    "}\n"
    "__inline\n"
    "b3Mat3x3 mtZero();\n"
    "__inline\n"
    "b3Mat3x3 mtIdentity();\n"
    "__inline\n"
    "b3Mat3x3 mtTranspose(b3Mat3x3 m);\n"
    "__inline\n"
    "b3Mat3x3 mtMul(b3Mat3x3 a, b3Mat3x3 b);\n"
    "__inline\n"
    "b3Float4 mtMul1(b3Mat3x3 a, b3Float4 b);\n"
    "__inline\n"
    "b3Float4 mtMul3(b3Float4 a, b3Mat3x3 b);\n"
    "__inline\n"
    "b3Mat3x3 mtZero()\n"
    "{\n"
    "	b3Mat3x3 m;\n"
    "	m.m_row[0] = (b3Float4)(0.f);\n"
    "	m.m_row[1] = (b3Float4)(0.f);\n"
    "	m.m_row[2] = (b3Float4)(0.f);\n"
    "	return m;\n"
    "}\n"
    "__inline\n"
    "b3Mat3x3 mtIdentity()\n"
    "{\n"
    "	b3Mat3x3 m;\n"
    "	m.m_row[0] = (b3Float4)(1,0,0,0);\n"
    "	m.m_row[1] = (b3Float4)(0,1,0,0);\n"
    "	m.m_row[2] = (b3Float4)(0,0,1,0);\n"
    "	return m;\n"
    "}\n"
    "__inline\n"
    "b3Mat3x3 mtTranspose(b3Mat3x3 m)\n"
    "{\n"
    "	b3Mat3x3 out;\n"
    "	out.m_row[0] = (b3Float4)(m.m_row[0].x, m.m_row[1].x, m.m_row[2].x, "
    "0.f);\n"
    "	out.m_row[1] = (b3Float4)(m.m_row[0].y, m.m_row[1].y, m.m_row[2].y, "
    "0.f);\n"
    "	out.m_row[2] = (b3Float4)(m.m_row[0].z, m.m_row[1].z, m.m_row[2].z, "
    "0.f);\n"
    "	return out;\n"
    "}\n"
    "__inline\n"
    "b3Mat3x3 mtMul(b3Mat3x3 a, b3Mat3x3 b)\n"
    "{\n"
    "	b3Mat3x3 transB;\n"
    "	transB = mtTranspose( b );\n"
    "	b3Mat3x3 ans;\n"
    "	//	why this doesn't run when 0ing in the for{}\n"
    "	a.m_row[0].w = 0.f;\n"
    "	a.m_row[1].w = 0.f;\n"
    "	a.m_row[2].w = 0.f;\n"
    "	for(int i=0; i<3; i++)\n"
    "	{\n"
    "//	a.m_row[i].w = 0.f;\n"
    "		ans.m_row[i].x = b3Dot3F4(a.m_row[i],transB.m_row[0]);\n"
    "		ans.m_row[i].y = b3Dot3F4(a.m_row[i],transB.m_row[1]);\n"
    "		ans.m_row[i].z = b3Dot3F4(a.m_row[i],transB.m_row[2]);\n"
    "		ans.m_row[i].w = 0.f;\n"
    "	}\n"
    "	return ans;\n"
    "}\n"
    "__inline\n"
    "b3Float4 mtMul1(b3Mat3x3 a, b3Float4 b)\n"
    "{\n"
    "	b3Float4 ans;\n"
    "	ans.x = b3Dot3F4( a.m_row[0], b );\n"
    "	ans.y = b3Dot3F4( a.m_row[1], b );\n"
    "	ans.z = b3Dot3F4( a.m_row[2], b );\n"
    "	ans.w = 0.f;\n"
    "	return ans;\n"
    "}\n"
    "__inline\n"
    "b3Float4 mtMul3(b3Float4 a, b3Mat3x3 b)\n"
    "{\n"
    "	b3Float4 colx = b3MakeFloat4(b.m_row[0].x, b.m_row[1].x, b.m_row[2].x, "
    "0);\n"
    "	b3Float4 coly = b3MakeFloat4(b.m_row[0].y, b.m_row[1].y, b.m_row[2].y, "
    "0);\n"
    "	b3Float4 colz = b3MakeFloat4(b.m_row[0].z, b.m_row[1].z, b.m_row[2].z, "
    "0);\n"
    "	b3Float4 ans;\n"
    "	ans.x = b3Dot3F4( a, colx );\n"
    "	ans.y = b3Dot3F4( a, coly );\n"
    "	ans.z = b3Dot3F4( a, colz );\n"
    "	return ans;\n"
    "}\n"
    "#endif\n"
    "#endif //B3_MAT3x3_H\n"
    "typedef struct b3Aabb b3Aabb_t;\n"
    "struct b3Aabb\n"
    "{\n"
    "	union\n"
    "	{\n"
    "		float m_min[4];\n"
    "		b3Float4 m_minVec;\n"
    "		int m_minIndices[4];\n"
    "	};\n"
    "	union\n"
    "	{\n"
    "		float	m_max[4];\n"
    "		b3Float4 m_maxVec;\n"
    "		int m_signedMaxIndices[4];\n"
    "	};\n"
    "};\n"
    "inline void b3TransformAabb2(b3Float4ConstArg "
    "localAabbMin,b3Float4ConstArg localAabbMax, float margin,\n"
    "						b3Float4ConstArg pos,\n"
    "						b3QuatConstArg orn,\n"
    "						b3Float4* aabbMinOut,b3Float4* "
    "aabbMaxOut)\n"
    "{\n"
    "		b3Float4 localHalfExtents = 0.5f*(localAabbMax-localAabbMin);\n"
    "		localHalfExtents+=b3MakeFloat4(margin,margin,margin,0.f);\n"
    "		b3Float4 localCenter = 0.5f*(localAabbMax+localAabbMin);\n"
    "		b3Mat3x3 m;\n"
    "		m = b3QuatGetRotationMatrix(orn);\n"
    "		b3Mat3x3 abs_b = b3AbsoluteMat3x3(m);\n"
    "		b3Float4 center = b3TransformPoint(localCenter,pos,orn);\n"
    "		\n"
    "		b3Float4 extent = "
    "b3MakeFloat4(b3Dot3F4(localHalfExtents,b3GetRow(abs_b,0)),\n"
    "									"
    "	 b3Dot3F4(localHalfExtents,b3GetRow(abs_b,1)),\n"
    "									"
    "	 b3Dot3F4(localHalfExtents,b3GetRow(abs_b,2)),\n"
    "									"
    "	 0.f);\n"
    "		*aabbMinOut = center-extent;\n"
    "		*aabbMaxOut = center+extent;\n"
    "}\n"
    "/// conservative test for overlap between two aabbs\n"
    "inline bool b3TestAabbAgainstAabb(b3Float4ConstArg "
    "aabbMin1,b3Float4ConstArg aabbMax1,\n"
    "								"
    "b3Float4ConstArg aabbMin2, b3Float4ConstArg aabbMax2)\n"
    "{\n"
    "	bool overlap = true;\n"
    "	overlap = (aabbMin1.x > aabbMax2.x || aabbMax1.x < aabbMin2.x) ? false "
    ": overlap;\n"
    "	overlap = (aabbMin1.z > aabbMax2.z || aabbMax1.z < aabbMin2.z) ? false "
    ": overlap;\n"
    "	overlap = (aabbMin1.y > aabbMax2.y || aabbMax1.y < aabbMin2.y) ? false "
    ": overlap;\n"
    "	return overlap;\n"
    "}\n"
    "#endif //B3_AABB_H\n"
    "#ifndef B3_COLLIDABLE_H\n"
    "#define B3_COLLIDABLE_H\n"
    "#ifndef B3_FLOAT4_H\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "#endif \n"
    "#endif //B3_FLOAT4_H\n"
    "#ifndef B3_QUAT_H\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "#endif \n"
    "#endif //B3_QUAT_H\n"
    "enum b3ShapeTypes\n"
    "{\n"
    "	SHAPE_HEIGHT_FIELD=1,\n"
    "	SHAPE_CONVEX_HULL=3,\n"
    "	SHAPE_PLANE=4,\n"
    "	SHAPE_CONCAVE_TRIMESH=5,\n"
    "	SHAPE_COMPOUND_OF_CONVEX_HULLS=6,\n"
    "	SHAPE_SPHERE=7,\n"
    "	MAX_NUM_SHAPE_TYPES,\n"
    "};\n"
    "typedef struct b3Collidable b3Collidable_t;\n"
    "struct b3Collidable\n"
    "{\n"
    "	union {\n"
    "		int m_numChildShapes;\n"
    "		int m_bvhIndex;\n"
    "	};\n"
    "	union\n"
    "	{\n"
    "		float m_radius;\n"
    "		int	m_compoundBvhIndex;\n"
    "	};\n"
    "	int m_shapeType;\n"
    "	union\n"
    "	{\n"
    "		int m_shapeIndex;\n"
    "		float m_height;\n"
    "	};\n"
    "};\n"
    "typedef struct b3GpuChildShape b3GpuChildShape_t;\n"
    "struct b3GpuChildShape\n"
    "{\n"
    "	b3Float4	m_childPosition;\n"
    "	b3Quat		m_childOrientation;\n"
    "	union\n"
    "	{\n"
    "		int			m_shapeIndex;//used for "
    "SHAPE_COMPOUND_OF_CONVEX_HULLS\n"
    "		int			m_capsuleAxis;\n"
    "	};\n"
    "	union \n"
    "	{\n"
    "		float		m_radius;//used for childshape of "
    "SHAPE_COMPOUND_OF_SPHERES or SHAPE_COMPOUND_OF_CAPSULES\n"
    "		int			m_numChildShapes;//used for compound "
    "shape\n"
    "	};\n"
    "	union \n"
    "	{\n"
    "		float		m_height;//used for childshape of "
    "SHAPE_COMPOUND_OF_CAPSULES\n"
    "		int	m_collidableShapeIndex;\n"
    "	};\n"
    "	int			m_shapeType;\n"
    "};\n"
    "struct b3CompoundOverlappingPair\n"
    "{\n"
    "	int m_bodyIndexA;\n"
    "	int m_bodyIndexB;\n"
    "//	int	m_pairType;\n"
    "	int m_childShapeIndexA;\n"
    "	int m_childShapeIndexB;\n"
    "};\n"
    "#endif //B3_COLLIDABLE_H\n"
    "#ifndef B3_RIGIDBODY_DATA_H\n"
    "#define B3_RIGIDBODY_DATA_H\n"
    "#ifndef B3_FLOAT4_H\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "#endif \n"
    "#endif //B3_FLOAT4_H\n"
    "#ifndef B3_QUAT_H\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "#endif \n"
    "#endif //B3_QUAT_H\n"
    "#ifndef B3_MAT3x3_H\n"
    "#ifdef __cplusplus\n"
    "#else\n"
    "#endif\n"
    "#endif //B3_MAT3x3_H\n"
    "typedef struct b3RigidBodyData b3RigidBodyData_t;\n"
    "struct b3RigidBodyData\n"
    "{\n"
    "	b3Float4				m_pos;\n"
    "	b3Quat					m_quat;\n"
    "	b3Float4				m_linVel;\n"
    "	b3Float4				m_angVel;\n"
    "	int 					m_collidableIdx;\n"
    "	float 				m_invMass;\n"
    "	float 				m_restituitionCoeff;\n"
    "	float 				m_frictionCoeff;\n"
    "};\n"
    "typedef struct b3InertiaData b3InertiaData_t;\n"
    "struct b3InertiaData\n"
    "{\n"
    "	b3Mat3x3 m_invInertiaWorld;\n"
    "	b3Mat3x3 m_initInvInertia;\n"
    "};\n"
    "#endif //B3_RIGIDBODY_DATA_H\n"
    "	\n"
    "void b3ComputeWorldAabb(  int bodyId, __global const b3RigidBodyData_t* "
    "bodies, __global const  b3Collidable_t* collidables, __global const  "
    "b3Aabb_t* localShapeAABB, __global b3Aabb_t* worldAabbs)\n"
    "{\n"
    "	__global const b3RigidBodyData_t* body = &bodies[bodyId];\n"
    "	b3Float4 position = body->m_pos;\n"
    "	b3Quat	orientation = body->m_quat;\n"
    "	\n"
    "	int collidableIndex = body->m_collidableIdx;\n"
    "	int shapeIndex = collidables[collidableIndex].m_shapeIndex;\n"
    "		\n"
    "	if (shapeIndex>=0)\n"
    "	{\n"
    "				\n"
    "		b3Aabb_t localAabb = localShapeAABB[collidableIndex];\n"
    "		b3Aabb_t worldAabb;\n"
    "		\n"
    "		b3Float4 aabbAMinOut,aabbAMaxOut;	\n"
    "		float margin = 0.f;\n"
    "		"
    "b3TransformAabb2(localAabb.m_minVec,localAabb.m_maxVec,margin,position,"
    "orientation,&aabbAMinOut,&aabbAMaxOut);\n"
    "		\n"
    "		worldAabb.m_minVec =aabbAMinOut;\n"
    "		worldAabb.m_minIndices[3] = bodyId;\n"
    "		worldAabb.m_maxVec = aabbAMaxOut;\n"
    "		worldAabb.m_signedMaxIndices[3] = body[bodyId].m_invMass==0.f? "
    "0 : 1;\n"
    "		worldAabbs[bodyId] = worldAabb;\n"
    "	}\n"
    "}\n"
    "#endif //B3_UPDATE_AABBS_H\n"
    "__kernel void initializeGpuAabbsFull(  const int numNodes, __global "
    "b3RigidBodyData_t* gBodies,__global b3Collidable_t* collidables, __global "
    "b3Aabb_t* plocalShapeAABB, __global b3Aabb_t* pAABB)\n"
    "{\n"
    "	int nodeID = get_global_id(0);\n"
    "	if( nodeID < numNodes )\n"
    "	{\n"
    "		b3ComputeWorldAabb(nodeID, gBodies, collidables, "
    "plocalShapeAABB,pAABB);\n"
    "	}\n"
    "}\n"
    "__kernel void clearOverlappingPairsKernel(  __global int4* pairs, int "
    "numPairs)\n"
    "{\n"
    "	int pairId = get_global_id(0);\n"
    "	if( pairId< numPairs )\n"
    "	{\n"
    "		pairs[pairId].z = 0xffffffff;\n"
    "	}\n"
    "}\n";
