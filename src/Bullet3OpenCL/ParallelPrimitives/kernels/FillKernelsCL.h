// this file is autogenerated using stringify.bat (premake --stringify) in the
// build folder of this project
static const char* fillKernelsCL =
    "/*\n"
    "Copyright (c) 2012 Advanced Micro Devices, Inc.  \n"
    "This software is provided 'as-is', without any express or implied "
    "warranty.\n"
    "In no event will the authors be held liable for any damages arising from "
    "the use of this software.\n"
    "Permission is granted to anyone to use this software for any purpose, \n"
    "including commercial applications, and to alter it and redistribute it "
    "freely, \n"
    "subject to the following restrictions:\n"
    "1. The origin of this software must not be misrepresented; you must not "
    "claim that you wrote the original software. If you use this software in a "
    "product, an acknowledgment in the product documentation would be "
    "appreciated but is not required.\n"
    "2. Altered source versions must be plainly marked as such, and must not "
    "be misrepresented as being the original software.\n"
    "3. This notice may not be removed or altered from any source "
    "distribution.\n"
    "*/\n"
    "//Originally written by Takahiro Harada\n"
    "#pragma OPENCL EXTENSION cl_amd_printf : enable\n"
    "#pragma OPENCL EXTENSION cl_khr_local_int32_base_atomics : enable\n"
    "typedef unsigned int u32;\n"
    "#define GET_GROUP_IDX get_group_id(0)\n"
    "#define GET_LOCAL_IDX get_local_id(0)\n"
    "#define GET_GLOBAL_IDX get_global_id(0)\n"
    "#define GET_GROUP_SIZE get_local_size(0)\n"
    "#define GROUP_LDS_BARRIER barrier(CLK_LOCAL_MEM_FENCE)\n"
    "#define GROUP_MEM_FENCE mem_fence(CLK_LOCAL_MEM_FENCE)\n"
    "#define AtomInc(x) atom_inc(&(x))\n"
    "#define AtomInc1(x, out) out = atom_inc(&(x))\n"
    "#define make_uint4 (uint4)\n"
    "#define make_uint2 (uint2)\n"
    "#define make_int2 (int2)\n"
    "typedef struct\n"
    "{\n"
    "	union\n"
    "	{\n"
    "		int4 m_data;\n"
    "		uint4 m_unsignedData;\n"
    "		float	m_floatData;\n"
    "	};\n"
    "	int m_offset;\n"
    "	int m_n;\n"
    "	int m_padding[2];\n"
    "} ConstBuffer;\n"
    "__kernel\n"
    "__attribute__((reqd_work_group_size(64,1,1)))\n"
    "void FillIntKernel(__global int* dstInt, 			int "
    "num_elements, int value, const int offset)\n"
    "{\n"
    "	int gIdx = GET_GLOBAL_IDX;\n"
    "	if( gIdx < num_elements )\n"
    "	{\n"
    "		dstInt[ offset+gIdx ] = value;\n"
    "	}\n"
    "}\n"
    "__kernel\n"
    "__attribute__((reqd_work_group_size(64,1,1)))\n"
    "void FillFloatKernel(__global float* dstFloat, 	int num_elements, "
    "float value, const int offset)\n"
    "{\n"
    "	int gIdx = GET_GLOBAL_IDX;\n"
    "	if( gIdx < num_elements )\n"
    "	{\n"
    "		dstFloat[ offset+gIdx ] = value;\n"
    "	}\n"
    "}\n"
    "__kernel\n"
    "__attribute__((reqd_work_group_size(64,1,1)))\n"
    "void FillUnsignedIntKernel(__global unsigned int* dstInt, const int num, "
    "const unsigned int value, const int offset)\n"
    "{\n"
    "	int gIdx = GET_GLOBAL_IDX;\n"
    "	if( gIdx < num )\n"
    "	{\n"
    "		dstInt[ offset+gIdx ] = value;\n"
    "	}\n"
    "}\n"
    "__kernel\n"
    "__attribute__((reqd_work_group_size(64,1,1)))\n"
    "void FillInt2Kernel(__global int2* dstInt2, 	const int num, const "
    "int2 value, const int offset)\n"
    "{\n"
    "	int gIdx = GET_GLOBAL_IDX;\n"
    "	if( gIdx < num )\n"
    "	{\n"
    "		dstInt2[ gIdx + offset] = make_int2( value.x, value.y );\n"
    "	}\n"
    "}\n"
    "__kernel\n"
    "__attribute__((reqd_work_group_size(64,1,1)))\n"
    "void FillInt4Kernel(__global int4* dstInt4, 		const int num, "
    "const int4 value, const int offset)\n"
    "{\n"
    "	int gIdx = GET_GLOBAL_IDX;\n"
    "	if( gIdx < num )\n"
    "	{\n"
    "		dstInt4[ offset+gIdx ] = value;\n"
    "	}\n"
    "}\n";
