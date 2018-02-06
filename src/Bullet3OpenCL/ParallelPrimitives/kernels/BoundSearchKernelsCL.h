// this file is autogenerated using stringify.bat (premake --stringify) in the
// build folder of this project
static const char* boundSearchKernelsCL =
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
    "typedef unsigned int u32;\n"
    "#define GET_GROUP_IDX get_group_id(0)\n"
    "#define GET_LOCAL_IDX get_local_id(0)\n"
    "#define GET_GLOBAL_IDX get_global_id(0)\n"
    "#define GET_GROUP_SIZE get_local_size(0)\n"
    "#define GROUP_LDS_BARRIER barrier(CLK_LOCAL_MEM_FENCE)\n"
    "typedef struct\n"
    "{\n"
    "	u32 m_key; \n"
    "	u32 m_value;\n"
    "}SortData;\n"
    "typedef struct\n"
    "{\n"
    "	u32 m_nSrc;\n"
    "	u32 m_nDst;\n"
    "	u32 m_padding[2];\n"
    "} ConstBuffer;\n"
    "__attribute__((reqd_work_group_size(64,1,1)))\n"
    "__kernel\n"
    "void SearchSortDataLowerKernel(__global SortData* src, __global u32 *dst, "
    "\n"
    "					unsigned int nSrc, unsigned int nDst)\n"
    "{\n"
    "	int gIdx = GET_GLOBAL_IDX;\n"
    "	if( gIdx < nSrc )\n"
    "	{\n"
    "		SortData first; first.m_key = (u32)(-1); first.m_value = "
    "(u32)(-1);\n"
    "		SortData end; end.m_key = nDst; end.m_value = nDst;\n"
    "		SortData iData = (gIdx==0)? first: src[gIdx-1];\n"
    "		SortData jData = (gIdx==nSrc)? end: src[gIdx];\n"
    "		if( iData.m_key != jData.m_key )\n"
    "		{\n"
    "//			for(u32 k=iData.m_key+1; k<=min(jData.m_key, nDst-1); "
    "k++)\n"
    "			u32 k = jData.m_key;\n"
    "			{\n"
    "				dst[k] = gIdx;\n"
    "			}\n"
    "		}\n"
    "	}\n"
    "}\n"
    "__attribute__((reqd_work_group_size(64,1,1)))\n"
    "__kernel\n"
    "void SearchSortDataUpperKernel(__global SortData* src, __global u32 *dst, "
    "\n"
    "					unsigned int nSrc, unsigned int nDst)\n"
    "{\n"
    "	int gIdx = GET_GLOBAL_IDX+1;\n"
    "	if( gIdx < nSrc+1 )\n"
    "	{\n"
    "		SortData first; first.m_key = 0; first.m_value = 0;\n"
    "		SortData end; end.m_key = nDst; end.m_value = nDst;\n"
    "		SortData iData = src[gIdx-1];\n"
    "		SortData jData = (gIdx==nSrc)? end: src[gIdx];\n"
    "		if( iData.m_key != jData.m_key )\n"
    "		{\n"
    "			u32 k = iData.m_key;\n"
    "			{\n"
    "				dst[k] = gIdx;\n"
    "			}\n"
    "		}\n"
    "	}\n"
    "}\n"
    "__attribute__((reqd_work_group_size(64,1,1)))\n"
    "__kernel\n"
    "void SubtractKernel(__global u32* A, __global u32 *B, __global u32 *C, \n"
    "					unsigned int nSrc, unsigned int nDst)\n"
    "{\n"
    "	int gIdx = GET_GLOBAL_IDX;\n"
    "	\n"
    "	if( gIdx < nDst )\n"
    "	{\n"
    "		C[gIdx] = A[gIdx] - B[gIdx];\n"
    "	}\n"
    "}\n";
