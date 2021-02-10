// unitTest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include "../PCLLearningNotesDll/PCLNOTES.h"
using namespace std;
#pragma comment(lib, "PCLLearningNotesDll.lib")
using namespace PCLNOTES;

int main(int argc, char* argv[])
{
	//char* param1[] = { argv[0],(char*)"-sf" };

	//3. 点云滤波
	//char* param1[] = { argv[0],(char*)"-r" };
	//remove_outliers(argc + 1, param1);
	//PassThrough();
	//downsample_voxel_grid();
	//StatisticalOutlierRemoval();

	//4. 深度图
	//char* param2[] = { argv[0],(char*)"-m",(char*)"C:\\Users\\admin\\Desktop\\ply\\shoeselected.pcd" };
	//range_image_border(argc + 2, param2);

	//5. 关键点
	//narf_keypoint_extraction(argc, argv);

	//6. 随机采样一致性算法
	//char* param1[] = { argv[0],(char*)"-sf" };
	//random_sample_consensus(argc + 1, param1);

	//7. 法向量估计
	//normal_estimation();

	//8.特征检测与描述子
	//pfh_estimation();

	//9.迭代最近点算法(ICP)
	//iterative_closest_point();

	char* param1[] = { argv[0],(char*)"-sf" };
	normal_distributions_transform(argc + 1, param1);


}

