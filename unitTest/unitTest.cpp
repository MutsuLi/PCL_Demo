// unitTest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include "../PCLLearningNotesDll/PCLNOTES.h"
using namespace std;
#pragma comment(lib, "PCLLearningNotesDll.lib")
using namespace PCLNOTES;

int main(int argc, char* argv[])
{
	//narf_keypoint_extraction(argc, argv);
	//char* param1[] = { argv[0],(char*)"-sf" };
	//random_sample_consensus(argc+1, param1);
	//normal_estimation();
	pfh_estimation();
}

