//
// Created by ty on 20-5-28.
//

#include <pcl/point_types.h>
#include <pcl/features/pfh.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include "../PCLLearningNotesDll/PCLNOTES.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
using namespace pcl;

using namespace PCLNOTES;
extern "C" PCLNOTES_API int pfh_estimation() {
	// load point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//    pcl::io::loadPCDFile("./data/target.pcd", *cloud);
	pcl::io::loadPCDFile("C:\\Users\\admin\\Desktop\\ply\\shoeselected.pcd", *cloud);

	// estimate normals ------------------------------------------------------------- 计算法向量
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for normal estimation.
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setNumberOfThreads(10);
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	//normalEstimation.setIndices()
	normalEstimation.setInputCloud(cloud);
	// For every point, use all neighbors in a radius of 3cm.
	normalEstimation.setRadiusSearch(1);

	normalEstimation.compute(*normals);
	// A kd-tree is a data structure that makes searches efficient. More about it later.
	// The normal estimation object will use it to find nearest neighbors.

	// Create the FPFH estimation class, and pass the input dataset+normals to it
	FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setNumberOfThreads(10);
	fpfh.setInputCloud(cloud);
	fpfh.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);

	fpfh.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch(2);

	// Compute the features
	fpfh.compute(*fpfhs);

	unsigned long size = fpfhs->points.size();
	for (int j = 0; j < size; ++j) {
		pcl::FPFHSignature33& signature33 = fpfhs->points[j];
		float* h = signature33.histogram;

		printf("%d: %f,%f,%f \n", j, h[1], h[2], h[3]);
	}

	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.5, "normals");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	return 0;
}