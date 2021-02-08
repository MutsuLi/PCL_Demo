#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include "../PCLLearningNotesDll/PCLNOTES.h"

using namespace PCLNOTES;
using namespace std;
/**
 * ����������
 */
extern "C" PCLNOTES_API int normal_estimation() {
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("C:\\Users\\admin\\Desktop\\ply\\shoeselected.pcd", *cloud);

    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Object for normal estimation.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    //normalEstimation.setIndices()
    normalEstimation.setInputCloud(cloud);
    // For every point, use all neighbors in a radius of 3cm.
    normalEstimation.setRadiusSearch(3);
    // A kd-tree is a data structure that makes searches efficient. More about it later.
    // The normal estimation object will use it to find nearest neighbors.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    // Calculate the normals.
    normalEstimation.compute(*normals);


    cout <<"normals count" <<normals->size() <<";"<< cloud->size() << endl; //should have the same size as the input cloud->size ()*
    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    // ����int level=2 ��ʾÿn�������һ��������
    // ����float scale=0.01 ��ʾ��������������Ϊ0.01��
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 2, 0.5, "normals");
    //    viewer.addCoordinateSystem(1.0);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return 0;
}