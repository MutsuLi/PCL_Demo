#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "../PCLLearningNotesDll/PCLNOTES.h"

using namespace PCLNOTES;

extern "C" PCLNOTES_API int planar_segmentation() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    /***
     * ����15��������ƣ�x,yΪ�������zΪ1.0
     * ��points��0��3��6����λ�õ�zֵ�����޸ģ���֮��Ϊ��Ⱥֵ
     */
     // Fill in the cloud data
    cloud->width = 15;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // Generate the data
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1.0;
    }

    // Set a few outliers
    cloud->points[0].z = 2.0;
    cloud->points[3].z = -2.0;
    cloud->points[6].z = 4.0;

    std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;
    for (std::size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
        << cloud->points[i].y << " "
        << cloud->points[i].z << std::endl;

    /**
     * �����ָ�ʱ����Ҫ��ģ��ϵ������ coefficients ���洢�ڵ�ĵ��������϶��� inliers .
     * ��Ҳ������ָ������ֵ����DistanceThreshold���ĵط����þ�����ֵȷ���������ģ���ж�Զ���ܱ���Ϊ��Ⱥ�㡣
     * ���������ֵ�� 0.01m ,��ֻҪ�㵽 z=1 ƽ�����С�ڸ���ֵ�ĵ㶼��Ϊ�ڲ��㿴��,�����ڸø�ֵ��������Ⱥ�㡣
     * ���ǽ�ʹ��RANSAC������`pcl::SAC_RANSAC`����Ϊ�ɿ��Ĺ���������ΪRANSAC�Ƚϼ򵥣�����ǿ��Ĺ��㹤��Ҳ�Դ�Ϊ����������������������ӵĸ����
     */
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // ��ѡ���ã��Ƿ��Ż�ģ��ϵ��
    seg.setOptimizeCoefficients(true);
    // ��ѡ���ã����÷ָ��ģ�����͡��ָ��㷨��������ֵ���������
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    // ִ�зָ���������洢�ָ������浽�㼯�� inliers ���洢ƽ��ģ��ϵ�� coefficients
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
    //    �˶δ���������ӡ�������ƽ��ģ�͵Ĳ������� ax+by+ca+d=0 ��ʽ��,���RANSAC����һ�����㷨��SACMODEL_PLANEƽ��ģ��
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        << coefficients->values[1] << " "
        << coefficients->values[2] << " "
        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
    for (std::size_t i = 0; i < inliers->indices.size(); ++i)
        std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
        << cloud->points[inliers->indices[i]].y << " "
        << cloud->points[inliers->indices[i]].z << std::endl;

    return (0);
}