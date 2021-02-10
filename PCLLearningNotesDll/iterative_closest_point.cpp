#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "../PCLLearningNotesDll/PCLNOTES.h"

using namespace PCLNOTES;

extern "C" PCLNOTES_API int iterative_closest_point() {
    // ����������������
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    // �������������
    cloud_in->width = 5;
    cloud_in->height = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
        << std::endl;
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
        std::cout << "    " <<
        cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
        cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;

    // �ڵ�����ִ�м򵥵ĸ��Ա任����cloud_out�е�xƽ��0.7f�ף�Ȼ���ٴ��������ֵ��
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    // ��ӡ��Щ��
    std::cout << "Transformed " << cloud_in->points.size() << " data points:"
        << std::endl;
    for (size_t i = 0; i < cloud_out->points.size(); ++i)
        std::cout << "    " << cloud_out->points[i].x << " " <<
        cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

    // ����IterativeClosestPoint��ʵ��
    // setInputSource��cloud_in��Ϊ�������
    // setInputTarget��ƽ�ƺ��cloud_out��ΪĿ�����
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    // ����һ�� pcl::PointCloud<pcl::PointXYZ>ʵ�� Final ����,�洢��׼�任���Դ����,
    // Ӧ�� ICP �㷨��, IterativeClosestPoint �ܹ����������Ƽ�,�������������ƥ����ȷ�Ļ�
    // ������������һ��Ӧ��ĳ�ָ���任���Ϳ��Եõ�������ͬһ����ϵ����ͬ�ĵ��ƣ�,��ô icp. hasConverged()= 1 (true),
    // Ȼ���������ձ任�����ƥ������ͱ任�������Ϣ��
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& matrix = icp.getFinalTransformation();
    std::cout << matrix << std::endl;
    return (0);
}