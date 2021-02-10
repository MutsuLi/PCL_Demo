#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "../PCLLearningNotesDll/PCLNOTES.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace PCLNOTES;
typedef pcl::PointXYZ PointT;


void showPointClouds(const pcl::PointCloud<PointT>::Ptr& cloud, const pcl::PointCloud<PointT>::Ptr& cloud2) {// 创建PCLVisualizer
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	// 设置背景色为灰色
	viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);

	// 添加一个普通点云 (可以设置指定颜色，也可以去掉single_color参数不设置)
	pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<PointT>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

	// 添加一个第二个点云 (可以设置指定颜色，也可以去掉single_color2参数不设置)
	pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color2(cloud, 255, 0, 0);
	viewer->addPointCloud<PointT>(cloud2, single_color2, "sample cloud 2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud 2");
	viewer->addCoordinateSystem(1.0);

	while (!viewer->wasStopped()) {
		viewer->spinOnce();
	}
}

extern "C" PCLNOTES_API int StatisticalOutlierRemoval()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// 从文件读取点云
	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ>("C:\\Users\\admin\\Desktop\\ply\\20210111152624.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// 创建过滤器，每个点分析计算时考虑的最近邻居个数为50个；
	// 设置标准差阈值为1，这意味着所有距离查询点的平均距离的标准偏差均大于1个标准偏差的所有点都将被标记为离群值并删除。
	// 计算输出并将其存储在cloud_filtered中。

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	// 设置平均距离估计的最近邻居的数量K
	sor.setMeanK(50);
	// 设置标准差阈值系数
	sor.setStddevMulThresh(1.0);
	// 执行过滤
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	// 将留下来的点保存到后缀为_inliers.pcd的文件
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("C:\\Users\\admin\\Desktop\\ply\\20210111152624_inliers.pcd", *cloud_filtered, false);

	//// 使用个相同的过滤器，但是对输出结果取反，则得到那些被过滤掉的点，保存到_outliers.pcd文件
	//sor.setNegative(true);
	//sor.filter(*cloud_filtered);
	//writer.write<pcl::PointXYZ>("C:\\Users\\admin\\Desktop\\ply\\20210111152624_outliers.pcd", *cloud_filtered, false);

	return (0);
}
extern "C" PCLNOTES_API int downsample_voxel_grid()
{
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// 从文件读取点云图
	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("C:\\Users\\admin\\Desktop\\ply\\shoeselected.pcd", *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	// 创建一个长宽高分别是1cm的体素过滤器，cloud作为输入数据，cloud_filtered作为输出数据
	float leftSize = 5.f;
	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leftSize, leftSize, leftSize);
	sor.filter(*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	// 将结果输出到文件
	pcl::PCDWriter writer;
	writer.write("C:\\Users\\admin\\Desktop\\ply\\shoeselected_downsampled.pcd", *cloud_filtered);

	return (0);
}
extern "C" PCLNOTES_API int remove_outliers(int argc, char* argv[]) {

	if (argc != 2) {
		std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
		exit(0);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("C:\\Users\\admin\\Desktop\\ply\\pc.pcd", *cloud); // Remember to download the file first!

	// Fill in the cloud data
	if (strcmp(argv[1], "-r") == 0) {
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		// build the filter
		outrem.setInputCloud(cloud);
		outrem.setRadiusSearch(1);
		outrem.setMinNeighborsInRadius(2);
		// outrem.setKeepOrganized(true);
		 // apply filter
		outrem.filter(*cloud_filtered);
	}
	else if (strcmp(argv[1], "-c") == 0) {
		// build the condition
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new
			pcl::ConditionAnd<pcl::PointXYZ>());
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
		// build the filter
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setCondition(range_cond);
		condrem.setInputCloud(cloud);
		condrem.setKeepOrganized(true);
		// apply filter
		condrem.filter(*cloud_filtered);
	}
	else {
		std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
		exit(0);
	}
	std::cerr << "Cloud before filtering: " << std::endl;
	// display pointcloud after filtering
	std::cerr << cloud->points.size() << std::endl;
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << cloud_filtered->points.size() << std::endl;

	showPointClouds(cloud, cloud_filtered);

	return (0);
}
extern "C" PCLNOTES_API int PassThrough() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cerr << "Cloud before filtering: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);          // 1. 设置输入源
	pass.setFilterFieldName("z");       // 2. 设置过滤域名
	pass.setFilterLimits(0.0, 1.0);     // 3. 设置过滤范围
//    pass.setFilterLimitsNegative(true); // 设置获取Limits之外的内容
	pass.filter(*cloud_filtered);       // 4. 执行过滤，并将结果输出到cloud_filtered

	std::cerr << "Cloud after filtering: " << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
		std::cerr << "    " << cloud_filtered->points[i].x << " "
		<< cloud_filtered->points[i].y << " "
		<< cloud_filtered->points[i].z << std::endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//这里会一直阻塞直到点云被渲染
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {
	}
	return (0);
}