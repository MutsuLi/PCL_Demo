#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include "MyLog.h"

#include <unordered_set>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension
using namespace std;
typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
bool setUnseenToMaxRange = false;

// --------------
// -----Help-----
// --------------

struct sortpoints {
	double x;
	double y;
	double z;
	double distance;
	int index;
};

bool sortCloudPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr boundary) {
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
	kdtree.setInputCloud(boundary);
	kdtree.setSortedResults(true);

	std::unordered_set <int> hashset(boundary->size());
	std::vector <sortpoints> sortedresult;
	int index = 0;
	int count = 1;
	double search_distance = 3;

	sortpoints next = { boundary->points[0].x,boundary->points[0].y,boundary->points[0].z,0,0 };
	sortpoints curr = { next.x,next.y,next.z,0,0 };
	sortpoints pre = { 0,0,0,0,0 };
	sortpoints zeropoint = { curr.x,curr.y,curr.z,0,0 };
	sortedresult.emplace_back(zeropoint);
	//sortedresult.insert(std::pair<int,sortpoints>(0, zeropoint));
	hashset.insert(0);
	sortpoints first = { 0,0,0,0,0 };




	while (count < boundary->size())
	{
		double k = search_distance;
		vector<int> nnh(k);
		vector<float> squaredistance(k);

		int numOfNeighbour = kdtree.radiusSearch(boundary->points[index], k, nnh, squaredistance);
		bool findflag = false;
		int min_index = INT32_MAX;
		int min_distance_index = INT32_MAX;
		int min_scalar_product = INT32_MAX;
		int min_distance = INT32_MAX;
		int mis_count = 0;

		for (int j = 0; j < nnh.size(); j++) {

			if (squaredistance[j] == 0) {
				mis_count++;
				continue;
			}
			if (hashset.find(nnh[j]) != hashset.end()) {
				mis_count++;
				continue;
			}
			auto next_point = boundary->points[nnh[j]];
			next = { next_point.x,next_point.y,next_point.z,squaredistance[j],nnh[j] };
			//next = cloud_filtered->points[nnh[j]];
			int cross_product = (curr.x - pre.x) * (next.y - curr.y) - (curr.y - pre.y) * (next.x - curr.x);

			// p12xp23=(x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)
			if (cross_product <= 0) {
				sortpoints sp = { next.x,next.y,next.z,squaredistance[j],nnh[j] };
				search_distance = squaredistance[j] * 1.5;
				hashset.insert(nnh[j]);
				//sortedresult.insert(std::pair<int, sortpoints>(nnh[j], sp));
				sortedresult.emplace_back(sp);
				if (count > 2)pre = curr;
				curr = next;
				index = nnh[j];
				count++;
				findflag = true;
				k = search_distance;
			}
			mis_count++;

			//int scalar_product = (next.x - curr.x) * (curr.x - pre.x) - (next.y - curr.y) * (curr.y - pre.y);

			//if (scalar_product < min_scalar_product) {
			//	min_scalar_product = scalar_product;
			//	min_index = nnh[j];
			//	min_distance_index = j;
			//	min_distance = squaredistance[j];
			//}
		}
		if (!findflag && squaredistance.size() && mis_count == squaredistance.size()) {
			search_distance = squaredistance[squaredistance.size() - 1];
			k = search_distance * 1.5;
		}

		//if (!findflag && min_index != INT32_MAX) {
		//	auto next_point = boundary->points[min_index];
		//	sortpoints sp = { next.x,next.y,next.z,squaredistance[min_distance_index],min_index };
		//	//sortedresult.insert(std::pair<int, sortpoints>(min_index, sp));
		//	sortedresult.emplace_back(sp);
		//	hashset.insert(min_index);
		//	k = 5;
		//	if (count > 2)pre = curr;
		//	curr = sp;
		//	index = min_index;
		//	count++;
		//}
		//if (!findflag && min_index == INT32_MAX && mis_count == nnh.size()) {
		//	k = k*2;
		//	/*		auto map_end = sortedresult.end();
		//			map_end--;
		//			auto test = map_end->first;
		//			sortedresult.erase(test);
		//			map_end = sortedresult.end();
		//			map_end--;
		//			curr = map_end->second;
		//			map_end--;
		//			pre = map_end->second;
		//			index = curr.index;
		//			count--;*/
		//}
		//cout << "numOfNeighbour: " << numOfNeighbour << endl;
		//cout<<everagedistance<<endl;
	}
	return true;
}


bool saveBoundary(const char* filename) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader reader;
	if (reader.read<pcl::PointXYZ>(filename, *cloud) == -1) {
		//PCL_ERROR("COULD NOT READ FILE\n");
		return -1;
	}
	std::cout << "points sieze is:" << cloud->size() << std::endl;

	pcl::PointCloud<pcl::Boundary> boundaries;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normEst;
	normEst.setNumberOfThreads(10);
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	normEst.setRadiusSearch(3);  //法向估计的半径
	//normEst.setKSearch(3);  //法向估计的点数
	normEst.compute(*normals);
	cout << "normal size is " << normals->size() << endl;

	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	//normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
	est.setInputCloud(cloud);
	est.setInputNormals(normals);
	//est.setAngleThreshold(M_PI / 4);
	//est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	est.setSearchMethod(tree);
	est.setKSearch(50);  //一般这里的数值越高，最终边界识别的精度越好
	//est.setRadiusSearch(everagedistance);  //搜索半径
	est.compute(boundaries);

	cout << "generating point cloud file...." << endl;

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	int countBoundaries = 0;
	for (int i = 0; i < cloud->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			(*boundPoints).push_back(cloud->points[i]);
			countBoundaries++;
		}

	}
	std::cout << "boudary size is：" << boundPoints->size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
	//sor.setInputCloud(boundPoints);                           //设置待滤波的点云
	//sor.setMeanK(30);                               //设置在进行统计时考虑的临近点个数
	//sor.setStddevMulThresh(1.0);                      //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
	//sor.filter(*cloud_filtered);                    //滤波结果存储到cloud_filtered

	pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  //创建滤波器对象
	pcFilter.setInputCloud(boundPoints);             //设置待滤波的点云
	pcFilter.setRadiusSearch(0.8);               // 设置搜索半径
	pcFilter.setMinNeighborsInRadius(2);      // 设置一个内点最少的邻居数目
	pcFilter.filter(*cloud_filtered);        //滤波结果存储到cloud_filtered

	std::cout << "boudary size is：" << cloud_filtered->size() << std::endl;
	pcl::io::savePLYFileASCII("C:\\Users\\admin\\Desktop\\ply\\Boundpoints.ply", *cloud_filtered);

	return true;

}

int load_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

	return (0);
}

int nearestSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// 创建KdTree的实现类KdTreeFLANN (Fast Library for Approximate Nearest Neighbor)
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	// pcl::search::KdTree<pcl::PointXYZ> kdtree;
	// 设置搜索空间，把cloud作为输入
	kdtree.setInputCloud(cloud);

	// 初始化一个随机的点，作为查询点
	pcl::PointXYZ searchPoint;
	searchPoint = cloud->points[0];

	// K nearest neighbor search
	// 方式一：搜索K个最近邻居

	// 创建K和两个向量来保存搜索到的数据
	// K = 10 表示搜索10个临近点
	// pointIdxNKNSearch        保存搜索到的临近点的索引
	// pointNKNSquaredDistance  保存对应临近点的距离的平方
	int K = 10;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (距离平方: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	// Neighbors within radius search
	// 方式二：通过指定半径搜索
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// 创建一个随机[0,256)的半径值
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;


	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (距离平方:: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

	pcl::PointXYZ originPoint(0.0, 0.0, 0.0);
	// 添加从原点到搜索点的线段
	viewer.addLine(originPoint, searchPoint);
	// 添加一个以搜索点为圆心，搜索半径为半径的球体
	viewer.addSphere(searchPoint, radius, "sphere", 0);
	// 添加一个放到200倍后的坐标系
	viewer.addCoordinateSystem(200);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}

	return 0;
}

void pointcloud2depth() {

	pcl::PointCloud<pcl::PointXYZ> pointCloud;

	// Generate the data
	for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
		for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
			pcl::PointXYZ point;
			point.x = 2.0f - y;
			point.y = y;
			point.z = z;
			pointCloud.points.push_back(point);
		}
	}
	pointCloud.width = pointCloud.size();
	pointCloud.height = 1;

	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;

	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	std::cout << rangeImage << "\n";
}

void printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
		<< "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
		<< "-m           Treat all unseen points to max range\n"
		<< "-h           this help\n"
		<< "\n\n";
}

int depthBorderExtract(int argc, char** argv) {
	// --------------------------------------
// -----Parse Command Line Arguments-----
// --------------------------------------
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	//if (pcl::console::find_argument(argc, argv, "-m") >= 0)
	//{
	setUnseenToMaxRange = true;
	std::cout << "Setting unseen values in range image to maximum range readings.\n";
	//}
	int tmp_coordinate_frame;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
	{
		coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
		std::cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
	}
	if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
		std::cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
	angular_resolution = pcl::deg2rad(angular_resolution);

	// ------------------------------------------------------------------
	// -----Read pcd file or create example point cloud if not given-----
	// ------------------------------------------------------------------
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
	//if (!pcd_filename_indices.empty())
	//{
	std::string filename = "C:\\Users\\admin\\Desktop\\ply\\shoeselected.pcd";
	if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
	{
		std::cout << "Was not able to open file \"" << filename << "\".\n";
		printUsage(argv[0]);
		return 0;
	}
	scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
		point_cloud.sensor_origin_[1],
		point_cloud.sensor_origin_[2])) *
		Eigen::Affine3f(point_cloud.sensor_orientation_);

	std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
	if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
		std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
	//}
	//else
	//{3
	//	std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
	//	for (float x = -0.5f; x <= 0.5f; x += 0.01f)
	//	{
	//		for (float y = -0.5f; y <= 0.5f; y += 0.01f)
	//		{
	//			PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
	//			point_cloud.points.push_back(point);
	//		}
	//	}
	//	point_cloud.width = point_cloud.size();  point_cloud.height = 1;
	//}

	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
		scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	range_image.integrateFarRanges(far_ranges);
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange();

	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0f, "global");
	pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr, 0, 0, 0);
	viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	//PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
	//viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	//viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");

	// -------------------------
	// -----Extract borders-----
	// -------------------------
	pcl::RangeImageBorderExtractor border_extractor(&range_image);
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_extractor.compute(border_descriptions);

	// ----------------------------------
	// -----Show points in 3D viewer-----
	// ----------------------------------
	pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
		veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
		shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
	pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
		& veil_points = *veil_points_ptr,
		& shadow_points = *shadow_points_ptr;
	for (int y = 0; y < (int)range_image.height; ++y)
	{
		for (int x = 0; x < (int)range_image.width; ++x)
		{
			if (border_descriptions[y * range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
				border_points.points.push_back(range_image[y * range_image.width + x]);
			if (border_descriptions[y * range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
				veil_points.points.push_back(range_image[y * range_image.width + x]);
			if (border_descriptions[y * range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
				shadow_points.points.push_back(range_image[y * range_image.width + x]);
		}
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler(border_points_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0);
	viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 255, 255);
	viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");

	//-------------------------------------
	// -----Show points on range image-----
	// ------------------------------------
	pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
	range_image_borders_widget =
		pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false,
			border_descriptions, "Range image with borders");
	// -------------------------------------


	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer.wasStopped())
	{
		range_image_borders_widget->spinOnce();
		viewer.spinOnce();
		Sleep(0.01);
	}
}

int main(int argc, char** argv)
{
	MyLog* myLog = MyLog::getInstance();

	LOG4CPLUS_DEBUG(myLog->root_logger, "DeleteService failed,errCode=[" << "123456" << "]");
	LOG4CPLUS_DEBUG(myLog->root_logger, "Service is removed");


	// if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yxg/pcl/pcd/mid.pcd",*cloud) == -1)
	//pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	//const char* filename = "C:\\Users\\admin\\Desktop\\ply\\ObjectModel3D41.ply";
	//saveBoundary(filename);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//const char* boundraryFile = "C:\\Users\\admin\\Desktop\\ply\\Boundpoints.ply";
	//pcl::PLYReader reader;
	//reader.read<pcl::PointXYZ>(boundraryFile, *cloud);
	//nearestSearch(cloud);
	//sortCloudPoints(cloud);
	//pcl::RangeImage::createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	//ObjectModel3D.pcd
	depthBorderExtract(argc, argv);
	return 0;

}