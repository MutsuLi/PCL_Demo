#pragma once

#ifdef PCLNOTES_EXPORTS
#define PCLNOTES_CLASS __declspec(dllexport)
#define PCLNOTES_API __declspec(dllexport)
#else
#define PCLNOTES_API __declspec(dllimport)
#define PCLNOTES_CLASS __declspec(dllimport) 
#endif
namespace PCLNOTES {

	extern "C" PCLNOTES_API int narf_keypoint_extraction(int argc, char* argv[]);
	extern "C" PCLNOTES_API int random_sample_consensus(int argc, char* argv[]);
	extern "C" PCLNOTES_API int range_image_border(int argc, char** argv);
	extern "C" PCLNOTES_API int remove_outliers(int argc, char* argv[]);
	extern "C" PCLNOTES_API int iterative_closest_point();
	extern "C" PCLNOTES_API int normal_distributions_transform(int argc, char* argv[]);
	extern "C" PCLNOTES_API int alignment_prerejective(int argc, char* argv[]);
	extern "C" PCLNOTES_API int interactive_icp(int argc, char* argv[]);
	extern "C" PCLNOTES_API int moment_of_inertia(int argc, char* argv[]);
	extern "C" PCLNOTES_API int template_alignment(int argc, char** argv);
	extern "C" PCLNOTES_API int normal_estimation();
	extern "C" PCLNOTES_API int downsample_voxel_grid();
	extern "C" PCLNOTES_API int pfh_estimation();
	extern "C" PCLNOTES_API int PassThrough();
	extern "C" PCLNOTES_API int StatisticalOutlierRemoval();
	extern "C" PCLNOTES_API int planar_segmentation();
	extern "C" PCLNOTES_API int cylinder_segmentation();
	extern "C" PCLNOTES_API int cluster_extraction();
	class BaseClass {
		public:
		float angular_resolution = 0.5f;
		bool setUnseenToMaxRange = false;
		virtual void printUsage(const char* progName) {};
	};
	
}