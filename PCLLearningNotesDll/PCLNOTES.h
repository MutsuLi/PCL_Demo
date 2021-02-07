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
}