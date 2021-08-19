#pragma once
#ifndef TEST_PATH_USE_STD_FILESYSTEM

#if __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define TEST_PATH_USE_STD_FILESYSTEM 1
#endif
#endif
#ifndef TEST_PATH_USE_STD_FILESYSTEM
#define TEST_PATH_USE_STD_FILESYSTEM 0
#endif

#if TEST_PATH_USE_STD_FILESYSTEM
#include <filesystem>
#else
#include <boost/filesystem.hpp>
#endif

namespace lanelet2_examples {
namespace test{

#if TEST_PATH_USE_STD_FILESYSTEM
static const std::filesystem::path projectRootDir{"/home/labdog/ai_ws/src/lanelet2/lanelet2_examples"};
#else
static const boost::filesystem::path projectRootDir{"/home/labdog/ai_ws/src/lanelet2/lanelet2_examples"};
#endif

} // namespace test
} // namespace lanelet2_examples

#endif
