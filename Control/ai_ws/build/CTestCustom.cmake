# This file configures ctest and has been generated by mrt_cmake_modules.
# Don't change this file, it will be overwritten by the next cmake run.
set(CTEST_CUSTOM_MAXIMUM_PASSED_TEST_OUTPUT_SIZE 0)
set(CTEST_CUSTOM_MAXIMUM_FAILED_TEST_OUTPUT_SIZE 0)
#set(CTEST_CUSTOM_PRE_TEST "/usr/bin/python2 /opt/ros/melodic/share/mrt_cmake_modules/cmake/../scripts/init_coverage.py lanelet2_core /home/labdog/ai_ws/build /home/labdog/ai_ws/src/lanelet2/lanelet2_core /home/labdog/ai_ws/build/test_results/lanelet2_core")
set(CTEST_CUSTOM_POST_TEST "/usr/bin/python2 /opt/ros/melodic/share/mrt_cmake_modules/cmake/../scripts/eval_coverage.py /home/labdog/ai_ws/src /home/labdog/ai_ws/build /home/labdog/ai_ws/build/test_results")
