add_library(cam_logic STATIC src/cam_logic.cpp)
target_include_directories(cam_logic PUBLIC src)
target_compile_options(cam_logic PRIVATE -Wall -Wextra)
set_target_properties(cam_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(cam_test tests/cam_test.cpp)
target_link_libraries(cam_test PRIVATE cam_logic)
add_test(NAME cam_test COMMAND cam_test)
target_compile_options(cam_test PRIVATE -UNDEBUG) # tests use plain assert

add_library(extra_cameras MODULE src/mod.cpp)
target_link_libraries(extra_cameras PRIVATE cam_logic)
target_include_directories(extra_cameras PRIVATE "${GT_SDK_DIR}")
target_compile_options(extra_cameras PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(extra_cameras PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
