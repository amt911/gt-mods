add_library(deformation_logic STATIC src/deform_logic.cpp)
target_include_directories(deformation_logic PUBLIC src)
target_compile_options(deformation_logic PRIVATE -Wall -Wextra)
set_target_properties(deformation_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(deformation_test tests/deform_test.cpp)
target_link_libraries(deformation_test PRIVATE deformation_logic)
add_test(NAME deformation_test COMMAND deformation_test)
target_compile_options(deformation_test PRIVATE -UNDEBUG) # tests use plain assert

add_library(deformation MODULE src/mod.cpp)
target_link_libraries(deformation PRIVATE deformation_logic)
target_include_directories(deformation PRIVATE "${GT_SDK_DIR}")
target_compile_options(deformation PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(deformation PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
