add_library(far_math STATIC src/far_math.cpp)
target_include_directories(far_math PUBLIC src)
target_compile_options(far_math PRIVATE -Wall -Wextra)
set_target_properties(far_math PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(far_math_test tests/far_math_test.cpp)
target_link_libraries(far_math_test PRIVATE far_math)
add_test(NAME far_math_test COMMAND far_math_test)
target_compile_options(far_math_test PRIVATE -UNDEBUG) # tests use plain assert

add_library(draw_distance MODULE src/mod.cpp)
target_link_libraries(draw_distance PRIVATE far_math)
target_include_directories(draw_distance PRIVATE "${GT_SDK_DIR}")
target_compile_options(draw_distance PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(draw_distance PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
