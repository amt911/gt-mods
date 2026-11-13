add_library(reset_logic STATIC src/reset_logic.cpp)
target_include_directories(reset_logic PUBLIC src)
target_compile_options(reset_logic PRIVATE -Wall -Wextra)
set_target_properties(reset_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(reset_test tests/reset_test.cpp)
target_link_libraries(reset_test PRIVATE reset_logic)
add_test(NAME reset_test COMMAND reset_test)
target_compile_options(reset_test PRIVATE -UNDEBUG) # tests use plain assert

add_library(reset_to_track MODULE src/mod.cpp)
target_link_libraries(reset_to_track PRIVATE reset_logic)
target_include_directories(reset_to_track PRIVATE "${GT_SDK_DIR}")
target_compile_options(reset_to_track PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(reset_to_track PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
