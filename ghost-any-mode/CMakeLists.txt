add_library(ghost_logic STATIC src/ghost_logic.cpp)
target_include_directories(ghost_logic PUBLIC src)
target_compile_options(ghost_logic PRIVATE -Wall -Wextra)
set_target_properties(ghost_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(ghost_test tests/ghost_test.cpp)
target_link_libraries(ghost_test PRIVATE ghost_logic)
add_test(NAME ghost_test COMMAND ghost_test)
target_compile_options(ghost_test PRIVATE -UNDEBUG) # tests use plain assert

add_library(ghost_any_mode MODULE src/mod.cpp)
target_link_libraries(ghost_any_mode PRIVATE ghost_logic)
target_include_directories(ghost_any_mode PRIVATE "${GT_SDK_DIR}")
target_compile_options(ghost_any_mode PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(ghost_any_mode PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
