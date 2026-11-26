add_library(weather_logic STATIC src/weather_logic.cpp)
target_include_directories(weather_logic PUBLIC src)
target_compile_options(weather_logic PRIVATE -Wall -Wextra)
set_target_properties(weather_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(weather_test tests/weather_test.cpp)
target_link_libraries(weather_test PRIVATE weather_logic)
add_test(NAME weather_test COMMAND weather_test)
target_compile_options(weather_test PRIVATE -UNDEBUG) # tests use plain assert

add_library(weather MODULE src/mod.cpp)
target_link_libraries(weather PRIVATE weather_logic)
target_include_directories(weather PRIVATE "${GT_SDK_DIR}")
target_compile_options(weather PRIVATE -Wall -Wextra -fvisibility=hidden)
set_target_properties(weather PROPERTIES PREFIX "lib" POSITION_INDEPENDENT_CODE ON)
