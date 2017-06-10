project(MPC_TEST)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources TestPath.cpp 		../src/Path.cpp 
			TestPolynomial.cpp 	../src/Polynomial.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc_test ${sources})

target_link_libraries(mpc_test gtest_main gtest)