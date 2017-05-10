project(PARTICLE_FILTER)
cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")


# Build the particle filter project and solution.
# Use C++11
set(sources src/main.cpp src/particle_filter.cpp)
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

# Create the executable
add_executable(particle_filter ${sources})

target_link_libraries(particle_filter z ssl uv uWS)

# Use C++11
#if [ ! -f ./src/particle_filter_sol.cpp]; then
#	echo "No solution file."
#else
#	echo "Solution file exists"
#fi

#if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/particle_filter_sol.cpp")
#	set(SRCS src/main.cpp src/particle_filter_sol.cpp)
#	set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

#	# Create the executable
#	add_executable(particle_filter_solution ${SRCS})
#endif()









