project(pbs-replan)
list(APPEND CMAKE_PREFIX_PATH "../json")
cmake_minimum_required (VERSION 2.6)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_executable(pbs-replan ${SOURCES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package(nlohmann_json REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(pbs-replan ${Boost_LIBRARIES} nlohmann_json::nlohmann_json)
