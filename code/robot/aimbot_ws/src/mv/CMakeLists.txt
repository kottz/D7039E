cmake_minimum_required (VERSION 3.13)

project ("line_tracer")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_executable(${PROJECT_NAME})
FIND_PACKAGE(OpenCV REQUIRED)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS} zbar pthread dl)




# Include sub-projects.
add_subdirectory ("src")
add_subdirectory("external")
