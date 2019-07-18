file(GLOB detectnetCameraSources *.cpp)
file(GLOB detectnetCameraIncludes *.h )

find_package(PythonLibs REQUIRED)

set(CMAKE_BUILD_TYPE Debug)


include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(usr/include)
include_directories(usr/local/include)
include_directories(./)


link_directories(${CMAKE_SOURCE_DIR}/paho.mqtt.cpp/build/src)
link_directories("/usr/include/python3.6m")

cuda_add_executable(detectnet-camera ${detectnetCameraSources})



target_link_libraries(detectnet-camera PUBLIC jetson-inference "/usr/local/lib/libmosquitto.so.1" ${PYTHON_LIBRARIES})
