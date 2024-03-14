cmake_minimum_required(VERSION 3.10)

project(FirstProject2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(OpenCV_DIR C:/opencv/build)

find_package(OpenCV REQUIRED)

# Added executable for imgDisplay.cp

add_executable(detectCorners src/detectCorners.cpp)
target_link_libraries(detectCorners ${OpenCV_LIBS})

add_executable(selectCalibrationImages src/selectCalibrationImages.cpp src/functions.cpp)
target_link_libraries(selectCalibrationImages ${OpenCV_LIBS})

add_executable(calibrateCamera src/calibrateCamera.cpp src/functions.cpp)
target_link_libraries(calibrateCamera ${OpenCV_LIBS})

add_executable(calculateCameraPosition src/calculateCameraPosition.cpp src/functions.cpp)
target_link_libraries(calculateCameraPosition ${OpenCV_LIBS})

add_executable(projectAxes src/projectAxes.cpp src/functions.cpp)
target_link_libraries(projectAxes ${OpenCV_LIBS})

add_executable(createObject src/createObject.cpp src/functions.cpp)
target_link_libraries(createObject ${OpenCV_LIBS})