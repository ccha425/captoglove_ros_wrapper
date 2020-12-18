cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_BUILD_TYPE Debug)

# Set CaptoGloveAPI (sources and library)
set(CAPTOGLOVE_API_INCLUDE_DIR $ENV{CAPTOGLOVEAPI_SRC_PATH}
message(STATUS "CAPTOGLOVEAPI SRC PATH: " ${CAPTOGLOVEAPI_INCLUDE_DIR})
set(CAPTOGLOVE_LIBRARY_DIR $ENV{CAPTOGLOVEAPI_LIBRARY_PATH})
message(STATUS "CAPTOGLOVEAPI LIBRARY PATH: " ${CAPTOGLOVEAPI_LIBRARY_DIR})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    cmake_modules
    std_msgs
    qt_build
    captoglove_ros_msgs
}

find_package(Qt5 REQUIRED COMPONENTS
         SerialPort
         Core
         DBus 
         Network
         Bluetooth
) 

message(STATUS "QT5: " ${Qt5_FOUND})
message(QT5_DIR "${Qt5_DIR})

find_package(Protobuf REQUIRED)
message(STATUS "PROTOBUF LIBRARY: " ${PROTOBUF_LIBRARY})
message(STATUS "PROTOBUF INCLUDE DIR:" ${PROTOBUF_INCLUDE_DIR})

catkin_package(
    INCLUDE_DIRS src include ${CAPTOGLOVEAPI_INCLUDE_DIR}
    LIBRARIES ${CAPTOGLOVE_LIBRARY} ${PROTOBUF_LIBRARY}
    CATKIN_DEPENDS message_runtime roscpp rospy std_msgs captoglove_ros_msgs 
    DEPENDS
}

include_directories(include 
    ${catkin_INCLUDE_DIRS}
    ${CAPTOGLOVEAPI_INCLUDE_DIR}
    ${CAPTOGLOVEAPI_INCLUDE_DIR}/proto_impl
    ${PROTOBUF_LIBRARIES}
)

set(captoglove_ros_wrapper_SOURCES src/captoglove_ros.cpp src/ros_translate.cpp src/main.cpp)
set(captoglove_ros_wrappe_HEADERS include/captoglove_ros/captoglove_ros.h include/captoglove_ros/ros_translate.h)

add_definitions(-std=c++11)

QT5_WRAP_CPP(captoglove_ros_wrapper_SRC ${captoglove_ros_wrapper_HEADERS})
add_library(captoglove_ros_wrapper/captoglove_ros_wrapper.cpp src/ros_translate.cpp ${captoglove_ros_wrapper_SRC})

target_link_libraries(captoglove_ros_wrapper 
    ${catkin_LIBRARIES}
    ${CAPTOGLOVEAPI_LIBRARY}
)

qt5_use_modules(captoglove_ros_wrapper Core Bluetooth Network DBus) 
add_executable(captoglove_ros_wrapper_node src/main.cpp ${captoglove_ros_wrapper_SRC})

target_link_libraries(captoglove_ros_wrapper_node
	captogolove_ros_wrapper
	${catkin_LIBRARIES}
	${CAPTOGLOVEAPI_LIBRARY}
	${PROTOBUF_LIBRARIES}
)

add_dependencies(infinity_ros_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp) 

