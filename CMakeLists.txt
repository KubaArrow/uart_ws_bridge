cmake_minimum_required(VERSION 3.0.2)
project(uart_ws_bridge)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
)

catkin_package(
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(log
        src/log.c
)

add_library(ws_bridge
        src/ws_bridge.c
)
target_link_libraries(ws_bridge
        log
)

add_library(serial_slip
        src/serial_slip.c
)
target_link_libraries(serial_slip
        log
)


add_executable(uwu_bridge src/main.cpp)
target_link_libraries(uwu_bridge
        log
        ws_bridge
        serial_slip
        ${catkin_LIBRARIES}
        cjson
        websockets
        ssl
        crypto
        pthread
)

target_compile_definitions(log PRIVATE
        WS_BRIDGE_LOG_LEVEL=LOG_TRACE
        LOG_USE_COLOR
        USE_HIGHRES_TIME
)

target_compile_definitions(ws_bridge PRIVATE
        WS_BRIDGE_LOG_LEVEL=LOG_TRACE
        LOG_USE_COLOR
        USE_HIGHRES_TIME
)

target_compile_definitions(uwu_bridge PRIVATE
        WS_BRIDGE_LOG_LEVEL=LOG_TRACE
        LOG_USE_COLOR
        USE_HIGHRES_TIME
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
