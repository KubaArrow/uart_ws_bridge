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


add_executable(my_node src/main.cpp)
target_link_libraries(my_node
        log
        ws_bridge
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

target_compile_definitions(my_node PRIVATE
        WS_BRIDGE_LOG_LEVEL=LOG_TRACE
        LOG_USE_COLOR
        USE_HIGHRES_TIME
)
