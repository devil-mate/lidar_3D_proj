set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -fPIC")


# 

# 如果找到了glog库，设置相关变量
find_library(GLOG_LIB
    NAMES glog
)
if (NOT GLOG_LIB)  
    message(FATAL_ERROR "Could not find glog library")
    if(${ARM})
        set(GLOG_LIB  libglog.a)
    endif()    
endif ()



# find_package(Boost  REQUIRED COMPONENTS filesystem)
# find_package(mysqlclient REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})


# set(MQ_SRC 
#     message/Message.cpp
# )
# add_library(MessageLib STATIC ${MQ_SRC})
# add_library(MessageT SHARED socket/MessageT.h)
# target_link_libraries(Message ${COMMON_LIB})
set(SIMPLE_LIB 
    ${COMMON_LIB}
    # RingBuffer
    # MessageLib
    pthread
    )
# add_library(MqProducer STATIC socket/MqProducer.cpp )
# target_link_libraries(MqProducer  ${SIMPLE_LIB} )
# add_library(MqConsumer  STATIC socket/MqConsumer.cpp )
# target_link_libraries(MqConsumer  ${SIMPLE_LIB} )


# aux_source_directory( nodeDev NODE_DEV_SRC)
# add_library(NodeDev  STATIC ${NODE_DEV_SRC}  )
# target_link_libraries(NodeDev    ${COMMON_LIB} )








add_library(LinearPathPlanner  LinearPathPlanner.cpp )
# target_link_libraries(LinearPathPlanner ${catkin_LIBRARIES})
# add_dependencies(LinearPathPlanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



# aux_source_directory(battery_driver_ros/ BATTERY_NODE_SRC)
# add_executable(battery_driver ${BATTERY_NODE_SRC}  )
# target_link_libraries(battery_driver ${COMMON_LIB}  ${WS_LIB} ${Utils} )

# aux_source_directory(arm_driver_ros/ ARM_NODE_SRC)
# add_executable(arm_driver ${ARM_NODE_SRC}  )
# target_link_libraries(arm_driver ${SIMPLE_LIB}  ${WS_LIB} ${Utils} )

# aux_source_directory(datas_capture_ros/ DATAS_CAPTURE_NODE_SRC)
# add_executable(datas_capture ${DATAS_CAPTURE_NODE_SRC}  )
# target_link_libraries(datas_capture ${COMMON_LIB}  ${WS_LIB} ${Utils} ${MYSQL_LIB})

