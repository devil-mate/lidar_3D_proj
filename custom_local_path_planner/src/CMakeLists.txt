set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -fPIC")


# 

# # 如果找到了glog库，设置相关变量
# find_library(GLOG_LIB
#     NAMES glog
# )
# if (NOT GLOG_LIB)  
#     message(FATAL_ERROR "Could not find glog library")
#     if(${ARM})
#         set(GLOG_LIB  libglog.a)
#     endif()    
# endif ()
# include_directories(${GLOG_INCLUDE_DIRS})



# find_package(Boost  REQUIRED COMPONENTS filesystem)
# find_package(mysqlclient REQUIRED)


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





aux_source_directory(utils UTILS_SRC)
add_library(Utils  STATIC ${UTILS_SRC}  )
# aux_source_directory(${} UTILS_SRC)
file(GLOB PID_TRACK_SRC "*.cpp")  
add_library(PIDTrackingControllerRos  ${PID_TRACK_SRC} )
target_link_libraries(PIDTrackingControllerRos    Utils )


