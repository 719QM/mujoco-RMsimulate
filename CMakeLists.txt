cmake_minimum_required(VERSION 3.0.0)
project(vctest VERSION 0.1.0 LANGUAGES C CXX)


# 安装路径
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install) 
# 库路径
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
# include路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# 抓代码
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

# 生成可执行程序MuJoCoDemo_1
add_executable(MuJoCoDemo_1 ${SOURCES})
add_executable(pendulum ${CMAKE_CURRENT_SOURCE_DIR}/src/pendulum.cpp)
add_executable(doublependulum ${CMAKE_CURRENT_SOURCE_DIR}/src/doublependulum.cpp)
add_executable(doublependulum_fsm ${CMAKE_CURRENT_SOURCE_DIR}/src/doublependulum_fsm.cpp)
add_executable(doublependulum_ik ${CMAKE_CURRENT_SOURCE_DIR}/src/doublependulum_ik.cpp)
add_executable(hybrid_pendulum ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_pendulum.cpp)
add_executable(arm ${CMAKE_CURRENT_SOURCE_DIR}/src/arm.cpp)
add_executable(ik ${CMAKE_CURRENT_SOURCE_DIR}/src/pendulum_ik.cpp)

# 链接glfw3和mujoco库
target_link_libraries(MuJoCoDemo_1 ${LIB_DIR}/mujoco.lib)
target_link_libraries(pendulum ${LIB_DIR}/mujoco.lib)
target_link_libraries(doublependulum ${LIB_DIR}/mujoco.lib)
target_link_libraries(doublependulum_fsm ${LIB_DIR}/mujoco.lib)
target_link_libraries(doublependulum_ik ${LIB_DIR}/mujoco.lib)
target_link_libraries(hybrid_pendulum ${LIB_DIR}/mujoco.lib)
target_link_libraries(arm ${LIB_DIR}/mujoco.lib)
target_link_libraries(ik ${LIB_DIR}/mujoco.lib)

target_link_libraries(MuJoCoDemo_1 ${LIB_DIR}/glfw3.dll)
target_link_libraries(pendulum ${LIB_DIR}/glfw3.dll)
target_link_libraries(doublependulum ${LIB_DIR}/glfw3.dll)
target_link_libraries(doublependulum_fsm ${LIB_DIR}/glfw3.dll)
target_link_libraries(doublependulum_ik ${LIB_DIR}/glfw3.dll)
target_link_libraries(hybrid_pendulum ${LIB_DIR}/glfw3.dll)
target_link_libraries(arm ${LIB_DIR}/glfw3.dll)
target_link_libraries(ik ${LIB_DIR}/glfw3.dll)

# install到一起去
install(FILES ${LIB_DIR}/mujoco.dll ${LIB_DIR}/glfw3.dll ${CMAKE_CURRENT_SOURCE_DIR}/model/ball.xml
        DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS MuJoCoDemo_1  
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})  
install(TARGETS pendulum  
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}) 
install(TARGETS doublependulum  
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})   
install(TARGETS doublependulum_fsm  
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}) 
install(TARGETS doublependulum_ik  
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS hybrid_pendulum  
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS arm  
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}) 
install(TARGETS ik  
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}) 
