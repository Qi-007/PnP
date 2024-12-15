# cmake_minimum_required(VERSION 3.17)

# project(test)

# #查找opencv
# find_package(OpenCV REQUIRED)

# #包含头文件路径
# include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

# #添加可执行文件
# set(SRC_DIR "/home/zjq/code/detectArmor/src")

# file(GLOB SRC_FILES "${SRC_DIR}/*cpp")

# add_executable(test ${SRC_FILES} main.cpp)

# #链接库
# target_link_libraries(test ${OpenCV_LIBS})



cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)

project(test)

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SRC hik_camera/src/*.cpp src/*.cpp)

add_executable(test main.cpp ${SRC})

target_include_directories(test PUBLIC ${OpenCV_INCLUDE_DIRS} include HikDriver)

target_link_libraries(test hik_driver)

target_link_libraries(test ${OpenCV_LIBS} )

target_link_libraries(${PROJECT_NAME}  -lX11)                    #底层显示后端

link_directories("${PROJECT_DIR}/hik_driver/lib/64")             #从该路径下寻找需要的库
