cmake_minimum_required(VERSION 3.0.0)
project(Xsens_Real-time_Visualization VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

include_directories(/usr/local/xsens/include)
include_directories(/usr/local/MATLAB/R2022b/extern/include)


add_executable(Xsens_Real-time_Visualization main.cpp)

target_link_libraries(Xsens_Real-time_Visualization 
        /usr/local/MATLAB/R2022b/extern/bin/glnxa64/libMatlabEngine.so
        /usr/local/MATLAB/R2022b/extern/bin/glnxa64/libMatlabDataArray.so)

# a fix to the pthread.h problem
find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
    target_compile_options(Xsens_Real-time_Visualization PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(Xsens_Real-time_Visualization  "${CMAKE_THREAD_LIBS_INIT}")
endif()

target_link_libraries(Xsens_Real-time_Visualization
    /usr/local/xsens/lib/libdataprocessor.so
    /usr/local/xsens/lib/libxsensdot_pc_sdk.so.2022
    /usr/local/xsens/lib/libdataprocessor.so.2022      
    /usr/local/xsens/lib/libxsensdot_pc_sdk.so.2022.2.0
    /usr/local/xsens/lib/libdataprocessor.so.2022.1.0  
    /usr/local/xsens/lib/libxstypes.so
    /usr/local/xsens/lib/libgattlib.so
    /usr/local/xsens/lib/libxstypes.so.2022
    /usr/local/xsens/lib/libxsensdot_pc_sdk_java64.so  
    /usr/local/xsens/lib/libxstypes.so.2022.1.0
    /usr/local/xsens/lib/libxsensdot_pc_sdk.so)



set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(CMAKE_AUTOMOC ON)
include(CPack)
