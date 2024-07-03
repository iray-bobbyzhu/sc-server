cmake_minimum_required(VERSION 3.26)

project(server LANGUAGES CXX VERSION 1.0.0)

find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(${PROJECT_NAME})
add_subdirectory(src)

# provide below variables with actual values
set(EXTERNAL_INCLUDES )
set(EXTERNAL_LIB_PATH )
set(EXTERNAL_LIBS )

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/inc ${EXTERNAL_INCLUDES})
target_link_directories(${PROJECT_NAME} PRIVATE ${EXTERNAL_LIB_PATH})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${EXTERNAL_LIBS} -static-libstdc++)
