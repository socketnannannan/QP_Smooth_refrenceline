cmake_minimum_required (VERSION 3.2)
project (osqp)

set(CMAKE_BUILD_TYPE "Debug")

find_package(Python3 3.10 COMPONENTS Interpreter Development NumPy REQUIRED)

find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories({PYTHON_INCLUDE_DIRS}
                    include
                    /usr/local/include/osqp
                    ${EIGEN3_INCLUDE_DIR})

add_executable(${CMAKE_PROJECT_NAME}  
                src/main.cpp
                src/QP.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME} 
                        Python3::Python
                        Python3::NumPy
                        ${CMAKE_SHARED_LIBRARY_PREFIX}osqp${CMAKE_SHARED_LIBRARY_SUFFIX}
                        OsqpEigen::OsqpEigen
                    )


