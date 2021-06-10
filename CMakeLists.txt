project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp)

# if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 

# find_package(ZLIB REQUIRED)
# find_package(OpenSSL REQUIRED)
# find_library(LIBUV_LIBRARY NAMES libuv uv uv1)
# find_library(libUWS_LIBRARY NAMES uWS)

# endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning ${sources})

# if(${CMAKE_SYSTEM_NAME} MATCHES "Linux") 
target_link_libraries(path_planning z ssl crypto uv uWS)
# endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux") 

# if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 
# include_directories("D:\\src\\vcpkg\\installed\\x64-windows\\include\\")
# target_link_libraries(path_planning ${ZLIB_LIBRARY} ${OPENSSL_SSL_LIBRARY} ${OPENSSL_CRYPTO_LIBRARY} ${LIBUV_LIBRARY} ${libUWS_LIBRARY})
# endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 






