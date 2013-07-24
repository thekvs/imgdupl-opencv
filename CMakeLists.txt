cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(imgdupl)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)

include_directories(${imgdupl_SOURCE_DIR}/)

include(FindThreads)

include(FindPkgConfig)
if (NOT PKG_CONFIG_FOUND)
    message(FATAL_ERROR, "Couldn't find pkg-config")
endif()

pkg_check_modules(LIBOPENCV QUIET opencv)
if (NOT LIBOPENCV_FOUND)
    message(FATAL_ERROR "Couldn't find OpenCV")
endif()

pkg_check_modules(SQLITE3 QUIET sqlite3)
if (NOT SQLITE3_FOUND)
    message(FATAL_ERROR "Couldn't find sqlite3")
endif()

find_package(Boost COMPONENTS thread filesystem serialization iostreams program_options system)

find_package(Yajl REQUIRED)
if (NOT YAJL_FOUND)
    message(FATAL_ERROR "Couldn't find libyajl")
endif()

include_directories(${LIBOPENCV_INCLUDE_DIRS})
include_directories(${SQLITE3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAJL_INCLUDE_DIRS})

link_directories(${LIBOPENCV_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -W")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W")

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
endif()

set(CHECKER_SRCS
    ${imgdupl_SOURCE_DIR}/mongoose.c
    ${imgdupl_SOURCE_DIR}/searcher.cpp
    ${imgdupl_SOURCE_DIR}/config.cpp
)
add_definitions(-DNO_SSL)

add_library(imgdupl-static STATIC ${CHECKER_SRCS})
set_target_properties(imgdupl-static PROPERTIES OUTPUT_NAME imgdupl)
set_target_properties(imgdupl-static PROPERTIES LINKER_LANGUAGE CXX)

set(IMGDUPL_LIBS
    imgdupl-static
    ${LIBOPENCV_LDFLAGS}
    ${Boost_LIBRARIES}
    ${SQLITE3_LIBRARIES}
    ${YAJL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(imgdupl ${imgdupl_SOURCE_DIR}/imgdupl.cpp)
target_link_libraries(imgdupl ${IMGDUPL_LIBS})

set(BUILD_DB_LIBS
    imgdupl-static
    ${LIBOPENCV_LDFLAGS}
    ${Boost_LIBRARIES}
    ${SQLITE3_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(build-imgdupl-db ${imgdupl_SOURCE_DIR}/build_imgdupl_db.cpp)
target_link_libraries(build-imgdupl-db ${BUILD_DB_LIBS})

add_subdirectory(tests)

install(PROGRAMS ${imgdupl_BINARY_DIR}/imgdupl DESTINATION bin/)
install(PROGRAMS ${imgdupl_BINARY_DIR}/build-imgdupl-db DESTINATION bin/)

