cmake_minimum_required(VERSION 2.4.7)
PROJECT(indi-astrolink4pi CXX C)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

include(GNUInstallDirs)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)

find_package(INDI REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_astrolink4pi.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_astrolink4pi.xml)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})

include(CMakeCommon)

set(GPIO_LIBRARIES "libgpiod.so")
set(PIGPIO_LIBRARIES "libpigpiod_if2.so")

################ AstroLink 4 Pi ################
set(indi_astrolink4pi_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/astrolink4pi.cpp
   )

IF (UNITY_BUILD)
    ENABLE_UNITY_BUILD(indi_astrolink4pi indi_astrolink4pi_SRCS 6 cpp)
ENDIF ()

add_executable(indi_astrolink4pi ${indi_astrolink4pi_SRCS})
#find_library(PIGPIO_LIBRARIES NAMES pigpiod_if2)
target_link_libraries(indi_astrolink4pi ${INDI_LIBRARIES} ${GPIO_LIBRARIES} ${PIGPIO_LIBRARIES})
install(TARGETS indi_astrolink4pi RUNTIME DESTINATION bin )
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_astrolink4pi.xml DESTINATION ${INDI_DATA_DIR})


