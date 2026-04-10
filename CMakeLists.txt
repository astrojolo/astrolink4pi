cmake_minimum_required(VERSION 3.10)
project(indi-astrolink4pi VERSION 2.1 LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

include(GNUInstallDirs)

find_package(INDI REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig)

pkg_check_modules(WIRINGPI wiringPi)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_astrolink4pi.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_astrolink4pi.xml)

include_directories(${INDI_INCLUDE_DIR}) # kept for any legacy includes; prefer target_include_directories below

include(CMakeCommon)

find_library(WIRINGPI_LIB wiringPi)

if(NOT WIRINGPI_LIB)
    message(FATAL_ERROR "wiringPi library not found!")
endif()


################ AstroLink 4 Pi ################
set(indi_astrolink4pi_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/astrolink4pi.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/boardio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/systeminfo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/pwm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/powermonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/shtreader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/mlxreader.cpp
)

if (UNITY_BUILD)
    ENABLE_UNITY_BUILD(indi_astrolink4pi indi_astrolink4pi_SRCS 6 cpp)
endif()

add_executable(indi_astrolink4pi ${indi_astrolink4pi_SRCS})

# modern target-based usage
target_include_directories(indi_astrolink4pi PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/modules
    ${INDI_INCLUDE_DIR}
)

target_compile_features(indi_astrolink4pi PRIVATE cxx_std_11)

target_link_libraries(indi_astrolink4pi PRIVATE
    ${INDI_LIBRARIES}
    ${WIRINGPI_LIB}
    Threads::Threads
)

install(TARGETS indi_astrolink4pi RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_astrolink4pi.xml DESTINATION ${INDI_DATA_DIR})
