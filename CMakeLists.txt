cmake_minimum_required(VERSION 3.10)
project(indi-astrolink4pi VERSION 1.5 LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

include(GNUInstallDirs)

find_package(INDI REQUIRED)
find_package(Threads REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_astrolink4pi.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_astrolink4pi.xml)

include_directories(${INDI_INCLUDE_DIR}) # kept for any legacy includes; prefer target_include_directories below

include(CMakeCommon)

# Prefer finding the library instead of hard-coding the .so filename
find_library(LGPIOLIB NAMES lgpio liblgpio)
if(NOT LGPIOLIB)
    message(FATAL_ERROR "lgpio library not found. Install liblgpio or set LGPIOLIB manually.")
endif()

################ AstroLink 4 Pi ################
set(indi_astrolink4pi_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/astrolink4pi.cpp
)

if (UNITY_BUILD)
    ENABLE_UNITY_BUILD(indi_astrolink4pi indi_astrolink4pi_SRCS 6 cpp)
endif()

add_executable(indi_astrolink4pi ${indi_astrolink4pi_SRCS})

# modern target-based usage
target_include_directories(indi_astrolink4pi PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${INDI_INCLUDE_DIR}
)

target_compile_features(indi_astrolink4pi PRIVATE cxx_std_11)

target_link_libraries(indi_astrolink4pi PRIVATE
    ${INDI_LIBRARIES}
    ${LGPIOLIB}
    Threads::Threads
)

install(TARGETS indi_astrolink4pi RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_astrolink4pi.xml DESTINATION ${INDI_DATA_DIR})
