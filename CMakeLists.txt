cmake_minimum_required(VERSION 3.5.7)

project(indi-astrolink4pi 
    VERSION 1.5
    LANGUAGES C CXX
)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Custom CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

include(GNUInstallDirs)

# ====== INDI ======
find_package(INDI REQUIRED)

# ====== Configure Files ======
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_astrolink4pi.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_astrolink4pi.xml
)

# ====== Include Directories ======
# Modern CMake: target_include_directories, not include_directories
set(PROJECT_INCLUDES
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${INDI_INCLUDE_DIRS}
)

# ====== Source Files ======
set(indi_astrolink4pi_SRCS
    astrolink4pi.cpp
)

# ====== Optional Unity Build ======
# Only if ENABLE_UNITY_BUILD() macro exists in included file
include(CMakeCommon)
if(UNITY_BUILD)
    ENABLE_UNITY_BUILD(indi_astrolink4pi indi_astrolink4pi_SRCS 6 cpp)
endif()

# ====== Libraries ======
# Proper detection of lgpio (do NOT link ".so")
# find_library(GPIO_LIBRARIES NAMES lgpio)
# if(NOT GPIO_LIBRARIES)
#     message(FATAL_ERROR "lgpio library not found")
# endif()

# ====== Build Executable ======
add_executable(indi_astrolink4pi ${indi_astrolink4pi_SRCS})

target_include_directories(indi_astrolink4pi PRIVATE ${PROJECT_INCLUDES})

target_link_libraries(indi_astrolink4pi
    PRIVATE
        ${INDI_LIBRARIES}
#        ${GPIO_LIBRARIES}
        pthread
)

# ====== Installation ======
install(TARGETS indi_astrolink4pi
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_astrolink4pi.xml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/indi
)
