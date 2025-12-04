# CMakeLists.txt for qFFTFilter plugin
# This file follows CloudCompare's standard plugin structure

cmake_minimum_required(VERSION 3.5)

# Option to enable/disable this plugin during CloudCompare build
option(PLUGIN_STANDARD_QFFTFILTER "Install qFFTFilter plugin" ON)

if (NOT PLUGIN_STANDARD_QFFTFILTER)
    return()
endif()

# Project name must match the plugin class name
project(QFFTFILTER_PLUGIN)

# Add the plugin (this macro is provided by CloudCompare's build system)
AddPlugin(NAME ${PROJECT_NAME})

# Source files
set(SOURCES
    src/qFFTFilter.cpp
    src/qFFTFilterDialog.cpp
)

# Header files
set(HEADERS
    include/qFFTFilter.h
    include/qFFTFilterDialog.h
)

# Qt UI files (will be processed by Qt's UIC)
set(UI_FILES
    ui/qFFTFilterDialog.ui
)

# Resource files (icons, etc.)
set(RESOURCE_FILES
    qFFTFilter.qrc
)

# Tell CMake where to find headers
target_include_directories(${PROJECT_NAME} PRIVATE include)

# Add source files to the target
target_sources(${PROJECT_NAME} PRIVATE
    ${SOURCES}
    ${HEADERS}
    ${UI_FILES}
    ${RESOURCE_FILES}
)

# Link against CloudCompare libraries
# CCCoreLib: Core data structures (point clouds, scalar fields)
# QCC_DB_LIB: Database and entity management
# QCC_GL_LIB: OpenGL rendering
target_link_libraries(${PROJECT_NAME}
    CCCoreLib
    QCC_DB_LIB
    QCC_GL_LIB
)

# Find and link Qt components (Qt5 or Qt6)
# Concurrent: Multi-threading support
# Widgets: UI elements
# Core: Qt fundamentals
find_package(Qt6 COMPONENTS Widgets Core Concurrent QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS Widgets Core Concurrent REQUIRED)
    target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets
        Qt5::Core
        Qt5::Concurrent
    )
else()
    target_link_libraries(${PROJECT_NAME}
        Qt6::Widgets
        Qt6::Core
        Qt6::Concurrent
    )
endif()

# Find and link FFTW3 library
# FFTW is a fast FFT implementation library
find_package(FFTW3 QUIET)

if(FFTW3_FOUND)
    # If FFTW3 is installed, use it
    message(STATUS "qFFTFilter: Using FFTW3 library (optimized)")
    target_link_libraries(${PROJECT_NAME} FFTW3::fftw3)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_FFTW)
    
    # Enable FFTW wisdom for persistent optimization
    target_compile_definitions(${PROJECT_NAME} PRIVATE FFTW_ENABLE_WISDOM)
else()
    # Otherwise, we'll use KissFFT (included in source as fallback)
    message(STATUS "qFFTFilter: FFTW3 not found, using KissFFT fallback (slower)")
    
    # Add KissFFT sources directly (simple, header-only alternative)
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/kissfft/kiss_fft.c
    )
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/kissfft
    )
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_KISSFFT)
endif()

# Generate Qt UI headers and MOC files
# This processes .ui files and Q_OBJECT macros
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOUIC ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOUIC_SEARCH_PATHS ui)

# Install the plugin to CloudCompare's plugin directory
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/plugins
    LIBRARY DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}/plugins
)