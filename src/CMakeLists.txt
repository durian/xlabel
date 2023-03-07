# Build script.
#
# Useful command line flags:
#   * -DCMAKE_BUILD_TYPE=XXX, where XXX is one of Debug, Release,
#     RelWithDebInfo and MinSizeRel. Default is Release.
#   

cmake_minimum_required(VERSION 3.9)
project(XLABEL VERSION 1.0 DESCRIPTION "X-Plane plugin")

# By default, use optimized release configuration.
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()


set(CMAKE_CXX_STANDARD 14)

if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS YES CACHE BOOL "Export all symbols")

# Set include directories used by our code and dependencies.
include_directories("${CMAKE_SOURCE_DIR}/XPSDK400/CHeaders/Widgets")
include_directories("${CMAKE_SOURCE_DIR}/XPSDK400/CHeaders/Wrappers")
include_directories("${CMAKE_SOURCE_DIR}/XPSDK400/CHeaders/XPLM")
include_directories("${CMAKE_SOURCE_DIR}/WinSDK/Include")
include_directories("${CMAKE_SOURCE_DIR}")
include_directories("${CMAKE_SOURCE_DIR}/")


if (WIN32)
  include_directories("${CMAKE_SOURCE_DIR}/WinSDK/Include")
endif ()

# Specify library search locations.
if (WIN32)
    list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_SOURCE_DIR}/XPSDK400/Libraries/Win")
    list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_SOURCE_DIR}/WinSDK/Lib/x64")
elseif (APPLE)
    list(APPEND CMAKE_FRAMEWORK_PATH "${CMAKE_SOURCE_DIR}/XPSDK400/Libraries/Mac")
elseif (UNIX)

endif ()

# Enable all X-Plane SDK APIs up to the newest version.
add_definitions(-DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM302=1)

# Define platform macros.
add_definitions(-DAPL=$<BOOL:${APPLE}> -DIBM=$<BOOL:${WIN32}> -DLIN=$<AND:$<BOOL:${UNIX}>,$<NOT:$<BOOL:${APPLE}>>>)

# Force-enable exception support. This is most likely redundant, although for C
# code the default is the opposite. Since we are mixing C++ and C libraries,
# safer to set it on?
add_compile_options(-fexceptions)
# On UNIX systems this makes symbols non-exported by default. On Windows this
# option is simply ignored, since symbol visibility works differently there.
add_compile_options(-fvisibility=hidden)

# Enable stricter warnings and then disable some we are not interested in.
#add_compile_options(-Wall -Wpedantic -Wshadow -Wfloat-equal -Wextra)
add_compile_options(-Wno-unused)
#add_compile_options(-Wno-format-overflow)

# Always use position-independent code and highest optimization level (FPS!).
add_compile_options(-O3)

# X-Plane plugin
list(APPEND XLABEL_SRCS
  toml.h
  dr.h
  global.h
  Log.h
  Smoker.h
  main.cpp
  global.cpp
  Log.cpp
  Smoker.cpp
  dr.cpp
  geohash.cpp
  custom_drefs.cpp
  ) 

add_library(XLABEL SHARED ${XLABEL_SRCS})

target_compile_features(XLABEL PUBLIC cxx_std_14) 

if (APPLE)
    # X-Plane supports OS X 10.10+, so this should ensure this can run on
    # all supported versions.
    target_compile_options(XLABEL PUBLIC -mmacosx-version-min=10.10 -std=c++14)
    target_link_libraries(XLABEL -mmacosx-version-min=10.10)
endif ()

# Link OpenGL and OpenAL related libraries.
find_package(OpenGL REQUIRED)  # apt install freeglut3-dev
find_library(OPENAL_LIBRARY NAMES openal OpenAL OpenAL32)  # apt install libopenal-dev
find_library(GLUT_LIBRARY NAMES glut GLUT glut64 glut32)  # apt install freeglut3-dev
#target_link_libraries(XLABEL ${OPENGL_LIBRARIES} ${OPENAL_LIBRARY} ${GLUT_LIBRARY})

# Link X-Plane plugin system libraries. They are only provided for OS X and Windows.
if (WIN32)
  find_library(XPLM_LIBRARY NAMES XPLM_64 XPLM_64.lib)
  find_library(XPWIDGETS_LIBRARY NAMES XPWidgets_64 XPWidgets_64.lib)
endif ()

# create target libraries for Windows or Mac
if (WIN32)
	target_link_libraries(XLABEL ${XPLM_LIBRARY} ${XPWIDGETS_LIBRARY})
elseif (APPLE)
	target_link_libraries(XLABEL ${XPLM_LIBRARY} ${XPWIDGETS_LIBRARY})
endif ()

# Link library for dynamic loading of shared objects on UNIX systems.
if (UNIX)
    add_compile_options(-fPIC)
endif ()

# Link OS X core system libraries.
if (APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(CORE_FOUNDATION_LIBRARY CoreFoundation)
    find_library(GLUT_LIBRARY GLUT)
    find_library(OpenGL_LIBRARY OpenGL)
    find_library(Cocoa_LIBRARY Cocoa)

    target_link_libraries(XLABEL ${IOKIT_LIBRARY} ${CORE_FOUNDATION_LIBRARY} ${GLUT_LIBRARY})
    target_link_libraries(XLABEL ${OpenGL_LIBRARY} ${Cocoa_LIBRARY})
endif ()

if (WIN32)
    # Unlike OS X and Linux we build standard libraries statically since X-Plane does not provide them.
    target_link_libraries(XLABEL -static-libgcc -static-libstdc++ opengl32)
    # MSYS2 for some reason always links against winpthread, so we have to work around it by statically linking the
    # entire plugin. This allows building the plugin nativaly on Windows.
    target_link_libraries(XLABEL -static)
elseif (APPLE)
    # Restrict set of symbols exported from the plugin. This reduces changes of
    # conflict with other plugins, in particular ones with Lua interpreter
    # embedded.
    target_link_libraries(XLABEL "-exported_symbols_list ${CMAKE_SOURCE_DIR}/XLABEL.sym_mac")
    target_link_libraries(XLABEL "-flat_namespace -undefined suppress"   -stdlib=libc++)
elseif (UNIX)
    # Specify additional runtime search laths for dynamically-linked libraries.
    # Restrict set of symbols exported from the plugin. This reduces changes of
    # conflict with other plugins, in particular ones with Lua interpreter
    # embedded.
    target_link_libraries(XLABEL -ldl -Wl,--version-script -Wl,${CMAKE_SOURCE_DIR}/XLABEL.sym)
endif ()


if (WIN32)
	set_target_properties(XLABEL PROPERTIES OUTPUT_NAME "win")
   	set(PLUGIN_OUTPUT_FILENAME "win.xpl")
elseif (APPLE)
	set_target_properties(XLABEL PROPERTIES OUTPUT_NAME "mac")
   	set(PLUGIN_OUTPUT_FILENAME "mac.xpl")
elseif (UNIX)
	set_target_properties(XLABEL PROPERTIES OUTPUT_NAME "lin")
   	set(PLUGIN_OUTPUT_FILENAME "lin.xpl")
endif ()

set_target_properties(XLABEL PROPERTIES PREFIX "")
set_target_properties(XLABEL PROPERTIES OUTPUT_NAME "xlabel")
set_target_properties(XLABEL PROPERTIES SUFFIX ".xpl")
