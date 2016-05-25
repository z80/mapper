
project( camlocator )
cmake_minimum_required(VERSION 2.8)

set( ROOT ${CMAKE_CURRENT_SOURCE_DIR} )
#set( CMAKE_INSTALL_PREFIX ${ROOT}/.build.app )
set( CMAKE_INSTALL_PREFIX ${ROOT}/app )
set( _3DPARTY ${CMAKE_CURRENT_SOURCE_DIR}/3dparty )

if ( WIN32 )
    include( "./settings.win32.cmake" )
    set( MSVC_RUNTIME static )
    include( cmake/MSVCRuntime.cmake )
    configure_msvc_runtime()
elseif ( UNIX )
    include( "./settings.unix.cmake" )
    set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11 )
endif ( WIN32 )

include(cmake/OpenCVUtils.cmake)

ocv_clear_vars(OpenCVModules_TARGETS)

#message( STATUS $ENV{PATH} )

add_subdirectory( "./calibration" )
add_subdirectory( "./camera" )
add_subdirectory( "./perspective" )




