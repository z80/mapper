
set( appname "camlib" )

find_package(Boost REQUIRED)
if ( UNIX )
    #find_package( OpenMP REQUIRED )
    #if (OPENMP_FOUND)
    #    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    #    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    #endif()
endif ( UNIX )

# Use the include path and library for Qt that is used by VTK.
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
  ./src
)


#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    #set (CMAKE_SHARED_LINKER_FLAGS ${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_CXX_FLAGS} )
#    set (CMAKE_STATIC_LINKER_FLAGS "-fopenmp" )
    #set (CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_FLAGS} )
#endif()



file( GLOB src "./src/*.cpp" )

add_library( ${appname} STATIC ${src} )
target_link_libraries( ${appname} ${Boost_LIBS} )

install( TARGETS ${appname} DESTINATION ./ )



