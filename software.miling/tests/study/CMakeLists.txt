
set( appname "study" )

#find_package( Boost REQUIRED COMPONENTS regex system filesystem REQUIRED )


#include(${BOOST_USE_FILE})

# Use the include path and library for Qt that is used by VTK.
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
  ./src
)

file( GLOB Srcs_c "./src/*.c" )
file( GLOB Srcs "./src/*.cpp" )
set( Srcs ${Srcs} ${Srcs_c} )
file( GLOB Hdrs "./src/*.h" )

add_executable( ${appname} ${Srcs} ${Hdrs} )
target_link_libraries(${appname} ${Boost_LIBRARIES} )


install( TARGETS ${appname} DESTINATION ./ )



