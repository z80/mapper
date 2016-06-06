
if ( WIN32 )
    include( "${ROOT}/settings.win32.cmake" )
elseif ( UNIX )
    include( "${ROOT}/settings.unix.cmake" )
endif ( WIN32 )


#set( CMAKE_MODULE_PATH ${VTK_DIR} ${CMAKE_MODULE_PATH} )

#set( CMAKE_MODULE_PATH ${VTK_DIR} )
set( VTK_RENDERING_BACKEND OpenGL )

find_package(VTK COMPONENTS
  vtkCommonCore
  vtkFiltersSources
  vtkInteractionStyle
  vtkRendering${VTK_RENDERING_BACKEND}
  PATHS ${VTK_DIR}
)
include(${VTK_USE_FILE})

set(RENDERING_EXAMPLES_SRCS
  Cylinder
  SpecularSpheres
  DiffuseSpheres
  AmbientSpheres
#  MaterialObjects
  )

foreach(name ${RENDERING_EXAMPLES_SRCS})
  add_executable(${name} MACOSX_BUNDLE ${name}.cxx)
  target_link_libraries(${name} ${VTK_LIBRARIES} )
endforeach()
