
set( appname "drill_gui" )

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)


find_package(VTK COMPONENTS
  vtkCommonCore
  vtkFiltersCore
  vtkInfovisCore
  vtkInteractionStyle
  vtkViewsQt
)
include(${VTK_USE_FILE})

# Use the include path and library for Qt that is used by VTK.
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS} 
  ./src
  ${ROOT}/drill-pos/src
  ${ROOT}/3dparty/camlib/src
)

# Set your files and resources here
#set( Srcs main.cxx SimpleView.cxx )
#set( Hdrs SimpleView.h )
#set( MOC_Hdrs SimpleView.h )
#set( UIs SimpleView.ui )
#set( QRCs Icons/icons.qrc )

file( GLOB Srcs "./src/*.cpp" )
file( GLOB Hdrs "./src/*.h" )
set( MOC_Hdrs ${Hdrs} )
file( GLOB UIs "./src/*.ui" )
file( GLOB QRCs "./icons/*.qrc" )
set( Srcs ${Srcs} ${ROOT}/drill-pos/src/positioner.cpp )

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# The rest should just work (sure...)
if(VTK_QT_VERSION VERSION_GREATER "4")
  # We have ui files, this will bring in the macro: qt5_wrap_ui
  find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
  qt5_wrap_ui(UI_Srcs ${UIs})
  qt5_add_resources(QRC_Srcs ${QRCs} )

  source_group("Resources" FILES
    ${UIs}
    ${QRCs}
    ${EXE_ICON} # Not present
  )

  source_group("Generated" FILES
    ${UI_Srcs}
    ${MOC_Srcs}
    ${QRC_Srcs}
    ${QRC_Srcs}
  )

  add_executable( ${appname} MACOSX_BUNDLE
    ${Srcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs})
  qt5_use_modules(${appname} Core Gui Widgets)
  target_link_libraries(${appname} ${VTK_LIBRARIES}
                                   ${OpenCV_LIBS}
                                   camlib )
else()
  find_package(Qt4 REQUIRED)
  include(${QT_USE_FILE})
  # Use what VTK built with
  set(QT_QMAKE_EXECUTABLE ${VTK_QT_QMAKE_EXECUTABLE} CACHE FILEPATH "")
  set(QT_MOC_EXECUTABLE ${VTK_QT_MOC_EXECUTABLE} CACHE FILEPATH "")
  set(QT_UIC_EXECUTABLE ${VTK_QT_UIC_EXECUTABLE} CACHE FILEPATH "")
  qt4_wrap_ui(UI_Srcs ${UIs})
  qt4_wrap_cpp(MOC_Srcs ${MOC_Hdrs} )
  qt4_add_resources(QRC_Srcs ${QRCs})

  source_group("Resources" FILES
    ${UIs}
    ${QRCs}
    ${EXE_ICON} # Not present
  )

  source_group("Generated" FILES
    ${UI_Srcs}
    ${QRC_Srcs}
    ${QRC_Srcs}
  )

  add_definitions(-DQT_GUI_LIBS -DQT_CORE_LIB -DQT3_SUPPORT)

  add_executable(${appname} 
    ${Srcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs})
  target_link_libraries(${appname}
    ${QT_LIBRARIES}
    ${VTK_LIBRARIES}
    ${OpenCV_LIBS}
    camlib
  )
endif()

install( TARGETS ${appname} DESTINATION ./ )



