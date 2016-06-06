
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
#find_package(OpenCV REQUIRED)

# Add OpenCV headers location to your include paths
include_directories( #${OpenCV_INCLUDE_DIRS}
                     ${CMAKE_CURRENT_BINARY_DIR} 
                     "./src" 
                     "${ROOT}/drill-pos/src" )


find_package(VTK COMPONENTS
  vtkCommonCore
  vtkFiltersCore
  vtkInfovisCore
  vtkInteractionStyle
  vtkViewsQt
)
include(${VTK_USE_FILE})


# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)

file( GLOB UIs      "./src/*.ui" )
file( GLOB MOC_Hdrs "./src/*.h" )
file( GLOB QRCs     "./src/icons/*.qrc" )

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

file( GLOB src "./src/*.cpp" )
#set( src ${src} "${ROOT}/drill-pos/src/positioner.cpp" )

# Declare the executable target built from your sources
add_executable( drill_gui ${src} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs} )

# Link your application with OpenCV libraries
qt5_use_modules( drill_gui Core Gui Widgets )
target_link_libraries( drill_gui ${VTK_LIBRARIES} ) #${OpenCV_LIBS}
install( TARGETS drill_gui DESTINATION ./ )
