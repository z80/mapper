
set( appname "qml" )

if ( WIN32 )
    set( MSVC_RUNTIME dynamic )
    include( ${ROOT}/cmake/MSVCRuntime.cmake )
    configure_msvc_runtime()
endif ( WIN32 )


# Use the include path and library for Qt that is used by VTK.
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ./src
)


file( GLOB Srcs_c "./src/*.c" )
file( GLOB Srcs "./src/*.cpp" )
set( Srcs ${Srcs} ${Srcs_c} )
file( GLOB Hdrs "./src/*.h" )
set( MOC_Hdrs ${Hdrs} )
file( GLOB UIs "./src/*.ui" )
file( GLOB QRCs "./src/*.qrc" )

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# The rest should just work (sure...)
# We have ui files, this will bring in the macro: qt5_wrap_ui
find_package(Qt5 COMPONENTS Widgets Quick Qml Core REQUIRED QUIET)
qt5_wrap_ui(UI_Srcs ${UIs})
qt5_add_resources(QRC_Srcs ${QRCs} )

include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS})

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

add_executable( ${appname} WIN32
    ${Srcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs})
qt5_use_modules(${appname} Core Gui Widgets Quick )
target_link_libraries(${appname} Qt5::WinMain Qt5::Widgets )

install( TARGETS ${appname} DESTINATION ./ )



