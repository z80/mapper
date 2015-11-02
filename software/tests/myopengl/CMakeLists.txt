
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5Core)
FIND_PACKAGE(Qt5Network)
FIND_PACKAGE(Qt5OpenGL)
SET(CMAKE_AUTOMOC ON)

# OpenGL & GLEW library
find_package(OpenGL)


include_directories( "./src"
                   )
file( GLOB src     "./src/*.cpp" )
file( GLOB headers "./src/*.h" )
file( GLOB ui      "./src/*.ui" )
file( GLOB qrc     "./src/*.qrc" )
message( STATUS "SRC = ${src}" )
message( STATUS "HEADERS = ${headers}" )
message( STATUS "ui = ${ui}" )

#qt5_wrap_cpp( moc_src ${headers} )
#set( src ${src} ${moc_src} )

qt5_wrap_ui( moc_src ${ui} )
set( src ${src} ${moc_src} )

qt5_add_resources( qrc_src ${qrc} )
set( src ${src} ${qrc_src} )

if ( WIN32 )
    list( APPEND src "src/icon.rc" )
    set( BLD_TYPE WIN32 )
else ( WIN32 )
    set( BLD_TYPE "" )
endif ( WIN32 )


add_executable( myopengl ${BLD_TYPE} ${src} )
qt5_use_modules( myopengl Widgets Core OpenGL )
set( libs ${libs} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} )
if ( WIN32 )
    set( libs ${libs} Qt5::WinMain )
endif ( WIN32 )
target_link_libraries( myopengl ${libs} )
install( TARGETS myopengl DESTINATION ./ )
