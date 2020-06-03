#  +---------------------------------------------------------------------------+
#  |                                                                           |
#  |               https://sites.google.com/site/bellonemauro/                 |
#  |                                                                           |
#  | Copyright (c) 2015, - All rights reserved.                                |
#  | Authors: Mauro Bellone                                                    |
#  | Released under BDS License.                                               |
#  +---------------------------------------------------------------------------+ */

project(PCL_upd_DEMO)

cmake_minimum_required(VERSION 2.8)

# --------------------------------------
#  including external libraries
# --------------------------------------
set(PCL_ML "/usr/local/include/pcl-1.8/pcl/ml/")
include_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories("/home/tarcisio/NetBeansProjects/OpenTraversability/libs/upd_lib/include/")
#include_directories("/usr/local/include/vtk-6.3")
include_directories(${PCL_ML})

# --------------------------------------
#  set variables for compilation 
# --------------------------------------
# select all headers in the include dir
FILE(GLOB PCL_upd_DEMO_HEADERS "${PCL_upd_DEMO_SOURCE_DIR}/*.h")   
# select all sources in the include dir
FILE(GLOB PCL_upd_DEMO_SOURCES "${PCL_upd_DEMO_SOURCE_DIR}/*.cpp")  
# select all files for the graphical interface
FILE(GLOB PCL_upd_DEMO_FORMS "${PCL_upd_DEMO_SOURCE_DIR}/*.ui")  
# select all files for the graphical interface
#FILE(GLOB EIGEN3 "/usr/include/eigen3/Eigen/*.h")  


# this is necessary to find in the code the ui____.h file built in the build folder
include_directories(${PCL_upd_DEMO_BINARY_DIR}) 
include_directories(${upd_lib_INCLUDE_DIR}) 

# --------------------------------------
#  VTK 6.3 Setup
# --------------------------------------
#set(VTK_DIR "/home/osboxes/vtk-6.3/vtk")
set(VTK_DIR "/home/tarcisio/vtk-5.8/vtk")
find_package(VTK 5.8)
set(VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)# vtkRendering vtkGraphics vtkHybrid QVTK)  #--> the specific libraries are not necessary in VTK6.1
#message(STATUS vtkRenderingFreeTypeOpenGL)
include(${VTK_USE_FILE})
#add_definitions(${VTK_DEFINITIONS})
message(STATUS "VTK_KITS--- ${VTK_KITS}")
message(STATUS "VTK_LIBRARY_DIRS--- ${VTK_LIBRARY_DIRS}")
message(STATUS "VTK_FOUND --- ${VTK_FOUND}")
message("*** VTK setup done ***")

# --------------------------------------
# PCL 1.7 Setup
# --------------------------------------
find_package(PCL 1.7 REQUIRED)
#include_directories("/usr/include/eigen3/Eigen/")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message("*** PCL setup done ***")

# --------------------------------------
# Qt 4 Setup
# --------------------------------------
#set(Qt4_DIR "/usr/local/Trolltech/Qt-4.8.6/bin/")
#set(QT_QMAKE_EXECUTABLE "/usr/local/Trolltech/Qt-4.8.6/bin/qmake")
message(STATUS "QT_QTCORE_INCLUDE_DIR >> ${QT_QTCORE_INCLUDE_DIR}")
find_package(Qt4 REQUIRED QtCore QtNetwork QtGui QtOpenGL QtWebkit)
link_directories(${QT_LIBRARY_DIR})
link_directories(${QT_BINARY_DIR})
add_definitions(${QT_DEFINITIONS})   #required to use QT and gui
include(${QT_USE_FILE})
set(QT_QMAKE_EXECUTABLE)
QT4_WRAP_CPP (PCL_upd_DEMO_HEADERS_MOC   ${PCL_upd_DEMO_HEADERS})
QT4_WRAP_UI  (PCL_upd_DEMO_FORMS_HEADERS ${PCL_upd_DEMO_FORMS})
message("*** QT setup done ***")
# --------------------------------------------------
# App Setup
# --------------------------------------------------
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable (PCL_upd_DEMO  	
                ${PCL_upd_DEMO_SOURCES} 
                ${PCL_upd_DEMO_HEADERS}
                ${PCL_upd_DEMO_FORMS}
		#${PCL_upd_DEMO_FORMS_HEADERS}
		#${PCL_upd_DEMO_HEADERS_MOC}
                #${PCL_INCLUDE_DIRS}
                ${PCL_ML}
                )

target_link_libraries (PCL_upd_DEMO
                       Qt4::QtGui 
                       QVTK
                       ${Boost_LIBRARIES}
                       ${PCL_LIBRARIES}
                       "/home/tarcisio/NetBeansProjects/OpenTraversability/libs/upd_lib/build/libupd_lib.so"
                       #"/home/tarcisio/pcl-1.8/pcl/build/lib/libpcl_ml.so"
                       "/usr/local/lib/libpcl_ml.so"
                       #"/usr/local/lib/vtk-5.8/libvtkRendering.so"
                       #"/home/osboxes/vtk-6.3/vtk/lib/libvtkRenderingCore-6.3.so.1"
                       #"/home/osboxes/vtk-6.3/vtk/lib/libvtkRenderingContextOpenGL-6.3.so.1"
                        )

# allows folders for MSVC
if (MSVC AND ENABLE_SOLUTION_FOLDERS) 
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Apps")
endif (MSVC AND ENABLE_SOLUTION_FOLDERS)