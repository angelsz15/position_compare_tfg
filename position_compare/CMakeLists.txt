cmake_minimum_required(VERSION 3.0.2)
project(position_compare)

## Buscar dependencias catkin
## Indicar las dependencias del paquete
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  eigen_conversions
)

find_package(Boost REQUIRED COMPONENTS system)


find_package(Eigen3 REQUIRED)

## Declarar el catkin package
## Indicar archivos y directorios a compilar
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_subscriber_node
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs eigen_conversions
  #DEPENDS system_lib
)

## Especificar el tipo de ejecutables
## Declarar ejecutables o nodos
## Agregar dependencias
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}	
)

## Especificar el nombre del ejecutable y los archivos fuente
## Agregar dependencias
add_executable(position_compare src/position_compare.cpp)

add_dependencies(position_compare ${position_compare_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Enlazar bibliotecas y ejecutables
## Agregar dependencias
target_link_libraries(position_compare
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)