cmake_minimum_required(VERSION 2.8.3)
project(open_abb_driver)

set( CMAKE_CXX_FLAGS "-g -Wall -std=c++11" )
set( CMAKE_BUILD_TYPE Release )

find_package( catkin 	REQUIRED
	COMPONENTS 			roscpp
						message_generation
						tf )

add_service_files(
	FILES 			GetCartesian.srv
					GetJoints.srv
					Ping.srv
					SetCartesian.srv
					SetDIO.srv
					SetJoints.srv
					SetSpeed.srv
					SetTool.srv
					SetWorkObject.srv
					SetZone.srv
)

generate_messages()

catkin_package(
	INCLUDE_DIRS	include
	
	LIBRARIES 		matvec
					abb_comm
					
	CATKIN_DEPENDS 	roscpp
					tf
)

include_directories( include
					 ${catkin_INCLUDE_DIRS} )

add_library( matvec 
	src/matvec/HomogTransf.cpp
	src/matvec/Mat.cpp
	src/matvec/Polynom.cpp
	src/matvec/Quaternion.cpp
	src/matvec/RotMat.cpp
	src/matvec/Vec.cpp )

add_library( abb_comm 
			 src/ABBProtocol.cpp
			 src/ABBControlInterface.cpp
			 src/ABBFeedbackInterface.cpp)
add_dependencies( abb_comm ${open_abb_driver_EXPORTED_TARGETS} )
target_link_libraries( abb_comm
	${catkin_LIBRARIES} )

add_executable( abb_node nodes/ABBNode.cpp )
add_dependencies( abb_node ${PROJECT_NAME}_gencfg )
target_link_libraries( abb_node abb_comm matvec )

install( TARGETS abb_comm matvec abb_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

install( DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
