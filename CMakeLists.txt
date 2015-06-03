cmake_minimum_required(VERSION 2.8.3)
project(open_abb_driver)

set( CMAKE_CXX_FLAGS "-g -Wall -std=c++11" )
set( CMAKE_BUILD_TYPE Release )

find_package( catkin 	REQUIRED
	COMPONENTS 			roscpp
						message_generation
						tf )

find_package(Eigen3 REQUIRED)

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
					SetSoftness.srv
)

generate_messages()

catkin_package(
	INCLUDE_DIRS	include
	
	LIBRARIES 		abb_comm
					abb2400_ikfast
					
	CATKIN_DEPENDS 	roscpp
					tf
					
	DEPENDS 		Eigen3
)

include_directories( include
					 ${EIGEN3_INCLUDE_DIR}
					 ${catkin_INCLUDE_DIRS} )

add_library( abb_comm 
			 src/ABBProtocol.cpp
			 src/ABBControlInterface.cpp
			 src/ABBFeedbackInterface.cpp )

add_library( abb2400_ikfast
			 src/IKFastSolver.cpp
			 src/ABBKinematics.cpp
			 src/PoseSE3.cpp )
			 
add_dependencies( abb_comm ${open_abb_driver_EXPORTED_TARGETS} )
target_link_libraries( abb_comm
	abb2400_ikfast
	${catkin_LIBRARIES} )

add_executable( abb_node nodes/ABBNode.cpp )
add_dependencies( abb_node ${PROJECT_NAME}_gencfg )
target_link_libraries( abb_node abb_comm )

install( TARGETS abb_comm abb_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

install( DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
