//
// Robot Node
//
// This node handles communication with the ABB robot. It serves 2 main
// functions. First, it communicates with the logging task on the ABB
// robot and broadcasts position and force information on topics. Second,
// it allows the user to control the robot, and allows for both blocking
// and non-blocking moves.
//

#include "abb_node.h"

/////////////////////////////////
// BEGIN RobotController Class //
/////////////////////////////////

RobotController::RobotController(ros::NodeHandle *n) 
{
  node = n;
}

RobotController::~RobotController() {
  /// Shut down services.
  handle_Ping.shutdown();
  handle_SetCartesian.shutdown();
  handle_GetCartesian.shutdown();
  handle_SetJoints.shutdown();
  handle_GetJoints.shutdown();
  handle_Stop.shutdown();
  handle_SetTool.shutdown();
  handle_SetWorkObject.shutdown();
  handle_SetSpeed.shutdown();
  handle_SetZone.shutdown();
  handle_SetTrackDist.shutdown();
  handle_SetComm.shutdown();
  handle_SetVacuum.shutdown();
  handle_SetDIO.shutdown();
  handle_SpecialCommand.shutdown();

  // Shut down topics.
  handle_CartesianLog.shutdown();
  handle_JointsLog.shutdown();
  handle_ForceLog.shutdown();

  //Close connections
  char message[MAX_BUFFER];
  strcpy(message, abb_comm::closeConnection().c_str());
  send(robotMotionSocket, message, strlen(message), 0);
  ros::Duration(1.0).sleep();
  close(robotMotionSocket);
  close(robotLoggerSocket);
}

// This method initializes the robot controller by connecting to the 
// Robot Motion and Robot Logger servers, setting up internal variables
// and setting the default robot configuration
bool RobotController::init()
{
  std::string robotIp;
  int robotMotionPort;
  int robotLoggerPort;

  motionConnected = false;
  loggerConnected = false;

  //Connection to Robot Motion server
  ROS_INFO("ROBOT_CONTROLLER: Connecting to the ABB motion server...");
  node->getParam("robot/robotIp",robotIp);
  node->getParam("robot/robotMotionPort",robotMotionPort);
  connectMotionServer(robotIp.c_str(), robotMotionPort);
  if(!motionConnected)
  {
    return false; 
  }

  //Connect to Robot Logger server
  ROS_INFO("ROBOT_CONTROLLER: Connecting to the ABB logger server...");
  node->getParam("robot/robotLoggerPort",robotLoggerPort);
  connectLoggerServer(robotIp.c_str(), robotLoggerPort);
  if(!loggerConnected)
  {
    ROS_INFO("ROBOT_CONTROLLER: Not able to connect to logger server. "
        "Continuing without robot feedback.");
  }

  // Setup our non-blocking variables
  non_blocking = false;
  do_nb_move = false;
  targetChanged = false;
  stopRequest = false;
  stopConfirm = false;
  changing_nb_speed = false;

  // Allocate space for all of our vectors
  curToolP = Vec(3);
  curWorkP = Vec(3);
  curP = Vec(3);
  curGoalP = Vec(3);
  curTargP = Vec(3);

  // Set the Default Robot Configuration
  ROS_INFO("ROBOT_CONTROLLER: Setting robot default configuration...");
  if(!defaultRobotConfiguration())
  {
    ROS_INFO("ROBOT_CONTROLLER: Not able to set the robot to "
        "default configuration.");
    return false;
  }

  return true;
}

// This function initializes the default configuration of the robot.
// It sets the work object, tool, zone, speed, and vacuum based on
// default parameters from the ROS parameter file
bool RobotController::defaultRobotConfiguration()
{
  double defWOx,defWOy,defWOz,defWOq0,defWOqx,defWOqy,defWOqz;
  double defTx,defTy,defTz,defTq0,defTqx,defTqy,defTqz;
  int zone;
  double speedTCP, speedORI;
  int vacuumMode;

  //WorkObject
  node->getParam("robot/workobjectX",defWOx);
  node->getParam("robot/workobjectY",defWOy);
  node->getParam("robot/workobjectZ",defWOz);
  node->getParam("robot/workobjectQ0",defWOq0);
  node->getParam("robot/workobjectQX",defWOqx);
  node->getParam("robot/workobjectQY",defWOqy);
  node->getParam("robot/workobjectQZ",defWOqz);
  if (!setWorkObject(defWOx,defWOy,defWOz,defWOq0,defWOqx,defWOqy,defWOqz))
    return false;

  //Tool
  node->getParam("robot/toolX",defTx);
  node->getParam("robot/toolY",defTy);
  node->getParam("robot/toolZ",defTz);
  node->getParam("robot/toolQ0",defTq0);
  node->getParam("robot/toolQX",defTqx);
  node->getParam("robot/toolQY",defTqy);
  node->getParam("robot/toolQZ",defTqz);
  if (!setTool(defTx,defTy,defTz,defTq0,defTqx,defTqy,defTqz))
    return false;

  //Zone
  node->getParam("robot/zone",zone);
  if (!setZone(zone))
    return false;

  //Speed
  node->getParam("robot/speedTCP",speedTCP);
  node->getParam("robot/speedORI",speedORI);
  if (!setSpeed(speedTCP, speedORI))
    return false;

  //Vacuum
  node->getParam("robot/vacuum",vacuumMode);
  if (!setVacuum(vacuumMode))
    return false;

  // If everything is set, our default configuration has been set up correctly
  return true;
}


//////////////////////////////////////////////////////////////////////////////
// Advertise Services and Topics
//////////////////////////////////////////////////////////////////////////////

// Advertise the topics that the robot node will be broadcasting
void RobotController::advertiseTopics()
{
  handle_CartesianLog = 
    node->advertise<geometry_msgs::PoseStamped>("pose", 100);
  handle_JointsLog = 
    node->advertise<sensor_msgs::JointState>("jointstate", 100);
  handle_ForceLog = 
    node->advertise<geometry_msgs::WrenchStamped>("wrench", 100);
}

// Advertise the services that the robot will be listening for
void RobotController::advertiseServices()
{
  handle_Ping = node->advertiseService("Ping", 
      &RobotController::robot_Ping, this);
  handle_SetCartesian = node->advertiseService("SetCartesian", 
      &RobotController::robot_SetCartesian, this);
  handle_GetCartesian = node->advertiseService("GetCartesian", 
      &RobotController::robot_GetCartesian, this);
  handle_SetJoints = node->advertiseService("SetJoints", 
      &RobotController::robot_SetJoints, this);
  handle_GetJoints = node->advertiseService("GetJoints", 
      &RobotController::robot_GetJoints, this);
  handle_Stop = node->advertiseService("Halt", 
      &RobotController::robot_Stop, this);
  handle_SetTool = node->advertiseService("SetTool", 
      &RobotController::robot_SetTool, this);
  handle_SetWorkObject = node->advertiseService("SetWorkObject", 
      &RobotController::robot_SetWorkObject, this);
  handle_SetSpeed = node->advertiseService("SetSpeed", 
      &RobotController::robot_SetSpeed, this);
  handle_SetZone = node->advertiseService("SetZone", 
      &RobotController::robot_SetZone, this);
  handle_SetTrackDist = node->advertiseService("SetTrackDist", 
      &RobotController::robot_SetTrackDist, this);
  handle_SetComm = node->advertiseService("SetComm", 
      &RobotController::robot_SetComm, this);
  handle_SpecialCommand = node->advertiseService("SpecialCommand",
      &RobotController::robot_SpecialCommand, this);
  handle_SetVacuum = node->advertiseService("SetVacuum", 
      &RobotController::robot_SetVacuum, this);
  handle_SetDIO = node->advertiseService("SetDIO", 
      &RobotController::robot_SetDIO, this);
  handle_IsMoving = node->advertiseService("IsMoving", 
      &RobotController::robot_IsMoving, this);
}


//////////////////////////////////////////////////////////////////////////////
// Service Callbacks
//////////////////////////////////////////////////////////////////////////////

// Simply pings the robot and makes sure we can still communicate with it
bool RobotController::robot_Ping(abb_node::robot_Ping::Request& req, 
    abb_node::robot_Ping::Response& res)
{
  if (ping())
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to ping the robot.";
    return false;
  }
}

// Moves the robot to a given cartesian position. If we are currently in
// non-blocking mode, then we simply setup the move and let the non-blocking
// thread handle the actual moving. If we are in blocking mode, we then 
// communicate with the robot to execute the move.
bool RobotController::robot_SetCartesian(
    abb_node::robot_SetCartesian::Request& req, 
    abb_node::robot_SetCartesian::Response& res)
{
  /*
  ROS_INFO("CMD: %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, mag = %f", 
      req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz, 
      sqrt(req.q0*req.q0 + req.qx*req.qx + req.qy*req.qy + req.qz*req.qz));
      */
  // If we are in non-blocking mode
  if (non_blocking)
  {
    // As we are changing some state variables, mutex this block to be sure
    // we don't get unpredictable actions
    pthread_mutex_lock(&nonBlockMutex);
    if (!do_nb_move)
    {
      // If no move is currently being executed, we will now be doing
      // a cartesian move.
      cart_move = true;

      // Our new target is simply the target passed by the user
      curTargP[0] = req.cartesian[0];
      curTargP[1] = req.cartesian[1];
      curTargP[2] = req.cartesian[2];
      curTargQ[0] = req.quaternion[0];
      curTargQ[1] = req.quaternion[1];
      curTargQ[2] = req.quaternion[2];
      curTargQ[3] = req.quaternion[3];

      // The last goal in the non-blocking move is simply the current position
      getCartesian(curGoalP[0], curGoalP[1], curGoalP[2],
          curGoalQ[0], curGoalQ[1], curGoalQ[2], curGoalQ[3]);

      // We are now ready to execute this non-blocking move
      do_nb_move = true;
    }
    else if (cart_move)
    {
      // If we are currently doing a non-blocking cartesian move, we simply
      // need to update our current target
      curTargP[0] = req.cartesian[0];
      curTargP[1] = req.cartesian[1];
      curTargP[2] = req.cartesian[2];
      curTargQ[0] = req.quaternion[0];
      curTargQ[1] = req.quaternion[1];
      curTargQ[2] = req.quaternion[2];
      curTargQ[3] = req.quaternion[3];
      
      // Remember that we changed our target, again to maintain concurrency
      targetChanged = true;
    }
    else
    {
      // If we are doing a non-blocking move, but it's a joint move,
      // it would be much too dangerous to switch
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Can't do a cartesian move while doing a ";
      res.msg += "non-blocking joint move!";
      return false;
    }
    pthread_mutex_unlock(&nonBlockMutex);
  }
  else
  {
    // If we are in blocking mode, simply execute the cartesian move
    if (!setCartesian(req.cartesian[0], req.cartesian[1], req.cartesian[2],
		      req.quaternion[0], req.quaternion[1], req.quaternion[2], req.quaternion[3]))
    {
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Not able to set cartesian coordinates ";
      res.msg += "of the robot.";
      return false;
    }
  }
  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}


// Queries the cartesian position of the robot
bool RobotController::robot_GetCartesian(
    abb_node::robot_GetCartesian::Request& req, 
    abb_node::robot_GetCartesian::Response& res)
{
  // Simply call our internal method to communicate with the robot and get
  // the cartesian position
  if (getCartesian(res.x, res.y, res.z, res.q0, res.qx, res.qy, res.qz))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to get cartesian coordinates of ";
    res.msg += "the robot.";
    return false;
  }
}


// Moves the robot to a given joint position. If we are currently in
// non-blocking mode, then we simply setup the move and let the non-blocking
// thread handle the actual moving. If we are in blocking mode, we then 
// communicate with the robot to execute the move.
bool RobotController::robot_SetJoints(
    abb_node::robot_SetJoints::Request& req, 
    abb_node::robot_SetJoints::Response& res)
{
  if (req.position.size() != NUM_JOINTS)
    {
      return false;
    }
  // If we are in non-blocking mode
  if (non_blocking)
  {
    // As we are changing state variables that are used in another thread,
    // be sure to mutex this if-block so we don't have any unanticipated
    // results
    pthread_mutex_lock(&nonBlockMutex);
    if (!do_nb_move)
    {
      // If we are not currently doing a move, we are now 
      // going to start a joint move
      cart_move = false;
      
      // Our new target is just the goal specified by the user
      for (uint i = 0; i < NUM_JOINTS; i++)
	{
	  curTargJ[i] = req.position[i];
	}

      // Our previous goal is simply the current position
      //getJoints(curGoalJ[0], curGoalJ[1], curGoalJ[2], 
      //	curGoalJ[3], curGoalJ[4], curGoalJ[5]);

      // Now that we have set everything up, execute the move
      do_nb_move = true;
    }
    else if (!cart_move)
    {
      // Otherwise, we are currently doing a joint move, and we need to update
      // our target position
      
      for (uint i = 0; i < NUM_JOINTS; i++)
	{
	  curTargJ[i] = req.position[i];
	}


      // Remember that we changed our position, again to make sure we don't
      // have unexpected results
      targetChanged = true;
    }
    else
    {
      // If we are in the middle of a non-blocking cartesian move, we 
      // cannot execute a joint move
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Can't do a joint move while doing a ";
      res.msg += "non-blocking cartesian move!";
      return false;
    }
    pthread_mutex_unlock(&nonBlockMutex);
  }
  else
  {
    // If we are in blocking mode, simply execute the entire joint move
    if (!setJoints(&req.position[0]))
    {
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Not able to set cartesian coordinates ";
      res.msg += "of the robot.";
      return false;
    }
  }

  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}

// Query the robot for the current position of its joints
bool RobotController::robot_GetJoints(
    abb_node::robot_GetJoints::Request& req, 
    abb_node::robot_GetJoints::Response& res)
{
  // Simply call our internal method to get the current position of the robot
  if (getJoints(res.j1, res.j2, res.j3, res.j4, res.j5, res.j6))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to get Joint coordinates of ";
    res.msg += "the robot.";
    return false;
  }
}

// If the robot is in non-blocking mode, stop the robot
bool RobotController::robot_Stop(
    abb_node::robot_Stop::Request& req, 
    abb_node::robot_Stop::Response& res)
{
  // If we are currently blocking, there's nothing to stop
  if(!non_blocking)
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Only able to stop the robot in ";
    res.msg += "non-blocking mode.";
    ROS_INFO("ROBOT_CONTROLLER: Stop command not sent. "
        "Only allowed in non-blocking mode.");
    return false;
  }
  else
  {
    // Otherwise, call our internal stop method
    stop_nb();
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
}

// Set the tool frame of the robot
bool RobotController::robot_SetTool(
    abb_node::robot_SetTool::Request& req, 
    abb_node::robot_SetTool::Response& res)
{
  // Simply call our internal method to set the tool
  if (setTool(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the tool of the robot.";
    return false;
  }
}

// Set the work object of the robot
bool RobotController::robot_SetWorkObject(
    abb_node::robot_SetWorkObject::Request& req, 
    abb_node::robot_SetWorkObject::Response& res)
{
  // Simply call our internal method to set the work object
  if(setWorkObject(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the work ";
    res.msg += "object of the robot.";
    ROS_WARN("ABB_NODE: %s",res.msg.c_str());
    return false;
  }
}

// Set the communication mode of our robot
bool RobotController::robot_SetComm(
    abb_node::robot_SetComm::Request& req, 
    abb_node::robot_SetComm::Response& res)
{
  if (req.mode == NON_BLOCKING)
  {
    // If the user wants to set non-blocking mode, we 
    // simply set our state variables appropriately
    if (!non_blocking)
    {
      // our mode is now non-blocking, but we are not yet executing a motion
      do_nb_move = false;
      non_blocking = true;

      // Set up step sizes based on the current speed
      setNonBlockSpeed(curSpd[0], curSpd[1]);
    }
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: Communication set to NON_BLOCKING.";
    return true;
  }
  else if (req.mode == BLOCKING)
  {
    // If the user wants to set blocking mode, make sure we stop any 
    // non-blocking move, and then set our state variable appropriately.
    if (non_blocking)
    {
      stop_nb();
      non_blocking = false;
    }
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: Communication set to BLOCKING.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Invalide communication mode ";
    res.msg += "(0-Nonblocking 1-Blocking).";
    ROS_INFO("ROBOT_CONTROLLER: SetComm command not sent. "
        "Invalid communication mode.");
    return false;
  }
}

// Set the speed of our robot. Note that if we are in non-blocking mode, we 
// call a separate method which sets step sizes in addition to speed.
// Otherwise, we just call our generic setSpeed method.
bool RobotController::robot_SetSpeed(
    abb_node::robot_SetSpeed::Request& req, 
    abb_node::robot_SetSpeed::Response& res)
{
  if (non_blocking)
  {
    if (!setNonBlockSpeed(req.tcp, req.ori))
    {
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Unable to change non-blocking ";
      res.msg += "speed of the robot.";
      return false;
    }
  }
  else if (!setSpeed(req.tcp, req.ori))
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the speed of the robot.";
    return false;
  }

  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}

// Set the zone of the robot. This is the distance before the end of a motion 
// that the server will respond. This enables smooth motions.
bool RobotController::robot_SetZone(
    abb_node::robot_SetZone::Request& req, 
    abb_node::robot_SetZone::Response& res)
{
  // Simply call our internal setZone method
  if (setZone(req.mode))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the zone of the robot.";
    return false;
  }
}

bool RobotController::robot_SetTrackDist(
    abb_node::robot_SetTrackDist::Request& req,
    abb_node::robot_SetTrackDist::Response& res)
{
  if (!non_blocking)
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: SetTrackDist only applies for non-blocking.";
    return false;
  }
  else if (setTrackDist(req.pos_dist, req.ang_dist))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the tracking distance ";
    res.msg += "of the robot.";
    return false;
  }
}

// Execute Special Command
bool RobotController::robot_SpecialCommand(
    abb_node::robot_SpecialCommand::Request& req, 
    abb_node::robot_SpecialCommand::Response& res)
{
  // Simply call our internal method
  if (specialCommand(req.command, req.param1, req.param2, req.param3, req.param4, req.param5))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to execute special command.";
    return false;
  }
}

// Turn the Vacuum on or off on the robot
bool RobotController::robot_SetVacuum(
    abb_node::robot_SetVacuum::Request& req, 
    abb_node::robot_SetVacuum::Response& res)
{
  // Simply call our internal method
  if (setVacuum(req.vacuum))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to toggle the vacuum state.";
    return false;
  }
}


// Lock/Unlock tool changer
bool RobotController::robot_SetDIO(
				   abb_node::robot_SetDIO::Request& req, 
				   abb_node::robot_SetDIO::Response& res)
{
  // Simply call our internal method
  if (setDIO(req.DIO_num, req.state))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to toggle the vacuum state.";
    return false;
  }
}


bool RobotController::setTrackDist(double pos_dist, double ang_dist)
{
  pthread_mutex_lock(&nonBlockMutex);
  
  // We set the tracking distance and adapt the speed of the robot
  // in order to avoid jerkiness.
  double desiredSpeedTCP;
  double desiredSpeedORI;
  
  //TRANSLATION
  //There is a minimum and maximum tracking distance
  if(pos_dist < MINIMUM_TRACK_DIST_TRANS)
    pos_dist = MINIMUM_TRACK_DIST_TRANS;
  if(pos_dist > MAXIMUM_TRACK_DIST_TRANS)
    pos_dist =  MAXIMUM_TRACK_DIST_TRANS;
  curDist[0] = pos_dist;
  curCartStep = curDist[0]/2.0;
  desiredSpeedTCP = 3.0*pos_dist*SAFETY_FACTOR;
  
  //ROTATION
  //There is a minimum and maximum tracking distance
  if(ang_dist < MINIMUM_TRACK_DIST_ORI)
    ang_dist = MINIMUM_TRACK_DIST_ORI;
  if(ang_dist > MAXIMUM_TRACK_DIST_ORI)
    ang_dist =  MAXIMUM_TRACK_DIST_ORI;
  curDist[1] = ang_dist * DEG2RAD;
  curOrientStep = curDist[1]/2.0;
  desiredSpeedORI = 3.0*ang_dist*SAFETY_FACTOR;
  
  //JOINTS
  //There is a minimum and maximum tracking distance
  curDist[2] = ang_dist;
  curJointStep = curDist[2]/2.0;   

  //Change speed in the ABB controller
  changing_nb_speed = true;
  setSpeed(desiredSpeedTCP, desiredSpeedORI);
  changing_nb_speed = false;

  pthread_mutex_unlock(&nonBlockMutex);
  return true;
}

bool RobotController::robot_IsMoving(
    abb_node::robot_IsMoving::Request& req, 
    abb_node::robot_IsMoving::Response& res)
{
  res.moving = is_moving();

  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}

bool RobotController::setNonBlockSpeed(double tcp, double ori)
{
  pthread_mutex_lock(&nonBlockMutex);

  double desiredSpeedTCP;
  double desiredSpeedORI;
  
  //TRANSLATION
  //There is a minimum speed
  if(tcp < MINIMUM_NB_SPEED_TCP)
    tcp = MINIMUM_NB_SPEED_TCP;
  desiredSpeedTCP = tcp;
  if(curSpd[0]<60.0)
    curDist[0] = (desiredSpeedTCP/3.0)/SAFETY_FACTOR;
  else
    curDist[0] = INFINITY_TRACK_DIST_TRANS;
  curCartStep = curDist[0]/2.0;
 
  //ROTATION
  if(ori < MINIMUM_NB_SPEED_ORI)
    ori = MINIMUM_NB_SPEED_ORI;
  desiredSpeedORI = ori;
  if(curSpd[1]<20.0)
    curDist[1] = (desiredSpeedORI/3.0)/SAFETY_FACTOR;
  else
    curDist[1] = INFINITY_TRACK_DIST_ORI;
  curOrientStep = curDist[1]/2.0;

  //JOINT
  curDist[2] = curDist[1];
  curJointStep = curOrientStep;

  //Change speed in the ABB controller
  changing_nb_speed = true;
  setSpeed(desiredSpeedTCP, desiredSpeedORI);
  changing_nb_speed = false;

  pthread_mutex_unlock(&nonBlockMutex);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
// Internal methods that let us execute certain robot functions without using 
// ROS service callbacks
//////////////////////////////////////////////////////////////////////////////

// Pings the robot
bool RobotController::ping()
{
  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
  strcpy(message, abb_comm::pingRobot(randNumber).c_str());

  if(sendAndReceive(message, strlen(message), reply, randNumber))
    return true;
  else
    return false;
}

// Command the robot to move to a given cartesian position
bool RobotController::setCartesian(double x, double y, double z, 
    double q0, double qx, double qy, double qz)
{
  // We will do some collision checking and sanity checking here
  // some time in the future
  
  /*
  // For debugging purposes
  ROS_INFO("ACTION: %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, mag = %f", 
      x, y, z, q0, qx, qy, qz, sqrt(q0*q0+qx*qx+qy*qy+qz*qz));*/

  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));

  strcpy(message, abb_comm::setCartesian(x, y, z, q0, qx, qy, qz, 
        randNumber).c_str());

  if (sendAndReceive(message, strlen(message), reply, randNumber))
  {
    // If this was successful, keep track of the last commanded position
    curGoalP[0] = x;
    curGoalP[1] = y;
    curGoalP[2] = z;
    curGoalQ[0] = q0;
    curGoalQ[1] = qx;
    curGoalQ[2] = qy;
    curGoalQ[3] = qz;
    return true;
  }
  else
    return false;
}

// Query the robot for the current position of the robot
bool RobotController::getCartesian(double &x, double &y, double &z, 
    double &q0, double &qx, double &qy, double &qz)
{
  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
  strcpy(message, abb_comm::getCartesian(randNumber).c_str());
  
  if(sendAndReceive(message, strlen(message), reply, randNumber))
    {
      // Parse the reply to get the cartesian coordinates
      abb_comm::parseCartesian(reply, &x, &y, &z, &q0, &qx, &qy, &qz);
      return true;
    }
  else
    {
      return false;
    }
}

// Command the robot to move to a given joint configuration
bool RobotController::setJoints(double position[])
{
  // We will do some collision and sanity checks here

  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));

  //ABB robots accept joint positions in degrees, so we convert from radians
  strcpy(message, abb_comm::setJoints(position[0]*57.2957795, 
				      position[1]*57.2957795,
				      position[2]*57.2957795,
				      position[3]*57.2957795,
				      position[4]*57.2957795,
				      position[5]*57.2957795,
				      randNumber).c_str());
  
  if (sendAndReceive(message, strlen(message), reply, randNumber))
  {
    // If the move was successful, keep track of the last commanded position
    for (int i=0; i < NUM_JOINTS; i++)
      {
	curGoalJ[i] = position[i];
      }
    return true;
  }
  else
    return false;
}

// Query the robot for the current joint positions
bool RobotController::getJoints(double &j1, double &j2, double &j3,
    double &j4, double &j5, double &j6)
{
  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
  strcpy(message, abb_comm::getJoints(randNumber).c_str());

  if(sendAndReceive(message, strlen(message), reply, randNumber))
  {
    // Parse the reply to get the joint angles
    abb_comm::parseJoints(reply, &j1, &j2, &j3, &j4, &j5, &j6);
    return true;
  }
  else
    return false;
}

// Stop the robot while it is in non_blocking mode
bool RobotController::stop_nb()
{
  // This function doesn't apply when we are in blocking mode
  if (!non_blocking)
    return false;

  // If the robot is currently not moving, we're done.
  if (!do_nb_move)
    return true;

  ros::Rate stop_check(STOP_CHECK_FREQ);

  // If we are currently non-blocking, get out of non-blocking move and wait
  // to return until we're sure that the robot has stopped moving.
  do_nb_move = false;

  // We wait for the non-blocking thread to set stopConfirm to 
  // true after we set stopRequest to true
  stopConfirm = false;
  stopRequest = true;
  while (!stopConfirm)
  {
    stop_check.sleep();
  }
  stopRequest = false; 

  return true;
}

// Set the tool frame of the robot
bool RobotController::setTool(double x, double y, double z, 
    double q0, double qx, double qy, double qz)
{
  // Only take action if the required values are different than the actual ones
  if(x!=curToolP[0] || y!=curToolP[1] || y!=curToolP[2] || 
     q0!=curToolQ[0] || qx!=curToolQ[1] || qy!=curToolQ[2] || qz!=curToolQ[3])
    {
      // This is dangerous if we are currently executing a non-blocking move
      if (do_nb_move)
	return false;
      
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, abb_comm::setTool(x, y, z, q0, qx, qy, qz, 
					      randNumber).c_str());

      if(sendAndReceive(message, strlen(message), reply, randNumber))
	{
	  // If the command was successful, remember our new tool frame
	  curToolP[0] = x;
	  curToolP[1] = y;
	  curToolP[2] = z;
	  curToolQ[0] = q0;
	  curToolQ[1] = qx;
	  curToolQ[2] = qy;
	  curToolQ[3] = qz;
	  // We also save it to the parameter server
	  node->setParam("robot/toolX",x);
	  node->setParam("robot/toolY",y);
	  node->setParam("robot/toolZ",z);
	  node->setParam("robot/toolQ0",q0);
	  node->setParam("robot/toolQX",qx);
	  node->setParam("robot/toolQY",qy);
	  node->setParam("robot/toolQZ",qz);
	  return true;
	}
      else
	return false;
    }
  return true;
}

// Set the work object frame of the robot 
bool RobotController::setWorkObject(double x, double y, double z, 
    double q0, double qx, double qy, double qz)
{
  // Only take action if the required values are different than the actual ones
  if(x!=curWorkP[0] || y!=curWorkP[1] || y!=curWorkP[2] || 
     q0!=curWorkQ[0] || qx!=curWorkQ[1] || qy!=curWorkQ[2] || qz!=curWorkQ[3])
    {
      // This is dangerous if we are in the middle of a non-blocking move
      if (do_nb_move)
	return false;
      
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, abb_comm::setWorkObject(x, y, z, q0, qx, qy, qz, 
						    randNumber).c_str());
      
      if(sendAndReceive(message, strlen(message), reply, randNumber))
	{
	  // If the command was successful, remember our new work object
	  curWorkP[0] = x;
	  curWorkP[1] = y;
	  curWorkP[2] = z;
	  curWorkQ[0] = q0;
	  curWorkQ[1] = qx;
	  curWorkQ[2] = qy;
	  curWorkQ[3] = qz;

	  //This transform is published from a callback every time an updated TCP is recieved from the robot
	  //So we need to lock the transform here
	  pthread_mutex_lock(&wobjUpdateMutex);
	  curWobjTransform.setOrigin(tf::Vector3(x*.001, y*.001, z*.001));
	  curWobjTransform.setRotation(tf::Quaternion(qx, qy, qz, q0));
	  pthread_mutex_unlock(&wobjUpdateMutex);


	  // We also save it to the parameter server
	  node->setParam("robot/workobjectX",x);
	  node->setParam("robot/workobjectY",y);
	  node->setParam("robot/workobjectZ",z);
	  node->setParam("robot/workobjectQ0",q0);
	  node->setParam("robot/workobjectQX",qx);
	  node->setParam("robot/workobjectQY",qy);
	  node->setParam("robot/workobjectQZ",qz);
	  return true;
	}
      else
	return false;
    }
  return true;
}

// Set the speed of the robot
bool RobotController::setSpeed(double tcp, double ori)
{
  // Only take action if the required values are different than the actual ones
  if(tcp!=curSpd[0] || ori!=curSpd[1])
    {
      // This is dangerous if we are currently executing a non-blocking move
      // (unless we're sure and set changing_nb_speed to true)
      if (!changing_nb_speed && do_nb_move)
	return false;

      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, abb_comm::setSpeed(tcp, ori, 
					       randNumber).c_str());
      if(sendAndReceive(message, strlen(message), reply, randNumber))
	{
	  // If we successfully changed the speed, remember our new speed values
	  curSpd[0] = tcp;
	  curSpd[1] = ori;
	  // We also save it to the parameter server
	  node->setParam("robot/speedTCP",tcp);
	  node->setParam("robot/speedORI",ori);
	  return true;
	}
      else
	return false;
    }
  return true;
}

// Set the zone of our robot
bool RobotController::setZone(int z)
{
 // Only take action if the required values are different than the actual ones
  if(z!=curZone)
    {
      // This is dangerous if we are in the middle of a non-blocking move
      if (do_nb_move)
	return false;
      
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      
      // Make sure the specified zone number exists
      if (z < 0 || z > NUM_ZONES)
	{
	  ROS_INFO("ROBOT_CONTROLLER: SetZone command not sent. Invalide zone mode.");
	  return false;
	}
      
      strcpy(message, abb_comm::setZone((z == ZONE_FINE), 
					      zone_data[z].p_tcp, zone_data[z].p_ori, zone_data[z].ori, 
					      randNumber).c_str());
      
      if(sendAndReceive(message, strlen(message), reply, randNumber))
	{
	  // If we set the zone successfully, remember our new zone
	  curZone = z;
	  // We also save it to the parameter server
	  node->setParam("robot/zone",z);
	  return true;
	}
      else
	return false;
    }
  return true;
}

// Call Special Command
bool RobotController::specialCommand(int command, double param1, double param2, double param3, double param4, double param5)
{
  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
  strcpy(message, abb_comm::specialCommand(command, param1, param2, param3, param4, param5, randNumber).c_str());

  if(sendAndReceive(message, strlen(message), reply, randNumber))
    return true;
  else
    return false;
}

// Open or close the vacuum 0 is open, 1 is closed
bool RobotController::setVacuum(int v)
{
  // Only take action if the required values are different than the actual ones
  if(v!=curVacuum)
    {
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      
      // Make sure we are either opening or closing the vacuum
      if((v != VACUUM_OPEN) && (v != VACUUM_CLOSE))
	{
	  ROS_INFO("ROBOT_CONTROLLER: SetVacuum command not sent. "
		   "Invalid communication mode.");
	  return false;
	}
      
      strcpy(message, abb_comm::setVacuum(v, randNumber).c_str());
      
      if(sendAndReceive(message, strlen(message), reply, randNumber))
	{
	  // Remember the current state of the vacuum if the command sent
	  curVacuum = v;
	  // We also save it to the parameter server
	  node->setParam("robot/vacuum",v);
	  return true;
	}
      else
	return false;
    }
  return true;
}


bool RobotController::setDIO(int dio_num, int dio_state)
{
  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
  strcpy(message, abb_comm::setDIO(dio_num, dio_state, randNumber).c_str());
  if(sendAndReceive(message, strlen(message), reply, randNumber))
    { return true; }
  else 
    { return false; }
}






// If we are in non-blocking mode, then we have a variable that keeps 
//  track of whether we are moving or not. If we are in blocking mode,
//  reaching this function means the robot is not moving
bool RobotController::is_moving()
{
  if (non_blocking)
    return do_nb_move;
  else
    return false;
}

//////////////////////////////////////////////////////////////////////////////
// Connect to Servers on Robot
//////////////////////////////////////////////////////////////////////////////

// Connect to the logger server
bool RobotController::connectLoggerServer(const char* ip, int port)
{
  if(loggerConnected)
  {
    ROS_INFO("ROBOT_CONTROLLER: Logger already connected.");
    return false;
  }

  // First, create a logger socket
  if ((robotLoggerSocket = socket(PF_INET, SOCK_STREAM, 0)) == -1)
  {
    ROS_INFO("ROBOT_CONTROLLER: Problem creating the logger socket. "
        "Error number: %d.",errno);
  }
  else
  {
    // Try connecting to the logger server
    struct sockaddr_in remoteSocket;
    remoteSocket.sin_family = AF_INET;
    remoteSocket.sin_port = htons(port);
    inet_pton(AF_INET, ip, &remoteSocket.sin_addr.s_addr);
    if (connect(robotLoggerSocket, (sockaddr*)&remoteSocket,
          sizeof(remoteSocket)) == -1)
    {
      ROS_INFO("ROBOT_CONTROLLER: Could not connect to the ABB logger "
          "server. Error number: %d.",errno);
    }
    else
    {
      //Set socket as non-blocking
      int flags;
      flags = fcntl(robotLoggerSocket,F_GETFL,0);
      if(flags != -1)
      {
        if(fcntl(robotLoggerSocket, F_SETFL, flags | O_NONBLOCK) != -1)
        {
          loggerConnected = true;
          return true;
        }
        else
        {
          ROS_INFO("ROBOT_CONTROLLER: Could not set the logger socket "
              "as non-blocking");
        }
      }
      else
      {
        ROS_INFO("ROBOT_CONTROLLER: Could not set the logger socket "
            "as non-blocking");
      }
    }
  }
  return false;
}

// Connect to the robot motion server
bool RobotController::connectMotionServer(const char* ip, int port)
{
  if(motionConnected)
  {
    ROS_INFO("ROBOT_CONTROLLER: Robot controller already connected.");
    return false;
  }

  // Create a socket for robot motion
  if ((robotMotionSocket = socket(PF_INET, SOCK_STREAM, 0)) == -1)
    ROS_INFO("ROBOT_CONTROLLER: Problem creating the socket. %d",errno);
  else
  {
    // Now try to connect to the robot motion server
    struct sockaddr_in remoteSocket;
    remoteSocket.sin_family = AF_INET;
    remoteSocket.sin_port = htons(port);
    inet_pton(AF_INET, ip, &remoteSocket.sin_addr.s_addr);
    if(connect(robotMotionSocket, (sockaddr*)&remoteSocket,
          sizeof(remoteSocket)) == -1)
    {
      ROS_INFO("ROBOT_CONTROLLER: Could not connect to the "
          "ABB robot. %d",errno);
    }
    else
    {
      motionConnected = true;
      return true;
    }
  }
  return false;
}

// Helper function to send a command to the robot, 
// wait for an answer and check for correctness.
bool RobotController::sendAndReceive(char *message, int messageLength, 
    char* reply, int idCode)
{
  pthread_mutex_lock(&sendRecvMutex);
  if (send(robotMotionSocket, message, messageLength, 0) == -1)
  {
    ROS_WARN("ROBOT_CONTROLLER: Failed to send command to ABB robot:"
        " Error number %d.", errno);
  }
  else
  {
    // Read the reply to the message we just sent, and make sure
    // it's not corrupt, and the command was executed successfully
    int t;
    if ((t=recv(robotMotionSocket, reply, MAX_BUFFER-1, 0)) > 0)
    {
      reply[t] = '\0';
      int ok, rcvIdCode;
      sscanf(reply,"%*d %d %d", &rcvIdCode, &ok);
      if(idCode!=-1)
      {  
        if ((ok == SERVER_OK) && (rcvIdCode == idCode))
        {
          pthread_mutex_unlock(&sendRecvMutex);
          return true;
        }
        else if ((ok == SERVER_COLLISION) && (rcvIdCode == idCode))
        {
          ROS_WARN("ROBOT_CONTROLLER: Collision Detected.");
        }
        else
        {
          ROS_WARN("ROBOT_CONTROLLER: Corrupt message.");
          ROS_WARN("msg = %s, reply = %s, idCode = %d, ok = %d, rcvCode = %d", message, reply, idCode, ok, rcvIdCode);
        }
      }
      else
      {
        if (ok == SERVER_OK)
        {
          pthread_mutex_unlock(&sendRecvMutex);
          return true;
        }
        else if (ok == SERVER_COLLISION)
        {
          ROS_WARN("ROBOT_CONTROLLER: Collision Detected.");
        }
        else
        {
          ROS_WARN("ROBOT_CONTROLLER: Corrupt message.");
          ROS_WARN("msg = %s, reply = %s, idCode = %d, ok = %d, rcvCode = %d", message, reply, idCode, ok, rcvIdCode);
        }
      }
    }
    else
      ROS_WARN("ROBOT_CONTROLLER: Failed to receive answer from ABB robot.");
  }

  pthread_mutex_unlock(&sendRecvMutex);
  return false;
}

// Compute the distance between the current position and the goal
double RobotController::posDistFromGoal()
{
  pthread_mutex_lock(&cartUpdateMutex);
  double pos_mag = (curGoalP - curP).norm();
  pthread_mutex_unlock(&cartUpdateMutex);
  return pos_mag;
}

// Compute the distance between the current orientation and the goal
double RobotController::orientDistFromGoal()
{
  pthread_mutex_lock(&cartUpdateMutex);
  //Quaternion diff = Quaternion("1 0 0 0") - (curGoalQ - curQ);
  //diff /= diff.norm();
  Quaternion diff = curGoalQ^(curQ.inverse());
  pthread_mutex_unlock(&cartUpdateMutex);
  double ang_mag = fabs(diff.getAngle());
  return ang_mag;
}

// Compute the distance between the current joint angles and the goal angles
double RobotController::jointDistFromGoal()
{
  double joint_mag = 0;
  pthread_mutex_lock(&jointUpdateMutex);
  for (int i=0; i < NUM_JOINTS; i++)
    {
      joint_mag += (curGoalJ[i] - curJ[i]) * (curGoalJ[i] - curJ[i]);
    }
  pthread_mutex_unlock(&jointUpdateMutex);

  return sqrt(joint_mag);
}


//////////////////////////////////////////////////////////////////////////////
// Log Call Back
//
// This function communicates with the robot to see if any new position
// or force information has been transmitted by tcp/ip. If any data has been
// sent, it reads all of it, saves the newest of each message, and publishes
// any new position, joint, or force information it gets. This function is
// called every time there is a timer event.
//////////////////////////////////////////////////////////////////////////////
void RobotController::logCallback(const ros::TimerEvent&)
{
  int t;
  int code;
  char buffer[MAX_BUFFER];
  char * partialBuffer;
  bool cartesianModif = false;
  bool jointsModif = false;
  bool forceModif = false;

  //I'd like to broadcast frames for abb_base -> abb_workobject -> abb_tcp
  //Non-TCP transforms change way less often, but I guess
  //the ROS way is to broadcast everything always...
  tf::Transform transform;
  
  //Messages to be published
  sensor_msgs::JointState msgJoints;
  geometry_msgs::WrenchStamped msgForce;
  geometry_msgs::PoseStamped msgCartesian;

  //Set the frame IDs on the messages 
  msgForce.header.frame_id = "abb_tcp";
  msgJoints.header.frame_id = "abb_base";
  msgCartesian.header.frame_id = "abb_workobject";

  //Joints has 6 positions, as the robot is 6 DOF
  msgJoints.position.resize(6);
  msgJoints.name.resize(6);
  msgJoints.name[0] = "joint1";
  msgJoints.name[1] = "joint2";
  msgJoints.name[2] = "joint3";
  msgJoints.name[3] = "joint4";
  msgJoints.name[4] = "joint5";
  msgJoints.name[5] = "joint6";
  

  // Read all information from the tcp/ip socket
  if ((t=recv(robotLoggerSocket, buffer, MAX_BUFFER-1, 0)) > 0)
  {
    // Add an end character to form our string
    buffer[t] = '\0';
    partialBuffer = buffer;

    // Each message starts with a '#' character. Read messages one at a time
    while((partialBuffer = strchr(partialBuffer,'#'))!=NULL)		
    {
      // The number after the start character is the type of message
      sscanf(partialBuffer,"# %d", &code);
      switch(code)
      {
        // Cartesian Message
        case 0:
          {
            char date[MAX_BUFFER];
            char time[MAX_BUFFER];
            int nParams = sscanf(partialBuffer,"# %*d %s %s %*f %lf %lf %lf %lf %lf %lf %lf",
				 date,
				 time,
				 //&msgCartesian.timeStamp,
				 &msgCartesian.pose.position.x,
				 &msgCartesian.pose.position.y,
				 &msgCartesian.pose.position.z,
				 &msgCartesian.pose.orientation.w,
				 &msgCartesian.pose.orientation.x,
				 &msgCartesian.pose.orientation.y,
				 &msgCartesian.pose.orientation.z);
            if(nParams == 9)
            {
              // If we read in the correct number of parameters, save this message
	      //Convert distance units from mm to meters
	      msgCartesian.pose.position.x *= .001;
	      msgCartesian.pose.position.y *= .001;
	      msgCartesian.pose.position.z *= .001;
              cartesianModif = true;
            }
            break;
          }

        // Joint Message
        case 1:
          {
            char date[MAX_BUFFER];
            char time[MAX_BUFFER];
            int nParams = sscanf(partialBuffer,"# %*d %s %s %*f %lf %lf %lf %lf %lf %lf",
				 date,
				 time,
				 //&msgJoints.timeStamp,
				 &msgJoints.position[0],
				 &msgJoints.position[1],
				 &msgJoints.position[2],
				 &msgJoints.position[3],
				 &msgJoints.position[4],
				 &msgJoints.position[5] );
            if (nParams == 8)  //9 with timestamp
            {
              // If we read in the correct number of parameters, save this message
       	      for (uint i =0; i < msgJoints.position.size(); i++) 
		{
		  //Convert joint angles to radians
		  msgJoints.position[i] *= 0.017453292;
		}
              jointsModif = true;
            }
            break;
          }

        // Force Message
        case 2:
          {
            char date[MAX_BUFFER];
            char time[MAX_BUFFER];
            int nParams = sscanf(partialBuffer,"# %*d %s %s %*f %lf %lf %lf %lf %lf %lf",
				 date,
				 time,
				 &msgForce.wrench.force.x,
				 &msgForce.wrench.force.y,
				 &msgForce.wrench.force.z,
				 &msgForce.wrench.torque.x,
				 &msgForce.wrench.torque.y,
				 &msgForce.wrench.torque.z);
	    if (nParams == 8) //9 with timestamp, which is ignored
            {
              // If we read in the correct number of parameters, save this message.
              forceModif = true;
            }
            break;
          }
      }
      // Increment partialBuffer, so we don't look at the same message again
      partialBuffer++;
    }

    // Now, only publish on a given topic if a new message was received.
    // Also, save this data internally so it can be used for other functions
    if(cartesianModif)
    {
      //update the TCP transform with message info
      //Need to convert from mm(robot) -> m (tf standard)
      transform.setOrigin(tf::Vector3(msgCartesian.pose.position.x, 
				      msgCartesian.pose.position.y, 
				      msgCartesian.pose.position.z) );
      transform.setRotation(tf::Quaternion(msgCartesian.pose.orientation.x, 
					   msgCartesian.pose.orientation.y, 
					   msgCartesian.pose.orientation.z, 
					   msgCartesian.pose.orientation.w));

      handle_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"abb_workobject",  "abb_tcp"));

      //Send the Work Object transform at the same time, so the tree abb_base -> abb_workobject -> abb_tcp is fully populated 
      pthread_mutex_lock(&wobjUpdateMutex);
      handle_tf.sendTransform(tf::StampedTransform(curWobjTransform, ros::Time::now(), "abb_base", "abb_workobject"));
      pthread_mutex_unlock(&wobjUpdateMutex); 
     
      //Publish XYZ/Quaternion message
      handle_CartesianLog.publish(msgCartesian);

      pthread_mutex_lock(&cartUpdateMutex);
      curP[0] = msgCartesian.pose.position.x;
      curP[1] = msgCartesian.pose.position.y;
      curP[2] = msgCartesian.pose.position.z;
      curQ[0] = msgCartesian.pose.orientation.w;
      curQ[1] = msgCartesian.pose.orientation.x;
      curQ[2] = msgCartesian.pose.orientation.y;
      curQ[3] = msgCartesian.pose.orientation.z;
      pthread_mutex_unlock(&cartUpdateMutex); 

    }
    if(jointsModif)
    {
      handle_JointsLog.publish(msgJoints);
      pthread_mutex_lock(&jointUpdateMutex);
      curJ[0] = msgJoints.position[0];
      curJ[1] = msgJoints.position[1];
      curJ[2] = msgJoints.position[2];
      curJ[3] = msgJoints.position[3];
      curJ[4] = msgJoints.position[4];
      curJ[5] = msgJoints.position[5];
      pthread_mutex_unlock(&jointUpdateMutex);
    }
    if(forceModif)
    {
      
      handle_ForceLog.publish(msgForce);
      pthread_mutex_lock(&forceUpdateMutex);
      curForce[0] = msgForce.wrench.force.x;
      curForce[1] = msgForce.wrench.force.y;
      curForce[2] = msgForce.wrench.force.z;
      curForce[3] = msgForce.wrench.torque.x;
      curForce[4] = msgForce.wrench.torque.y;
      curForce[5] = msgForce.wrench.torque.z;
      pthread_mutex_unlock(&forceUpdateMutex);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// Main Thread for Logger
//
// This is the main function for our logger thread. We simply start a ROS 
// timer event and get it to call our loggerCallback function at a 
// specified interval. This exits when ROS shuts down.
//////////////////////////////////////////////////////////////////////////////
void *loggerMain(void *args)
{
  //Recover the pointer to the main node
  RobotController* ABBrobot;
  ABBrobot = (RobotController*) args;

  // Create a timer to look at the log data
  ros::Timer loggerTimer;
  loggerTimer = ABBrobot->node->createTimer(ros::Duration(0.003), 
      &RobotController::logCallback, ABBrobot);

  // Now simply wait until the program is shut down
  ros::waitForShutdown();
  loggerTimer.stop();

  pthread_exit((void*) 0);
}

//////////////////////////////////////////////////////////////////////////////
// Main Thread for Non-Blocking
//
// This is the main function of the non-blocking thread. When we are in
// non-blocking mode, it updates it's current target position, figures
// out what step to take, and moves there. 
//////////////////////////////////////////////////////////////////////////////
void *nonBlockMain(void *args)
{
  // Recover the pointer to the main node
  RobotController* robot;
  robot = (RobotController*) args;

  // Initiate our loop frequency variable
  ros::Rate nb_rate(NB_FREQ);

  // We will stay in this loop until ROS shuts down
  while(ros::ok())
  {
    // While we are inside this loop, the robot is moving
    while (robot->non_blocking && robot->do_nb_move)
    {
      // If we are doing a cartesian move, execute the sequence below
      if (robot->cart_move)
      {
        // Read in the current cartesian target, and don't use it again
        // for this iteration in case it changed in between
        pthread_mutex_lock (&nonBlockMutex);
        HomogTransf target(robot->curTargQ.getRotMat(), robot->curTargP);
        robot->targetChanged = false;
        /*ROS_INFO("Target: %f, %f, %f", robot->curTargP[0], 
            robot->curTargP[1], robot->curTargP[2]);*/
        pthread_mutex_unlock(&nonBlockMutex);

        // Get the last goal position, and find the 
        // difference between here and our target
        HomogTransf pos(robot->curGoalQ.getRotMat(), robot->curGoalP);
        HomogTransf diff = (pos.inv())*target;
        
        // Get the orientation and translational change
        Vec diffV = diff.getTranslation();
        Quaternion diffQ = diff.getRotation().getQuaternion();
        diffQ /= diffQ.norm();

        // Compute the magnitude of each change
        double transMag = diffV.norm();
        double rotMag = diffQ.getAngle();

        // Compute the number of loops it will take us to reach our 
        // translation and orientation goals. Note that since our speed and
        // step size is variable, we make sure that nothing funny happens while
        pthread_mutex_lock(&nonBlockMutex);
        double linSteps = transMag / robot->curCartStep;
        double angSteps = rotMag / robot->curOrientStep;
        pthread_mutex_unlock(&nonBlockMutex);
        
        // This will hold the distance to translate 
        // and rotate for this iteration
        double transDist = 0;
        double angDist = 0;

        // Keep track of whether this is our last step
        bool reachedGoal = false;

        // If we have more linear steps than angular steps left, make sure 
        // we go the full distance for the linear step, but only a scaled 
        // distance for the angular step
        if (linSteps >= angSteps)
        {
          // If we have more than a step left, make sure we only move 
          // 1 step's worth in both rotation and translation
          if (linSteps > 1.0)
          {
            transDist = transMag / linSteps;
            angDist = rotMag / linSteps;
          }
          else
          {
            // Otherwise, we are less than a step away from our goal
            reachedGoal = true;
          }
        }
        else
        {
          // If there are more angular steps than linear steps remaining, 
          // make sure we scale everything by the number of angular 
          // steps left
          if (angSteps > 1.0)
          {
            // If we have more than a step left, only move 1 steps's worth
            angDist = rotMag / angSteps;
            transDist = transMag / angSteps;
          }
          else
          {
            // Otherwise, we are less than a step away from our goal
            reachedGoal = true;
          }
        }

        // Now that we have computed the magnitude of our steps, compute
        // the actual translation and rotation to do for this step
        Vec incTrans(3);
        Quaternion incRot("1 0 0 0");

        if (!reachedGoal)
        {
          // Simply scale the total difference by the current magnitude's step
          // to get the translation for this step
          if (transMag > 0)
            incTrans = diffV * transDist / transMag;

          // Interpolate between not rotating (unit quaternion) to the full
          // rotation, and scale by the magnitude for the current step
          if (rotMag > 0)
          {
            incRot = Quaternion("1 0 0 0") + 
               (diffQ - Quaternion("1 0 0 0")) * angDist / rotMag;

            // Make sure that we renormalize our quaternion
            incRot /= incRot.norm();
          }
        }
        else
        { 
          // If we have less than a step to go, the translation and rotation 
          // is simply the changed we calculated above
          incTrans = diffV;
          incRot = diffQ;
        }

        // Now form homogeneous matrices to calculate the resulting 
        // position and orientation from this step
        HomogTransf incStep(incRot.getRotMat(), incTrans);
        HomogTransf newGoal = pos * incStep;

        Vec newGoalV = newGoal.getTranslation();
        Quaternion newGoalQ = newGoal.getRotation().getQuaternion();

        // Wait here until we're within range to last commanded goal
	ros::Rate check_rate(DIST_CHECK_FREQ);

	while ((robot->posDistFromGoal() > robot->curDist[0]) ||
	       (robot->orientDistFromGoal() > robot->curDist[1]))
	  check_rate.sleep();

        /*
        ROS_INFO("New Goal: %f, %f, %f", newGoalV[0], newGoalV[1], newGoalV[2]);
        ROS_INFO("Old Goal: %f, %f, %f", robot->curGoalP[0], 
            robot->curGoalP[1], robot->curGoalP[2]);
        pthread_mutex_lock (&cartUpdateMutex);
        ROS_INFO("Cur Pos: %f, %f, %f", 
            robot->curP[0], robot->curP[1], robot->curP[2]); 
        pthread_mutex_unlock(&cartUpdateMutex);
*/

        // Now do the cartesian move towards our new goal
        if (!robot->setCartesian(newGoalV[0], newGoalV[1], newGoalV[2],
              newGoalQ[0], newGoalQ[1], newGoalQ[2], newGoalQ[3]))
        {
          ROS_INFO("Non-Blocking move error!");
          pthread_mutex_lock (&nonBlockMutex);
          robot->do_nb_move = false;
          pthread_mutex_unlock(&nonBlockMutex);
        }

        // If we have reached our goal, and the target hasn't been changed 
        // while we were doing the last move, we're done.
        pthread_mutex_lock (&nonBlockMutex);
        if (reachedGoal && !robot->targetChanged)
          robot->do_nb_move = false;
        pthread_mutex_unlock(&nonBlockMutex);
      }
      // Otherwise, we are executing a non-blocking joint move
      else
      {
        int i;
	double diffJ[NUM_JOINTS];
	double newGoalJ[NUM_JOINTS];
        bool reachedJ = false;
        double maxNumSteps = 0.0;
        double jointStepSize;

        // Compute the difference between the current goal and joint target 
        // for each joint. Note that this is the only time we access the 
        // current joint target, as it's possible that it could change 
        // while we are executing this iteration.
        pthread_mutex_lock (&nonBlockMutex);
        for (i=0; i<NUM_JOINTS; i++)
        {
          diffJ[i] = (robot->curTargJ[i]) - (robot->curGoalJ[i]);
        }
        robot->targetChanged = false;
        jointStepSize = robot->curJointStep;

        // Find the joint with the furthest to go, and compute the number
        // of iterations it will take to get there
        for (i=0; i<NUM_JOINTS; i++)
        {
          double numSteps = fabs(diffJ[i]) / jointStepSize;
          if (numSteps > maxNumSteps)
            maxNumSteps = numSteps;
        }
        pthread_mutex_unlock(&nonBlockMutex);

        // If we have more than one iteration to go, scale the total 
        // difference by the magnitude of the current step and add it 
        // to the current position to get our new goal
        if (maxNumSteps > 1.0)
        {
          for (i=0; i<NUM_JOINTS; i++)
	    {
	      newGoalJ[i] = robot->curGoalJ[i] + diffJ[i] / maxNumSteps;
	    }
	}
        else
        {
          // Otherwise, we will reach our goal during this step, so simply
          // add the entire difference to the current position to get our goal
          for (i=0; i<NUM_JOINTS; i++)
	    {
	      newGoalJ[i] = robot->curGoalJ[i] + diffJ[i];
	    }
	  reachedJ = true;
        }

        // Wait here until we're within range to last commanded goal
	ros::Rate check_rate(DIST_CHECK_FREQ);
	while (robot->jointDistFromGoal() > robot->curDist[2])
	  {
	    check_rate.sleep();
	  }
	// Now do the joint move towards our new goal
        if (!robot->setJoints(newGoalJ))
        {
          ROS_INFO("Non-Blocking move error!");
          pthread_mutex_lock (&nonBlockMutex);
          robot->do_nb_move = false;
          pthread_mutex_unlock(&nonBlockMutex);
        }

        // If we have reached our goal, and the target hasn't been changed 
        // while we were doing the last move, we're done.
        pthread_mutex_lock (&nonBlockMutex);
        if (reachedJ && !robot->targetChanged)
          robot->do_nb_move = false;
        pthread_mutex_unlock(&nonBlockMutex);
      }

      // Make sure we don't hog all the CPU by running too fast
      nb_rate.sleep();
    }

    // If we get a stop request, confirm that we've stopped
    if (robot->stopRequest)
      robot->stopConfirm = true;

    // Make sure we don't hog all the CPU by running too fast
    nb_rate.sleep();
  }

  pthread_exit((void*) 0);
}



//////////////////////////////////////////////////////////////////////////////
// Main Loop for Robot Node
//
// This is what is executed when the robot node is run in ROS. We initialize
// the robot, start our logging and non-blocking threads, and start listening
// for messages. The program exits when ROS is shutdown.
//////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_controller");
  ros::NodeHandle node;
  RobotController ABBrobot(&node);

  // Initialize the Robot Node
  if (!ABBrobot.init())
    exit(-1);
 
  // Initialize the mutex's we will be using in our threads
  pthread_mutex_init(&nonBlockMutex, NULL);
  pthread_mutex_init(&jointUpdateMutex, NULL);
  pthread_mutex_init(&cartUpdateMutex, NULL);
  pthread_mutex_init(&forceUpdateMutex, NULL);
  pthread_mutex_init(&sendRecvMutex, NULL);

  // Create a dedicated thread for logger broadcasts
  pthread_t loggerThread;
  pthread_attr_t attrL;
  pthread_attr_init(&attrL);
  pthread_attr_setdetachstate(&attrL, PTHREAD_CREATE_JOINABLE);

  if (pthread_create(&loggerThread, &attrL, 
        loggerMain, (void*)&ABBrobot) != 0)
  {
    ROS_INFO("ROBOT_CONTROLLER: Unable to create logger thread. "
        "Error number: %d.",errno);
  }

  // Create a dedicated thread for non-blocking moves
  pthread_t nonBlockThread;
  pthread_attr_t attrB;
  pthread_attr_init(&attrB);
  pthread_attr_setdetachstate(&attrB, PTHREAD_CREATE_JOINABLE);

  if(pthread_create(&nonBlockThread,  &attrB, 
        nonBlockMain, (void*)&ABBrobot) != 0)
  {
    ROS_INFO("ROBOT_CONTROLLER: Unable to create non-blocking thread."
        " Error number: %d.",errno);
  }

  //Advertise ROS services
  ROS_INFO("ROBOT_CONTROLLER: Advertising ROS services...");
  ABBrobot.advertiseServices();

  //Advertise ROS topics
  ROS_INFO("ROBOT_CONTROLLER: Advertising ROS topics...");
  ABBrobot.advertiseTopics();

  //Main ROS loop
  ROS_INFO("ROBOT_CONTROLLER: Running node /robot_controller...");
  // Multithreaded spinner so that callbacks 
  // can be handled on separate threads.
  ros::MultiThreadedSpinner spinner(3); // We have 3 total threads
  spinner.spin();
  ROS_INFO("ROBOT_CONTROLLER: Shutting down node /robot_controller...");

  //End threads
  void *statusL, *statusB;
  pthread_attr_destroy(&attrL);
  pthread_attr_destroy(&attrB);
  pthread_mutex_destroy(&nonBlockMutex);
  pthread_mutex_destroy(&jointUpdateMutex);
  pthread_mutex_destroy(&cartUpdateMutex);
  pthread_mutex_destroy(&forceUpdateMutex);
  pthread_mutex_destroy(&sendRecvMutex);
  pthread_join(loggerThread, &statusL);
  pthread_join(nonBlockThread, &statusB);

  ROS_INFO("ROBOT_CONTROLLER: Done.");
  return 0;
}
