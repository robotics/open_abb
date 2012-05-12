#include <string>
#include <math.h>

#include <SFML/Network.hpp>
#include <SFML/System.hpp>
#include <oad.h>

#define MAX_STRING 80
#define MAX_BUFFER 1024
#define ROBOT_IP "192.168.1.07"
#define ROBOT_PORT   5000
#define LOGGER_PORT   5001


int main(int argc, char **argv)
{
  //Local Variables
  char strReceived[MAX_STRING];
  std::size_t sizeReceived;
  sf::SocketTCP RobotClient;

  //Set communication with robot
  RobotClient.SetBlocking(true); //hold the execution until we receive ACK from the server 	
  sf::IPAddress robotIp(ROBOT_IP); 
  if (RobotClient.Connect(ROBOT_PORT, robotIp) != sf::Socket::Done)
    {
      printf("There has been an error while connecting to the robot controller.\n");
      RobotClient.Close();
      exit(-1);
    }
	
  //Set workObject
  RobotClient.Send(oad::setWorkObject(500.0, 0.0, 500.0, 0.0, 0.0, 1.0, 0.0).c_str(),MAX_STRING); 
	RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  if(oad::checkReply(strReceived))  
    printf("WorkObject set.\n");	
  else
    exit(-1);

  //Set tool
  RobotClient.Send(oad::setTool(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  if(oad::checkReply(strReceived))  
    printf("Tool set.\n");	
  else
    exit(-1);
	
  //Set speed
  RobotClient.Send(oad::setSpeed(100,30).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  if(oad::checkReply(strReceived))  
    printf("Speed set.\n");	
  else
    exit(-1);
	
  //Set zone
  RobotClient.Send(oad::setPredefinedZone(1).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  if(oad::checkReply(strReceived))  
    printf("Zone set.\n");	
  else
    exit(-1);
  
  //Motion
  RobotClient.Send(oad::setCartesian(100.0, 100.0, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  sf::Sleep(1.0);
  RobotClient.Send(oad::setCartesian(100.0, -100.0, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  RobotClient.Send(oad::setCartesian(-100.0, -100.0, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  RobotClient.Send(oad::setCartesian(-100.0, 100.0, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  RobotClient.Send(oad::setCartesian(100.0, 100.0, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  sf::Sleep(1.0);
  RobotClient.Send(oad::setCartesian(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  
  //Close connection with ABB Server
  RobotClient.Send(oad::closeConnection().c_str(),MAX_STRING);
  //RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  RobotClient.Close();
  return 0;
}
