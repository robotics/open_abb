#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>

#include <oad.h>

#define MAX_STRING   80
#define MAX_BUFFER   1024
#define ROBOT_IP     "192.168.1.7"
#define LOGGER_IP    "192.168.1.7"
#define ROBOT_PORT   5000
#define LOGGER_PORT  5001

bool logging;

// Function prototypes
bool sockSend(int socketId, std::string toSend);
bool sockReceive(int socketId, std::string &received);
void *loggerMain(void *args);


int main(int argc, char **argv)
{ 
  //Set communication with robot
  std::string msgReceived;
  int robotSocket; 
  if ((robotSocket = socket(PF_INET, SOCK_STREAM, 0)) == -1)
  {
    printf("Problem creating robot socket. Error number: %d",errno);
    exit(-1);
  }
  struct sockaddr_in remoteSocket;
  remoteSocket.sin_family = AF_INET;
  int port = ROBOT_PORT;
  remoteSocket.sin_port = htons(port);
  inet_pton(AF_INET, ROBOT_IP , &remoteSocket.sin_addr.s_addr);
  if(connect(robotSocket, (sockaddr*)&remoteSocket,sizeof(remoteSocket)) == -1)
  {
    printf("Impossible to connect to the ABB robot. Error number: %d",errno);
    exit(-1);    
  }

  logging = false;
  // Create a dedicated thread for logger
  pthread_t loggerThread;
  pthread_attr_t attrL;
  pthread_attr_init(&attrL);
  pthread_attr_setdetachstate(&attrL, PTHREAD_CREATE_JOINABLE);

  if (pthread_create(&loggerThread, &attrL, loggerMain, NULL) != 0)
  {
    printf("Unable to create logger thread. Error number: %d.",errno);
    exit(-1);
  }
  logging = true;



  //Set workObject
  sockSend(robotSocket, oad::setWorkObject(500.0, 0.0, 500.0, 0.0, 0.0, 1.0, 0.0));
  sockReceive(robotSocket, msgReceived);
  if(!oad::checkReply(msgReceived))
  {
    printf("Problem setting the WorkObject.\n");	
    exit(-1);
  }
 
  //Set tool
  sockSend(robotSocket, oad::setTool(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
  sockReceive(robotSocket, msgReceived);
  if(!oad::checkReply(msgReceived))
  {
    printf("Problem setting the Tool.\n");	
    exit(-1);
  }

  //Set speed
  sockSend(robotSocket, oad::setSpeed(100,30));
  sockReceive(robotSocket, msgReceived);
  if(!oad::checkReply(msgReceived))
  {
    printf("Problem setting the Speed of the robot.\n");	
    exit(-1);
  }
 
  //Set zone
  sockSend(robotSocket, oad::setPredefinedZone(1));
  sockReceive(robotSocket, msgReceived);
  if(!oad::checkReply(msgReceived))
  {
    printf("Problem setting the Zone of the robot.\n");	
    exit(-1);
  }
   
  //Motion
  sockSend(robotSocket, oad::setCartesian(100.0, 100.0, 0.0, 1.0, 0.0, 0.0, 0.0));
  sockReceive(robotSocket, msgReceived);

  sockSend(robotSocket, oad::setCartesian(100.0, -100.0, 0.0, 1.0, 0.0, 0.0, 0.0));
  sockReceive(robotSocket, msgReceived);

  sockSend(robotSocket, oad::setCartesian(-100.0, -100.0, 0.0, 1.0, 0.0, 0.0, 0.0));
  sockReceive(robotSocket, msgReceived);
 
  sockSend(robotSocket, oad::setCartesian(-100.0, 100.0, 0.0, 1.0, 0.0, 0.0, 0.0));
  sockReceive(robotSocket, msgReceived);

  sockSend(robotSocket, oad::setCartesian(100.0, 100.0, 0.0, 1.0, 0.0, 0.0, 0.0));
  sockReceive(robotSocket, msgReceived);

  //Close communication with ABB controller
  sockSend(robotSocket, oad::closeConnection());
  sleep(1);
  close(robotSocket);

  //Stop logger thread
  void *statusL;
  logging = false;
  pthread_join(loggerThread, &statusL);
  pthread_attr_destroy(&attrL);

  return 0;
}

void *loggerMain(void *args)
{
  //Set communication with logger
  std::string msgReceived;
  int loggerSocket; 
  if ((loggerSocket = socket(PF_INET, SOCK_STREAM, 0)) == -1)
  {
    printf("Problem creating logger socket. Error number: %d",errno);
    exit(-1);
  }
  struct sockaddr_in remoteSocket;
  remoteSocket.sin_family = AF_INET;
  int port = LOGGER_PORT;
  remoteSocket.sin_port = htons(port);
  inet_pton(AF_INET, LOGGER_IP , &remoteSocket.sin_addr.s_addr);
  if(connect(loggerSocket, (sockaddr*)&remoteSocket,sizeof(remoteSocket)) == -1)
  {
    printf("Impossible to connect to the logger in the ABB robot. Error number: %d",errno);
    exit(-1);    
  }
 
  //Start logging
  while(logging)
  {
    sockReceive(loggerSocket, msgReceived);
    int code;
    sscanf(msgReceived.c_str(),"# %d", &code);
    if(code==0)
      printf("%s\n",msgReceived.c_str());
   }
  close(loggerSocket);
  pthread_exit((void*) 0);
}


// Helper function to send a command through a socket
bool sockSend(int socketId, std::string toSend)
{
  //printf("toSend: %s\n",toSend.c_str());
  if (send(socketId, toSend.c_str(), toSend.size(), 0) == -1)
  {
    printf("Failed to send command to ABB robot: Error number %d.", errno);
    return false;
  }
  return true;
}


// Helper blocking function to receive a message from a socket.
bool sockReceive(int socketId, std::string &received)
{
  int t;
  char buffer[MAX_BUFFER];
  if ((t=recv(socketId, buffer, MAX_BUFFER-1, 0)) > 0)
  {
    //printf("received: %s\n",buffer); 
    buffer[t] = '\0';
    received.assign(buffer,strlen(buffer));
    return true;
  }
  printf("Problem receiving message");
  return false;
}
