// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

// System includes
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>

////////////////////////////////////////////////////////////////////////
// Constants

const int LOCAL_PORT = 1511;

////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{ 
  
  // Initialize ROS node
  ros::init(argc, argv, "dummyserver_node");
  ros::NodeHandle n("~");

  //create a UDP socket
  int s;
  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    ROS_ERROR("Cannot create the server socket");
     
  // zero out the structure
  struct sockaddr_in si_me;
  memset((char *) &si_me, 0, sizeof(si_me));
     
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(LOCAL_PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
     
  //bind socket to port
  if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    ROS_ERROR("Cannot bind the server socket");
  else
    ROS_INFO("Server waiting for connection");
  
  //watiting to receive some data to get the client IP
  int recv_len;
  const int buf_len = 255;
  char buf[buf_len];
  struct sockaddr_in si_other;
  socklen_t slen = sizeof(si_other);

  if ((recv_len = recvfrom(s, buf, buf_len, 0, (struct sockaddr *) &si_other, &slen)) == -1)
  {
    ROS_ERROR("Cannot receive data from client");
    exit(1);
  }

  ROS_INFO("Client connected, packet received from %s:%i", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
     
  //keep sending data
  sprintf(buf, "%s", "pippo\0");

  ros::Rate LoopRate(100.0);
  while(ros::ok())
  {
    if (sendto(s, buf, buf_len, 0, (struct sockaddr*) &si_other, slen) == -1)
      ROS_ERROR("Error sending data");
    else
      ROS_INFO("Packet sent to %s:%i", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));

    ros::spinOnce();
    
    LoopRate.sleep();
  }
 
  close(s);

  return 0;
}
