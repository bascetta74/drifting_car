/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
///
/// ROS node that translates motion capture data from an OptiTrack rig to tf transforms.
/// The node receives the binary packages that are streamed by the Arena software,
/// decodes them and broadcasts the poses of rigid bodies as tf transforms.
///
/// Currently, this node supports the NatNet streaming protocol v1.4.

#define UNICAST
//#define MULTICAST

// Local includes
#ifdef UNICAST
  #include "mocap_optitrack/socket_unicast.h"
#endif
#ifdef MULTICAST
  #include "mocap_optitrack/socket_multicast.h"
#endif
#include "mocap_optitrack/mocap_datapackets.h"
#include "mocap_optitrack/mocap_config.h"
#include "mocap_optitrack/skeletons.h"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

// System includes
#include <string>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////
// Constants

// ip on multicast group - cannot be changed in Arena
const std::string MULTICAST_IP_KEY = "optitrack_config/multicast_address";
const std::string MULTICAST_IP_DEFAULT = "224.0.0.1";

const std::string SERVER_IP_KEY = "optitrack_config/server_address";
const std::string SERVER_IP_DEFAULT = "10.0.0.245";

const std::string MOCAP_MODEL_KEY = "mocap_model";
const std::string RIGID_BODIES_KEY = "rigid_bodies";
const char ** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;

const int LOCAL_PORT = 1511;

////////////////////////////////////////////////////////////////////////

void processMocapData( const char** mocap_model,
                       RigidBodyMap& published_rigid_bodies,
                       const std::string& ip)
{
#ifdef MULTICAST
  UdpMulticastSocket client_socket( LOCAL_PORT, ip );
#endif
#ifdef UNICAST
  UdpUnicastSocket client_socket( LOCAL_PORT, ip );
#endif

  ushort payload;
  int numberOfPackets = 0;
  while(ros::ok())
  {
    bool packetread = false;
    int numBytes = 0;

    do
    {
      // Receive data from mocap device
      numBytes = client_socket.recv();
      if (numBytes>0)
        ROS_INFO("Message received, %d bytes", numBytes);

      // Parse mocap data
      if( numBytes > 0 )
      {
        const char* buffer = client_socket.getBuffer();
        unsigned short header = *((unsigned short*)(&buffer[0]));

        // Look for the beginning of a NatNet package
        if (header == 7)
        {
          payload = *((ushort*) &buffer[2]);
          MoCapDataFormat format(buffer, payload);
          format.parse();
          packetread = true;
          numberOfPackets++;

          if( format.model.numRigidBodies > 0 )
          {
            for( int i = 0; i < format.model.numRigidBodies; i++ )
            {
              int ID = format.model.rigidBodies[i].ID;
              RigidBodyMap::iterator item = published_rigid_bodies.find(ID);

              if (item != published_rigid_bodies.end())
              {
                  item->second.publish(format.model.rigidBodies[i]);
              }
            }
          }
        }
        // else skip packet
      }
    } while( numBytes > 0 );

    // Don't try again immediately
    if( !packetread )
    {
      usleep( 10 );
    }
  }
}



////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{ 
  
  // Initialize ROS node
  ros::init(argc, argv, "mocap_node");
  ros::NodeHandle n("~");

  // Get configuration from ROS parameter server  
  const char** mocap_model( DEFAULT_MOCAP_MODEL );
  if( n.hasParam( MOCAP_MODEL_KEY ) )
  {    std::string tmp;
    if( n.getParam( MOCAP_MODEL_KEY, tmp ) )
    {
      if( tmp == "SKELETON_WITH_TOES" )
        mocap_model = SKELETON_WITH_TOES;
      else if( tmp == "SKELETON_WITHOUT_TOES" )
        mocap_model = SKELETON_WITHOUT_TOES;
      else if( tmp == "OBJECT" )
        mocap_model = OBJECT;
    }
  }

  // Get configuration from ROS parameter server
#ifdef MULTICAST
  std::string multicast_ip( MULTICAST_IP_DEFAULT );
  if( n.hasParam( MULTICAST_IP_KEY ) )
  {
    n.getParam( MULTICAST_IP_KEY, multicast_ip );
  }
  else {
    ROS_WARN_STREAM("Could not get multicast address, using default: " << multicast_ip);
  }
#endif

#ifdef UNICAST
  std::string server_ip( SERVER_IP_DEFAULT );
  if( n.hasParam( SERVER_IP_KEY ) )
  {
    n.getParam( SERVER_IP_KEY, server_ip );
  }
  else {
    ROS_WARN_STREAM("Could not get server address, using default: " << server_ip);
  }
#endif

  RigidBodyMap published_rigid_bodies;

  if (n.hasParam(RIGID_BODIES_KEY))
  {
      XmlRpc::XmlRpcValue body_list;
      n.getParam("rigid_bodies", body_list);
      if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0)
      {
          XmlRpc::XmlRpcValue::iterator i;
          for (i = body_list.begin(); i != body_list.end(); ++i) {
              if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                  PublishedRigidBody body(i->second);
                  string id = (string&) (i->first);
                  RigidBodyItem item(atoi(id.c_str()), body);

                  std::pair<RigidBodyMap::iterator, bool> result = published_rigid_bodies.insert(item);
                  if (!result.second)
                  {
                      ROS_ERROR("Could not insert configuration for rigid body ID %s", id.c_str());
                  }
              }
          }
      }
  }

  // Process mocap data until SIGINT
#ifdef MULTICAST
  processMocapData(mocap_model, published_rigid_bodies, multicast_ip);
#endif
#ifdef UNICAST
  processMocapData(mocap_model, published_rigid_bodies, server_ip);
#endif

  return 0;
}
