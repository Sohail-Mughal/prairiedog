/*  
 *  Copyrights:
 *  Michael Otte July. 2010
 *
 *  This file is part of intercom_cu.
 *
 *  intercom_cu is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  client_server_cu is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with client_server_cu. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 *
 *
 *  This listens to data from the client computer, then broadcasts on the approperiate topics 
 *  server computer.
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <time.h>
#include <math.h>
#include <list>
#include <sstream>
#include <iostream>   // file I/O
#include <fstream>   // file I/O
#include <vector>
#include <pthread.h>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tinyxml/tinyxml.h>

#include "roslib/Header.h"

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point32.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/GridCells.h"

#include "sensor_msgs/PointCloud.h"

#include "move_base_msgs/MoveBaseActionGoal.h"

#include "hokuyo_listener_cu/PointCloudWithOrigin.h"

#include "multi_robot_planner_cu/PolygonArray.h"

#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include "intercom_cu/PoseStamped_CU_ID.h"
#include "intercom_cu/Int32_CU_ID.h"
#include "intercom_cu/PointCloud_CU_ID.h"
#include "intercom_cu/Path_CU_ID.h"
#include "intercom_cu/Pose2D_CU_ID.h"
#include "intercom_cu/PointCloudWithOrigin_CU_ID.h"
#include "intercom_cu/Polygon_CU_ID.h"
#include "intercom_cu/PolygonArray_CU_ID.h"
#include "intercom_cu/Float32_CU_ID.h"

#include "ros_to_buffer.cpp"

#ifndef PI
  #define PI 3.1415926535897
#endif
      
#ifndef SQRT2
  #define SQRT2 1.414213562373095
#endif
      
#define SCANNER_RANGE 5.5 // the range of the scanner in meters;
        
using namespace std;      
        
// globals that can be reset using parameter server, see main();
bool using_tf = true;              // when set to true, use the tf package

int max_message_size = 500000;
size_t max_network_message_size = 15000; // messages larger than this are split into multiple messages to be sent

int NUM_MESSAGE_TYPES = 19;
  
// global ROS subscriber handles
ros::Subscriber selected_robot_sub;

ros::Subscriber new_goal_sub;
ros::Subscriber new_pose_sub;
ros::Subscriber user_control_sub;
ros::Subscriber user_state_sub;

ros::Subscriber pose_sub;
ros::Subscriber global_path_sub;
ros::Subscriber goal_sub;
ros::Subscriber laser_scan_sub;
ros::Subscriber map_changes_sub;
ros::Subscriber system_state_sub;
ros::Subscriber system_update_sub;

ros::Subscriber target_pose_sub;
ros::Subscriber planning_area_sub;
ros::Subscriber turn_circle_sub;
ros::Subscriber obstacles_sub;

ros::Subscriber time_ahead_sub;

// global ROS publisher handles
ros::Publisher pose_pub;
ros::Publisher global_path_pub;
ros::Publisher goal_pub;
ros::Publisher laser_scan_pub;
ros::Publisher map_changes_pub;
ros::Publisher system_state_pub;
ros::Publisher system_update_pub;

ros::Publisher new_goal_pub;
ros::Publisher new_pose_pub;
ros::Publisher user_control_pub;
ros::Publisher user_state_pub;

ros::Publisher target_pose_pub;
ros::Publisher planning_area_pub;
ros::Publisher turn_circle_pub;
ros::Publisher obstacles_pub;

ros::Publisher time_ahead_pub;

// global ROS provide service server handles
ros::ServiceServer get_map_srv;

// helper functions
void error(const char *msg)
{
    perror(msg);
}

// store globals in this class so that we can have access to them from different threads
class GlobalVariables
{
  public:

   GlobalVariables();
   GlobalVariables(int id, int total_agents); 
   ~GlobalVariables();
   
   void Populate(int id, int total_agents);  // populate GlobalVariables
   
   bool read_IPS_from_file(std::string& config_file);       // reads the IPS from a file
   bool set_up_OtherAddressesUDP();                         // sets other address data for UDP socket connections
   bool set_up_MyAddressUDP();                              // sets up incomming UDP sockets
   bool set_up_IncommingTCP();                              // sets up incomming TCP sockets
   bool set_up_OutgoingTCP();                               // sets up outgoing TCP sockets
   bool set_up_single_IncommingTCP(int s);                  // sets up incomming TCP socket, s is the agent we want to set it up to
   bool set_up_single_OutgoingTCP(int s);                   // sets up outgoing TCP socket, s is the agent we want to set it up to
   
   void send_to_agent(void* buffer, size_t buffer_size, int ag);                    // sends data to agent ag
   void send_to_all_agents(void* buffer, size_t buffer_size);                       // sends data to all other agents
   void send_to_destination_matrix(void* buffer, size_t buffer_size, int msg_type); // sends data to all destination_matrix[msg_type]==true
   void send_message_type(void* buffer, size_t buffer_size, int type);              // sends data of message-type send_mode_list as defined by send_mode_list[type]
   
   bool using_tcp();    // returns true if any messages need tcp
   bool using_udp();    // returns true if any messages need udp
   
   // ip addresses
   std::string MyIP;
   vector<std::string> OtherIPs;
   
   // ports
   int MyInPortUDP;
   int MyOutPortUDP;
   
   vector<int> OtherInPortsUDP;
   
   vector<int> MyInPortsTCP;
   int MyDestinationPortTCP;
   
   // socket numbers
   int MyInSockUDP;
   int MyOutSockUDP;
   
   vector<int> MyInSocksTCP;
   vector<int> MyOutSocksTCP;
   
   // sockaddr_in addresses
   struct sockaddr_in MyAddressUDP;
   vector<struct sockaddr_in> OtherAddressesUDP;
   
   vector<struct sockaddr_in> MyInAddressesTCP;
   vector<struct sockaddr_in> DestinationAddressesTCP;
   
   // my agent id
   int my_id;
   
   // total number of agents (not including the visualization agent)
   int num_agents;
   
   // the agent we are currently communicating to (if we don't wish to broadcast to all)
   int target_agent;
   
   // message types we send, 'send_list[i] = true' denotes message type i is sent (via udp)
   vector<bool> send_list;
   vector<int>  send_mode_list;  // 0=broadcast (default), 1=send only to the current target_agent, 2=send only to a subset of agents as defined in the config file 
   vector<vector<bool> > destination_matrix; // destination_matrix[x][y] means that we need to send message x to agent y
   vector<int> protocol; // 0 = udp (default), 1 = tcp
   
   // message types we recieve 'listen_list[i] = true' denotes message type i is listened for (via udp)
   vector<bool> listen_list;
   vector<int>  listen_mode_list;  //0=publish original message , 1=wrap original message so that message contains sending robot's id and send on /cu_multi/ topic instead
   
   // global inter-thread flags and storage for service interaction
   bool service_received_map;
   nav_msgs::OccupancyGrid service_response_map;
   
   // counter for messages that require being chopped up
   uint message_counter;
   
   bool send_mutext; // used to keep everybody from sending at the same time
};
GlobalVariables Globals;

GlobalVariables::GlobalVariables()  
{   
  my_id = -1;
  
  MyOutSockUDP = -1;
  MyInSockUDP = -1;
  
  num_agents = 0;
  
  target_agent = 0;
  
  service_received_map = false;
  
  message_counter = 0;
  
  send_mutext = false;
}

GlobalVariables::GlobalVariables(int id, int total_agents)  
{  
   Populate(id, total_agents);
}

GlobalVariables::~GlobalVariables()  // destructor
{
}

void GlobalVariables::Populate(int id, int total_agents) // populate GlobalVariables
{
  my_id = id;  
  
  num_agents = total_agents;
  
  service_received_map = false;
       
  MyInPortUDP = 57001 + id;
  MyOutPortUDP = 67001 + id;
  
  OtherInPortsUDP.resize(num_agents);
  for(int i = 0; i < num_agents; i++)
    OtherInPortsUDP[i] = 57001 + i;   
  
  MyInPortsTCP.resize(num_agents);    // this is the reserved port on us for robot i to contact us
  for(int i = 0; i < num_agents; i++)
    MyInPortsTCP[i] = 37001 + i;
    
  MyDestinationPortTCP = 37001 + id;  // this is the reserved port on other agents for us to contact them
  
  MyInSockUDP = -1;
  MyOutSockUDP = -1;
  
  MyInSocksTCP.resize(num_agents, -1);
  MyOutSocksTCP.resize(num_agents, -1);
  
  OtherIPs.resize(num_agents);
  OtherAddressesUDP.resize(num_agents);
  MyInAddressesTCP.resize(num_agents);
  DestinationAddressesTCP.resize(num_agents);
  
  message_counter = 0;

  send_mutext = false;
}


bool GlobalVariables::read_IPS_from_file(std::string& config_file)       // reads the IPs from a file
{
  TiXmlDocument doc(config_file.c_str());
  if(!doc.LoadFile())
  {
    printf("Failed to load file: %s \n", config_file.c_str());
    return false;
  }

  printf("loaded config file:\n");  

  // init to not send or recieve anything
  send_list.resize(NUM_MESSAGE_TYPES, false);
  send_mode_list.resize(NUM_MESSAGE_TYPES, 0);
  destination_matrix.resize(NUM_MESSAGE_TYPES);
  protocol.resize(NUM_MESSAGE_TYPES, 0);
  listen_list.resize(NUM_MESSAGE_TYPES, false);
  listen_mode_list.resize(NUM_MESSAGE_TYPES, 0);
  
  //ADD ERROR CHECKING TO THE NEXT PART (look if TiXmlElement are null or not)
  // read data about each node
  TiXmlElement* agents_el = doc.RootElement();
  TiXmlElement* agent_el = 0;
  int i = 0;
  while((agent_el = (TiXmlElement*)(agents_el->IterateChildren(agent_el))) && i < num_agents)
  {
    int i_id = atoi(agent_el->Attribute("id"));
    
    if(i_id >= num_agents)
    {
      printf("warning: agent id (%d) >= num agents (%d), so skipping it. \n", i_id, num_agents);
      continue;
    }
    
    const char* ip = agent_el->Attribute("ip");
    OtherIPs[i_id].assign(ip);
    printf("%d: %s\n", i_id, OtherIPs[i].c_str());
    
    if(i_id == my_id) // need to get lists of what we send and receive
    {
      TiXmlElement* broadcast_el = agent_el->FirstChildElement("broadcast");
      TiXmlElement* message_el = 0;
        
      printf(" broadcast: \n");
      while((message_el = (TiXmlElement *)(broadcast_el->IterateChildren("message",message_el))))
      {
        int message_type = -1;
        if(message_el->Attribute("type"))
        {
          message_type = atoi(message_el->Attribute("type"));
          send_list[message_type] = true;
          
          if(message_el->Attribute("mode"))
            send_mode_list[message_type] = atoi(message_el->Attribute("mode"));
          
          if(message_el->Attribute("protocol"))
            protocol[message_type] = atoi(message_el->Attribute("protocol"));
          
          printf("  %d (using mode %d), on protocol %d\n", message_type, send_mode_list[message_type], protocol[message_type]);
          
          if(send_mode_list[message_type] == 2) // then we need to read the list of destination agents
          {
            destination_matrix[message_type].resize(num_agents, false);
             
            TiXmlElement* to_el = message_el->FirstChildElement("to");;
            TiXmlElement* to_agent_el = 0;
            
            printf("    to:\n");
            
            while((to_agent_el = (TiXmlElement *)(to_el->IterateChildren(to_agent_el))))
            {
              if(to_agent_el->Attribute("id"))
              {
                int dest_agent = atoi(to_agent_el->Attribute("id"));
                destination_matrix[message_type][dest_agent] = true;
                    
                printf("    -> %d \n", dest_agent);
              }
            }
          }
        }
      }
    
      TiXmlElement *receive_el = agent_el->FirstChildElement("receive");
      message_el = 0;
        
      printf(" receive: \n");
      while((message_el = (TiXmlElement *)(receive_el->IterateChildren("message",message_el))))
      {
        int message_type = 0;
        
        if(message_el->Attribute("type"))
        {
          message_type = atoi(message_el->Attribute("type"));
          listen_list[message_type] = true;
          
          if(message_el->Attribute("mode"))
            listen_mode_list[message_type] = atoi(message_el->Attribute("mode"));
          
          printf("  %d (using mode %d) \n", message_type, listen_mode_list[message_type]);
        }
      }
    }  
    
    i++;   
  }
  MyIP = OtherIPs[my_id];  

  return true;
}

bool GlobalVariables::set_up_OtherAddressesUDP() // sets up other address data for UDP connections
{
  for(int i = 0; i < num_agents; i++)
  {
    if(i == my_id)
      continue;
    
    struct hostent *hp_this = gethostbyname(OtherIPs[i].c_str());
     
    if(hp_this==0) 
    {
      error("Unknown host");
      continue;
    }

    // populate OtherAddressesUDP
    OtherAddressesUDP[i].sin_family = AF_INET;

    bcopy((char *)hp_this->h_addr, (char *)&OtherAddressesUDP[i].sin_addr, hp_this->h_length); // copy in address

    OtherAddressesUDP[i].sin_port = htons(OtherInPortsUDP[i]);       
  }

  return true;
}

bool GlobalVariables::set_up_MyAddressUDP() // sets up incomming UDP socket
{
  if(MyOutSockUDP < 0)
  {
    MyOutSockUDP = socket(AF_INET, SOCK_DGRAM, 0); 
    if(MyOutSockUDP < 0)        // failed to create socket
    {
      error("problems creating socket");
      return false;
    }
  }

  if(MyInSockUDP < 0)
  {
    MyInSockUDP = socket(AF_INET, SOCK_DGRAM, 0);
    if(MyInSockUDP < 0)  // failed to open socket
    {
      error("Problems opening socket\n");
      return false;
    }
  }
  
  // clear all memory of MyAddressUDP structure
  int MyAddressUDP_length = sizeof(MyAddressUDP);
  memset(&(MyAddressUDP), NULL, MyAddressUDP_length); 
   
  // populate MyAddressUDP structure
  MyAddressUDP.sin_family = AF_INET;
  MyAddressUDP.sin_addr.s_addr = INADDR_ANY; // ip address of this machine
  MyAddressUDP.sin_port = htons(MyInPortUDP);  // htons() converts 'number' to proper network byte order

  // bind in_socket with MyAddressUDP
  if(bind(MyInSockUDP, (struct sockaddr *)&(MyAddressUDP), MyAddressUDP_length)<0) 
  {
    error("problems binding MyInSockUDP");
    return false;
  }

  return true;
}

bool GlobalVariables::set_up_IncommingTCP()  // sets up incomming TCP sockets
{   
  bool r_value = true; 
  for(int i = 0; i < num_agents; i++)
    r_value = r_value & set_up_single_IncommingTCP(i);
  
  return r_value;     
}



bool GlobalVariables::set_up_OutgoingTCP()   // sets up outgoing TCP sockets
{
  bool r_value = true;
  for(int i = 0; i < num_agents; i++)
    r_value = r_value & set_up_single_OutgoingTCP(i);

  return r_value;   
}

bool GlobalVariables::set_up_single_IncommingTCP(int s)  // sets up incomming TCP socket, s is the agent we want to set it up for
{   
  bool r_value = true; 
   
  int i = s;

  if(MyInSocksTCP[i] < 0)
    MyInSocksTCP[i] = socket(AF_INET, SOCK_STREAM, 0);   
  if(MyInSocksTCP[i] < 0) 
  {
    error("error opening socket");  
    r_value = false;
  }
  bzero((char *) &(MyInAddressesTCP[i]), sizeof(MyInAddressesTCP[i])); 
     
  MyInAddressesTCP[i].sin_family = AF_INET;
  MyInAddressesTCP[i].sin_addr.s_addr = INADDR_ANY;
  MyInAddressesTCP[i].sin_port = htons(MyInPortsTCP[i]);
     
  size_t size_temp = sizeof(MyInAddressesTCP[i]);
    
  if(bind(MyInSocksTCP[i], (struct sockaddr *) &(MyInAddressesTCP[i]), size_temp) < 0) 
  {
    error("problems binding socket");
    r_value = false;
  }
  else
    printf("binding socket %d \n", MyInSocksTCP[i]);

  return true;  
}

bool GlobalVariables::set_up_single_OutgoingTCP(int s)   // sets up outgoing TCP socket, s is the agent we want to set it up to
{
  int i = s;

  if(MyOutSocksTCP[i]  < 0)
    MyOutSocksTCP[i] = socket(AF_INET, SOCK_STREAM, 0);
  
  if(MyOutSocksTCP[i]  < 0)
  {
    error("problems opening socket");
    return false;
  }
    
  struct hostent *server;
  server = gethostbyname(OtherIPs[i].c_str());
    
  if(server == NULL) 
  {
    error("problems opening socket: cannot locate host");
    return false;
  }
    
  bzero((char *) &DestinationAddressesTCP[i], sizeof(DestinationAddressesTCP[i]));
  DestinationAddressesTCP[i].sin_family = AF_INET;
    
  
  bcopy((char *)server->h_addr, (char *)&DestinationAddressesTCP[i].sin_addr.s_addr, server->h_length);
  DestinationAddressesTCP[i].sin_port = htons(MyDestinationPortTCP);
  printf("trying to connect to : %d \n", i);
  if(connect(MyOutSocksTCP[i], (struct sockaddr *) &(DestinationAddressesTCP[i]),sizeof(DestinationAddressesTCP[i])) < 0) 
  {
    printf("problems connecting to other agent: %d :\n", i);
    error("");
    return false;
  }
  printf("connected : %d \n", i);
  return true;   
}

void GlobalVariables::send_to_agent(void* buffer, size_t buffer_size, int ag) // sends data to agent ag
{
  if(send_mutext) // somebody else is sending, so drop the message
    return;
  
  // extract message type (and other info)
  size_t buffer_max = (size_t)buffer + max_message_size;
  size_t buffer_ptr = size_t(buffer);
  int sending_agent;
  uint message_type;
  uint temp_message_counter;
  uint total_packets;
  uint packet_number;
  size_t header_end = extract_from_buffer_ethernetheader(buffer_ptr, sending_agent, message_type, temp_message_counter, total_packets, packet_number, buffer_max); // extracts an ethernet header from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
  size_t header_size = header_end - buffer_ptr;
  
  if(protocol[message_type] == 0) // want to send using UDP
  {
    send_mutext = true;
    
    //printf("trying to send: %s\n", (char*)buffer);
    //printf("trying to send message type: %d \n", message_type);
  
    if(buffer_size <= max_network_message_size)  // message is small enough to fit in one packet
    {
      // replace buffer with current message counter
      message_counter++;
      add_to_buffer_ethernetheader(buffer_ptr, my_id, message_type, message_counter, 1, 0, buffer_max);
      
      int sent_size = sendto(MyOutSockUDP, buffer, buffer_size, 0, (struct sockaddr *)&(OtherAddressesUDP[ag]), sizeof(struct sockaddr_in));
      if(sent_size < 0) 
      {
        error("Problems sending data");
      }
    }
    else // message must be split into multiple packets
    {
      size_t adjusted_data_size = max_network_message_size - header_size;
      //printf("must split message into multiple packets \n");
    
      // replace buffer with current message counter and total packets we need to send
      message_counter++;
      total_packets = buffer_size/(max_network_message_size-header_size) + 1; // number of packets we need to send
      add_to_buffer_ethernetheader(buffer_ptr, my_id, message_type, message_counter, total_packets, 0, buffer_max);
    
      // send first packet
      //printf("sending %u of %u \n", 0, total_packets);
      int sent_size = sendto(MyOutSockUDP, buffer, max_network_message_size, 0, (struct sockaddr *)&(OtherAddressesUDP[ag]), sizeof(struct sockaddr_in));
      if(sent_size < 0) 
      {
        error("Problems sending data");
      }
    
      // send the rest of the packets
      for(packet_number = 1; packet_number < total_packets; packet_number++)
      {
        // add buffer before next part of data to send
        buffer_ptr += adjusted_data_size; 
        add_to_buffer_ethernetheader(buffer_ptr, my_id, message_type, message_counter, total_packets, packet_number, buffer_max);   
    
        // send nth packet
        //printf("sending %u of %u \n", packet_number, total_packets);
        int sent_size = sendto(MyOutSockUDP, (void *)buffer_ptr, max_network_message_size, 0, (struct sockaddr *)&(OtherAddressesUDP[ag]), sizeof(struct sockaddr_in));
        if(sent_size < 0) 
        {
          error("Problems sending data");
        }
      } 
    }
    send_mutext = false;
  }
  else if(protocol[message_type] == 1) // want to send using TCP
  {  
    message_counter++;
    add_to_buffer_ethernetheader(buffer_ptr, my_id, message_type, message_counter, (uint)buffer_size, 0, buffer_max);  
    
    //printf("sending message %d of size %d \n", message_type, (int)buffer_size);
    int n = -1;
    
    while(n < 0)
    {  
      if(MyOutSocksTCP[ag] < 0)  
        Globals.set_up_single_OutgoingTCP(ag);
      
      //printf("sending message %d of size %d \n", message_type, (int)buffer_size);
      
      n = write(MyOutSocksTCP[ag],(const void*)buffer_ptr,buffer_size);
      if(n < 0)
      {
        printf("S problems writing to socket %d: \n", (int)MyOutSocksTCP[ag]);
        error("");
        Globals.set_up_single_OutgoingTCP(ag);
      }
    }
    
  }
}

void GlobalVariables::send_to_all_agents(void* buffer, size_t buffer_size) // sends data to all other agents
{
  for(int i = 0; i < num_agents; i++)
  {
    if(i == my_id)
      continue;
    send_to_agent(buffer, buffer_size, i);
  } 
}

void GlobalVariables::send_to_destination_matrix(void* buffer, size_t buffer_size, int msg_type) // sends data to all destination_matrix[msg_type]==true
{
  for(int i = 0; i < num_agents; i++)
  {
    if(i == my_id)
      continue;
    if(destination_matrix[msg_type][i])
      send_to_agent(buffer, buffer_size, i);
  }  
}

void GlobalVariables::send_message_type(void* buffer, size_t buffer_size, int type)  // sends data of message-type send_mode_list as defined by send_mode_list[type]
{
  // printf("sending message type %d \n", type);
  if(send_mode_list[type] == 0)
    send_to_all_agents(buffer, buffer_size);            // broadcast
  else if(send_mode_list[type] == 1)
    send_to_agent(buffer, buffer_size, target_agent);   // send to target agent 
  else if(send_mode_list[type] == 2)
    send_to_destination_matrix(buffer, buffer_size, type);    // send to all agents in destination matrix
}
 
bool GlobalVariables::using_tcp()    // returns true if any messages need tcp
{
  for(int i = 0; i <= NUM_MESSAGE_TYPES; i++)  
  {
    if(protocol[i] == 1)
      return true;
  }
  return false;
}

bool GlobalVariables::using_udp()    // returns true if any messages need udp
{
  for(int i = 0; i <= NUM_MESSAGE_TYPES; i++)  
  {
    if(protocol[i] == 0)
      return true;
  }
  return false;     
}

void extract_and_publish_message_type(int message_type, size_t buffer_ptr, size_t buffer_max, GlobalVariables* G, int sending_agent) // extracts the message denoted by type starting at buffer_ptr and publishes on approperiate topic
{   
  if(message_type == 0 && G->listen_list[0]) // it is a pose message
  {
    if(G->listen_mode_list[0] == 0)
    {
      geometry_msgs::PoseStamped msg;
      buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      pose_pub.publish(msg);  
    }
    else if(G->listen_mode_list[0] == 1)
    {
      intercom_cu::PoseStamped_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg.data, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      pose_pub.publish(msg);
    }
  }
  else if(message_type == 1 && G->listen_list[1]) // it is a goal message
  {
    if(G->listen_mode_list[1] == 0)
    {
      geometry_msgs::PoseStamped msg;
      buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      goal_pub.publish(msg);
    }
    else if(G->listen_mode_list[1] == 1)
    {
      intercom_cu::PoseStamped_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg.data, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      goal_pub.publish(msg);  
    }
  } 
  else if(message_type == 2 && G->listen_list[2]) // it is a system state message
  {
    if(G->listen_mode_list[2] == 0)
    {
      std_msgs::Int32 msg;
      buffer_ptr = extract_from_buffer_int(buffer_ptr, msg.data, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      system_state_pub.publish(msg);
    }
    else if(G->listen_mode_list[2] == 1)
    {
      intercom_cu::Int32_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_int(buffer_ptr, msg.data.data, buffer_max); 
      if(buffer_ptr == 0)
         return; 
      
      system_state_pub.publish(msg);
    }
  } 
  else if(message_type == 3 && G->listen_list[3]) // it is a system update message
  {
    if(G->listen_mode_list[3] == 0)
    {
      std_msgs::Int32 msg;
      buffer_ptr = extract_from_buffer_int(buffer_ptr, msg.data, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      system_update_pub.publish(msg);
    }
    else if(G->listen_mode_list[3] == 1)
    {
      intercom_cu::Int32_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_int(buffer_ptr, msg.data.data, buffer_max); 
      if(buffer_ptr == 0)
         return; 
      
      system_update_pub.publish(msg);
    }
  }
  else if(message_type == 4 && G->listen_list[4]) // it is a map changes message
  {
    if(G->listen_mode_list[4] == 0)
    {
      sensor_msgs::PointCloud msg;
      buffer_ptr = extract_from_buffer_PointCloud(buffer_ptr, msg, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      map_changes_pub.publish(msg);
    }
    else if(G->listen_mode_list[4] == 1)
    {
      intercom_cu::PointCloud_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_PointCloud(buffer_ptr, msg.data, buffer_max); 
      if(buffer_ptr == 0)
         return; 
      
      map_changes_pub.publish(msg);   
    }
  } 
  else if(message_type == 5 && G->listen_list[5]) // it is a global path message
  {
    if(G->listen_mode_list[5] == 0)
    {
      nav_msgs::Path msg;
      buffer_ptr = extract_from_buffer_Path(buffer_ptr, msg, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      global_path_pub.publish(msg);
    }
    else if(G->listen_mode_list[5] == 1)
    {
      intercom_cu::Path_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_Path(buffer_ptr, msg.data, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      global_path_pub.publish(msg);  
    }
  }
  else if(message_type == 6 && G->listen_list[6]) // it is a goal reset message
  {
    if(G->listen_mode_list[6] == 0)
    {
      geometry_msgs::PoseStamped msg;
      buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      new_goal_pub.publish(msg);
    }
    else if(G->listen_mode_list[6] == 1)
    {
      intercom_cu::PoseStamped_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg.data, buffer_max);
      if(buffer_ptr == 0)
         return;
        
      new_goal_pub.publish(msg); 
    }
  } 
  else if(message_type == 7 && G->listen_list[7]) // it is a pose reset message
  {
    if(G->listen_mode_list[7] == 0)
    {
      geometry_msgs::PoseStamped msg;
      buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      new_pose_pub.publish(msg);
    }
    else if(G->listen_mode_list[7] == 1)
    {
      intercom_cu::PoseStamped_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg.data, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      new_pose_pub.publish(msg);
    }
  } 
  else if(message_type == 8 && G->listen_list[8]) // it is a user control message
  {
    if(G->listen_mode_list[8] == 0)
    {
      geometry_msgs::Pose2D msg;
      buffer_ptr = extract_from_buffer_Pose2D(buffer_ptr, msg, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      user_control_pub.publish(msg);
    }
    else if(G->listen_mode_list[8] == 1)
    {
      intercom_cu::Pose2D_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_Pose2D(buffer_ptr, msg.data, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      user_control_pub.publish(msg);   
    }
  }
  else if(message_type == 9 && G->listen_list[9]) // it is a user state message
  {
    if(G->listen_mode_list[9] == 0)
    {
      std_msgs::Int32 msg;
      buffer_ptr = extract_from_buffer_int(buffer_ptr, msg.data, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      user_state_pub.publish(msg);
    }
    else if(G->listen_mode_list[9] == 1)
    {
      intercom_cu::Int32_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_int(buffer_ptr, msg.data.data, buffer_max); 
      if(buffer_ptr == 0)
         return; 
      
      user_state_pub.publish(msg);
    }
  }
  else if(message_type == 10 && G->listen_list[10]) // it is a laser scan message
  {
    if(G->listen_mode_list[10] == 0)
    {
      hokuyo_listener_cu::PointCloudWithOrigin msg;
      buffer_ptr = extract_from_buffer_PointCloudWithOrigin(buffer_ptr, msg, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      laser_scan_pub.publish(msg);
    }
    else if(G->listen_mode_list[10] == 1)
    {
      intercom_cu::PointCloudWithOrigin_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_PointCloudWithOrigin(buffer_ptr, msg.data, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      laser_scan_pub.publish(msg);    
    }
  }
  else if(message_type == 11 && G->listen_list[11]) // it is a map service request message
  {
    if(G->listen_mode_list[11] == 0)
    {
      nav_msgs::GetMap::Request  req;
      nav_msgs::GetMap::Response resp;
      
      if(G->send_list[12])
      {
        if(ros::service::call("/cu/get_map_cu", req, resp) )
        {
          // send response back to the sending agent
          uint this_msg_size = max_message_size; 
          char buffer[this_msg_size];
          size_t buffer_ptr = (size_t)buffer;
          size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
   
          buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 12, 0, 0, 0, buffer_max); // add space for ethernet header 
          buffer_ptr = add_to_buffer_OccupancyGrid(buffer_ptr, resp.map, buffer_max);                    // add occupancy grid
            
          //printf("sending message type 12 to %d at %s\n", sending_agent, G->OtherIPs[2].c_str());
          Globals.send_to_agent(buffer, buffer_ptr-(size_t)buffer, sending_agent);
        }
      }
    }
    else if(G->listen_mode_list[11] == 1)
    {

    }
  }
  else if(message_type == 12 && G->listen_list[12]) // it is a map service response message
  {
    if(!Globals.service_received_map)
    {
      if(G->listen_mode_list[12] == 0)
      {
        // store in globals so service provider can access it
        buffer_ptr = extract_from_buffer_OccupancyGrid(buffer_ptr, Globals.service_response_map, buffer_max);
        if(buffer_ptr == 0)
         return;
        
        Globals.service_received_map = true;
      }
      else if(G->listen_mode_list[12] == 1)
      {
          
      }
    }
  }
  else if(message_type == 13 && G->listen_list[13]) // it is a "/map_cu" -> "/world_cu" transform
  {
    if(using_tf)
    {
      if(G->listen_mode_list[13] == 0)
      {
        static tf::TransformBroadcaster br;  
        tf::StampedTransform transform;   

        buffer_ptr = extract_from_buffer_StampedTransform(buffer_ptr, transform, buffer_max);
        if(buffer_ptr == 0)
         return;
        
        transform.frame_id_ = std::string("/map_cu");
        transform.child_frame_id_ = std::string("/world_cu");
        
        br.sendTransform(transform);
      }
      else if(G->listen_mode_list[13] == 1)
      {
          
      }
    }
  }
  else if(message_type == 14 && G->listen_list[14]) // it is a target pose
  {
    if(G->listen_mode_list[14] == 0)
    {
      geometry_msgs::Pose2D msg;
      buffer_ptr = extract_from_buffer_Pose2D(buffer_ptr, msg, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      target_pose_pub.publish(msg);
    }
    else if(G->listen_mode_list[14] == 1)
    {
      intercom_cu::Pose2D_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_Pose2D(buffer_ptr, msg.data, buffer_max); 
      if(buffer_ptr == 0)
         return;

      target_pose_pub.publish(msg);   
    }
  }
  else if(message_type == 15 && G->listen_list[15]) // it is a planning area
  {
    if(G->listen_mode_list[15] == 0)
    {
      geometry_msgs::Polygon msg;
      buffer_ptr = extract_from_buffer_Polygon(buffer_ptr, msg, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      planning_area_pub.publish(msg);
    }
    else if(G->listen_mode_list[15] == 1)
    {
      intercom_cu::Polygon_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_Polygon(buffer_ptr, msg.data, buffer_max); 
      if(buffer_ptr == 0)
         return;

      planning_area_pub.publish(msg);   
    }
  }
  else if(message_type == 16 && G->listen_list[16]) // it is a turn circle
  {
    if(G->listen_mode_list[16] == 0)
    {
      geometry_msgs::Pose2D msg;
      buffer_ptr = extract_from_buffer_Pose2D(buffer_ptr, msg, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      turn_circle_pub.publish(msg);
    }
    else if(G->listen_mode_list[16] == 1)
    {
      intercom_cu::Pose2D_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_Pose2D(buffer_ptr, msg.data, buffer_max); 
      if(buffer_ptr == 0)
         return;

      turn_circle_pub.publish(msg);   
    }
  }
  else if(message_type == 17 && G->listen_list[17]) // it is an obstacle array
  {
    if(G->listen_mode_list[17] == 0)
    {
      multi_robot_planner_cu::PolygonArray msg;
      buffer_ptr = extract_from_buffer_PolygonArray(buffer_ptr, msg, buffer_max); 
      if(buffer_ptr == 0)
         return;
      
      obstacles_pub.publish(msg);
    }
    else if(G->listen_mode_list[17] == 1)
    {
      intercom_cu::PolygonArray_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_PolygonArray(buffer_ptr, msg.data, buffer_max); 
      if(buffer_ptr == 0)
         return;

      obstacles_pub.publish(msg);   
    }
  }
  else if(message_type == 18 && G->listen_list[18]) // it is a time ahead message
  {
    if(G->listen_mode_list[18] == 0)
    {
      std_msgs::Float32 msg;
      buffer_ptr = extract_from_buffer_float(buffer_ptr, msg.data, buffer_max);
      if(buffer_ptr == 0)
         return;
      
      time_ahead_pub.publish(msg);
    }
    else if(G->listen_mode_list[18] == 1)
    {
      intercom_cu::Float32_CU_ID msg;
      msg.id.data = sending_agent;
      buffer_ptr = extract_from_buffer_float(buffer_ptr, msg.data.data, buffer_max); 
      if(buffer_ptr == 0)
         return; 
      
      time_ahead_pub.publish(msg);
    }
  }
}  
    
    



/*-------------------------- UDP listner thread -------------------------*/
void *Listner_UDP(void * inG)
{
  GlobalVariables* G = (GlobalVariables*)inG;  
  char network_message_buffer[max_network_message_size]; // used for a single message
  char large_message_buffer[max_message_size];           // used if messages need to be broken up becasue they are too large
  char* message_buffer;                                  // used to switch between either of the above
  struct sockaddr_in senders_address;
  
  // calculate the header size
  size_t buffer_ptr = (size_t)network_message_buffer;
  size_t buffer_max = buffer_ptr + (size_t)max_network_message_size;
  size_t header_size = add_to_buffer_ethernetheader((size_t)network_message_buffer, 0,0,0,0,0, buffer_ptr + (size_t)max_network_message_size) - buffer_ptr;
  size_t adjusted_network_data_size = max_network_message_size - header_size;
  
  while(true)
  {    
    int senders_address_length = sizeof(struct sockaddr_in);  // get the memory size of a sockaddr_in struct  
    memset(&network_message_buffer,'\0',sizeof(network_message_buffer)); 
    int message_length = recvfrom(G->MyInSockUDP, network_message_buffer, sizeof(network_message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks untill a message is recieved
    //if(message_length < 0) 
    //  printf("had problems getting a message \n");
    
    buffer_max = (size_t)network_message_buffer + (size_t)max_network_message_size;
    
    //extract header elements
    int sending_agent;
    uint message_type;
    uint sent_message_counter;
    uint total_packets;
    uint packet_number;
    buffer_ptr = extract_from_buffer_ethernetheader((size_t)network_message_buffer, sending_agent, message_type, sent_message_counter, total_packets, packet_number, buffer_max); // extracts an ethernet header from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer

    //printf("recieved message %u: %u, %u of %u \n", message_type, sent_message_counter, packet_number, total_packets);
    
    if(total_packets > 1) // this message has more packets still to come
    {
      // copy this into where it should go in the large message buffer 
      memcpy(large_message_buffer + header_size + (((size_t)packet_number)*adjusted_network_data_size), (void *)buffer_ptr, adjusted_network_data_size); 
        
      //set up structure to record what we have received
      vector<bool> msg_rec(total_packets, false);
      msg_rec[packet_number] = true;
              
      // now we try to get the rest of the message
      bool keep_going = true;
      while(keep_going)
      { 
        int senders_address_length_b = sizeof(struct sockaddr_in);  // get the memory size of a sockaddr_in struct 
        memset(&network_message_buffer,'\0',sizeof(network_message_buffer)); 
        int message_length = recvfrom(G->MyInSockUDP, network_message_buffer, sizeof(network_message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length_b);  // blocks untill a message is recieved
        //if(message_length < 0) 
        //  printf("had problems getting a message \n");
    
        buffer_max = (size_t)network_message_buffer + (size_t)max_network_message_size;
        
        //extract header elements
        int sending_agent_b;
        uint message_type_b;
        uint sent_message_counter_b;
        uint total_packets_b;
        uint packet_number_b;
        buffer_ptr = extract_from_buffer_ethernetheader((size_t)network_message_buffer, sending_agent_b, message_type_b, sent_message_counter_b, total_packets_b, packet_number_b, buffer_max); // extracts an ethernet header from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer

        //printf("recieved message %u(%u): %u, %u of %u \n", message_type_b, message_type, sent_message_counter_b, packet_number_b, total_packets_b);
        
        if(sending_agent_b != sending_agent || sent_message_counter_b < sent_message_counter)
        {
          //printf("continuing \n");
          continue;
        }
        
        //MAYBE CHANGE THE FOLLOWING TO A TIMEOUT
        if(sent_message_counter_b > sent_message_counter + total_packets*1.5+1) // then we conclude we failed to get all of the message 
        {
          //printf("failed to receive all packets \n");
          message_type = -1;  
          break;
        }        
        
        if(sending_agent_b != sending_agent || message_type_b != message_type || total_packets_b != total_packets)
        {
          //printf("continuing \n");
          continue;
        }
        
        //printf("waiting for %u > %u \n", sent_message_counter_b, sent_message_counter + total_packets);
        
        // copy this into where it should go in the large message buffer 
        memcpy(large_message_buffer + header_size + (((size_t)packet_number_b)*adjusted_network_data_size), (void*)buffer_ptr, adjusted_network_data_size);
        msg_rec[packet_number_b] = true;
        
        keep_going = false;
        for(uint j = 0; j < total_packets; j++)
        {
          if(! msg_rec[j]) // still need to receive a packet  
          {
            keep_going = true;
            break;
          }
        }
      }
     
      message_buffer = large_message_buffer;
      buffer_ptr = (size_t)large_message_buffer + header_size;
      buffer_max = buffer_ptr + (size_t)max_message_size - header_size;
    }
    else // single message is in network_message_buffer
    {
      message_buffer = network_message_buffer;
      buffer_ptr = (size_t)network_message_buffer + header_size;
      buffer_max = buffer_ptr + adjusted_network_data_size;      
    }
    
    //printf("--recieved message type %u from %d \n", message_type, sending_agent); 

    extract_and_publish_message_type(message_type, buffer_ptr, buffer_max, G, sending_agent);
  }
}   
    
    

/*-------------------------- TCP listner thread -------------------------*/
// this class stores the stuff that needs to be sent to a (TCP) thread
class ThreadHelper
{
  public:
   GlobalVariables* G;  // pointer to the global variables structure
   int listen_id;       // the id of the robot this is listening for
};
vector<ThreadHelper> TCPThreadHelpers; // resized and populated once we know how many agents there are

void *Listner_TCP(void * inT)
{
  ThreadHelper* T = (ThreadHelper*)inT;
  GlobalVariables* G = T->G;  
  int i = T->listen_id;
  
  char network_message_buffer[max_message_size];
  
  int l = listen(G->MyInSocksTCP[i],5);
  if(l < 0)
  {
    printf("L%d error listining to socket %d\n", i, G->MyInSocksTCP[i]);
    G->set_up_single_IncommingTCP(i); 
  }
     
  int newsockfd = accept(G->MyInSocksTCP[i], NULL, NULL);
  if(newsockfd < 0)
  {
    printf("L%d ", i);
    error("error accepting connection");
    G->set_up_single_IncommingTCP(i);
  }
  
  while(true)
  {
    int n = -1;
    n = read(newsockfd,network_message_buffer,max_message_size-1);
    if(n < 0) 
    {
      printf("L%d ", i);
      error("error reading from socket");
      G->set_up_single_IncommingTCP(i);
      continue;
    }
   
    //extract header elements
    int sending_agent;
    uint message_type;
    uint sent_message_counter;
    uint total_bytes;
    uint packet_number;
    size_t buffer_max = (size_t)network_message_buffer + max_message_size;
    size_t buffer_ptr = extract_from_buffer_ethernetheader((size_t)network_message_buffer, sending_agent, message_type, sent_message_counter, total_bytes, packet_number, buffer_max); // extracts an ethernet header from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer

    // read the rest of the message
    while((uint)n < total_bytes-1)
    {
      int m = read(newsockfd,network_message_buffer+n,max_message_size-1);
      if(m < 0) 
      {
        printf("L%d ", i);
        error("error reading from socket");
        G->set_up_single_IncommingTCP(i);
        break;
      }
      //printf("waiting for the rest \n");
      
      n += m;
    }
    
    if((uint)n >= total_bytes-1)
    {
      //printf("L%d received message %u: %u, %u of %u --- %d\n", i, message_type, sent_message_counter, packet_number, total_bytes, max_message_size-1);
      extract_and_publish_message_type(message_type, buffer_ptr, buffer_max, G, sending_agent);
    }
  }
}


/*----------------------- ROS Callbacks ---------------------------------*/
void selected_robot_callback(const std_msgs::Int32::ConstPtr& msg)
{        
  Globals.target_agent = msg->data;
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if(msg->header.frame_id != "/map_cu")
      ROS_INFO("Received unknown pose message");
    
  uint this_msg_size = 500; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 0,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_PoseStamped(buffer_ptr, *msg, buffer_max);                      // add posestamped

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 0);                           // send 
    
  //printf("sending:\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n\n",msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z );
  //double secs = msg->header.stamp.toSec();
  //printf("header:\n %d, %s, %f\n", (int)msg->header.seq, msg->header.frame_id.c_str(), secs);
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
  uint this_msg_size = 500; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 1,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_PoseStamped(buffer_ptr, *msg, buffer_max);                      // add posestamped
  
  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 1);                           // send 
}

void system_state_callback(const std_msgs::Int32::ConstPtr& msg)
{        
  uint this_msg_size = 100; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 2,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_int(buffer_ptr,  msg->data, buffer_max);                        // add int

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 2);                           // send   
}

void system_update_callback(const std_msgs::Int32::ConstPtr& msg)
{      
  uint this_msg_size = 100; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 3,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_int(buffer_ptr,  msg->data, buffer_max);                        // add int

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 3);                           // send 
}

void map_changes_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{    
  uint this_msg_size = max_message_size; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 4,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_PointCloud(buffer_ptr, *msg, buffer_max);                       // add pointcloud

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 4);                           // send 
  
}
void global_plan_callback(const nav_msgs::Path::ConstPtr& msg)
{     
  uint this_msg_size = max_message_size; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 5,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_Path(buffer_ptr, *msg, buffer_max);                             // add path

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 5);                           // send 
}

void goal_reset_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
  uint this_msg_size = 500; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 6,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_PoseStamped(buffer_ptr, *msg, buffer_max);                      // add posestamped

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 6);                           // send 
}

void pose_reset_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
  uint this_msg_size = 500; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 7,0,0,0, buffer_max); // add space for ethernet header
  buffer_ptr = add_to_buffer_PoseStamped(buffer_ptr, *msg, buffer_max);                      // add posestamped

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 7);                           // send 
}

void user_control_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{    
  uint this_msg_size = 500; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 8,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_Pose2D(buffer_ptr, *msg, buffer_max);                           // add posestamped

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 8);                           // send 
}

void user_state_callback(const std_msgs::Int32::ConstPtr& msg)
{        
  uint this_msg_size = 100; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 9,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_int(buffer_ptr,  msg->data, buffer_max);                        // add int

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 9);                           // send 
}

 
void laser_scan_callback(const hokuyo_listener_cu::PointCloudWithOrigin::ConstPtr& msg)
{    
  uint this_msg_size = max_message_size; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 10,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_PointCloudWithOrigin(buffer_ptr, *msg, buffer_max);              // add pointcloud with origin

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 10);                           // send   
}

bool get_map_callback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &resp)
{
  // need to send message to client requesting map
  ros::Rate loop_rate(5);
  
  uint this_msg_size = 100; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 11,0,0,0, buffer_max); // add space for ethernet header 
  
  Globals.service_received_map = false;
  
  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 11);                         // send 
  while(!Globals.service_received_map) // wait for response
  {
    if(Globals.protocol[11] == 0) // being sent via UDP so may be dropped (NOTE: if TCP is used, i.e. protocol[12] == 1, then we assume the return message is also sent TCP) 
      Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 11);                     // re-send 
    
    loop_rate.sleep();
  }
  
  // now the map info is available in Globals.service_response_map
  resp.map = Globals.service_response_map;
  
  return true;
}


void transform_sender(const tf::StampedTransform& t)
{      
  uint this_msg_size = 500; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 13,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_StampedTransform(buffer_ptr, t, buffer_max);                     // add StampedTransform 
         
  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 13);                           // send 
}

void target_pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{    
  uint this_msg_size = 500; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 14,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_Pose2D(buffer_ptr, *msg, buffer_max);                            // add posestamped

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 14);                           // send 
}

void planning_area_callback(const geometry_msgs::Polygon::ConstPtr& msg)
{    
  uint this_msg_size = max_message_size; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 15,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_Polygon(buffer_ptr, *msg, buffer_max);                           // add posestamped

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 15);                           // send 
}

void turn_circle_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{    
  uint this_msg_size = 500; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 16,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_Pose2D(buffer_ptr, *msg, buffer_max);                            // add posestamped

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 16);                           // send 
}

void obstacles_callback(const multi_robot_planner_cu::PolygonArray::ConstPtr& msg)
{    
  uint this_msg_size = max_message_size; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 17,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_PolygonArray(buffer_ptr, *msg, buffer_max);                      // add posestamped
  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 17);                           // send 
}

void time_ahead_callback(const std_msgs::Float32::ConstPtr& msg)
{      
  uint this_msg_size = 100; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 18,0,0,0, buffer_max); // add space for ethernet header 
  buffer_ptr = add_to_buffer_float(buffer_ptr,  msg->data, buffer_max);                       // add float

  Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 18);                           // send 
}

int main(int argc, char * argv[]) 
{    
  // init ROS
  ros::init(argc, argv, "intercom_cu");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
 
  int my_id_default = 0;
  int num_agents_default = 1;
  std::string config_file("config.xml");
  
  // load globals from command line
  if(argc > 1)
    my_id_default = atoi(argv[1]);      // this robots id
  if(argc > 2)    
    num_agents_default = atoi(argv[2]); // total number of agents
  
  // load (or overwrite) globals from parameter server
  bool bool_input;
  int int_input;
  std::string string_input;
  if(ros::param::get("intercom_cu/my_id", int_input)) 
    my_id_default = int_input;          // this robot's id
  if(ros::param::get("intercom_cu/num_agents", int_input)) 
    num_agents_default = int_input;     // total number of agents
  if(ros::param::get("intercom_cu/config_file", string_input))                       
    config_file = string_input;         // the config file with ip and other data
  if(ros::param::get("prairiedog/using_tf", bool_input)) 
    using_tf = bool_input;              // when set to true, use the tf package 
  
  printf("I am agent %d of %d \n", my_id_default, num_agents_default); 
    
  Globals.Populate(my_id_default, num_agents_default);
  Globals.read_IPS_from_file(config_file);
  
  Globals.set_up_MyAddressUDP();
  Globals.set_up_OtherAddressesUDP();
  
  Globals.set_up_IncommingTCP();
  //Globals.set_up_OutgoingTCP();  // OUTGOING CONNECTION HAVE BEEN MOVED TO SET UP ONLY WHEN NEEDED DUE TO BLOCKING PROBLEMS
    
  // set up ROS topic subscriber callbacks
  selected_robot_sub = nh.subscribe("/cu/selected_robot_cu", 1, selected_robot_callback);
  if(Globals.send_list[0])
    pose_sub = nh.subscribe("/cu/pose_cu", 1, pose_callback);
  if(Globals.send_list[1]) 
    goal_sub = nh.subscribe("/cu/goal_cu", 1, goal_callback);
  if(Globals.send_list[2])
    system_state_sub = nh.subscribe("/cu/system_state_cu", 10, system_state_callback);
  if(Globals.send_list[3])
    system_update_sub = nh.subscribe("/cu/system_update_cu", 10, system_update_callback);
  if(Globals.send_list[4])
    map_changes_sub = nh.subscribe("/cu/map_changes_cu", 10, map_changes_callback);
  if(Globals.send_list[5])
    global_path_sub = nh.subscribe("/cu/global_path_cu", 2, global_plan_callback);
  if(Globals.send_list[6])
    new_goal_sub = nh.subscribe("/cu/reset_goal_cu", 1, goal_reset_callback);
  if(Globals.send_list[7])
    new_pose_sub = nh.subscribe("/cu/user_pose_cu", 1, pose_reset_callback); 
  if(Globals.send_list[8])
    user_control_sub = nh.subscribe("/cu/user_control_cu", 1, user_control_callback);
  if(Globals.send_list[9])
    user_state_sub = nh.subscribe("/cu/user_state_cu", 1, user_state_callback); 
  if(Globals.send_list[10])
    laser_scan_sub = nh.subscribe("/cu/laser_scan_cu", 1, laser_scan_callback);
  if(Globals.send_list[14])
    target_pose_sub = nh.subscribe("/cu/target_pose_cu", 1, target_pose_callback);
  if(Globals.send_list[15])
    planning_area_sub = nh.subscribe("/cu/planning_area_cu", 1, planning_area_callback);
  if(Globals.send_list[16])
    turn_circle_sub = nh.subscribe("/cu/turn_circle_cu", 1, turn_circle_callback);
  if(Globals.send_list[17])
    obstacles_sub = nh.subscribe("/cu/obstacles_cu", 1, obstacles_callback);
  if(Globals.send_list[18])
    time_ahead_sub = nh.subscribe("/cu/time_ahead_cu", 1, time_ahead_callback);

  // set up ROS topic publishers
  if(Globals.listen_list[0])
  { 
    if(Globals.listen_mode_list[0] == 0)
      pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/pose_cu", 1);
    else if(Globals.listen_mode_list[0] == 1)
      pose_pub = nh.advertise<intercom_cu::PoseStamped_CU_ID>("/cu_multi/pose_cu", 1);
  }
  if(Globals.listen_list[1])
  {
    if(Globals.listen_mode_list[1] == 0)
      goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/goal_cu", 1);
    else if(Globals.listen_mode_list[1] == 1)
      goal_pub = nh.advertise<intercom_cu::PoseStamped_CU_ID>("/cu_multi/goal_cu", 1);
  }
  if(Globals.listen_list[2])
  {
    if(Globals.listen_mode_list[2] == 0)
      system_state_pub = nh.advertise<std_msgs::Int32>("/cu/system_state_cu", 1);
    else if(Globals.listen_mode_list[2] == 1)
      system_state_pub = nh.advertise<intercom_cu::Int32_CU_ID>("/cu_multi/system_state_cu", 1);
  }
  if(Globals.listen_list[3])
  {
    if(Globals.listen_mode_list[3] == 0)  
      system_update_pub = nh.advertise<std_msgs::Int32>("/cu/system_update_cu", 1);
    else if(Globals.listen_mode_list[3] == 1)
      system_update_pub = nh.advertise<intercom_cu::Int32_CU_ID>("/cu_multi/system_update_cu", 1);
  }
  if(Globals.listen_list[4])
  {
    if(Globals.listen_mode_list[4] == 0)  
      map_changes_pub = nh.advertise<sensor_msgs::PointCloud>("/cu/map_changes_cu", 1);
    else if(Globals.listen_mode_list[4] == 1)
      map_changes_pub = nh.advertise<intercom_cu::PointCloud_CU_ID>("/cu_multi/map_changes_cu", 1);
  }
  if(Globals.listen_list[5])
  {
    if(Globals.listen_mode_list[5] == 0)  
      global_path_pub = nh.advertise<nav_msgs::Path>("/cu/global_path_cu", 1);
    else if(Globals.listen_mode_list[5] == 1)  
      global_path_pub = nh.advertise<intercom_cu::Path_CU_ID>("/cu_multi/global_path_cu", 1);
  }
  if(Globals.listen_list[6])
  {
    if(Globals.listen_mode_list[6] == 0)  
      new_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/reset_goal_cu", 1);
    else if(Globals.listen_mode_list[6] == 1)  
      new_goal_pub = nh.advertise<intercom_cu::PoseStamped_CU_ID>("/cu_multi/reset_goal_cu", 1);
  }
  if(Globals.listen_list[7])
  {
    if(Globals.listen_mode_list[7] == 0)  
      new_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/user_pose_cu", 1);
    else if(Globals.listen_mode_list[7] == 1)
      new_pose_pub = nh.advertise<intercom_cu::PoseStamped_CU_ID>("/cu_multi/user_pose_cu", 1);
  }
  if(Globals.listen_list[8])
  {
    if(Globals.listen_mode_list[8] == 0) 
      user_control_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/user_control_cu", 1);
    else if(Globals.listen_mode_list[8] == 1) 
      user_control_pub = nh.advertise<intercom_cu::Pose2D_CU_ID>("/cu_multi/user_control_cu", 1);
  }
  if(Globals.listen_list[9])
  {
    if(Globals.listen_mode_list[9] == 0) 
      user_state_pub = nh.advertise<std_msgs::Int32>("/cu/user_state_cu", 1);
    else if(Globals.listen_mode_list[9] == 1) 
      user_state_pub = nh.advertise<intercom_cu::Int32_CU_ID>("/cu_multi/user_state_cu", 1);
  }
  if(Globals.listen_list[10])
  {
    if(Globals.listen_mode_list[10] == 0) 
      laser_scan_pub = nh.advertise<hokuyo_listener_cu::PointCloudWithOrigin>("/cu/laser_scan_cu", 1);
    else if(Globals.listen_mode_list[10] == 1) 
      laser_scan_pub = nh.advertise<intercom_cu::PointCloudWithOrigin_CU_ID>("/cu_multi/laser_scan_cu", 1);
  }
  if(Globals.listen_list[14])
  {
    if(Globals.listen_mode_list[14] == 0) 
      target_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/target_pose_cu", 1);
    else if(Globals.listen_mode_list[14] == 1) 
      target_pose_pub = nh.advertise<intercom_cu::Pose2D_CU_ID>("/cu_multi/target_pose_cu", 1);
  }
  if(Globals.listen_list[15])
  {
    if(Globals.listen_mode_list[15] == 0) 
      planning_area_pub = nh.advertise<geometry_msgs::Polygon>("/cu/planning_area_cu", 1);
    else if(Globals.listen_mode_list[15] == 1) 
      planning_area_pub = nh.advertise<intercom_cu::Polygon_CU_ID>("/cu_multi/planning_area_cu", 1);
  }
  if(Globals.listen_list[16])
  {
    if(Globals.listen_mode_list[16] == 0) 
      turn_circle_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/turn_circle_cu", 1);
    else if(Globals.listen_mode_list[16] == 1) 
      turn_circle_pub = nh.advertise<intercom_cu::Pose2D_CU_ID>("/cu_multi/turn_circle_cu", 1);
  }
  if(Globals.listen_list[17])
  {
    if(Globals.listen_mode_list[16] == 0) 
      obstacles_pub = nh.advertise<multi_robot_planner_cu::PolygonArray>("/cu/obstacles_cu", 1);
    else if(Globals.listen_mode_list[16] == 1) 
      obstacles_pub = nh.advertise<intercom_cu::PolygonArray_CU_ID>("/cu_multi/obstacles_cu", 1);
  }
  if(Globals.listen_list[18])
  {
    if(Globals.listen_mode_list[18] == 0)  
      time_ahead_pub = nh.advertise<std_msgs::Float32>("/cu/time_ahead_cu", 1);
    else if(Globals.listen_mode_list[18] == 1)
      time_ahead_pub = nh.advertise<intercom_cu::Float32_CU_ID>("/cu_multi/time_ahead_cu", 1);
  }  

  // set up service servers
  if(Globals.send_list[11])
    get_map_srv = nh.advertiseService("/cu/get_map_cu", get_map_callback);
  
  //kick off udp listener thread
  pthread_t Listener_thread_UDP;
  pthread_create(&Listener_thread_UDP, NULL, Listner_UDP, &Globals); // listens for incomming messages on udp

  // populate TCPThreadHelpers
  TCPThreadHelpers.resize(Globals.num_agents);
  for(int i = 0; i < Globals.num_agents; i++)
  {
    TCPThreadHelpers[i].G = &Globals;
    TCPThreadHelpers[i].listen_id = i;
  }
  
  // kick off TCP threads (one for each robot to listen for
  vector<pthread_t> Listener_threads_TCP(Globals.num_agents);
  for(int i = 0; i < Globals.num_agents; i++)
    pthread_create(&(Listener_threads_TCP[i]), NULL, Listner_TCP, &(TCPThreadHelpers[i])); // listens for incomming messages on tcp
  
  // 'prime' transform listners
  bool setup_tf = false;             // flag used to help init tf
  static tf::TransformListener listener_map_world;  
  tf::StampedTransform transform_map_world; 
  ros::Time get_most_recent(0);
  
  
  if(Globals.send_list[13])
  {
    if(using_tf)
    {
      while(setup_tf)  // there is usually a problem looking up the first transform, so do this to avoid that
      {
        try
        {
          // "/map_cu" -> "/world_cu"
          listener_map_world.waitForTransform("/map_cu", "/world_cu", ros::Time(0), ros::Duration(3.0));
          listener_map_world.lookupTransform(std::string("/map_cu"), std::string("/world_cu"), get_most_recent, transform_map_world);
        }
        catch(tf::TransformException ex)
        { 
          //printf("attempt failed \n");
          ROS_ERROR("intercom_cu: %s",ex.what());
          setup_tf = true;  
        }   
      } 
    }
  }
  
  while (nh.ok()) 
  {   
    // listen for transforms
    if(Globals.send_list[13])
    {
      if(using_tf)
      {
        // if using tf then we want to send in the world_cu frame
        bool no_problems_with_transform = true;
        try
        {  
          // "/map_cu" -> "/world_cu" transform
          listener_map_world.lookupTransform(std::string("/map_cu"), std::string("/world_cu"), get_most_recent, transform_map_world);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("client_server: %s",ex.what());
          no_problems_with_transform = false;
        }
        
        if(no_problems_with_transform) 
          transform_sender(transform_map_world);  
      } 
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  } // end main control loop
  
  
  
  // destroy subscribers
  selected_robot_sub.shutdown();
  
  new_goal_sub.shutdown();
  new_pose_sub.shutdown();
  user_control_sub.shutdown();
  user_state_sub.shutdown();
 
  pose_sub.shutdown();
  global_path_sub.shutdown();
  goal_sub.shutdown();
  laser_scan_sub.shutdown();
  map_changes_sub.shutdown();
  system_state_sub.shutdown();
  system_update_sub.shutdown();
  
  target_pose_sub.shutdown();
  planning_area_sub.shutdown();
  turn_circle_sub.shutdown();
  obstacles_sub.shutdown();
       
  time_ahead_sub.shutdown();
   
  // destroy publishers
  pose_pub.shutdown();
  global_path_pub.shutdown();
  goal_pub.shutdown();
  laser_scan_pub.shutdown();
  map_changes_pub.shutdown();
  system_state_pub.shutdown();
  system_update_pub.shutdown();
  
  new_goal_pub.shutdown();
  new_pose_pub.shutdown();
  user_control_pub.shutdown();
  user_state_pub.shutdown();
  
  target_pose_pub.shutdown();
  planning_area_pub.shutdown();
  turn_circle_pub.shutdown();
  obstacles_pub.shutdown();
          
  time_ahead_pub.shutdown();

  // destroy service providers
  get_map_srv.shutdown();
  
  return 0;
}
