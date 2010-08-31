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
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point32.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/GridCells.h"

#include "sensor_msgs/PointCloud.h"

#include "move_base_msgs/MoveBaseActionGoal.h"

#include "hokuyo_listener_cu/PointCloudWithOrigin.h"

#include "std_msgs/Int32.h"


#include "intercom_cu/PoseStamped_CU_ID.h"

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
   bool set_up_OtherAddresseses();                         // sets other address data
   bool set_up_MyAddress();                                // sets up outgoing socket
   
   void send_to_agent(void* buffer, size_t buffer_size, int ag);                    // sends data to agent ag
   void send_to_all_agents(void* buffer, size_t buffer_size);                       // sends data to all other agents
   void send_to_destination_matrix(void* buffer, size_t buffer_size, int msg_type); // sends data to all destination_matrix[msg_type]==true
   void send_message_type(void* buffer, size_t buffer_size, int type);              // sends data of message-type send_mode_list as defined by send_mode_list[type]
   
   // ip addresses
   std::string MyIP;
   vector<std::string> OtherIPs;
      
   // ports
   int MyInPort;
   int MyOutPort;
   
   vector<int> OtherInPorts;
   
   // socket numbers
   int  MyInSock;
   int  MyOutSock;
      
   // addresses
   struct sockaddr_in MyAddress;
   vector<struct sockaddr_in> OtherAddresses;
   
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
   
   // message types we recieve 'listen_list[i] = true' denotes message type i is listened for (via udp)
   vector<bool> listen_list;
   vector<int>  listen_mode_list;  //0=publish original message , 1=wrap original message so that message contains sending robot's id and send on /cu_multi/ topic instead
   
   // global inter-thread flags and storage for service interaction
   bool service_received_map;
   nav_msgs::OccupancyGrid service_response_map;
   
   // counter for messages that require being chopped up
   uint message_counter;
};
GlobalVariables Globals;

GlobalVariables::GlobalVariables()  
{   
  my_id = -1;
  
  MyOutSock = -1;
  MyInSock = -1;
  
  num_agents = 0;
  
  target_agent = 0;
  
  service_received_map = false;
  
  message_counter = 0;
}

GlobalVariables::GlobalVariables(int id, int total_agents)  
{  
  my_id = id;  
  
  MyInSock = -1;
  MyOutSock = -1;
  
  num_agents = total_agents;
  
  service_received_map = false;
       
  MyInPort = 57001 + id;
  MyOutPort = 67001 + id;
            
  OtherInPorts.resize(num_agents);
  for(int i = 0; i < num_agents; i++)
    OtherInPorts[i] = 57001 + i;  
  
  OtherIPs.resize(num_agents);
  OtherAddresses.resize(num_agents);
  
  message_counter = 0;
}

GlobalVariables::~GlobalVariables()  // destructor
{
}

void GlobalVariables::Populate(int id, int total_agents) // populate GlobalVariables
{
  my_id = id;  
  
  MyInSock = -1;
  MyOutSock = -1;
  
  num_agents = total_agents;
  
  service_received_map = false;
       
  MyInPort = 57001 + id;
  MyOutPort = 67001 + id;
  
  OtherInPorts.resize(num_agents);
  for(int i = 0; i < num_agents; i++)
    OtherInPorts[i] = 57001 + i;  
  
  OtherIPs.resize(num_agents);
  OtherAddresses.resize(num_agents);
  
  message_counter = 0;
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
  int NUM_MESSAGE_TYPES = 14;
  send_list.resize(NUM_MESSAGE_TYPES, false);
  send_mode_list.resize(NUM_MESSAGE_TYPES, 0);
  destination_matrix.resize(NUM_MESSAGE_TYPES);
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
      while((message_el = (TiXmlElement *)(broadcast_el->IterateChildren(message_el))))
      {
        int message_type = -1;
        if(message_el->Attribute("type"))
        {
          message_type = atoi(message_el->Attribute("type"));
          send_list[message_type] = true;
          
          if(message_el->Attribute("mode"))
            send_mode_list[message_type] = atoi(message_el->Attribute("mode"));
          
          printf("  %d (using mode %d)\n", message_type, send_mode_list[message_type]);
          
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
      while((message_el = (TiXmlElement *)(receive_el->IterateChildren(message_el))))
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

bool GlobalVariables::set_up_OtherAddresseses() // sets up other address data
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

    // populate OtherAddresseses
    OtherAddresses[i].sin_family = AF_INET;

    bcopy((char *)hp_this->h_addr, (char *)&OtherAddresses[i].sin_addr, hp_this->h_length); // copy in address

    OtherAddresses[i].sin_port = htons(OtherInPorts[i]);       
  }

  return true;
}

bool GlobalVariables::set_up_MyAddress() // sets up outgoing socket
{
  if(MyOutSock < 0)
  {
    MyOutSock = socket(AF_INET, SOCK_DGRAM, 0); 
    if(MyOutSock < 0)        // failed to create socket
    {
      error("problems creating socket");
      return false;
    }
  }

  if(MyInSock < 0)
  {
    MyInSock = socket(AF_INET, SOCK_DGRAM, 0);
    if(MyInSock < 0)  // failed to open socket
    {
      error("Problems opening socket\n");
      return false;
    }
  }
  
  // clear all memory of MyAddress structure
  int MyAddress_length = sizeof(MyAddress);
  memset(&(MyAddress), NULL, MyAddress_length); 
   
  // populate MyAddress structure
  MyAddress.sin_family = AF_INET;
  MyAddress.sin_addr.s_addr = INADDR_ANY; // ip address of this machine
  MyAddress.sin_port = htons(MyInPort);  // htons() converts 'number' to proper network byte order

  // bind in_socket with MyAddress
  if(bind(MyInSock, (struct sockaddr *)&(MyAddress), MyAddress_length)<0) 
  {
    error("problems binding MyInSock");
    return false;
  }

  return true;
}

void GlobalVariables::send_to_agent(void* buffer, size_t buffer_size, int ag) // sends data to agent ag
{
  //printf("trying to send: %s\n", (char*)buffer);  
  if(buffer_size <= max_network_message_size)  // message is small enough to fit in one packet
  {
    int sent_size = sendto(MyOutSock, buffer, buffer_size, 0, (struct sockaddr *)&(OtherAddresses[ag]), sizeof(struct sockaddr_in));
    if(sent_size < 0) 
    {
      error("Problems sending data");
    }
  }
  else // message must be split into multiple packets
  {
    printf("must split message into multiple packets \n");
    
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
    size_t adjusted_data_size = max_network_message_size - header_size;
    
    total_packets = buffer_size/(max_network_message_size-header_size) + 1; // number of packets we need to send
    message_counter++; // increment global message counter
    
    // replace buffer of first packet
    add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 13, message_counter, total_packets, 0, buffer_max);
    
    // send first packet
    printf("sending %u of %u \n", 0, total_packets);
    int sent_size = sendto(MyOutSock, buffer, max_network_message_size, 0, (struct sockaddr *)&(OtherAddresses[ag]), sizeof(struct sockaddr_in));
    if(sent_size < 0) 
    {
      error("Problems sending data");
    }
    
    // send the rest of the packets
    for(packet_number = 1; packet_number < total_packets; packet_number++)
    {
      // add buffer before next part of data to send
      buffer_ptr += adjusted_data_size; 
      add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 13, message_counter, total_packets, packet_number, buffer_max);   
    
      // send nth packet
      printf("sending %u of %u \n", packet_number, total_packets);
      int sent_size = sendto(MyOutSock, (void *)buffer_ptr, max_network_message_size, 0, (struct sockaddr *)&(OtherAddresses[ag]), sizeof(struct sockaddr_in));
      if(sent_size < 0) 
      {
        error("Problems sending data");
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
 
/*--------------------------  listner thread ----------------------------*/
void *Listner(void * inG)
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
    int message_length = recvfrom(G->MyInSock, network_message_buffer, sizeof(network_message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks untill a message is recieved
    if(message_length < 0) 
      printf("had problems getting a message \n");
    
    message_buffer = network_message_buffer;
    buffer_ptr = (size_t)network_message_buffer;
    buffer_max = buffer_ptr + (size_t)max_network_message_size;
    
    printf("extracting header \n");
    //extract header elements
    int sending_agent;
    uint message_type;
    uint sent_message_counter;
    uint total_packets;
    uint packet_number;
    buffer_ptr = extract_from_buffer_ethernetheader(buffer_ptr, sending_agent, message_type, sent_message_counter, total_packets, packet_number, buffer_max); // extracts an ethernet header from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer


    printf("recieved message %u: %u, %u of %u \n", message_type, sent_message_counter, packet_number, total_packets);
    
    if(total_packets > 1) // this message has more packets still to come
    {
      // copy this into where it should go in the large message buffer 
      memcpy(large_message_buffer + header_size + (((size_t)packet_number)*adjusted_network_data_size), (void *)buffer_ptr, adjusted_network_data_size); 
        
      //set up structure to record what we have received
      vector<bool> msg_rec(total_packets, false);
      msg_rec[packet_number] = true;
              
      // now we try to get the rest of the message
      while(true)
      { 
        int senders_address_length_b = sizeof(struct sockaddr_in);  // get the memory size of a sockaddr_in struct 
        memset(&network_message_buffer,'\0',sizeof(network_message_buffer)); 
        int message_length = recvfrom(G->MyInSock, network_message_buffer, sizeof(network_message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length_b);  // blocks untill a message is recieved
        if(message_length < 0) 
          printf("had problems getting a message \n");
    
        buffer_max = buffer_ptr + (size_t)max_network_message_size;
        
        //extract header elements
        int sending_agent_b;
        uint message_type_b;
        uint sent_message_counter_b;
        uint total_packets_b;
        uint packet_number_b;
        buffer_ptr = extract_from_buffer_ethernetheader((size_t)network_message_buffer, sending_agent_b, message_type_b, sent_message_counter_b, total_packets_b, packet_number_b, buffer_max); // extracts an ethernet header from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer

        printf("recieved message %u: %u, %u of %u \n", message_type_b, sent_message_counter_b, packet_number_b, total_packets_b);
        
        if(sending_agent_b != sending_agent || message_type_b != message_type || sent_message_counter_b < sent_message_counter || total_packets_b != total_packets)
        {
            printf("continuing \n");
            continue;
        }
        //MAYBE CHANGE THE FOLLOWING TO A TIMEOUT
        if(sent_message_counter_b > sent_message_counter + total_packets*3) // then we conclude we failed to get all of the message 
        {
          printf("failed to receive all packets \n");
          message_type = -1;  
          break;
        }        
        
        // copy this into where it should go in the large message buffer 
        memcpy(large_message_buffer + header_size + (((size_t)packet_number_b)*adjusted_network_data_size), (void*)buffer_ptr, adjusted_network_data_size);
        msg_rec[packet_number_b] = true;
        
        for(uint j = 0; j < total_packets; j++)
        {
          if(msg_rec[j])   
            printf("1");
          else
            printf("0");
        }
        printf("\n");
        
      }
     
      message_buffer = large_message_buffer;
      buffer_ptr = (size_t)large_message_buffer + header_size;
      buffer_max = buffer_ptr + (size_t)max_message_size - header_size;
    }
    
    

    printf("recieved message type %d from %d \n", (int)message_type, sending_agent); 
    
    if(message_type == 0 && G->listen_list[0]) // it is a pose message
    {
      if(G->listen_mode_list[0] == 0)
      {
        geometry_msgs::PoseStamped msg;
        buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg, buffer_max); 
        pose_pub.publish(msg);
      
        //printf("pose message:\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n\n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);       
        //double secs = msg.header.stamp.toSec();
        //printf("header: %d, %f, %s \n", (int)msg.header.seq, secs, msg.header.frame_id.c_str());
      }
      else if(G->listen_mode_list[0] == 1)
      {
        intercom_cu::PoseStamped_CU_ID msg;
        msg.id.data = sending_agent;
        buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg.data, buffer_max); 
        pose_pub.publish(msg);
      }
    }
    else if(message_type == 1 && G->listen_list[1]) // it is a goal message
    {
      if(G->listen_mode_list[1] == 0)
      {
        geometry_msgs::PoseStamped msg;
        buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg, buffer_max); 
        goal_pub.publish(msg);
      }
      else if(G->listen_mode_list[1] == 1)
      {
          
      }
    } 
    else if(message_type == 2 && G->listen_list[2]) // it is a system state message
    {
      if(G->listen_mode_list[2] == 0)
      {
        std_msgs::Int32 msg;
        buffer_ptr = extract_from_buffer_int(buffer_ptr, msg.data, buffer_max); 
        system_state_pub.publish(msg);
      }
      else if(G->listen_mode_list[2] == 1)
      {
          
      }
    } 
    else if(message_type == 3 && G->listen_list[3]) // it is a system update message
    {
      if(G->listen_mode_list[3] == 0)
      {
        std_msgs::Int32 msg;
        buffer_ptr = extract_from_buffer_int(buffer_ptr, msg.data, buffer_max); 
        system_update_pub.publish(msg);
      }
      else if(G->listen_mode_list[3] == 1)
      {
          
      }
    }
    else if(message_type == 4 && G->listen_list[4]) // it is a map changes message
    {
      if(G->listen_mode_list[4] == 0)
      {
        sensor_msgs::PointCloud msg;
        buffer_ptr = extract_from_buffer_PointCloud(buffer_ptr, msg, buffer_max);
        map_changes_pub.publish(msg);
      }
      else if(G->listen_mode_list[4] == 1)
      {
          
      }
    } 
    else if(message_type == 5 && G->listen_list[5]) // it is a global path message
    {
      if(G->listen_mode_list[5] == 0)
      {
        nav_msgs::Path msg;
        buffer_ptr = extract_from_buffer_Path(buffer_ptr, msg, buffer_max);
        global_path_pub.publish(msg);
      }
      else if(G->listen_mode_list[5] == 1)
      {
          
      }
    }
    else if(message_type == 6 && G->listen_list[6]) // it is a goal reset message
    {
      if(G->listen_mode_list[6] == 0)
      {
        geometry_msgs::PoseStamped msg;
        buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg, buffer_max); 
        new_goal_pub.publish(msg);
      }
      else if(G->listen_mode_list[6] == 1)
      {
          
      }
    } 
    else if(message_type == 7 && G->listen_list[7]) // it is a pose reset message
    {
      if(G->listen_mode_list[7] == 0)
      {
        geometry_msgs::PoseStamped msg;
        buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, msg, buffer_max); 
        new_pose_pub.publish(msg);
      }
      else if(G->listen_mode_list[7] == 1)
      {
          
      }
    } 
    else if(message_type == 8 && G->listen_list[8]) // it is a user control message
    {
      if(G->listen_mode_list[8] == 0)
      {
        geometry_msgs::Pose2D msg;
        buffer_ptr = extract_from_buffer_Pose2D(buffer_ptr, msg, buffer_max); 
        user_control_pub.publish(msg);
      }
      else if(G->listen_mode_list[8] == 1)
      {
          
      }
    }
    else if(message_type == 9 && G->listen_list[9]) // it is a user state message
    {
      if(G->listen_mode_list[9] == 0)
      {
        std_msgs::Int32 msg;
        buffer_ptr = extract_from_buffer_int(buffer_ptr, msg.data, buffer_max); 
        user_state_pub.publish(msg);
      }
      else if(G->listen_mode_list[9] == 1)
      {
          
      }
    }
    else if(message_type == 10 && G->listen_list[10]) // it is a laser scan message
    {
      if(G->listen_mode_list[10] == 0)
      {
        hokuyo_listener_cu::PointCloudWithOrigin msg;
        buffer_ptr = extract_from_buffer_PointCloudWithOrigin(buffer_ptr, msg, buffer_max);
        laser_scan_pub.publish(msg);
      }
      else if(G->listen_mode_list[10] == 1)
      {
          
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
        
            //Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 12);                            // send      
            printf("sending message type 12 to %d at %s\n", sending_agent, G->OtherIPs[2].c_str());
            Globals.send_to_agent(buffer, buffer_ptr-(size_t)buffer, 2);
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
          transform.frame_id_ = std::string("/map_cu");
          transform.child_frame_id_ = std::string("/world_cu");
        
          br.sendTransform(transform);
        }
        else if(G->listen_mode_list[13] == 1)
        {
          
        }
      }
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
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 7,0,0,0, buffer_max); // add space for ethernet header 7
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
  ros::Rate loop_rate(100);
  
  uint this_msg_size = 100; 
  char buffer[this_msg_size];
  size_t buffer_ptr = (size_t)buffer;
  size_t buffer_max = buffer_ptr + (size_t)this_msg_size;
  
  buffer_ptr = add_to_buffer_ethernetheader(buffer_ptr, Globals.my_id, 11,0,0,0, buffer_max); // add space for ethernet header 
  
  Globals.service_received_map = false;
  while(!Globals.service_received_map) // wait for response
  {
    Globals.send_message_type(buffer, buffer_ptr-(size_t)buffer, 11);                         // send 
    
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

int main(int argc, char * argv[]) 
{    
  // init ROS
  ros::init(argc, argv, "intercom_cu");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
 
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

  // set up ROS topic publishers
  if(Globals.listen_list[0])
  { 
    if(Globals.listen_mode_list[0] == 0)
      pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/pose_cu", 1);
    else if(Globals.listen_mode_list[0] == 1)
      pose_pub = nh.advertise<intercom_cu::PoseStamped_CU_ID>("/cu_multi/pose_cu", 1);
  }
  if(Globals.listen_list[1]) 
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/goal_cu", 1);
  if(Globals.listen_list[2])
    system_state_pub = nh.advertise<std_msgs::Int32>("/cu/system_state_cu", 1);
  if(Globals.listen_list[3])
    system_update_pub = nh.advertise<std_msgs::Int32>("/cu/system_update_cu", 1);
  if(Globals.listen_list[4])
    map_changes_pub = nh.advertise<sensor_msgs::PointCloud>("/cu/map_changes_cu", 1);
  if(Globals.listen_list[5])
    global_path_pub = nh.advertise<nav_msgs::Path>("/cu/global_path_cu", 1);
  if(Globals.listen_list[6])
    new_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/reset_goal_cu", 1);
  if(Globals.listen_list[7])
    new_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/user_pose_cu", 1);
  if(Globals.listen_list[8])
    user_control_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/user_control_cu", 1);
  if(Globals.listen_list[9])
     user_state_pub = nh.advertise<std_msgs::Int32>("/cu/user_state_cu", 1);
  if(Globals.listen_list[10])
    laser_scan_pub = nh.advertise<hokuyo_listener_cu::PointCloudWithOrigin>("/cu/laser_scan_cu", 1);
      
  // set up service servers
  if(Globals.send_list[11])
    get_map_srv = nh.advertiseService("/cu/get_map_cu", get_map_callback);

  while(! Globals.set_up_MyAddress())
  {}

  while(! Globals.set_up_OtherAddresseses())
  {}

  pthread_t Listener_thread;
  pthread_create(&Listener_thread, NULL, Listner, &Globals); // listens for incomming messages

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
  
  // destroy service providers
  get_map_srv.shutdown();
  
  return 0;
}
