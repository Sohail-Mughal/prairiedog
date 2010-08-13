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

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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

/*-------------- functions for adding ros data to a buffer --------------*/
size_t add_to_buffer_int8(size_t buffer_ptr, const int8_t& i, size_t buffer_max) // adds i at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  if(buffer_ptr + sizeof(i) < buffer_max)
  {
    memcpy((void*)(buffer_ptr), (void*)&(i), sizeof(i));
    buffer_ptr += sizeof(i);
  }
  else
    printf("could not add int (int8) to buffer, buffer too small\n");
    
  return buffer_ptr;
}
        
size_t add_to_buffer_int(size_t buffer_ptr, const int& i, size_t buffer_max) // adds i at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  if(buffer_ptr + sizeof(i) < buffer_max)
  {
    memcpy((void*)(buffer_ptr), (void*)&(i), sizeof(i));
    buffer_ptr += sizeof(i);
  }
  else
    printf("could not add int (int32) to buffer, buffer too small\n");
    
  return buffer_ptr;
}

size_t add_to_buffer_uint(size_t buffer_ptr, const uint& u, size_t buffer_max) // adds u at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  if(buffer_ptr + sizeof(u) < buffer_max)
  {
    memcpy((void*)(buffer_ptr), (void*)&(u), sizeof(u));
    buffer_ptr += sizeof(u);
  }
  else
    printf("could not add uint to buffer, buffer too small\n");
  
  return buffer_ptr;
}

size_t add_to_buffer_float(size_t buffer_ptr, const float& f, size_t buffer_max) // adds f at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  if(buffer_ptr + sizeof(f) < buffer_max)
  {
    memcpy((void*)(buffer_ptr), (void*)&(f), sizeof(f));
    buffer_ptr += sizeof(f);
  }
  else
    printf("could not add float (float32) to buffer, buffer too small\n");
    
  return buffer_ptr;
}

size_t add_to_buffer_double(size_t buffer_ptr, const double& d, size_t buffer_max) // adds d at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  if(buffer_ptr + sizeof(d) < buffer_max)
  {
    memcpy((void*)(buffer_ptr), (void*)&(d), sizeof(d));
    buffer_ptr += sizeof(d);
  }
  else
    printf("could not add double (float64) to buffer, buffer too small\n"); 
    
  return buffer_ptr;
}

size_t add_to_buffer_string(size_t buffer_ptr, const std::string& s, size_t buffer_max) // adds s at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  size_t size_of_s = s.size() * sizeof(char);
      
  if(buffer_ptr + sizeof(size_t) + size_of_s < buffer_max)
  { 
    // add size of s 
    memcpy((void*)(buffer_ptr), (void*)&(size_of_s), sizeof(size_of_s));
    buffer_ptr += sizeof(size_of_s);
  
    // add s
    memcpy((void*)(buffer_ptr), (void*)(s.data()), size_of_s);
    buffer_ptr += size_of_s;
  }
  else
    printf("could not add string to buffer, buffer too small\n");
  
  return buffer_ptr;
}

size_t add_to_buffer_btScalar(size_t buffer_ptr, const btScalar& d, size_t buffer_max) // adds d at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  if(buffer_ptr + sizeof(d) < buffer_max)
  {
    memcpy((void*)(buffer_ptr), (void*)&(d), sizeof(d));
    buffer_ptr += sizeof(d);
  }
  else
    printf("could not add btScalar to buffer, buffer too small\n");
    
  return buffer_ptr;
}

size_t add_to_buffer_btVector3(size_t buffer_ptr, const btVector3& v, size_t buffer_max) // adds v at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  buffer_ptr = add_to_buffer_btScalar(buffer_ptr, v.x(), buffer_max);
  buffer_ptr = add_to_buffer_btScalar(buffer_ptr, v.y(), buffer_max);
  buffer_ptr = add_to_buffer_btScalar(buffer_ptr, v.z(), buffer_max);
  buffer_ptr = add_to_buffer_btScalar(buffer_ptr, v.w(), buffer_max);
  
  return buffer_ptr;
}

size_t add_to_buffer_btMatrix3x3(size_t buffer_ptr, const btMatrix3x3& m, size_t buffer_max) // adds m at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  buffer_ptr = add_to_buffer_btVector3(buffer_ptr, m[0], buffer_max);
  buffer_ptr = add_to_buffer_btVector3(buffer_ptr, m[1], buffer_max);
  buffer_ptr = add_to_buffer_btVector3(buffer_ptr, m[2], buffer_max);
  
  return buffer_ptr;
}

size_t add_to_buffer_Time(size_t buffer_ptr, const ros::Time& t, size_t buffer_max) // adds t at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  buffer_ptr = add_to_buffer_int(buffer_ptr, t.sec, buffer_max);
  buffer_ptr = add_to_buffer_int(buffer_ptr, t.nsec, buffer_max);
  
  return buffer_ptr;
}


size_t add_to_buffer_Pose(size_t buffer_ptr, const geometry_msgs::Pose& pose, size_t buffer_max) // adds the Pose at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  size_t size_of_pose = sizeof(pose);
  if(buffer_ptr + sizeof(size_t) + size_of_pose < buffer_max)
  { 
    // add size of pose
    memcpy((void*)(buffer_ptr), (void*)&(size_of_pose), sizeof(size_of_pose));
    buffer_ptr += sizeof(size_of_pose);
  
    // add pose
    memcpy((void*)(buffer_ptr), (void*)&(pose), size_of_pose);
    buffer_ptr += size_of_pose;
  }
  else
    printf("could not add Pose to buffer, buffer too small\n");
  
  return buffer_ptr;
}

size_t add_to_buffer_Pose2D(size_t buffer_ptr, const geometry_msgs::Pose2D& pose, size_t buffer_max) // adds the Pose2D at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  buffer_ptr = add_to_buffer_double(buffer_ptr, pose.x, buffer_max);
  buffer_ptr = add_to_buffer_double(buffer_ptr, pose.y, buffer_max);
  buffer_ptr = add_to_buffer_double(buffer_ptr, pose.theta, buffer_max);
  
  return buffer_ptr;
}

size_t add_to_buffer_Header(size_t buffer_ptr, const roslib::Header& header, size_t buffer_max)  // adds the Header at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{ 
  buffer_ptr = add_to_buffer_uint(buffer_ptr, header.seq, buffer_max);
  buffer_ptr = add_to_buffer_Time(buffer_ptr, header.stamp, buffer_max);
  buffer_ptr = add_to_buffer_string(buffer_ptr, header.frame_id, buffer_max);
  
  return buffer_ptr;
}

size_t add_to_buffer_PoseStamped(size_t buffer_ptr, const geometry_msgs::PoseStamped& posestamped, size_t buffer_max) // adds the PoseStamped at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  buffer_ptr = add_to_buffer_Pose(buffer_ptr, posestamped.pose, buffer_max);
  buffer_ptr = add_to_buffer_Header(buffer_ptr, posestamped.header, buffer_max);
  return buffer_ptr;
}

size_t add_to_buffer_Point32(size_t buffer_ptr, const geometry_msgs::Point32& point, size_t buffer_max) // adds the Point32 at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  size_t size_of_point = sizeof(point);
  if(buffer_ptr + sizeof(size_t) + size_of_point < buffer_max)
  { 
    // add size of pose
    memcpy((void*)(buffer_ptr), (void*)&(size_of_point), sizeof(size_of_point));
    buffer_ptr += sizeof(size_of_point);
  
    // add pose
    memcpy((void*)(buffer_ptr), (void*)&(point), size_of_point);
    buffer_ptr += size_of_point;
  }
  else
    printf("could not add Point32 to buffer, buffer too small\n");
  
  return buffer_ptr;
}

size_t add_to_buffer_PointCloud(size_t buffer_ptr, const sensor_msgs::PointCloud& cloud, size_t buffer_max) // adds the PointCloud at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  // add header  
  buffer_ptr = add_to_buffer_Header(buffer_ptr, cloud.header, buffer_max);
  
  // add number of points
  uint num_points = cloud.points.size();
  buffer_ptr = add_to_buffer_uint(buffer_ptr, num_points, buffer_max);
  
  // add each point
  for(uint i = 0; i < num_points; i++)
    buffer_ptr = add_to_buffer_Point32(buffer_ptr, cloud.points[i], buffer_max);
  
  // add number of channels
  uint num_channels = cloud.channels.size();
  buffer_ptr = add_to_buffer_uint(buffer_ptr, num_channels, buffer_max);
  
  // add each channels
  for(uint i = 0; i < num_channels; i++)
    printf("warning: channels in PointCloud not implimented yet\n");
  
  return buffer_ptr;
}

size_t add_to_buffer_Path(size_t buffer_ptr, const nav_msgs::Path& path, size_t buffer_max) // adds the Path at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  // add header  
  buffer_ptr = add_to_buffer_Header(buffer_ptr, path.header, buffer_max);
  
  // add number of poses
  uint num_poses = path.poses.size();
  buffer_ptr = add_to_buffer_uint(buffer_ptr, num_poses, buffer_max);
  
  // add each pose
  for(uint i = 0; i < num_poses; i++)
    buffer_ptr = add_to_buffer_PoseStamped(buffer_ptr, path.poses[i], buffer_max);
  
  return buffer_ptr;
}

size_t add_to_buffer_PointCloudWithOrigin(size_t buffer_ptr, const hokuyo_listener_cu::PointCloudWithOrigin& pcwo, size_t buffer_max) // adds the PointCloudWithOrigin at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  // add origin  
  buffer_ptr = add_to_buffer_Point32(buffer_ptr, pcwo.origin, buffer_max);
  
  // add cloud
  buffer_ptr = add_to_buffer_PointCloud(buffer_ptr, pcwo.cloud, buffer_max);
  
  return buffer_ptr;
}


size_t add_to_buffer_OccupancyGrid(size_t buffer_ptr, const nav_msgs::OccupancyGrid& map, size_t buffer_max) // adds the OccupancyGrid at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  // add header  
  buffer_ptr = add_to_buffer_Header(buffer_ptr, map.header, buffer_max);
  
  // add info
  buffer_ptr = add_to_buffer_Time(buffer_ptr, map.info.map_load_time, buffer_max);
  buffer_ptr = add_to_buffer_float(buffer_ptr, map.info.resolution, buffer_max);
  buffer_ptr = add_to_buffer_uint(buffer_ptr, map.info.width, buffer_max);
  buffer_ptr = add_to_buffer_uint(buffer_ptr, map.info.height, buffer_max);
  buffer_ptr = add_to_buffer_Pose(buffer_ptr, map.info.origin, buffer_max);      
          
  // add number of data
  uint num_data = map.data.size();
  buffer_ptr = add_to_buffer_uint(buffer_ptr, num_data, buffer_max);
  
  // add each data
  for(uint i = 0; i < num_data; i++)
    buffer_ptr = add_to_buffer_int8(buffer_ptr, map.data[i], buffer_max);
  
  return buffer_ptr;
}

size_t add_to_buffer_StampedTransform(size_t buffer_ptr, const tf::StampedTransform& t, size_t buffer_max) // adds the tf::StampedTransform at the buffer pointed to by (void*)buffer_ptr, returns the next free location in the buffer, errors if try to insert past buffer_max
{
  buffer_ptr = add_to_buffer_Time(buffer_ptr, t.stamp_, buffer_max);  
  buffer_ptr = add_to_buffer_string(buffer_ptr, t.frame_id_, buffer_max);  
  buffer_ptr = add_to_buffer_string(buffer_ptr, t.child_frame_id_, buffer_max);
  buffer_ptr = add_to_buffer_btVector3(buffer_ptr, t.getOrigin(), buffer_max); 
  buffer_ptr = add_to_buffer_btMatrix3x3(buffer_ptr, t.getBasis(), buffer_max); 
  return buffer_ptr;
}

/*------------ functions for extracting ros data to a buffer ------------*/
size_t extract_from_buffer_int8(size_t buffer_ptr, int8_t& i, size_t buffer_max) // extracts int i from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  if(buffer_ptr + sizeof(i) >= buffer_max)
  {
    printf("could not extract int (int8) from buffer, buffer too small\n");
    return  buffer_ptr;
  } 
  memcpy((void*)&(i), (void*)(buffer_ptr), sizeof(i));
  buffer_ptr += sizeof(i);
 
  return  buffer_ptr;  
}

size_t extract_from_buffer_int(size_t buffer_ptr, int& i, size_t buffer_max) // extracts int i from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  if(buffer_ptr + sizeof(i) >= buffer_max)
  {
    printf("could not extract int (int32) from buffer, buffer too small\n");
    return  buffer_ptr;
  } 
  memcpy((void*)&(i), (void*)(buffer_ptr), sizeof(i));
  buffer_ptr += sizeof(i);
 
  return  buffer_ptr;  
}

size_t extract_from_buffer_uint(size_t buffer_ptr, uint& u, size_t buffer_max) // extracts uint u from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  if(buffer_ptr + sizeof(u) >= buffer_max)
  {
    printf("could not extract uint from buffer, buffer too small\n");
    return  buffer_ptr;
  } 
  memcpy((void*)&(u), (void*)(buffer_ptr), sizeof(u));
  buffer_ptr += sizeof(u);
  
  return  buffer_ptr; 
}

size_t extract_from_buffer_float(size_t buffer_ptr, float& f, size_t buffer_max) // extracts double f from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  if(buffer_ptr + sizeof(f) >= buffer_max)
  {
    printf("could not extract double (float64) from buffer, buffer too small\n");
    return  buffer_ptr;
  } 
  memcpy((void*)&(f), (void*)(buffer_ptr), sizeof(f));
  buffer_ptr += sizeof(f);
 
  return  buffer_ptr;  
}

size_t extract_from_buffer_double(size_t buffer_ptr, btScalar& d, size_t buffer_max) // extracts double d from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  if(buffer_ptr + sizeof(d) >= buffer_max)
  {
    printf("could not extract double (float64) from buffer, buffer too small\n");
    return  buffer_ptr;
  } 
  memcpy((void*)&(d), (void*)(buffer_ptr), sizeof(d));
  buffer_ptr += sizeof(d);
 
  return  buffer_ptr;  
}

size_t extract_from_buffer_string(size_t buffer_ptr, std::string& s, size_t buffer_max) // extracts std::string s from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  // get size of string
  if(buffer_ptr + sizeof(size_t) >= buffer_max)
  {
    printf("could not extract string (length) from buffer, buffer too small\n");
    return  buffer_ptr;
  } 
  size_t size_of_s;
  memcpy((void*)&(size_of_s), (void*)(buffer_ptr), sizeof(size_of_s));
  buffer_ptr += sizeof(size_of_s);
      
  // get string
  if(buffer_ptr + size_of_s >= buffer_max)
  {
    printf("could not extract string from buffer, buffer too small\n");
    return  buffer_ptr;
  }   
  std::string temp((char*)(buffer_ptr), size_of_s);
  s = temp;
  buffer_ptr += size_of_s;
  
  return  buffer_ptr; 
}

size_t extract_from_buffer_btScalar(size_t buffer_ptr, btScalar& d, size_t buffer_max) // extracts btScalar d from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  if(buffer_ptr + sizeof(d) >= buffer_max)
  {
    printf("could not extract double (float64) from buffer, buffer too small\n");
    return  buffer_ptr;
  } 
  memcpy((void*)&(d), (void*)(buffer_ptr), sizeof(d));
  buffer_ptr += sizeof(d);
 
  return  buffer_ptr;  
}

size_t extract_from_buffer_btVector3(size_t buffer_ptr, btVector3& v, size_t buffer_max) // extracts btVector3 v from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{ 
  btScalar temp;  
  buffer_ptr = extract_from_buffer_btScalar(buffer_ptr, temp, buffer_max);
  v.setX(temp);
  buffer_ptr = extract_from_buffer_btScalar(buffer_ptr, temp, buffer_max);
  v.setY(temp);
  buffer_ptr = extract_from_buffer_btScalar(buffer_ptr, temp, buffer_max);
  v.setZ(temp);
  buffer_ptr = extract_from_buffer_btScalar(buffer_ptr, temp, buffer_max);
  v.setW(temp);
  
  return  buffer_ptr;  
}

size_t extract_from_buffer_btMatrix3x3(size_t buffer_ptr, btMatrix3x3& m, size_t buffer_max) // extracts btMatrix3x3 m from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{ 
  btVector3 temp;  
  buffer_ptr = extract_from_buffer_btVector3(buffer_ptr, temp, buffer_max);
  m[0] = temp;
  buffer_ptr = extract_from_buffer_btVector3(buffer_ptr, temp, buffer_max);
  m[1] = temp;
  buffer_ptr = extract_from_buffer_btVector3(buffer_ptr, temp, buffer_max);
  m[2] = temp;

  return  buffer_ptr;  
}

size_t extract_from_buffer_Time(size_t buffer_ptr, ros::Time& t, size_t buffer_max) // extracts ros::time t from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{ 
  int temp;  
  buffer_ptr = extract_from_buffer_int(buffer_ptr, temp, buffer_max);
  t.sec = temp;
  buffer_ptr = extract_from_buffer_int(buffer_ptr, temp, buffer_max);
  t.nsec = temp;

  return  buffer_ptr;  
}

size_t extract_from_buffer_Pose(size_t buffer_ptr, geometry_msgs::Pose& pose, size_t buffer_max) // extracts Pose from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{    
  // get size of pose
  if(buffer_ptr + sizeof(size_t) >= buffer_max)
  {
    printf("could not extract Pose from buffer, buffer too small\n");
    return  buffer_ptr;
  }  
  size_t size_of_pose;
  memcpy((void*)&(size_of_pose), (void*)(buffer_ptr), sizeof(size_of_pose));
  buffer_ptr += sizeof(size_of_pose); 
 
  // get pose
  if(buffer_ptr + size_of_pose >= buffer_max)
  {
    printf("could not extract Pose from buffer, buffer too small\n");
    return  buffer_ptr;
  }
  memcpy((void*)&(pose), (void*)(buffer_ptr), size_of_pose);
  buffer_ptr += size_of_pose;
  
  return  buffer_ptr;
}

size_t extract_from_buffer_Pose2D(size_t buffer_ptr, geometry_msgs::Pose2D& pose, size_t buffer_max) // extracts Pose2D from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{    
  buffer_ptr = extract_from_buffer_double(buffer_ptr, pose.x, buffer_max);
  buffer_ptr = extract_from_buffer_double(buffer_ptr, pose.y, buffer_max);
  buffer_ptr = extract_from_buffer_double(buffer_ptr, pose.theta, buffer_max);
  return  buffer_ptr;
}

size_t extract_from_buffer_Header(size_t buffer_ptr, roslib::Header& header, size_t buffer_max) // extracts Header from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{ 
  buffer_ptr = extract_from_buffer_uint(buffer_ptr, header.seq, buffer_max);
  buffer_ptr = extract_from_buffer_Time(buffer_ptr, header.stamp, buffer_max);
  buffer_ptr = extract_from_buffer_string(buffer_ptr, header.frame_id, buffer_max);
  return buffer_ptr;
}

size_t extract_from_buffer_PoseStamped(size_t buffer_ptr, geometry_msgs::PoseStamped& posestamped, size_t buffer_max) // extracts PoseStamped from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  buffer_ptr = extract_from_buffer_Pose(buffer_ptr, posestamped.pose, buffer_max); 
  buffer_ptr = extract_from_buffer_Header(buffer_ptr, posestamped.header, buffer_max);
  return buffer_ptr;       
}      
       
size_t extract_from_buffer_Point32(size_t buffer_ptr, geometry_msgs::Point32& point, size_t buffer_max) // extracts Point32 from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{    
  // get size of point
  if(buffer_ptr + sizeof(size_t) >= buffer_max)
  {
    printf("could not extract Point32 from buffer, buffer too small\n");
    return  buffer_ptr;
  }  
  size_t size_of_point;
  memcpy((void*)&(size_of_point), (void*)(buffer_ptr), sizeof(size_of_point));
  buffer_ptr += sizeof(size_of_point); 
 
  // get point
  if(buffer_ptr + size_of_point >= buffer_max)
  {
    printf("could not extract Point32 from buffer, buffer too small\n");
    return  buffer_ptr;
  }
  memcpy((void*)&(point), (void*)(buffer_ptr), size_of_point);
  buffer_ptr += size_of_point;
  
  return  buffer_ptr;
}

size_t extract_from_buffer_PointCloud(size_t buffer_ptr, sensor_msgs::PointCloud& cloud, size_t buffer_max) // extracts PointCloud from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  // extract header  
  buffer_ptr = extract_from_buffer_Header(buffer_ptr, cloud.header, buffer_max);
    
  // extract number of points
  uint num_points;
  buffer_ptr = extract_from_buffer_uint(buffer_ptr, num_points, buffer_max);
  
  // extract each point
  cloud.points.resize(num_points);
  
  for(uint i = 0; i < num_points; i++)
  {
    geometry_msgs::Point32 point;
    buffer_ptr = extract_from_buffer_Point32(buffer_ptr, point, buffer_max);  
    cloud.points[i] = point;
  }
  
  // extract number of chanels
  uint num_channels;
  buffer_ptr = extract_from_buffer_uint(buffer_ptr, num_channels, buffer_max);
  
  // extract each channel
  cloud.channels.resize(num_channels);
  for(uint i = 0; i < num_channels; i++)
    printf("warning: channels in PointCloud not implimented yet\n");
  
  return buffer_ptr;
}

size_t extract_from_buffer_Path(size_t buffer_ptr, nav_msgs::Path& path, size_t buffer_max) // extracts PointCloud from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  // extract header  
  buffer_ptr = extract_from_buffer_Header(buffer_ptr, path.header, buffer_max);
    
  // extract number of poses
  uint num_poses;
  buffer_ptr = extract_from_buffer_uint(buffer_ptr, num_poses, buffer_max);
  
  // extract each point
  path.poses.resize(num_poses);
  
  for(uint i = 0; i < num_poses; i++)
  {
    geometry_msgs::PoseStamped pose;
    buffer_ptr = extract_from_buffer_PoseStamped(buffer_ptr, pose, buffer_max);  
    path.poses[i] = pose;
  }
  
  return buffer_ptr;
}

size_t extract_from_buffer_PointCloudWithOrigin(size_t buffer_ptr, hokuyo_listener_cu::PointCloudWithOrigin& pcwo, size_t buffer_max) // extracts PointCloudWithOrigin from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  // extract origin  
  buffer_ptr = extract_from_buffer_Point32(buffer_ptr, pcwo.origin, buffer_max);
    
  // extract cloud
  buffer_ptr = extract_from_buffer_PointCloud(buffer_ptr, pcwo.cloud, buffer_max);
   
  return buffer_ptr;        
}  

size_t extract_from_buffer_OccupancyGrid(size_t buffer_ptr, nav_msgs::OccupancyGrid& map, size_t buffer_max) // extracts OccupancyGrid from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  // extract header  
  buffer_ptr = extract_from_buffer_Header(buffer_ptr, map.header, buffer_max);
  
  // extract info
  buffer_ptr = extract_from_buffer_Time(buffer_ptr, map.info.map_load_time, buffer_max);
  buffer_ptr = extract_from_buffer_float(buffer_ptr, map.info.resolution, buffer_max);
  buffer_ptr = extract_from_buffer_uint(buffer_ptr, map.info.width, buffer_max);
  buffer_ptr = extract_from_buffer_uint(buffer_ptr, map.info.height, buffer_max);
  buffer_ptr = extract_from_buffer_Pose(buffer_ptr, map.info.origin, buffer_max);    

  // extract number of data
  uint num_data = -1;
  buffer_ptr = extract_from_buffer_uint(buffer_ptr, num_data, buffer_max);
  map.data.resize(num_data);

  // extract each data
  for(uint i = 0; i < num_data; i++)
    buffer_ptr = extract_from_buffer_int8(buffer_ptr, map.data[i], buffer_max);
  
  return buffer_ptr;
}

size_t extract_from_buffer_StampedTransform(size_t buffer_ptr, tf::StampedTransform& t, size_t buffer_max) // extracts tf::StampedTransform from (void*)buffer_ptr, errors if try to extract past buffer_max, returns the next free location in the buffer
{
  buffer_ptr = extract_from_buffer_Time(buffer_ptr, t.stamp_, buffer_max);  
  buffer_ptr = extract_from_buffer_string(buffer_ptr, t.frame_id_, buffer_max);  
  buffer_ptr = extract_from_buffer_string(buffer_ptr, t.child_frame_id_, buffer_max);
  
  btVector3 m_origin;
  buffer_ptr = extract_from_buffer_btVector3(buffer_ptr, m_origin, buffer_max); 
  t.setOrigin(m_origin);
  
  btMatrix3x3 m_basis;
  buffer_ptr = extract_from_buffer_btMatrix3x3(buffer_ptr, m_basis, buffer_max); 
  t.setBasis(m_basis);
  
  return buffer_ptr;
}

