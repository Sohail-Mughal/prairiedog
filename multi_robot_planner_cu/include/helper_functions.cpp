/* ------------------------- helper functions -------------------------- */
void redude_3_to_2(const vector<float>& V3, vector<float>& V2) // puts the Workspace3 vector int a Workapace2 vector
{
  uint num = V3.size()/3;

  V2.resize(num*2);
  
  for(uint i = 0; i < num; i++)
  {
    V2[2*i] = V3[3*i];
    V2[2*i+1] = V3[3*i+1];   
  }
}

void populate_int_vector(vector<int>& V, int* A, int s) // populates V with A of size s
{
  V.resize(s);
  for(int i = 0; i < s; i++)
    V[i] = A[i];
}

void populate_float_vector(vector<float>& V, float* A, int s) // populates V with A of size s
{
  V.resize(s);
  for(int i = 0; i < s; i++)
    V[i] = A[i];
}

bool equal_float_vector(const vector<float>& A, const vector<float>& B, float tol) // returns true if the vectors are the same size and contain the same elements to within tolerance tol
{
  if(A.size() != B.size())
    return false;
  
  int size = A.size();
  
  for(int i = 0; i < size; i++)
  {
    if(A[i] != B[i])
    {
      if(fabs(A[i] - B[i]) > tol)  
        return false;
    }
  }
  return true;
}


void draw_circle(float* pos, float rad, float* color)
{
  glPushMatrix();
  glTranslatef(pos[0], pos[1], pos[2]);
  glScaled(rad, rad, 1);  
  glColor3f(color[0],color[1],color[2]);
  glBegin(GL_LINE_LOOP); 
  float points = 20;
  for (float i = 0; i < points; i++)
  {    
    float a = 2*PI*i/points; 
    glVertex2f(cos(a), sin(a)); 
  } 
  glEnd();
  glPopMatrix();
}

void draw_x(float* pos, float rad, float* color) // draws an X with rad rad
{
  glPushMatrix();
  glTranslatef(pos[0], pos[1], pos[2]);
  glScaled(rad, rad, 1);  
  glColor3f(color[0],color[1],color[2]);
  glBegin(GL_LINES);
  float points = 4;
  for (float i = 0; i < points; i++)
  {    
    float a = 2*PI*i/points; 
    glVertex2f(cos(a+PI/4), sin(a+PI/4)); 
    glVertex2f(cos(a+PI+PI/4), sin(a+PI+PI/4)); 
    
  } 
  glEnd();
  glPopMatrix();
}
    
// draws an arrow from pos1 to pos2
void draw_arrow(float* pos1, float* pos2, float* color)
{
    
  float r = sqrt((pos1[0]-pos2[0])*(pos1[0]-pos2[0]) + (pos1[1]-pos2[1])*(pos1[1]-pos2[1]));  
  float theta = atan2(pos2[1]-pos1[1],pos2[0]-pos1[0]); 
  float pos3[] = {(pos1[0]+2*pos2[0])/3+r/3*cos(theta-PI/2), (pos1[1]+2*pos2[1])/3+r/3*sin(theta-PI/2), pos1[2]};
  float pos4[] = {(pos1[0]+2*pos2[0])/3+r/3*cos(theta+PI/2), (pos1[1]+2*pos2[1])/3+r/3*sin(theta+PI/2), pos1[2]};
  
  glColor3f(color[0],color[1],color[2]);
  glBegin(GL_LINE_STRIP);
    glVertex3f(pos1[0],pos1[1],pos1[2]);
    glVertex3f(pos2[0],pos2[1],pos2[2]);
    glVertex3f(pos3[0],pos3[1],pos3[2]);
    glVertex3f(pos4[0],pos4[1],pos4[2]);
    glVertex3f(pos2[0],pos2[1],pos2[2]);
  glEnd();
}

void draw_points(const vector<float>& xs, const vector<float>& ys) // draws points at locations specified in the vectors
{
    glBegin(GL_POINTS);
    glColor3f(1,1,1);
    for(int i = 0; i < (int)xs.size(); i++)
    {
      glVertex2f(xs[i],ys[i]); 
    }
    glEnd();
    
    printf("num points: %d\n", xs.size());
}

int rand_int(int left_bound, int right_bound) // returns a random integer between left_bound and right_bound, inclusive
{
  if(  left_bound == right_bound)
    return left_bound;
  return (rand() % (right_bound - left_bound +1)) + left_bound;  
}

void print_float_vector(const vector<float>& vec) // prints the vector to the command line
{   
  for(int i = 0; i < (int)vec.size(); i++)
    printf("%f, ", vec[i]);
  printf("\n");
}

float line_dist_to_point_helper(float ax, float ay, float bx, float by, float cx, float cy) // returns the minimum distance between the line segment [a b] and the point c
{
  float top_m1 =  (by-ay);
  float bottom_m1 = (bx-ax);
  float dx, dy;
  
  if(top_m1 != 0 && bottom_m1 != 0) // the segment is neither horizontal nor vertical
  {
    float m1 = top_m1/bottom_m1;
    float m2 = -1/m1;

    dx = (cy-ay+m1*ax-m2*cx)/(m1-m2);
    dy = m2*(dx-cx)+cy;
  }
  else if(bottom_m1 != 0) // && top_m1 == 0 // the segment is horizontal
  {
    dx = cx;
    dy = ay;
  }
  else if(top_m1 != 0) // && bottom_m1 == 0 // the segment is vertical 
  {
    dx = ax;
    dy = cy; 
  }
  else // (top_m1 == 0 && bottom_m1 == 0) // the segment is actually a point, we'll assume that it is a small vertical line
  {
    dx = ax;
    dy = cy;  
  }
  
  if(((ax <= dx && dx <= bx) || (bx <= dx && dx <= ax)) && ((ay <= dy && dy <= by) || (by <= dy && dy <= ay))) // then the intersect point is on the line segment 
    return sqrt((cx-dx)*(cx-dx) + (cy-dy)*(cy-dy));   
   
  // otherwise the intersect point is not within the line segment, 
  // so we return the min dist between the (input) point and the segments' end points
  
  float dist1 = sqrt((cx-ax)*(cx-ax) + (cy-ay)*(cy-ay));  
  float dist2 = sqrt((cx-bx)*(cx-bx) + (cy-by)*(cy-by));
  
  if(dist1 < dist2)
    return dist1;
  return dist2;
}

float line_dist_to_point(float ax, float ay, float bx, float by, float cx, float cy) // returns the minimum distance between the line segment [a b] and the point c
{
  // help minimize of illconditioning
  if(fabs(ax-bx) >= fabs(ay-by))
    return line_dist_to_point_helper(ay, ax, by, bx, cy, cx);
  else
    return line_dist_to_point_helper(ax, ay, bx, by, cx, cy);
}

// appends info to a file per agent in directory 
void data_dump(const char* directory, float prob_success, float min_clock_to_plan, float phase_two_time, Cspace& C, MultiAgentSolution& M, float actual_clock_to_plan, float total_time)
{   
  char this_file[100];
  sprintf(this_file, "../%s/agent_%d_stats.txt", directory, MultAgSln.agent_id);
  
  FILE* ofp = fopen(this_file,"a");
  while(ofp == NULL)
  {
      ofp = fopen(this_file,"a");
      printf("trying to open stats file in: %s\n",directory);
      return;
  }
  
  
  /* these things go into a file:  
  prob_success
  message_wait_time      
  min_clock_to_plan
  phase_two_time      
  MultAgSln.num_agents
  MultAgSln.best_solution_length    
  actual_clock_to_plan  
  total_time    
  */

  fprintf(ofp, "%f, %f, %f, %f, %d, %f, %f, %f, %f\n", prob_success, message_wait_time, min_clock_to_plan, phase_two_time, M.num_agents, M.best_solution_length, actual_clock_to_plan, total_time, sync_message_wait_time);
  
  fclose(ofp);
  
//   // also save a blank file so that we know this robot is done with this run
//   sprintf(this_file, "ticks/%d.txt", MultAgSln.agent_id);
//   ofp = NULL;
//   ofp = fopen(this_file,"w");
//   while(ofp == NULL)
//   {
//     ofp = fopen(this_file,"w");
//     printf("trying to open file 4 \n");
//     return;
//   }
//   
//   fclose(ofp);
  
}

#ifdef save_time_data
void time_data_dump(const char* directory, float prob_success, float min_clock_to_plan)
{   
  char this_file[100];
  sprintf(this_file, "%s/agent_%d_time_stats.txt", directory, MultAgSln.agent_id);
  
  FILE* ofp = fopen(this_file,"a");
  while(ofp == NULL)
  {
    ofp = fopen(this_file,"a");
    printf("trying to open time stats file in: %s\n",directory);
    return;
  }
  
  fprintf(ofp, "T: %f, %f, %f, %u\n", prob_success, message_wait_time, min_clock_to_plan, nodes_in_tree_stats.size());
  
  for(uint i = 0; i < time_stats.size(); i++)
    fprintf(ofp, "%f, %f, %f, %d\n", time_stats[i], best_path_len_stats[i], nodes_in_tree_stats[i], collision_checks_stats[i]);
  
  fclose(ofp);
}
#endif

float difftime_clock(const clock_t& clock_time_1, const clock_t& clock_time_2) // returns the difference in seconds between the clock time 1 and 2, (2 is assumed earlier than 1)
{
  return ((float)clock_time_1-(float)clock_time_2)/((float)CLOCKS_PER_SEC);
}

bool string_printf_s(int &sp, char* buffer, char* buffer2, int buffer_len) // this takes the string in buffer 2 and puts it into buffer starting at buffer[sp], it then resets sp to be the new end of the string, returns false if there is not enough space in buffer
{  
  if((int)strlen(buffer2) + sp >= buffer_len)
  {
    printf(" error trying to use string_printf_s and buffer is all full \n");
    return false;
  }
  
  strcpy(&(buffer[sp]),buffer2);
  
  while(buffer[sp] != '\0')
    sp++;
  
  return true;
}

void double_up_points(const vector<vector<float> >& V1, vector<vector<float> >& V2) // this doubles each point from V1 and puts it in V2 (1,2,3 becomes 1,1,2,2,3,3)
{
  int size_v1 = V1.size();
  int size_v2 = size_v1*2;
  V2.resize(size_v2);
  
  
  int i = 0;
  for(int j = 0; j < size_v1; j++)
  {
    V2[i] = V1[j];
    i++;
    V2[i] = V1[j];
    i++;
  } 
}

void extract_and_translate_solution(vector<vector<float> >& AgentSolution, vector<vector<float> >& MultiSolution, vector<float>& Translation, int agent_id, int dims) // extracts the agent solution if agent_id where dims is the workspace dimensions and Translate holds the inverse ammount to translate along each
{
  // make sure solution lengths are the same
  AgentSolution.resize(MultiSolution.size());
  
  // extract each point
  for(int i = 0; i < (int)MultiSolution.size(); i++)
  {
    AgentSolution[i].resize(dims);
    
    for(int j = 0; j < dims; j++)
      AgentSolution[i][j] = MultiSolution[i][j+dims*agent_id] - Translation[j];   
  }
}

void remove_unnecessary_rotation(vector<vector<float> >& MultiSolution) // removes unnecessary rotation, assumes 2D workspace and orientation for each robot
{
  if(MultiSolution.size() < 1)
    return;

  for(int j =  (int)MultiSolution.size()-2; j >= 0; j--)
  {
    for(int i = 0; i < (int)MultiSolution[0].size(); i += 3)
    {  
      if(MultiSolution[j][i] == MultiSolution[j+1][i] && MultiSolution[j][i+1] == MultiSolution[j+1][i+1])
        MultiSolution[j][i+2] = MultiSolution[j+1][i+2];
    }    
  }    
}

void calculate_rotation(vector<vector<float> >& MultiSolution) // calculates the rotation at each point based on the position of the next point
{
  if(MultiSolution.size() < 1)
    return;
  
  float theta;
  
  for(uint j = 0; j < MultiSolution.size()-1; j++) //note: leave start pose alone by j>0
  { 
    //printf("%d: \n", (int)j);
    for(int i = 0; i < (int)MultiSolution[0].size(); i += 3)
    {   
      if(MultiSolution[j][i] == MultiSolution[j+1][i] && MultiSolution[j][i+1] == MultiSolution[j+1][i+1])  // no movement
      {
        if(j+1 != MultiSolution.size()-1)  // not goal (want goal theta to remain the same)
          MultiSolution[j+1][i+2] = MultiSolution[j][i+2];  // use this theta at the next point (will be overwritten if it needs to be on next pass) 
      }
      else  // there is movement, so calculate the angle based on the relative angle of this time-step location vs next time-step location
      {
        theta = atan2(MultiSolution[j+1][i+1] - MultiSolution[j][i+1], MultiSolution[j+1][i] - MultiSolution[j][i]); 
          
        while(theta < -PI)
         theta += 2*PI;
        while(theta > PI)
         theta -= 2*PI;
         
        MultiSolution[j][i+2] = theta;
        if(j+1 != MultiSolution.size()-1)  // not goal (want goal theta to remain the same)
          MultiSolution[j+1][i+2] = theta; // assuming two point where there is rotation
      }
      //printf("%f, %f, %f --- ", MultiSolution[j][i], MultiSolution[j][i+1], MultiSolution[j][i+2]);
    }
    //printf("\n");
  }
  
  //printf("%d (g): \n", (int)MultiSolution.size()-1);
  //for(int i = 0; i < (int)MultiSolution[MultiSolution.size()-1].size(); i += 3)
  //  printf("%f, %f, %f --- ", MultiSolution[MultiSolution.size()-1][i], MultiSolution[MultiSolution.size()-1][i+1], MultiSolution[MultiSolution.size()-1][i+2]);
  //printf("\n");
  //getchar();
}

void calculate_times(vector<float>& Times, vector<vector<float> >& MultiSolution, float mps_target, float rps_target) // calculates the time parametery of the MultiSolution, where mps_target is the target meters per second of the fastest moving robot and rps is the target radians per second of fastest moving robot, note that these should be slightly slower than the max possible values to allow a behind robot to catch up, assumes 2D workspace and orientation for each robot
{
  Times.resize(MultiSolution.size());
  
  if(Times.size() < 1)
    return;
          
  Times[0] = 0; // start time
    
  float this_max, this_val, x_dif, y_dif, t_dif; // t_diff is theta diff (not time diff)
  
  //printf("times: 0");
  
  int num_of_points = MultiSolution.size();
  for(int i = 1; i < num_of_points; i++)
  {
    int num_of_dimensions = MultiSolution[i].size();
              
    // find max time required by any robot
    this_max = 0;
    for(int j = 0; j < num_of_dimensions; j +=3)
    {
      // find max time based on translation
      x_dif = MultiSolution[i][j] - MultiSolution[i-1][j];
      y_dif = MultiSolution[i][j+1] - MultiSolution[i-1][j+1];
      this_val = sqrt((x_dif*x_dif) + (y_dif*y_dif))/mps_target;
      
      if(this_val > this_max)
        this_max = this_val;
              
      // find max time based on rotation
      t_dif = MultiSolution[i][j+2] - MultiSolution[i-1][j+2];
      while(t_dif < -PI)
        t_dif += PI;
      
      while(t_dif > PI)
        t_dif -= PI;
      
      if(t_dif < 0)
         t_dif *= -1;
              
      this_val = t_dif/rps_target;
      if(this_val > this_max)
        this_max = this_val;  
    }
    Times[i] = Times[i-1] + this_max;
    //printf("%f \n", Times[i]);
  }
  //printf(". ---- MultiSolution.size()=%u \n", MultiSolution.size());
}

void verrify_start_angle(vector<vector<float> >& MultiSolution, vector<float>& start_config)  // makes sure start angles are correct
{
  for(int i = 0; i < (int)MultiSolution[0].size(); i += 3)
  {  
    printf("robot %d solution start: %f, %f, %f \n", i/3, MultiSolution[0][i], MultiSolution[0][i+1], MultiSolution[0][i+2]); 
    printf("robot %d actual start:   %f, %f, %f \n\n", i/3, start_config[i], start_config[i+1], start_config[i+2]);
  }
}

bool quads_overlap(float x1_min, float x1_length, float y1_min, float y1_length, float x2_min, float x2_length, float y2_min, float y2_length ) // returns true if the quads overlap
{
  float rad_1 = x1_length/2.0;  
  float rad_2 = x2_length/2.0;  
  float center_1 = x1_min + rad_1;
  float center_2 = x2_min + rad_2;  
  
  float center_dist = center_1 - center_2;
  if(center_dist < 0)
    center_dist *= -1;
    
  // now center_dist has the distance in the x direction between the two centers
  
  if(center_dist > rad_1+rad_2) // quads cannot possibly intersect
    return false;
  // otherwise the x overlap, so we need to look at y direction
  
  
  rad_1 = y1_length/2.0;  
  rad_2 = y2_length/2.0;  
  center_1 = y1_min + rad_1;
  center_2 = y2_min + rad_2;  
  
  center_dist = center_1 - center_2;
  if(center_dist < 0)
    center_dist *= -1;
    
  // now center_dist has the distance in the y direction between the two centers
  
  if(center_dist > rad_1+rad_2) // quads cannot possibly intersect
    return false;
  
  return true;
}

float PointSafe(const vector<float>& point, int index, float the_robot_rad, const vector<vector<vector<float> > >& obstacle_list) // this checks if a point is safe with respect to the obstacle list, where the points' coords start at index in vectors, it returns the minimum distance to an obstacle
{ 
  int num_polygons = obstacle_list.size(); 
  if(num_polygons == 0)
     return LARGE;
  
  float r_x, r_y, o_x1, o_x2, o_y1, o_y2, dist_temp;
  float min_dist = LARGE;
  int this_edge_num;
  
  r_x = point[index];
  r_y = point[index+1];
  
  //  do collision detection
  for(int i = 0; i < num_polygons; i++) // each polynomial
  {
    this_edge_num = obstacle_list[i].size();
    for(int j = 0; j < this_edge_num; j++) // each edge
    {        
      if(j == 0)   
      {   
        o_x1 = obstacle_list[i][this_edge_num-1][0];
        o_x2 = obstacle_list[i][0][0];
        o_y1 = obstacle_list[i][this_edge_num-1][1];
        o_y2 = obstacle_list[i][0][1];  
      }
      else
      {
        o_x1 = obstacle_list[i][j-1][0];
        o_x2 = obstacle_list[i][j][0];
        o_y1 = obstacle_list[i][j-1][1];
        o_y2 = obstacle_list[i][j][1]; 
      }
     
      dist_temp = line_dist_to_point(o_x1, o_y1, o_x2, o_y2, r_x, r_y) - the_robot_rad;
      if(dist_temp < min_dist)
        min_dist = dist_temp;
      
      dist_temp = line_dist_to_point(o_x1, o_y1, o_x2, o_y2, r_x, r_y) - the_robot_rad; 
      if(dist_temp < min_dist)
        min_dist = dist_temp;
      
      if(min_dist <= 0)
      {
        return min_dist;  // early break out if there is a collision
      }
    }    
  }    
  
  return min_dist; 
}

bool EdgeSafe(const vector<float>& point1, const vector<float>& point2, int index, const vector<vector<vector<float> > >& obstacle_list, float the_robot_rad) // this checks if an edge is safe vs obstacle_list, where the points' coords start at index in vectors (note assumes 2d points)
{
  int num_polygons = obstacle_list.size(); 
  if(num_polygons == 0)
     return true;

  float r_x1, r_x2, r_y1, r_y2, o_x1, o_x2, o_y1, o_y2, Mr_top, Mr_bottom, Mo_top, Mo_bottom, Mr, Mo, x, y;
  int this_edge_num;
  
  r_x1 = point1[index];
  r_x2 = point2[index];
  r_y1 = point1[index+1];
  r_y2 = point2[index+1];
    
  // do collision detection
  Mr_top = r_y2 - r_y1;
  Mr_bottom = r_x2 - r_x1;
            
  for(int i = 0; i < num_polygons; i++) // each polygon
  {
    this_edge_num = obstacle_list[i].size();
    for(int j = 0; j < this_edge_num; j++) // each edge
    {    
      if(j == 0)   
      {   
        o_x1 = obstacle_list[i][this_edge_num-1][0];
        o_x2 = obstacle_list[i][0][0];
        o_y1 = obstacle_list[i][this_edge_num-1][1];
        o_y2 = obstacle_list[i][0][1];  
      }
      else
      {
        o_x1 = obstacle_list[i][j-1][0];
        o_x2 = obstacle_list[i][j][0];
        o_y1 = obstacle_list[i][j-1][1];
        o_y2 = obstacle_list[i][j][1]; 
      }
     
      Mo_top = o_y2 - o_y1;
      Mo_bottom = o_x2 - o_x1;
      
      if(Mo_bottom != 0 && Mr_bottom != 0) // neither obstacle nor path is vertical 
      {
        Mr = Mr_top/Mr_bottom;
        Mo = Mo_top/Mo_bottom;
      
        // for numerical stability break this up based on which slope is larger
        if(fabs(Mr) <= fabs(Mo))
        {
          x = (Mo*o_x1-Mr*r_x1+r_y1-o_y1)/(Mo-Mr);
          y = Mr*(x-r_x1)+r_y1;
        }
        else
        {
          x = (Mr*r_x1-Mo*o_x1+o_y1-r_y1)/(Mr-Mo);
          y = Mo*(x-o_x1)+o_y1; 
        }
      }
      else if(Mo_bottom != 0) // && Mr_bottom == 0 // the path is vertical
      {
        Mo = Mo_top/Mo_bottom;
        x = r_x1;    
        y = Mo*(x-o_x1)+o_y1;   
      }
      else if(Mr_bottom != 0) // && Mo_bottom == 0 // the obstacle is vertical
      {
        Mr = Mr_top/Mr_bottom;
        x = o_x1;
        y = Mr*(x-r_x1)+r_y1;  
      }
      else // Mo_bottom == 0 && Mr_bottom == 0 // both path and obstacle are vertical
      {
        if(fabs(o_x1 - r_x1) > the_robot_rad)
          continue; // they are vertical and further then robot rad away, so no collision
          
        if(Mo_top != 0 && Mr_top != 0)  // neither the obstacle nor the path are points (both are vertical segments)  
        {
          if(((o_y1 <= r_y1 && r_y1 <= o_y2) || (o_y2 <= r_y1 && r_y1 <= o_y1)) ||
             ((o_y1 <= r_y2 && r_y2 <= o_y2) || (o_y2 <= r_y2 && r_y2 <= o_y1)) ||
             ((r_y1 <= o_y1 && o_y1 <= r_y2) || (r_y2 <= o_y1 && o_y1 <= r_y1)) ||
             ((r_y1 <= o_y2 && o_y2 <= r_y2) || (r_y2 <= o_y2 && o_y2 <= r_y1))) // collision
          { 
            return false;  
          }   
        } 
        else if(Mo_top != 0) // && Mr_top == 0 // the path is a point
        {
          x = r_x1;
          y = r_y1;
        }
        else if(Mr_top != 0) // && Mo_top == 0 // the obstacle is a point
        {
          x = o_x1;
          y = o_y1;  
        }
        else //(Mo_top == 0 && Mr_top == 0) // both obstacle and path are points
        {
          x = r_x1;
          y = r_y1;
        }
      }
      
      if((line_dist_to_point(o_x1, o_y1, o_x2, o_y2, x, y) <= the_robot_rad) &&
         ((o_x1 <= x && x <= o_x2) || (o_x2 <= x && x <= o_x1)) &&
         ((o_y1 <= y && y <= o_y2) || (o_y2 <= y && y <= o_y1)) &&
         ((r_x1 <= x && x <= r_x2) || (r_x2 <= x && x <= r_x1)) &&
         ((r_y1 <= y && y <= r_y2) || (r_y2 <= y && y <= r_y1))) // collision */
      {
        return false; 
      }
     
      // the lines do not intersect, but it is still possible that they are too close together
      // need 4 tests, one each for each end point tested against the other line
      
      
      /* assuming the path segment ends have already been checked against the obstacle
       * we can ignor the firt two tests */
      float dist_temp = line_dist_to_point(o_x1, o_y1, o_x2, o_y2, r_x1, r_y1); 
      if(dist_temp < the_robot_rad)
      {
        return false;
      }
     
      dist_temp = line_dist_to_point(o_x1, o_y1, o_x2, o_y2, r_x2, r_y2); 
      if(dist_temp < the_robot_rad)
      { 
        return false;
      }
      
      dist_temp = line_dist_to_point(r_x1, r_y1, r_x2, r_y2, o_x1, o_y1); 
      if(dist_temp < the_robot_rad)
      {
        return false;
      }
      
      dist_temp = line_dist_to_point(r_x1, r_y1, r_x2, r_y2, o_x2, o_y2); 
      if(dist_temp < the_robot_rad)
      {
        return false;
      }
    }    
  }    
  return true; 
}

bool SolutionSafe(const vector<vector<float> >& solution, const vector<vector<vector<float> > >& obstacle_list, float the_robot_rad, int path_dims) // this if the solution is safe vs obstacle_list, path_dims is the dimensions of 1 robot's path
{
  if(solution.size() == 0)
    return true;
  
  if(obstacle_list.size() == 0)
    return true;
  
  int num_solution_points = (int)solution.size();
  int num_solution_dims = (int)solution[0].size();
  
  // check per each solution edge
  for(int i = 0; i < num_solution_points - 1; i++)
  {   
    // check per each robot
    for(int j = 0; j < num_solution_dims; j += path_dims)
    {
      if(!EdgeSafe(solution[i], solution[i+1], j, obstacle_list, the_robot_rad))
        return false;
    }
  }
  return true;
}