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


// appends info to a file per agent in directory 
void data_dump_dynamic_team(const char* directory, const Cspace& C, const MultiAgentSolution& M, const GlobalVariables& G, POSE* robot_pose)
{   
  char this_file[100];
  sprintf(this_file, "../%s/agent_%d_team_stats.txt", directory, MultAgSln.agent_id);
  
  FILE* ofp = fopen(this_file,"a");
  while(ofp == NULL)
  {
      ofp = fopen(this_file,"a");
      printf("trying to open stats file in: %s\n",directory);
      return;
  }
  
  clock_t now_time = clock();
  float time_elapsed = difftime_clock(now_time, start_time);

  char sent_to_us[100];
  char temp[20];
  sprintf(sent_to_us,"s: ");  
  for(int i = 0; i < G.number_of_agents; i++)
  {  
    sprintf(temp,"%f ", M.messages_sent_to_us[i]);  
    strcat(sent_to_us, temp); 
  }
  
  char recieved_by_us[100];
  sprintf(recieved_by_us,"r: ");  
  for(int i = 0; i < G.number_of_agents; i++)
  {  
    sprintf(temp,"%f ", M.messages_recieved_by_us[i]);  
    strcat(recieved_by_us, temp); 
  }
  
  char in_team[100];
  sprintf(in_team,"t: ");  
  for(int i = 0; i < G.number_of_agents; i++)
  {  
    if(G.InTeam[i])
      sprintf(temp,"1 ");
    else
      sprintf(temp,"0 "); 
    strcat(in_team, temp); 
  }
  
  fprintf(ofp, "%d %f %f %f %f %f %d %d %s%s%s\n", G.agent_number, time_elapsed, M.best_solution_length, robot_pose->x, robot_pose->y, robot_pose->alpha, G.number_of_agents, G.team_size, sent_to_us, recieved_by_us, in_team);
  
  fclose(ofp);
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
  return ((float)(clock_time_1-clock_time_2))/((float)CLOCKS_PER_SEC);
}

float difftime_timeval(const timeval& time_1, const timeval& time_2) // returns the difference in seconds between the timeval time 1 and 2, (2 is assumed earlier than 1)
{
  return ((float)time_1.tv_sec + (((float)time_1.tv_usec)/1000000.0)) - ((float)time_2.tv_sec + (((float)time_2.tv_usec)/1000000.0));
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

void extract_and_translate_solution(vector<vector<float> >& AgentSolution, vector<vector<float> >& MultiSolution, vector<float>& Translation, int agent_id, int dims) // extracts the agent solution of agent_id where dims is the workspace dimensions and Translate holds the inverse ammount to translate along each
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

  float r_x1, r_x2, r_y1, r_y2, o_x1, o_x2, o_y1, o_y2, Mr_top, Mr_bottom, Mo_top, Mo_bottom, Mr, Mo;
  float x = LARGE;
  float y = LARGE;
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


int add_int_to_buffer(const int &i, void* buffer) // add int to buffer and returns the size in chars
{
  size_t buffer_ptr = (size_t)buffer;
  memcpy((void*)buffer_ptr, (void*)&i, sizeof(i));
  buffer_ptr += sizeof(i);

  return (int)(buffer_ptr - ( (size_t)buffer));
}

int extract_int_from_buffer(int &i, void* buffer) // extracts int from buffer and places it in i, returns the number of chars that were used
{
  size_t buffer_ptr = (size_t)buffer;

  memcpy(&i, (void*)buffer_ptr, sizeof(int));
  buffer_ptr += sizeof(int);

  return (int)(buffer_ptr - ((size_t)buffer));
}

int add_2d_vector_to_buffer(const vector<vector<float> > &v, void* buffer) // add v to buffer and returns the size in chars
{
  size_t buffer_ptr = (size_t)buffer;

  int r = (int)v.size();

  int c = 0;
  if(r > 0)
    c = (int)v[0].size();

  // add size of first dimension
  memcpy((void*)buffer_ptr, (void*)&r, sizeof(r));
  buffer_ptr += sizeof(r);

  // add size of second dimension
  memcpy((void*)buffer_ptr, &c, sizeof(int));
  buffer_ptr += sizeof(int);

  //printf("adding: [%d, %d]\n", r, c);

  // add all points
  for(int i = 0; i < r; i++)
  {
    for(int j = 0; j < c; j++)
    {
      float t = v[i][j];
      //printf("%f, ", t);

      memcpy((void*)buffer_ptr, &t, sizeof(float));
      buffer_ptr += sizeof(float);
    }
    //printf("\n");
  }

  return (int)(buffer_ptr - ( (size_t)buffer));
}

int extract_2d_vector_from_buffer(vector<vector<float> > &v, void* buffer) // extracts a 2d vector from buffer and places it in v, returns the number of chars that were used
{
  size_t buffer_ptr = (size_t)buffer;
  int r, c;

  // extract size of first dimension
  memcpy(&r, (void*)buffer_ptr, sizeof(int));
  buffer_ptr += sizeof(int);

  // extract size of second dimension
  memcpy(&c, (void*)buffer_ptr, sizeof(int));
  buffer_ptr += sizeof(int);

  //printf("extracting [%d, %d]: \n", r, c);

  // add all points
  v.resize(r);
  for(int i = 0; i < r; i++)
  {
    v[i].resize(c);
    for(int j = 0; j < c; j++)
    {
      float t;
      memcpy(&t, (void*)buffer_ptr, sizeof(float));
      buffer_ptr += sizeof(float);
      v[i][j] = t;

       //printf("%f, ", t);
    }
    //printf("\n");
  }

  return (int)(buffer_ptr - ((size_t)buffer));
}



// returns the 2D eudlidian distance between two points (using first 2 dims)
float euclid_dist(const vector<float> & A, const vector<float> &B)
{
  return sqrt( ( (A[0] - B[0]) * (A[0] - B[0]) ) + ( (A[1] - B[1]) * (A[1] - B[1]) ) );
}

// finds the closest point along the edge [edge0, edge1] to the point within resolution, returns false if closest_point is not populated
bool find_closest_point_along_edge(const vector<float> & point, const vector<float> & edge0, const vector<float> & edge1, float resolution, vector<float> & closest_point, float & best_dist_to_point)
{
  float obs_edge_length = euclid_dist(edge0, edge1);
  float obs_step = obs_edge_length*resolution;
  float obs_step_x = (obs_step/obs_edge_length)*(edge1[0] - edge0[0]);
  float obs_step_y = (obs_step/obs_edge_length)*(edge1[1] - edge0[1]);

  vector<float> obs_point = edge0;

  best_dist_to_point = LARGE;
  for(float obs_d = 0; obs_d <= obs_edge_length; obs_d += obs_step)
  {
    //printf("obs_point: [%f %f] ", obs_point[0], obs_point[1]);

    float dist_to_point = euclid_dist(obs_point, point);

    if(dist_to_point < best_dist_to_point)
    {       
      //printf("(new best %f)", dist_to_point);
      best_dist_to_point = dist_to_point;
      closest_point = obs_point;
    }
    //printf("\n");
    obs_point[0] += obs_step_x;
    obs_point[1] += obs_step_y;
  }

  if(best_dist_to_point == LARGE)
    return false;

  return true;
}


// finds the closest point along the path to the point within resolution, returns false if closest_point is not populated
bool find_closest_point_along_path(const vector<float> & point, const vector<vector<float> > & path, float resolution, vector<float> & closest_point, float & best_dist_to_point)
{
  best_dist_to_point = LARGE;
  uint size_of_path = path.size()-1;
  for(uint i = 0; i < size_of_path; i++)
  {
    float dist_to_point;
    vector<float> this_point;

    //printf(" :::: [%f %f] [%f %f] [%f %f]\n", point[0], point[1], path[i][0], path[i][1], path[i+1][0], path[i+1][1]);

    if(find_closest_point_along_edge(point, path[i], path[i+1], resolution, this_point, dist_to_point))
    {     
      if(dist_to_point < best_dist_to_point)
      {
        //printf("new best, edge ind %d to %d at %f\n", i, i+1, dist_to_point);
   
        best_dist_to_point = dist_to_point;
        closest_point = this_point;
      }
    }
  }

  if(best_dist_to_point == LARGE)
    return false;

  return true;
}

// helps the function below, returns true of the point is within robot_rad of the edge to within resolution
bool edge_and_point_conflict(const vector<float> & point, const vector<vector<float> > & edge, float robot_rad, float resolution)
{
  float obs_edge_length = euclid_dist(edge[0], edge[1]);
  float obs_step = obs_edge_length*resolution;
  float obs_step_x = (obs_step/obs_edge_length)*(edge[1][0] - edge[0][0]);
  float obs_step_y = (obs_step/obs_edge_length)*(edge[1][1] - edge[0][1]);

  vector<float> obs_point = edge[0];

  for(float obs_d = 0; obs_d <= obs_edge_length; obs_d += obs_step)
  {
    if(robot_rad > euclid_dist(obs_point, point))
    {       
      // then there is a collision
      return true;
    }
    obs_point[0] += obs_step_x;
    obs_point[1] += obs_step_y;
  }

  return false;
}


// used in the function below
bool edge_and_edges_conflict(const vector<vector<float> > & robot_forward_conflict, 
                             const vector<vector<vector<float> > > & obstacle_forward_conflicts,
                             const vector<vector<vector<float> > > & obstacle_backward_conflicts,
                             bool found_obstacle_forward_conflict,
                             bool found_obstacle_backward_conflict,
                             vector<float> & robot_forward_point_conflict, 
                             float robot_rad, float resolution)
{
  // find first robot_forward conflict point

  float edge_length = euclid_dist(robot_forward_conflict[0], robot_forward_conflict[1]);

  if(edge_length < SMALL)
    return false;

  float step = edge_length*resolution;
  float step_x = (step/edge_length)*(robot_forward_conflict[1][0] - robot_forward_conflict[0][0]);
  float step_y = (step/edge_length)*(robot_forward_conflict[1][1] - robot_forward_conflict[0][1]);

  vector<float> point = robot_forward_conflict[0];

  for(float d = 0; d <= edge_length; d += step)
  {
    if(found_obstacle_forward_conflict)
    {
      bool found_robot_forward_point_conflict = false;
      for(uint i = 0; i < obstacle_forward_conflicts.size(); i++)
      {
        if(edge_and_point_conflict(point, obstacle_forward_conflicts[i], robot_rad, resolution))
        {    
          // then there is a collision
          if(!found_robot_forward_point_conflict)
          {
            // first one
            robot_forward_point_conflict = point;
            found_robot_forward_point_conflict = true;
          }
          else if( euclid_dist(robot_forward_conflict[0], point) < euclid_dist(robot_forward_conflict[0], robot_forward_point_conflict) )
          {
            // not first one, but only get here if it is closer to start of path 
            robot_forward_point_conflict = point;
          }
        }
      }

      if(found_robot_forward_point_conflict)
      {
        return true;
      }
    }

    if(found_obstacle_backward_conflict)
    {
      bool found_robot_forward_point_conflict = false;
      for(uint i = 0; i < obstacle_backward_conflicts.size(); i++)
      {
        if(edge_and_point_conflict(point, obstacle_backward_conflicts[i], robot_rad, resolution))
        {    
          // then there is a collision
          if(!found_robot_forward_point_conflict)
          {
            // first one
            robot_forward_point_conflict = point;
            found_robot_forward_point_conflict = true;
          }
          else if( euclid_dist(robot_forward_conflict[0], point) < euclid_dist(robot_forward_conflict[0], robot_forward_point_conflict) )
          {
            // not first one, but only get here if it is closer to start of path 
            robot_forward_point_conflict = point;
          }
        }
      }
      if(found_robot_forward_point_conflict)
      {
        return true;
      }
    }

    point[0] += step_x;
    point[1] += step_y;
  }

  return false;
}


// finds the first and last conflict (with respect to robot path) between robot path and obstacle path. Edges are check at resolution, and conflicts take into account robot rad. Returns true if there is a conflict, else false
bool find_conflict_points(const vector<vector<float> > &robot_path, const vector<vector<float> > &obstacle_path, float robot_rad, float resolution, vector<float> &first_r_conflict,  float &dist_to_first_r_conflict,
vector<float> &last_r_conflict, float &dist_to_last_r_conflict,
vector<float> &first_o_conflict, float &dist_to_first_o_conflict,  
vector<float> &last_o_conflict,  float &dist_to_last_o_conflict)
{
  // store obstacle path in way that lets it be used with code I've already written
  vector<vector<vector<float> > > obstacle_path_list(1,obstacle_path);

  // store robot path in way that lets it be used with code I've already written
  vector<vector<vector<float> > > robot_path_list(1,robot_path);

  bool found_robot_forward_conflict = false;
  bool found_robot_backward_conflict = false;
  bool found_obstacle_forward_conflict = false;
  bool found_obstacle_backward_conflict = false;

  vector<vector<vector<float> > > robot_forward_conflicts;
  vector<vector<vector<float> > > robot_backward_conflicts;
  vector<vector<vector<float> > > obstacle_forward_conflicts;
  vector<vector<vector<float> > > obstacle_backward_conflicts;

  // robot_path forward direction:

  // for each edge in the robot path, check if it conflicts with the obstacle path
  for(uint i = 0; i < robot_path.size()-1; i++)
  {
    if(EdgeSafe(robot_path[i], robot_path[i+1], 0, obstacle_path_list, robot_rad))
      continue;

    // if here then this edge had a conflict
    vector<vector<float> > robot_forward_conflict(2);
    robot_forward_conflict[0] = robot_path[i];
    robot_forward_conflict[1] = robot_path[i+1];
    robot_forward_conflicts.push_back(robot_forward_conflict);

    if(!found_robot_forward_conflict) // first forward conflict
    {
      // save distance up until start of conflict edge for later use
      dist_to_first_r_conflict = 0;
      for(uint j = 0; j <= i; j++)
        dist_to_first_r_conflict += euclid_dist(robot_path[j], robot_path[j+1]);

      found_robot_forward_conflict = true;
    }
  }

  // robot_path backward direction:

  // for each edge in the robot path, check if it conflicts with the obstacle path
  for(uint i = robot_path.size()-1; i > 0; i--)
  {
    if(EdgeSafe(robot_path[i], robot_path[i-1], 0, obstacle_path_list, robot_rad))
      continue;

    // if here then this edge had a conflict
    vector<vector<float> > robot_backward_conflict(2);
    robot_backward_conflict[0] = robot_path[i];
    robot_backward_conflict[1] = robot_path[i-1];
    robot_backward_conflicts.push_back(robot_backward_conflict);

    if(!found_robot_backward_conflict) // first backward conflict
    {
      // save distance up until start of conflict edge for later use
      dist_to_last_r_conflict = 0;
      for(uint j = i; j < robot_path.size()-1; j++)
        dist_to_last_r_conflict += euclid_dist(robot_path[j], robot_path[j+1]);

      found_robot_backward_conflict = true;
    }
  }

  // obstacle_path forward direction:

  // for each edge in the obstacle path, check if it conflicts with the robot path
  for(uint i = 0; i < obstacle_path.size()-1; i++)
  {
    if(EdgeSafe(obstacle_path[i], obstacle_path[i+1], 0, robot_path_list, robot_rad))
      continue;

    // if here then this edge had a conflict
    vector<vector<float> > obstacle_forward_conflict(2);
    obstacle_forward_conflict[0] = obstacle_path[i];
    obstacle_forward_conflict[1] = obstacle_path[i+1];
    obstacle_forward_conflicts.push_back(obstacle_forward_conflict);
 
    if(!found_obstacle_forward_conflict) // first forward conflict
    {
      // save distance up until start of conflict edge for later use
      dist_to_first_o_conflict = 0;
      for(uint j = 0; j <= i; j++)
        dist_to_first_o_conflict += euclid_dist(obstacle_path[j], obstacle_path[j+1]);

      found_obstacle_forward_conflict = true;
    }
  }


  // obstacle_path backward direction:

  // for each edge in the obstacle path, check if it conflicts with the robot path
  for(uint i = obstacle_path.size()-1; i > 0; i--)
  {
    if(EdgeSafe(obstacle_path[i], obstacle_path[i-1], 0, robot_path_list, robot_rad))
      continue;

    // if here then this edge had a conflict
    vector<vector<float> > obstacle_backward_conflict(2);
    obstacle_backward_conflict[0] = obstacle_path[i];
    obstacle_backward_conflict[1] = obstacle_path[i-1];
    obstacle_backward_conflicts.push_back(obstacle_backward_conflict);

    if(!found_obstacle_backward_conflict) // first backward conflict
    {
      // save distance up until start of conflict edge for later use
      dist_to_last_o_conflict = 0;
      for(uint j = i; j < obstacle_path.size()-1; j++)
        dist_to_last_o_conflict += euclid_dist(obstacle_path[j], obstacle_path[j+1]);

      found_obstacle_backward_conflict = true;
    }
  }

  // search for points that actually conflict to within resolution
  // only need to compare conflict edges between obstacle and 

  bool found_robot_forward_point_conflict = false;
  bool found_robot_backward_point_conflict = false;
  bool found_obstacle_forward_point_conflict = false;
  bool found_obstacle_backward_point_conflict = false;

  vector<float> robot_forward_point_conflict;
  vector<float> robot_backward_point_conflict;
  vector<float> obstacle_forward_point_conflict;
  vector<float> obstacle_backward_point_conflict;

  if(found_robot_forward_conflict)
  {
    // find first robot_forward conflict point

    found_robot_forward_point_conflict = edge_and_edges_conflict(robot_forward_conflicts[0], 
                                                                  obstacle_forward_conflicts, obstacle_backward_conflicts,
                                                                  found_obstacle_forward_conflict, found_obstacle_backward_conflict,
                                                                  robot_forward_point_conflict, robot_rad, resolution);
  }

  if(found_robot_backward_conflict)
  {
    // find first robot backward conflict point
    found_robot_backward_point_conflict = edge_and_edges_conflict(robot_backward_conflicts[0], 
                                                                  obstacle_forward_conflicts, obstacle_backward_conflicts, 
                                                                  found_obstacle_forward_conflict, found_obstacle_backward_conflict, 
                                                                  robot_backward_point_conflict, robot_rad, resolution);
  }

  if(found_obstacle_forward_conflict)
  {
    // find first obstacle_forward conflict point

    found_obstacle_forward_point_conflict = edge_and_edges_conflict(obstacle_forward_conflicts[0], 
                                                                    robot_forward_conflicts, robot_backward_conflicts, 
                                                                    found_robot_forward_conflict, found_robot_backward_conflict, 
                                                                    obstacle_forward_point_conflict, robot_rad, resolution);
  }


  if(found_obstacle_backward_conflict)
  {
    // find first obstacle_backward conflict point

    found_obstacle_backward_point_conflict = edge_and_edges_conflict(obstacle_backward_conflicts[0], 
                                                                     robot_forward_conflicts, robot_backward_conflicts, 
                                                                     found_robot_forward_conflict, found_robot_backward_conflict,
                                                                     obstacle_backward_point_conflict, robot_rad, resolution);
  }


  // add remaining distance along paths to conflict points
  if(found_robot_forward_point_conflict)
    dist_to_first_r_conflict += euclid_dist(robot_forward_conflicts[0][0], robot_forward_point_conflict);
  else
    dist_to_first_r_conflict = LARGE;

  if(found_robot_backward_point_conflict)
    dist_to_last_r_conflict += euclid_dist(robot_backward_conflicts[0][0], robot_backward_point_conflict);
  else
    dist_to_last_r_conflict = LARGE;

  if(found_obstacle_forward_point_conflict)
    dist_to_first_o_conflict += euclid_dist(obstacle_forward_conflicts[0][0], obstacle_forward_point_conflict);
  else
    dist_to_first_o_conflict = LARGE;

  if(found_obstacle_backward_point_conflict)
    dist_to_last_o_conflict += euclid_dist(obstacle_backward_conflicts[0][0], obstacle_backward_point_conflict);
  else
    dist_to_last_o_conflict = LARGE;



  if(!found_robot_forward_point_conflict && !found_robot_backward_point_conflict &&
     !found_obstacle_forward_point_conflict && !found_obstacle_backward_point_conflict)
  {
    return false;
  }
  else if(!found_robot_forward_point_conflict && !found_robot_backward_point_conflict)
  {
    // this should probably never happen
    printf("this should probably never happen\n");
    return false;
  }
  else if(!found_obstacle_forward_point_conflict && !found_obstacle_backward_point_conflict)
  {
    // this should probably never happen
    printf("this should probably never happen\n");
    return false;
  }

  if(!found_robot_forward_point_conflict && found_robot_backward_point_conflict)
  {
    robot_forward_point_conflict = robot_backward_point_conflict;
  }
  else if(found_robot_forward_point_conflict && !found_robot_backward_point_conflict)
  {
    robot_backward_point_conflict = robot_forward_point_conflict;
  } 

  if(!found_obstacle_forward_point_conflict && found_obstacle_backward_point_conflict)
  {
    obstacle_forward_point_conflict = obstacle_backward_point_conflict;
  }
  else if(found_obstacle_forward_point_conflict && !found_obstacle_backward_point_conflict)
  {
    obstacle_backward_point_conflict = obstacle_forward_point_conflict;
  } 

  first_r_conflict = robot_forward_point_conflict;
  last_r_conflict = robot_backward_point_conflict;

  first_o_conflict = obstacle_forward_point_conflict;
  last_o_conflict = obstacle_backward_point_conflict;

  return true;
}

// moves the point that is located on edge with inds [i , i+1] time resolution along the path P
void move_time_res_along_path(const vector<vector<float> > & P, float time_res, int &i, vector<float> & point, bool & at_end_P)
{
  while(P[i+1][2] - point[2] < time_res) // not enough space on the current edge
  {
    // remove the ammount of time that it takes to get to the end of the current edge
    time_res -= P[i+1][2] - point[2];
    // increase the edge
    i++;

    // make the point the first point on the new edge
    point = P[i];

    if(i+1 >= (int)P.size()) // this is the last edge
    {
      at_end_P = true;
      return;
    }
  }

  //  calculate the point on the edge [i, i+1] where time is time_res ahead of the current point
  float proportion_along_edge_of_new_point = time_res/(P[i+1][2] - P[i][2]);

  point[0] += proportion_along_edge_of_new_point*(P[i+1][0] -  point[0]);
  point[1] += proportion_along_edge_of_new_point*(P[i+1][1] -  point[1]);
  point[2] += time_res;
}  

// finds the first conflict (with respect to time) between paths (A and B) accounting for time (unlike the above function),  each path point is assumed [x y time]. time_res is the time granularity to use
bool find_first_time_conflict_points(const vector<vector<float> > &A, const vector<vector<float> > &B, float robot_rad, float time_res, vector<float> &A_conflict, vector<float> &B_conflict)
{
  int a = 0;  // index for A
  int b = 0;  // index for b
  bool at_end_A = false; // gets set to true 
  bool at_end_B = false;

  int length_A = A.size();
  int length_B = B.size();

  if(length_A < 1 || length_B < 1)
    return false;

  vector<float> point_on_A = A[0];
  vector<float> point_on_B = B[0];

  robot_rad *= 2; //since 1 rad for each robot

  // times may not start or end at the same time, 
  // find the earliest time that is common to either path

  if(A[a][2] == B[0][2])
  {
    // do nothing
    // printf("A = B\n");
  }
  else if(A[a][2] < B[0][2]) // A starts earlier than B
  {
    // printf("A < B\n");

    // find the last edge node on A that is earlier than B start
    while(A[a][2] < B[0][2])
    {
      if(a+1 >= length_A ) // at end of A
      {
        at_end_A = true;
        break;
      }

      if(A[a+1][2] > B[0][2])
      {
        // found edge in A that contains the point that is at the same time as the point in B, now find the actual point
        float proportion_along_A_of_B = (B[0][2] - A[a][2])/(A[a+1][2] - A[a][2]);

        point_on_A[0] = A[a][0] + proportion_along_A_of_B*(A[a+1][0] -  A[a][0]);
        point_on_A[1] = A[a][1] + proportion_along_A_of_B*(A[a+1][1] -  A[a][1]);
        point_on_A[2] = B[0][2];

        break;
      }

      a++;
    }
  }
  else // B starts earlier than A
  {
    // printf("A > B\n");

    // find the last edge node on B that is earlier than A start
    while(B[b][2] < A[0][2])
    {
      if(b+1 >= length_B ) // at end of B
      {
        at_end_B = true;
        break;
      }

      if(B[b+1][2] > A[0][2])
      {
        // found edge in B that contains the point that is at the same time as the point in A, now find the actual point
        float proportion_along_B_of_A = (A[0][2] - B[b][2])/(B[b+1][2] - B[b][2]);

        point_on_B[0] = B[b][0] + proportion_along_B_of_A*(B[b+1][0] -  B[b][0]);
        point_on_B[1] = B[b][1] + proportion_along_B_of_A*(B[b+1][1] -  B[b][1]);
        point_on_B[2] = A[0][2];

        break;
      }

      b++;
    }
  }

  if(at_end_A)               // at the end A (we know we are still at the start of B)
  {
    // printf("at end A\n");

    if(A[a][2] <= B[0][2])    // also know that the last point on A is earlier than the forst point on B
    {
      // check if they conflict
      if(euclid_dist(A[a],B[0]) <= robot_rad) // there is a conflict
      {
        A_conflict = A[a];
        B_conflict = B[0];
        return true;
      }
      else // no conflict
      {
        return false;
      }
    }
  }

  if(at_end_B)               // at the end B (we know we are still at the start of A)
  {
    // printf("at end B\n");

    if(B[b][2] <= A[0][2])    // also know that the last point on B is earlier than the forst point on A
    {
      // check if they conflict
      if(euclid_dist(B[b],A[0]) <= robot_rad) // there is a conflict
      {
        A_conflict = A[0];
        B_conflict = B[b];
        return true;
      }
      else // no conflict
      {
        return false;
      }
    }
  }

  // move through paths at time_res checking if there are conflicts

  while(!at_end_A || !at_end_B)
  {
    // check if the current points conflict
    if( euclid_dist(point_on_A, point_on_B) <= robot_rad) // note euclid_dist only checks forst 2 dims
    {
      // they do conflict
      A_conflict = point_on_A;
      B_conflict = point_on_B;
      return true;
    }
  
    // move time_res along A
    if(!at_end_A)
      move_time_res_along_path(A, time_res, a, point_on_A, at_end_A);

    // move time_res along B
    if(!at_end_B)
      move_time_res_along_path(B, time_res, b, point_on_B, at_end_B);
  }

  return false;
}



// find edge that most likely contains point and return the index of that edge or -1 if nothing is found, and also return the point in the path that was the best match
int find_edge_containing_point(const vector<vector<float> > & robot_path, const vector<float> & point, vector<float> & best_point_found)
{
  vector<float> best_point;
  int best_i = -1;
  float best_dist = LARGE;

  int num_edges = robot_path.size()-1;
  for(int i = 0; i < num_edges; i++)
  {
    float edge_length = euclid_dist(robot_path[i], robot_path[i+1]);
    float dist_to_point = euclid_dist(robot_path[i], point);


    // check if dist_to_point along edge is approximatly start_point
    vector<float> approx_p(2);
    approx_p[0] = robot_path[i][0] + (dist_to_point/edge_length)*(robot_path[i+1][0] - robot_path[i][0]);
    approx_p[1] = robot_path[i][1] + (dist_to_point/edge_length)*(robot_path[i+1][1] - robot_path[i][1]);

    float this_dist = euclid_dist(approx_p, point);


    if(this_dist <= best_dist)
    {
      best_dist = this_dist;
      best_i = i;
      best_point = approx_p;
    }
  }

  best_point_found = best_point;
  return best_i;
}


// given data about a robot's path, start point along that path, and a planning region, this returns the last point along the path in the region
bool calculate_exit_point(const vector<vector<float> > & robot_path, const vector<float> & start_point, float area_min_x, float area_max_x, float  area_min_y, float area_max_y, float resolution, float robot_rad, vector<float> & exit_point)
{
  // find edge that contains point and remember both
  vector<float> best_point;
  int best_i = find_edge_containing_point(robot_path, start_point, best_point);

  if(best_i == -1) // did not find point along path
  {
    printf("did not find point along path \n");
    return false;
  }

  // create an edge list containing the boundary so can use collision detections stuff to check when an edge crosses it, put in a robot_rad buffer
  vector<float> the_boundary_0(2);
  the_boundary_0[0] = area_min_x - robot_rad; the_boundary_0[1] = area_min_y - robot_rad;
  vector<float> the_boundary_1(2);
  the_boundary_1[0] = area_min_x - robot_rad; the_boundary_1[1] = area_max_y + robot_rad;
  vector<float> the_boundary_2(2);
  the_boundary_2[0] = area_max_x + robot_rad; the_boundary_2[1] = area_max_y + robot_rad;
  vector<float> the_boundary_3(2);
  the_boundary_3[0] = area_max_x + robot_rad; the_boundary_3[1] = area_min_y - robot_rad;

  vector<vector<vector<float> > > boundary(4);
  boundary[0].push_back(the_boundary_0); boundary[0].push_back(the_boundary_1);
  boundary[1].push_back(the_boundary_1); boundary[1].push_back(the_boundary_2);
  boundary[2].push_back(the_boundary_2); boundary[2].push_back(the_boundary_3);
  boundary[3].push_back(the_boundary_3); boundary[3].push_back(the_boundary_0);

  vector<float> this_point;
  vector<float> last_point_in_boundary;
  bool found_last_point_in_boundary = false;
  int edge_that_intersects_boundary_i;

  int num_edges = robot_path.size()-1;
  for(int i = best_i; i < num_edges; i++)
  {
    if(i == best_i)
      this_point = best_point; 
    else
      this_point = robot_path[i];

    if(EdgeSafe(this_point, robot_path[i+1], 0, boundary, robot_rad))
    {
      continue;
    }

    // if here then this edge leaves the boundary, so find the last point within the boundary
    vector<vector<float> > robot_forward_conflict(2);
    robot_forward_conflict[0] = this_point;
    robot_forward_conflict[1] = robot_path[i+1];

    if(edge_and_edges_conflict(robot_forward_conflict, boundary, boundary, true, false, last_point_in_boundary, robot_rad, resolution))
    {
       // if here than last_point_in_boundary has been populated
       found_last_point_in_boundary = true;
       edge_that_intersects_boundary_i = i;
       break;
    }
  }

  if(!found_last_point_in_boundary)
  {
    // made it all the way to the goal without finding a conflict with the boundary
    return false;
  }
  // if here than found a last point in boundary
  exit_point = last_point_in_boundary;
  return true;
}


// returns true if any two points in sub_goals conflict within robot_rad, returns their inds in conflicting_ind_a, conflicting_ind_b
bool any_two_points_conflict(const vector<vector<float> > & sub_goals, const vector<bool> & InTeam, float robot_rad, int & conflicting_ind_a, int & conflicting_ind_b)
{
  robot_rad *= 2; // one for each robot

  int max_num_robots = (int)sub_goals.size();
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    for(int j = i+1; j < max_num_robots; j++)
    {
      if(!InTeam[j])
        continue;
 
      //printf("checking goals of robot %d vs %d\n", i, j);
      float diff_dist = euclid_dist(sub_goals[i], sub_goals[j]);
      //printf(":::: %f > %f, [%f %f] [%f %f] \n", diff_dist, robot_rad, sub_goals[i][0], sub_goals[i][1], sub_goals[j][0], sub_goals[j][1]);
      if(diff_dist < robot_rad)
      {
        //printf("two goals are too close: %f < %f, [%f %f] [%f %f] \n", diff_dist, robot_rad, sub_goals[i][0], sub_goals[i][1], sub_goals[j][0], sub_goals[j][1]);

       conflicting_ind_a = i; 
       conflicting_ind_b = j;
       return true;
      }
    }
  }
  //printf("no points conflict \n");
  return false;
}

// moves the point step along the path, assumes that point is already on path
void move_point_along_path(vector<float> & point, const vector<vector<float> > & path, float step)
{
  // find edge that most likely contains point and return the index of that edge or -1 if nothing is found, and also return the point in the path that was the best match
  vector<float> best_point_found;
  int ind_of_edge = find_edge_containing_point(path, point, best_point_found);

  float total_dist_moved = 0;  
  while(total_dist_moved < step)
  {

    float edge_length = euclid_dist(best_point_found, path[ind_of_edge+1]); // length of edge remaining
  
    while(edge_length < SMALL && ind_of_edge+1 < ((int)path.size())-1) // while almost at the next edge and next edge is less than the final edge
    {
      total_dist_moved += edge_length;
      ind_of_edge++;

      if(ind_of_edge+1 < ((int)path.size())-1) // at last edge
        break;

      best_point_found = path[ind_of_edge];
      edge_length = euclid_dist(best_point_found, path[ind_of_edge+1]);
    }

    edge_length = euclid_dist(best_point_found, path[ind_of_edge+1]);

    if(ind_of_edge+1 >= ((int)path.size())-1) // if this is the last edge
    {
      if(edge_length <= step - total_dist_moved)  // if the end of the path is within step - total_dist_moved
      {
        // set point to the end of the path
        printf("setting sub_goal to last point in path \n");
        point = path[ind_of_edge+1];
        return; 
      }
    }

    if(edge_length <= step - total_dist_moved) // move to the end of the current edge
    {
      best_point_found = path[ind_of_edge+1];
      total_dist_moved += edge_length;
      continue;
    }
    else
    {
      break;
    }
  }

  // if moved enough
  if(total_dist_moved >= step)
  {
    point = best_point_found;
  }

  // otherwise move the rest of what we need along the current edge
  // we know edge_length > step - total_dist_moved = dist_ramaining_to_move
  float edge_length = euclid_dist(best_point_found, path[ind_of_edge+1]); // length of edge remaining
  float dist_ramaining_to_move = step - total_dist_moved;
  best_point_found[0] += (dist_ramaining_to_move/edge_length)*(path[ind_of_edge+1][0] - best_point_found[0]);
  best_point_found[1] += (dist_ramaining_to_move/edge_length)*(path[ind_of_edge+1][1] - best_point_found[1]);
  point = best_point_found;
}



// calculates the sub_start and sub goal that this robot should use for multi-robot path planning based on single robot paths, returns true if finds a new sub_start and sub_goal. no_conflicts_between_single_paths gets set to true if there is no conflicts between anybody
bool calculate_sub_goal(const vector<vector<vector<float> > > & robot_paths, const vector<bool> & InTeam, int this_agent_id, float preferred_min_planning_area_side_length, float preferred_max_planning_area_side_length, float robot_rad, float resolution, float time_res, vector<float> & sub_start, vector<float> & sub_goal, bool & no_conflicts_between_single_paths)
{
  // Note: There are probably better ways to do this than duplicating it on each agent, but this works and runtime is only robots squared

  no_conflicts_between_single_paths = false;

  // see if all paths fit within the preferred planning area
  float max_x = -LARGE;
  float max_y = -LARGE;

  float min_x = LARGE;
  float min_y = LARGE;

  int max_num_robots = InTeam.size();
  for(int r = 0; r < max_num_robots; r++)
  {
    if(!InTeam[r])
      continue;

    int num_points = robot_paths[r].size();
    for(int i = 0; i < num_points; i++)
    {
      if(robot_paths[r][i][0] > max_x)
        max_x = robot_paths[r][i][0];

      if(robot_paths[r][i][0] < min_x)
        min_x = robot_paths[r][i][0];

      if(robot_paths[r][i][1] > max_y)
        max_y = robot_paths[r][i][1];

      if(robot_paths[r][i][1] < min_y)
        min_y = robot_paths[r][i][1];
    }
  }

  if( (max_x - min_x <= preferred_max_planning_area_side_length) && 
      (max_y - min_y <= preferred_max_planning_area_side_length) )  // then we can just use normal start and goal
  {

    // share extra space evenly among different sides
    if(max_x - min_x < preferred_min_planning_area_side_length) // need to increase x 
    {
      float x_increase = (preferred_min_planning_area_side_length - (max_x - min_x))/2;
      max_x += x_increase;
      min_x -= x_increase;
    }

    if(max_y - min_y < preferred_min_planning_area_side_length) // need to increase y 
    {
      float y_increase = (preferred_min_planning_area_side_length - (max_y - min_y))/2;
      max_y += y_increase;
      min_y -= y_increase;
    }

    printf("region bounds: x:[%f, %f] y:[%f, %f]\n", min_x, max_x, min_y, max_y);
    printf("all paths fit, just use normal start and goal\n");

    return false;
  }

  // go through and find all robot vs robot first conflicts, as well as bounds on the area containing all first conflicts
  vector<vector<float> > first_conflicts(max_num_robots);
  vector<float> time_of_first_conflicts(max_num_robots, LARGE);

  bool found_this_robots_conflicts = false;

  float max_x_first_only = -LARGE;
  float max_y_first_only = -LARGE;

  float min_x_first_only = LARGE;
  float min_y_first_only = LARGE;

  bool there_is_at_least_one_conflict = false;
  bool there_is_at_least_one_robot_without_conflicts = false;
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    for(int j = i+1; j < max_num_robots; j++)
    { 
      if(!InTeam[j])
        continue;

      vector<float> first_i_conflict;
      vector<float> first_j_conflict;

      if(find_first_time_conflict_points(robot_paths[i], robot_paths[j], robot_rad, time_res, first_i_conflict, first_j_conflict) )
      {
        //printf("agent %d conflicts with agent %d at [%f %f %f] %f\n", i, j, first_i_conflict[0], first_i_conflict[1], first_i_conflict[2], euclid_dist(first_i_conflict, first_j_conflict));
        if(first_i_conflict[2] <= time_of_first_conflicts[i]) // this i conflict is the soonest for i that we have found
        {
          first_conflicts[i] = first_i_conflict;
          time_of_first_conflicts[i] = first_i_conflict[2];

          if(first_i_conflict[0] > max_x_first_only)
            max_x_first_only = first_i_conflict[0];

          if(first_i_conflict[1] > max_y_first_only)
            max_y_first_only = first_i_conflict[1];

          if(first_i_conflict[0] < min_x_first_only)
            min_x_first_only = first_i_conflict[0];

          if(first_i_conflict[1] < min_y_first_only)
            min_y_first_only = first_i_conflict[1];
        }

        if(first_j_conflict[2] <= time_of_first_conflicts[j]) // this j conflict is the soonest for j that we have found
        {
          first_conflicts[j] = first_j_conflict;
          time_of_first_conflicts[j] = first_j_conflict[2];

          if(first_j_conflict[0] > max_x_first_only)
            max_x_first_only = first_j_conflict[0];

          if(first_j_conflict[1] > max_y_first_only)
            max_y_first_only = first_j_conflict[1];

          if(first_j_conflict[0] < min_x_first_only)
            min_x_first_only = first_j_conflict[0];

          if(first_j_conflict[1] < min_y_first_only)
            min_y_first_only = first_j_conflict[1];
        }

        there_is_at_least_one_conflict = true;
      }
      else
      {
        there_is_at_least_one_robot_without_conflicts = true;
      }
    }
  }


  // check if any robots did not conflict, this may happen if a multi-path was 1/2 way through planning when a new robot was added to the team
  // that does not conflict with the multi-path but does conflict with the single robot path
  // (we need a portion of their path to be in the sub area so they are accounted for) (yes, this is a bit hacky, but it shouldn't happen much)
  if(there_is_at_least_one_robot_without_conflicts) // at least one robot needs a first conflict
  {
    // check if there is at least 1 conflict
    if(there_is_at_least_one_conflict) // there is at least one conflict
    {
      // for all robots that are in the team and do not have sub_start and sub_goal, call sub_start (and sub_goal) the closest
      // point on their path to the centroid of all start_conflicts

      // calculate centroid
      float center_x = 0;
      float center_y = 0;
      float num = 0;
      for(int i = 0; i < max_num_robots; i++)
      {
        if(!InTeam[i])
          continue;

        if(first_conflicts[i].size() < 2)  // in team but no conflict
          continue;

        center_x += first_conflicts[i][0];
        center_y += first_conflicts[i][1];
        num++;
      }
      center_x /= num;
      center_y /= num;

      vector<float> centroid(2);
      centroid[0] = center_x;
      centroid[1] = center_y;
      //printf("centrod: [%f %f]\n", centroid[0], centroid[1]);
      // go through and find closest points as first conflicts for robots that need it
      for(int i = 0; i < max_num_robots; i++)
      {
        if(!InTeam[i])
          continue;

        if(first_conflicts[i].size() < 2)  // in team and needs a conflict
        {
          vector<float> closest_point;
          float best_dist_to_point;

          //printf("robot %d %f\n", i, resolution);

          if(find_closest_point_along_path(centroid, robot_paths[i], resolution, closest_point, best_dist_to_point) )
          {
            first_conflicts[i] = closest_point;
            time_of_first_conflicts[i] = closest_point[2]; // note, not dist along path, but main thing is its less than LARGE

         
          }
          else
          {
            // this should not happen
            printf("unable to find closest point \n");
          }
        }
      }
    }
    else // no conflicts between any robots single paths
    {
      // send a flag that lets system know that it can ignore planning a new path and can just use its single robot path
      // now that robots are in the same team, they will not try to recombine unless another robot conflicts with one or more of them

      no_conflicts_between_single_paths = true;
      return false;
    }
  }

  vector<float> this_robots_first_conflict;

  if(time_of_first_conflicts[this_agent_id] < LARGE)
  {
    this_robots_first_conflict = first_conflicts[this_agent_id];
    found_this_robots_conflicts = true;

    printf("first conflict: [%f %f]\n", this_robots_first_conflict[0], this_robots_first_conflict[1]);
    //getchar();
  }
  else
  {
    printf("didn't find conflicts of this agent\n");

    if(there_is_at_least_one_conflict)
      printf("...but other agents have conflicts \n");
    else
      printf("...and no other agents have conflicts \n");
  }

  // init current_point to the conflict points and find the inds of edges that contain them
  vector<vector<float> > current_point = first_conflicts;
  vector<float> ind_of_current_edges(max_num_robots, -1); // find first edges that contain the conflict points for all robots

  for(int r = 0; r < max_num_robots; r++)
  {
    if(!InTeam[r])
      continue;

    // init to final point in path (usually reset below)
    ind_of_current_edges[r] = robot_paths[r].size()-1;

    for(uint i = 0; i < robot_paths[r].size(); i++)
    {
      if(robot_paths[r][i][2] > first_conflicts[r][2]) // point at i is greater in time than the conflict point, so edge [i-1, i] contains it 
      {
        if(i == 0) // i is first point in path r
          ind_of_current_edges[r] = 0;
        else
          ind_of_current_edges[r] = i - 1;
        
        if(ind_of_current_edges[r] < 0)
          ind_of_current_edges[r] = 0;

        break;
      }
    }
  }

  // expand the planning area up to the minimum size, we'll start with the first conflicts, and expand the planning area by moving it out along all robot's paths simultaniously
  bool need_to_expand_up_to_maximum = true;

  // start planning area based on first conflicts
  min_x = min_x_first_only;
  max_x = max_x_first_only;
  min_y = min_y_first_only;
  max_y = max_y_first_only;

  float total_distance = 0;

  vector<bool> at_goal(max_num_robots,false);

  // now expand along each path at an equal rate based on resolution until we achieve the preferred planning area
  while( (max_x - min_x <= preferred_max_planning_area_side_length) && 
         (max_y - min_y <= preferred_max_planning_area_side_length) )
  {
    if(need_to_expand_up_to_maximum)
    {
      if((max_x - min_x > preferred_max_planning_area_side_length) || 
         (max_y - min_y > preferred_max_planning_area_side_length) )
      {
        break;
      }
    }

    // check if some robots still have path left to traverse along
    bool some_not_at_goal = false;
    for(int r = 0; r < max_num_robots; r++)
    {
      if(!InTeam[r])
        continue;

      if(!at_goal[r])
      {
        some_not_at_goal = true;
        break;
      }
    }

    if(!some_not_at_goal)
    {
      printf("everybody at goal \n");
      break;
    }

    printf("total distance: %f \n", total_distance);
    
    // move each point resolution distance along path 
    for(int r = 0; r < max_num_robots; r++)
    {
      if(!InTeam[r])
        continue;

      if(at_goal[r])
        continue;

      if(robot_paths[r].size() <= 1)
        continue;

      int i = ind_of_current_edges[r];

      if(i+1 >= ((int)robot_paths[r].size()))
      {
        at_goal[r] = true;
        break;
      }

      float dist_to_end_of_edge = euclid_dist(current_point[r], robot_paths[r][i+1]);
      float dist_to_move = resolution;

      while(dist_to_end_of_edge < resolution) // need to move to a new edge
      {
        if(i+1 >= (int)robot_paths[r].size())
        {
          at_goal[r] = true;
          break;
        }
        // account for the end of the current edge
        if(robot_paths[r][i+1][0] > max_x)
          max_x = robot_paths[r][i+1][0];

        if(robot_paths[r][i+1][0] < min_x)
          min_x = robot_paths[r][i+1][0];
 
        if(robot_paths[r][i+1][1] > max_y)
          max_y = robot_paths[r][i+1][1];

        if(robot_paths[r][i+1][1] < min_y)
          min_y = robot_paths[r][i+1][1];

        i++;
        // check if we've reached the end of the final edge
        if(i >= ((int)robot_paths[r].size()) - 1) // reached this robot's goal
        {
          at_goal[r] = true;
          break;
        }
        else // not final edge, ok to move forward
        {
          dist_to_move -= dist_to_end_of_edge;
          current_point[r] = robot_paths[r][i];
          dist_to_end_of_edge = euclid_dist(current_point[r], robot_paths[r][i+1]);
        }
        total_distance += dist_to_move;
      }
      // remember the index we are at
      ind_of_current_edges[r] = i;

      if(at_goal[r])
        continue;

    printf("here h \n");


      current_point[r][0] += (dist_to_move/dist_to_end_of_edge)*(robot_paths[r][i+1][0] - current_point[r][0]);
      current_point[r][1] += (dist_to_move/dist_to_end_of_edge)*(robot_paths[r][i+1][1] - current_point[r][1]);

      if(current_point[r][0] > max_x)
        max_x = current_point[r][0];

      if(current_point[r][0] < min_x)
        min_x = current_point[r][0];
 
      if(current_point[r][1] > max_y)
        max_y = current_point[r][1];

      if(current_point[r][1] < min_y)
        min_y = current_point[r][1];
    printf("here i \n");
    }
    printf("here j \n");
  }
    printf("here k \n");
  //printf("current_point 0: [%f, %f]\n", current_point[0][0], current_point[0][1]);
  //printf("current_point 1: [%f, %f]\n", current_point[1][0], current_point[1][1]);

  printf("region bounds: x:[%f, %f] y:[%f, %f]\n", min_x, max_x, min_y, max_y);
  //printf("(soft boundry)\n");
 
  // now we have a (soft) boundry, but need to calculate sub_start and sub_goal based on boundry and/or start/goals 
  // (NOTE using goal and not last conflict points) in order to insure that there will not be 
  // goal conflicts. (even though we trash this data and wait for other robots to send them later)

  // find sub goal:

  // record all robot goal locations
  //printf("(Recording all global goals) \n");
  vector<vector<float> > robot_global_goals(max_num_robots);
printf("here l \n");
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;
printf("here m \n");

    int goal_ind = ((int)robot_paths[i].size())-1;
    robot_global_goals[i] = robot_paths[i][goal_ind];
printf("here n \n");
  }
  printf("(Done recording all goals)\n");

  // remember which goals we need to set
  vector<vector<float> > sub_goals(max_num_robots);
  vector<bool> sub_goal_found(max_num_robots, false);

  printf("(First pass)\n");

  // first pass, robot's with goals in planning area get priority as long as they are able to reach those goals without leaving the planning area
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    if(at_goal[i]) // can reach goal without leaving planning area
    {
      //printf("(robot %d can reach goal without leaving the planning area)\n", i);

      sub_goals[i] = robot_global_goals[i];
      sub_goal_found[i] = true;
    }
  }

  printf("(Done with first pass)\n");

  printf("(Second pass)\n");

  // second pass, find goals for robots that do not yet have their goals set, this is the last point in the planning area that can be reached from the first conflict without leaving the planning area
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    if(sub_goal_found[i])
      continue;

    vector<float> exit_point;
    if(calculate_exit_point(robot_paths[i], first_conflicts[i], min_x, max_x, min_y, max_y, resolution, robot_rad, exit_point))
    {
      // if here then use exit_point as goal
      sub_goals[i] = exit_point;
      sub_goal_found[i] = true;
    }
    else
    { 
      // no exit point
      //printf("(did not find exit point for robot %d, so using global goal)",i);
      sub_goals[i] = robot_global_goals[i];
      sub_goal_found[i] = true;
    }
  }  
  printf("(Done with second pass)\n");

  if(!sub_goal_found[this_agent_id])
  {
    printf("(Return false, no subgoal found for this agent)\n");
    return false;
  }

  printf("This agents initial sub_goal: [%f %f] \n",  sub_goals[this_agent_id][0],  sub_goals[this_agent_id][1]);

  // modify goals that conflict until they no longer conflict by moving the robot with lower id along its path toward its global goal
  // if robot with lower id is at its global goal, then move the robot with the higher id toward its global goal
  //printf("(Removing sub_goal conflicts)\n");
  int conflicting_ind_a = -1;
  int conflicting_ind_b = -1;
  while(any_two_points_conflict(sub_goals, InTeam, robot_rad, conflicting_ind_a, conflicting_ind_b))
  {
    //printf("(two points are conflicting, so modifying goals)\n");
    // if here then conflicting_ind_a and conflicting_ind_b conflict

    if(resolution <= euclid_dist(sub_goals[conflicting_ind_a], robot_global_goals[conflicting_ind_a]))
    {
      // conflicting_ind_a is not yet at its global goal, so move it toward its global goal
      move_point_along_path(sub_goals[conflicting_ind_a], robot_paths[conflicting_ind_a], robot_rad);
    }
    else if(resolution <= euclid_dist(sub_goals[conflicting_ind_b], robot_global_goals[conflicting_ind_b]))
    {
       // conflicting_ind_b is not yet at its global goal, so move it toward its global goal
       move_point_along_path(sub_goals[conflicting_ind_b], robot_paths[conflicting_ind_b], robot_rad);
    }
    else
    {
      // this should not happen
      printf("(two global goals are conflicting)");
      return false;
    }
  }
  printf("(Done removing sub_goal conflicts)\n");

  printf("This agents final sub_goal: [%f %f] \n",  sub_goals[this_agent_id][0],  sub_goals[this_agent_id][1]);

  // find sub start:

  // record all robot goal locations
  //printf("(Recording global starts)\n");
  vector<vector<float> > robot_global_starts(max_num_robots);
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    robot_global_starts[i] = robot_paths[i][0];
  }
  printf("(Done recording global starts)\n");

  // for coding ease I will use stuff I've already written, and just reverse all robot paths and then swap start and goals
  printf("(Reversing all paths)\n");
  vector<vector<vector<float> > > reverse_robot_paths = robot_paths;
  for(int r = 0; r < max_num_robots; r++)
  {
    int num_points = reverse_robot_paths[r].size();
    for(int i = 0; i < num_points; i++)
    {
      int j = (num_points - 1) - i;
      reverse_robot_paths[r][i] = robot_paths[r][j];
    }
  }
  printf("(Done reversing all paths)\n");

  // remember which starts we need to set
  vector<vector<float> > sub_starts(max_num_robots);
  vector<bool> sub_start_found(max_num_robots, false);

  printf("(Third pass)\n");

  // third pass, robot's with starts in planning area get priority as long as they were able to reach the first intersect point without leaving the planning area
  vector<bool> at_start(max_num_robots,false);
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    // see if robot i can reach start from first conflict point without leaving the planning area
    vector<float> entrance_point;
    if(!calculate_exit_point(reverse_robot_paths[i], first_conflicts[i], min_x, max_x, min_y, max_y, resolution, robot_rad, entrance_point))
    {
      // if here then did not exit the boundary before finding start, so use start as start
      sub_starts[i] = robot_global_starts[i];
      sub_start_found[i] = true;
    }
  }

  printf("(Done with third pass)\n");
  printf("(Fourth pass)\n");

  // fourth pass, find starts for robots that do not yet have their starts set
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    if(sub_start_found[i])
      continue;

    vector<float> entrance_point;
    if(calculate_exit_point(reverse_robot_paths[i], first_conflicts[i], min_x, max_x, min_y, max_y, resolution, robot_rad, entrance_point))
    {
      // if here then use entrance_point as start
      sub_starts[i] = entrance_point;
      sub_start_found[i] = true;
    }
    else
    {
      sub_starts[i] = robot_global_starts[i];
      sub_start_found[i] = true;
    }
  }

  printf("(Done with fourth pass)\n");

  if(!sub_start_found[this_agent_id])
  {
    printf("(unable to find sub_start of this agent, returning false)\n");
    return false;
  }

  printf("This agents initial sub_start: [%f %f] \n",  sub_starts[this_agent_id][0],  sub_starts[this_agent_id][1]);


  // modify starts that conflict until they no longer conflict by moving the robot with lower id along its path toward its global start
  // if robot with lower id is at its global start, then move the robot with the higher id toward its global start
  printf("(Removing sub_start conflicts)\n");
  conflicting_ind_a = -1;
  conflicting_ind_b = -1;
  while(any_two_points_conflict(sub_starts, InTeam, robot_rad, conflicting_ind_a, conflicting_ind_b))
  {
    //printf("(two points are conflicting, so modifying starts)\n");

    // if here then conflicting_ind_a and conflicting_ind_b conflict
    if(resolution <= euclid_dist(sub_starts[conflicting_ind_a], robot_global_starts[conflicting_ind_a]))
    {
      // conflicting_ind_a is not yet at its global start, so move it toward its global start
      move_point_along_path(sub_starts[conflicting_ind_a], reverse_robot_paths[conflicting_ind_a], robot_rad);
    }
    else if(resolution <= euclid_dist(sub_starts[conflicting_ind_b], robot_global_starts[conflicting_ind_b]))
    {
       // conflicting_ind_b is not yet at its global start, so move it toward its global start
       move_point_along_path(sub_starts[conflicting_ind_b], reverse_robot_paths[conflicting_ind_b], robot_rad);
    }
    else
    {
      // this should not happen, but just explicitly set each to the global start
      printf("(Two global starts are conflictiong)\n");

      sub_starts[conflicting_ind_a] = robot_paths[conflicting_ind_a][0];
      sub_starts[conflicting_ind_b] = robot_paths[conflicting_ind_b][0];

      //return false;
    }
  }
  printf("(Done removing sub_start conflicts)\n");

  //printf("This agents final sub_start: [%f %f] \n",  sub_starts[this_agent_id][0],  sub_starts[this_agent_id][1]);

  sub_start = sub_starts[this_agent_id];
  sub_goal = sub_goals[this_agent_id];

  printf("This agents final sub_goal  : [%f %f] \n",  sub_goal[0],  sub_goal[1]);
  printf("This agents final sub_start : [%f %f] \n",  sub_start[0],  sub_start[1]);
  return true;
}
