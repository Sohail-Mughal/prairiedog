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
  float step = edge_length*resolution;
  float step_x = (step/edge_length)*(robot_forward_conflict[1][0] - robot_forward_conflict[0][0]);
  float step_y = (step/edge_length)*(robot_forward_conflict[1][1] - robot_forward_conflict[0][1]);

  vector<float> point = robot_forward_conflict[0];

  for(float d = 0; d <= edge_length; d += step)
  {
    if(found_obstacle_forward_conflict)
    {
      bool found_robot_forward_point_conflict = false;
      for(int i = 0; i < obstacle_forward_conflicts.size(); i++)
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
      for(int i = 0; i < obstacle_backward_conflicts.size(); i++)
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
  for(int i = 0; i < robot_path.size()-1; i++)
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
      for(int j = 0; j <= i; j++)
        dist_to_first_r_conflict += euclid_dist(robot_path[j], robot_path[j+1]);

      found_robot_forward_conflict = true;
    }
  }

  // robot_path backward direction:

  // for each edge in the robot path, check if it conflicts with the obstacle path
  for(int i = robot_path.size()-1; i > 0; i--)
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
      for(int j = i; j < robot_path.size()-1; j++)
        dist_to_last_r_conflict += euclid_dist(robot_path[j], robot_path[j+1]);

      found_robot_backward_conflict = true;
    }
  }

  // obstacle_path forward direction:

  // for each edge in the obstacle path, check if it conflicts with the robot path
  for(int i = 0; i < obstacle_path.size()-1; i++)
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
      for(int j = 0; j <= i; j++)
        dist_to_first_o_conflict += euclid_dist(obstacle_path[j], obstacle_path[j+1]);

      found_obstacle_forward_conflict = true;
    }
  }


  // obstacle_path backward direction:

  // for each edge in the obstacle path, check if it conflicts with the robot path
  for(int i = obstacle_path.size()-1; i > 0; i--)
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
      for(int j = i; j < obstacle_path.size()-1; j++)
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


// find edge that mot likely contains point and return the index of that edge or -1 if nothing is found, and also return the point in the path that was the best match
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
// also handles conflicts with existing sub-goals in case that point is already used (note, may put slightly out of region in that case)
bool calculate_exit_point(const vector<vector<float> > & robot_path, const vector<float> & start_point, float area_min_x, float area_max_x, float  area_min_y, float area_max_y, const vector<vector<float> > sub_goals, const vector<bool> & sub_goal_found, float robot_rad, float resolution, vector<float> & exit_point)
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
      continue;

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
    printf("didn't find last point in boundary\n");
    return false;
  }
  // if here than found a last point in boundary, but still need to make sure it is compatible with the other robot goals that have already been selected

  bool compatible = true;
  int max_num_robots = sub_goal_found.size();
  while(true) // break out when done
  {
    for(int r = 0; r < max_num_robots; r++)
    {
      if(!sub_goal_found[r])
        continue;

      if(robot_rad > euclid_dist(last_point_in_boundary, sub_goals[r]) )
      {
        // there is a conflict
        compatible = false;
        break;
      }
    }

    if(compatible)
      break;

    // if here then not compatible, so move further along robot_path
    float dist_remaining = euclid_dist(last_point_in_boundary, robot_path[edge_that_intersects_boundary_i + 1]);
    if(robot_rad > dist_remaining) // need to move to next edge
    {
      last_point_in_boundary = robot_path[edge_that_intersects_boundary_i + 1];
      edge_that_intersects_boundary_i++; 
    }
    else // stay on same edge
    {
      last_point_in_boundary[0] += (resolution/dist_remaining)*(robot_path[edge_that_intersects_boundary_i+1][0] - last_point_in_boundary[0]);
      last_point_in_boundary[1] += (resolution/dist_remaining)*(robot_path[edge_that_intersects_boundary_i+1][1] - last_point_in_boundary[1]);
    }

    // if close to normal robot goal, just use that
    if(robot_rad > euclid_dist(last_point_in_boundary, robot_path[robot_path.size()-1]) )
    {
      return false;
    }
  }

  exit_point = last_point_in_boundary;
  return true;
}

// calculates the sub goal that this robot should use for multi-robot path planning based on single robot paths, returns true if finds a new sub_start and sub_goal
bool calculate_sub_goal(const vector<vector<vector<float> > > & robot_paths, const vector<bool> & InTeam, int this_agent_id, float preferred_planning_area_side_length, float robot_rad, float resolution, vector<float> & sub_start, vector<float> & sub_goal)
{
  // Note: There are probably better ways to do this than duplicating it on each agent, but this works and runtime is only robots squared

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

  if( (max_x - min_x <= preferred_planning_area_side_length) && 
      (max_y - min_y <= preferred_planning_area_side_length) )  // then we can just use normal start and goal
  {
    printf("region bounds: x:[%f, %f] y:[%f, %f]\n", min_x, max_x, min_y, max_y);

    printf("all paths fit, just use normal start and goal\n");

    return false;
  }

  // go through and find all robot vs robot first and last conflicts and see if paths between them fit within the preferred planning area
  // also save this robot's first and last conflicts seperately, as well as bounds on the area containing all first conflicts
  vector<vector<float> > first_conflicts(max_num_robots);
  vector<vector<float> > last_conflicts(max_num_robots);

  vector<float> dist_to_first_conflicts(max_num_robots, LARGE);
  vector<float> dist_to_last_conflicts(max_num_robots, LARGE);

  vector<float> this_robots_first_conflict;
  vector<float> this_robots_last_conflict;
  bool found_this_robots_conflicts;

  float max_x_first_only = -LARGE;
  float max_y_first_only = -LARGE;

  float min_x_first_only = LARGE;
  float min_y_first_only = LARGE;

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

      vector<float> last_i_conflict;
      vector<float> last_j_conflict;

      float dist_to_first_i_conflict; 
      float dist_to_last_i_conflict;

      float dist_to_first_j_conflict;
      float dist_to_last_j_conflict;

      if(find_conflict_points(robot_paths[i], robot_paths[j], robot_rad, resolution, first_i_conflict,  dist_to_first_i_conflict, 
                                                                                     last_i_conflict, dist_to_last_i_conflict,
                                                                                     first_j_conflict, dist_to_first_j_conflict,
                                                                                     last_j_conflict, dist_to_last_j_conflict) )
      {
        if(first_i_conflict[0] > max_x_first_only)
          max_x_first_only = first_i_conflict[0];

        if(first_i_conflict[1] > max_y_first_only)
          max_y_first_only = first_i_conflict[1];

        if(first_i_conflict[0] < min_x_first_only)
          min_x_first_only = first_i_conflict[0];

        if(first_i_conflict[1] < min_y_first_only)
          min_y_first_only = first_i_conflict[1];


        if(first_j_conflict[0] > max_x_first_only)
          max_x_first_only = first_j_conflict[0];

        if(first_j_conflict[1] > max_y_first_only)
          max_y_first_only = first_j_conflict[1];

        if(first_j_conflict[0] < min_x_first_only)
          min_x_first_only = first_j_conflict[0];

        if(first_j_conflict[1] < min_y_first_only)
          min_y_first_only = first_j_conflict[1];


        // need to only do the following if this is still the first/last compared to what we alerady know
        if(dist_to_first_i_conflict < dist_to_first_conflicts[i])
        {
          first_conflicts[i] = first_i_conflict;
          dist_to_first_conflicts[i] = dist_to_first_i_conflict;
        }

        if(dist_to_first_j_conflict < dist_to_first_conflicts[j])
        {
          first_conflicts[j] = first_j_conflict;
          dist_to_first_conflicts[j] = dist_to_first_j_conflict;
        }

        if(dist_to_last_i_conflict < dist_to_last_conflicts[i])
        {
          last_conflicts[i] = last_i_conflict;
          dist_to_last_conflicts[i] = dist_to_last_i_conflict;
        }

        if(dist_to_last_j_conflict < dist_to_last_conflicts[j])
        {
          last_conflicts[j] = last_j_conflict;
          dist_to_last_conflicts[j] = dist_to_last_j_conflict;
        }




        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if(i == this_agent_id)
        {
          this_robots_first_conflict = first_i_conflict;
          this_robots_last_conflict = last_i_conflict;
          found_this_robots_conflicts = true;
        }
        else if(j == this_agent_id)
        {
          this_robots_first_conflict = first_j_conflict;
          this_robots_last_conflict = last_j_conflict;
          found_this_robots_conflicts = true;
        }
      }



    }
  }


  printf("first conflict: [%f %f]\n", this_robots_first_conflict[0], this_robots_first_conflict[1]);
  printf("last conflict: [%f %f]\n", this_robots_last_conflict[0], this_robots_last_conflict[1]);

  // see if path sub-sections between each robot's first and last conflict fit within the preferred planning area, 
  // save bounds on the planning area as we go
  
  max_x = -LARGE;
  max_y = -LARGE;

  min_x = LARGE;
  min_y = LARGE;

  vector<float> ind_of_current_edges(max_num_robots, -1); // also remember first edges for all robots
  vector<vector<float> > current_point(max_num_robots);   // and init this with the best matches

  for(int r = 0; r < max_num_robots; r++)
  {
    if(!InTeam[r])
      continue;

    vector<float> first_path_point;
    vector<float> last_path_point;

    // find the edge containing the first conflict
    int edge_first = find_edge_containing_point(robot_paths[r], first_conflicts[r], first_path_point);

    if(edge_first == -1)
    {
      // this should not happen
      printf("hmmm this should not happen 1\n");
      return false;
    }
     
    ind_of_current_edges[r] = edge_first;
    current_point[r] = first_path_point;

    // find edge containing last conflict
    int edge_last = find_edge_containing_point(robot_paths[r], last_conflicts[r], last_path_point);

    if(edge_last == -1)
    {
      // this should not happen
      printf("hmmm this should not happen 2\n");
      return false;
    }

    // record bounds for first_path_point
    if(first_path_point[0] > max_x)
      max_x = first_path_point[0];

    if(first_path_point[0] < min_x)
      min_x = first_path_point[0];
 
    if(first_path_point[1] > max_y)
      max_y = first_path_point[1];

    if(first_path_point[1] < min_y)
      min_y = first_path_point[1];

    // walk from end of edge first to start of edge last and record bounds
    for(int i = edge_first+1; i <= edge_last; i++)
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

    // record bounds for last_path_point
    if(last_path_point[0] > max_x)
      max_x = last_path_point[0];

    if(last_path_point[0] < min_x)
      min_x = last_path_point[0];
 
    if(last_path_point[1] > max_y)
      max_y = last_path_point[1];

    if(last_path_point[1] < min_y)
      min_y = last_path_point[1];
  }

  if( (max_x - min_x <= preferred_planning_area_side_length) && 
      (max_y - min_y <= preferred_planning_area_side_length) ) // then we can just use first and last intersect as start and goal
  {
    printf("region bounds: x:[%f, %f] y:[%f, %f]\n", min_x, max_x, min_y, max_y);

    printf("use first and last conflicts as start and goal\n");

    if(found_this_robots_conflicts)
    {
      sub_start = this_robots_first_conflict;
      sub_goal = this_robots_last_conflict;
      return true;
    }
    else
    {
       // this should never happen
       printf("no conflicts found?\n");
       return false;
    }
  }

  // if we are here than planning area based on first and last conflicts is too big, so we'll start with the first conflicts, and expand
  // the planning area by moving it out along all robot's paths simultaniously

  sub_start = this_robots_first_conflict;

  // start planning area based on first conflicts
  min_x = min_x_first_only;
  max_x = max_x_first_only;
  min_y = min_y_first_only;
  max_y = max_y_first_only;

  float total_distance = 0;

  vector<bool> at_goal(max_num_robots,false);

  // now expand along each path at an equal rate based on resolution until we achieve the preferred planning area
  while( (max_x - min_x <= preferred_planning_area_side_length) && 
         (max_y - min_y <= preferred_planning_area_side_length) )
  {
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

    //printf("total distance: %f \n", total_distance);
    
    // move each point resolution distance along path 
    for(int r = 0; r < max_num_robots; r++)
    {
      if(!InTeam[r])
        continue;

      if(at_goal[r])
        continue;

      int i = ind_of_current_edges[r];
      float dist_to_end_of_edge = euclid_dist(current_point[r], robot_paths[r][i+1]);
      float dist_to_move = resolution;

      while(dist_to_end_of_edge < resolution) // need to move to a new edge
      {

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
        if(i >= robot_paths[r].size() - 1) // reached this robot's goal
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
    }
  }

  //printf("current_point 0: [%f, %f]\n", current_point[0][0], current_point[0][1]);
  //printf("current_point 1: [%f, %f]\n", current_point[1][0], current_point[1][1]);

  printf("region bounds: x:[%f, %f] y:[%f, %f]\n", min_x, max_x, min_y, max_y);

  // now we have a (soft) boundry, but need to calculate goal based on boundry and/or goals 
  // (NOTE using goal and not last conflict points) in order to insure that there will not be 
  // goal conflicts. (even though we trash this data and wait for other robots to send them later)

  // record all robot goal locations
  vector<vector<float> > robot_goals;

  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    int goal_ind = robot_paths[i].size()-1;
    robot_goals.push_back(robot_paths[i][goal_ind]);
  }

  // remember which goals we need to set
  vector<vector<float> > sub_goals(max_num_robots);
  vector<bool> sub_goal_found(max_num_robots, false);


  // first pass, robot's with goals in planning area get priority as long as they were able to reach those goals without leaving the planning area
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    if(at_goal[i]) // can reach goal without leaving planning area
    {
      if(i == this_agent_id)
      {
        // if i is this agent, then we are done
        sub_goal = robot_paths[i][robot_paths[i].size()-1];
        return true;
      }
      sub_goals[i] = robot_paths[i][robot_paths[i].size()-1];
      sub_goal_found[i] = true;
    }
  }

  // second pass, find goals for robots that do not yet have their goals set
  for(int i = 0; i < max_num_robots; i++)
  {
    if(!InTeam[i])
      continue;

    if(sub_goal_found[i])
      continue;

    vector<float> exit_point;
    if(calculate_exit_point(robot_paths[i], first_conflicts[i], min_x, max_x, min_y, max_y, sub_goals, sub_goal_found, robot_rad, resolution, exit_point))
    {
      // if here then use exit_point as goal

      if(i == this_agent_id)
      {
        // if i is this agent, then we are done
        sub_goal = exit_point;
        return true;
      }
      sub_goals[i] = exit_point;
      sub_goal_found[i] = true;
    }
    else
    {
      // this should never happen
      printf("unable to calculate sub goal?\n");

      if(i == this_agent_id)
        break;
    }
  }

  return false;
}



