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
  
  for(int j =  (int)MultiSolution.size()-2; j > 0; j--) //note: leave start pose alone by j>0
  {
    for(int i = 0; i < (int)MultiSolution[0].size(); i += 3)
    { 
      if(MultiSolution[j][i] == MultiSolution[j+1][i] && MultiSolution[j][i+1] == MultiSolution[j+1][i+1])  // no movement, so use angle of next time step
        MultiSolution[j][i+2] = MultiSolution[j+1][i+2];
      else  // there is movement, so calculate the angle based on the relative angle of this time-step location vs next time-step location
      {
         theta = atan2(MultiSolution[j+1][i+1] - MultiSolution[j][i+1], MultiSolution[j+1][i] - MultiSolution[j][i]); 
          
         while(theta < -PI)
          theta += 2*PI;
         while(theta > PI)
          theta -= 2*PI;
         
         MultiSolution[j][i+2] = theta;
      }
    }    
  }    
}

void calculate_times(vector<float>& Times, vector<vector<float> >& MultiSolution, float mps_target, float rps_target) // calculates the time parametery of the MultiSolution, where mps_target is the target meters per second of the fastest moving robot and rps is the target radians per second of fastest moving robot, note that these should be slightly slower than the max possible values to allow a behind robot to catch up, assumes 2D workspace and orientation for each robot
{
  Times.resize(MultiSolution.size());
  
  if(Times.size() < 1)
    return;
          
  Times[0] = 0; // start time
    
  float this_max, this_val, x_dif, y_dif, t_dif; // t_diff is theta diff (not time diff)
  
  //printf("times: 0");
  
  for(int i = 1; i < (int)MultiSolution.size(); i++)
  {
    // find max time required by any robot
    this_max = 0;
    for(int j = 0; j < (int)MultiSolution.size(); j +=3)
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
  //printf(".\n");
}

void verrify_start_angle(vector<vector<float> >& MultiSolution, vector<float>& start_config)  // makes sure start angles are correct
{
  for(int i = 0; i < (int)MultiSolution[0].size(); i += 3)
  {  
    printf("robot %d solution start: %f, %f, %f \n", i/3, MultiSolution[0][i], MultiSolution[0][i+1], MultiSolution[0][i+2]); 
    printf("robot %d actual start:   %f, %f, %f \n\n", i/3, start_config[i], start_config[i+1], start_config[i+2]);
  }
}
