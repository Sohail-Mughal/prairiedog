MultiRobotWorkspace::MultiRobotWorkspace() // default constructor    
{
  num_robots = 0;
  dims = 3;
  dim_max.resize(dims);
  dim_max[0] = 1;    // x
  dim_max[1] = 1;    // y
  dim_max[2] = 2*PI; // theta
}

MultiRobotWorkspace::MultiRobotWorkspace(int n_robots, float robot_r)         // constructor
{
   Populate(n_robots, robot_r);
}

MultiRobotWorkspace::MultiRobotWorkspace(const MultiRobotWorkspace& W)   // copy constructor
{
  num_robots = W.num_robots;
  robot_radius = W.robot_radius;
  dim_max = W.dim_max;
}

MultiRobotWorkspace::~MultiRobotWorkspace() // destructor
{  

}

void MultiRobotWorkspace::Populate(int n_robots, float robot_r)         // populates or re-populates the structure
{
  num_robots = n_robots;
  robot_radius.resize(n_robots); 
  for(int i = 0; i < n_robots; i++)
    robot_radius[i] = robot_r;    
   
  dims = 3;
  dim_max.resize(dims);
  dim_max[0] = 1;
  dim_max[1] = 1;
  dim_max[2] = 2*PI;
}

void MultiRobotWorkspace::Populate(int n_robots, float robot_r, const vector<float>& dim_max_in)         // populates or re-populates the structure
{
  num_robots = n_robots;
  robot_radius.resize(n_robots); 
  for(int i = 0; i < n_robots; i++)
    robot_radius[i] = robot_r;    
   
  dims = 3;
  dim_max.resize(dims);
  dim_max[0] = dim_max_in[0];
  dim_max[1] = dim_max_in[1];
  dim_max[2] = dim_max_in[2];
}

void MultiRobotWorkspace::Print() // displays info about the class on the command line
{
  printf("There are %d robots:\n", num_robots);
  for(int i = 0; i < num_robots; i++)
    printf("robot %d, radius:%f \n",i, robot_radius[i]); 
}

void MultiRobotWorkspace::Draw(const vector<float>& config, float* clr)             // draws the workspace in the given configuration
{
  glPushMatrix(); 
  glTranslatef(-1, -1, 0);
  
  float map_rad; 
  if(dim_max[0] > dim_max[1])
    map_rad = 2/dim_max[0];
  else
    map_rad = 2/dim_max[1];
      
  glScaled(map_rad,map_rad,1);
   
  int dims_all = config.size();
    
  float* clr_temp;
  for(int i = 0; i < dims_all; i+=dims)
  {        
    if(clr != NULL)
      clr_temp = clr;
    else 
      clr_temp = RAINBOW[i/dims];
      
    float pos[] = {config[i], config[i+1], 0};
    draw_circle(pos, robot_radius[i/dims] , clr_temp);
    float pos2[] = {config[i] + robot_radius[i/dims]*cos(config[i+2]), config[i+1] + robot_radius[i/dims]*sin(config[i+2]), 0};
    draw_arrow(pos, pos2, clr_temp);
      
  }
  //draw_points(x_temp_vec,y_temp_vec); //used for debugging
  
  glPopMatrix();   
}

void MultiRobotWorkspace::DrawGoal(const vector<float>& config, float* clr)             // draws the workspace in the given configuration as a goal
{
  glPushMatrix(); 
  glTranslatef(-1, -1, 0);
  
  float map_rad; 
  if(dim_max[0] > dim_max[1])
    map_rad = 2/dim_max[0];
  else
    map_rad = 2/dim_max[1];
      
  glScaled(map_rad,map_rad,1);
   
  int dims_all = config.size();
    
  float* clr_temp;
  for(int i = 0; i < dims_all; i+=dims)
  {        
    if(clr != NULL)
      clr_temp = clr;
    else 
      clr_temp = RAINBOW[i/dims];
      
    float pos[] = {config[i], config[i+1], 0};
    draw_x(pos, robot_radius[i/dims]/2 , clr_temp); 
    float pos2[] = {config[i] + robot_radius[i/dims]/2*cos(config[i+2]), config[i+1] + robot_radius[i/dims]/2*sin(config[i+2]), 0};
    draw_arrow(pos, pos2, clr_temp);
  }
    
  glPopMatrix();   
}

void MultiRobotWorkspace::DrawEdge(const vector<float>& config1, const vector<float>& config2)             // draws a line(s) in the workspace between the two configurations 
{
    glPushMatrix(); 
    glTranslatef(-1, -1, 0);
  
    float map_rad; 
    if(dim_max[0] > dim_max[1])
      map_rad = 2/dim_max[0];
    else
      map_rad = 2/dim_max[1];
      
    glScaled(map_rad,map_rad,1);
  
    glBegin(GL_LINES);
    
    int dims_all = config1.size();
    int robot_ind;
    for(int i = 0; i < dims_all; i+=dims)
    {       
      robot_ind = i/dims;
      glColor3f(RAINBOW[robot_ind][0], RAINBOW[robot_ind][1], RAINBOW[robot_ind][2]);

      glVertex2f(config1[i], config1[i+1]);
      glVertex2f(config2[i], config2[i+1]);
    }
    
    glEnd();
    
    glPopMatrix();   
}

void MultiRobotWorkspace::RandMove(const vector<float>& old_config, vector<float>& new_config, float safe_dist, float max_dist, float max_theta, float prob_at_goal, const vector<float>& the_goal) // calculates a new configuration point that is whithin max_dist of the old point, but with prob_at_goal moves directly at the goal (often the goal will be the start), safe_dist is the believed safe distance to move
{
  int x_ind, y_ind, theta_ind;
  float dist_each = max_dist/num_robots;
  
  if(((float)rand_int(0, 100000))/100000 < prob_at_goal) // want to move directly at the goal configuration (which is where robots start)
  {
    // have each robot move the max possible    
    
    float goal_x, goal_y, goal_theta, bot_x, bot_y, bot_theta;
    float dist_to_goal;
    float direction_to_goal, d_theta;
    bool not_done_yet = true;
    float frac_to_goal = 1;
    while(not_done_yet)
    {
      not_done_yet = false;
      for(int r = 0; r < num_robots; r++)
      {    
        x_ind = r*dims;
        y_ind = x_ind+1;
        theta_ind = y_ind+1;
              
        goal_x = the_goal[x_ind];
        goal_y = the_goal[y_ind];
        goal_theta = the_goal[theta_ind] + PI;  // plus PI because we are searching in the reverse direction that the robots will drive in
                
        bot_x = old_config[x_ind];
        bot_y = old_config[y_ind];
        bot_theta = old_config[theta_ind] + PI;  // plus PI because we are searching in the reverse direction that the robots will drive in
        
        while(bot_theta < -PI)
          bot_theta += 2*PI;
        while(bot_theta > PI)
          bot_theta -= 2*PI;
        
        direction_to_goal = atan2(goal_y-bot_y,goal_x-bot_x);
      
        while(direction_to_goal < -PI)
          direction_to_goal += 2*PI;
        while(direction_to_goal > PI)
          direction_to_goal -= 2*PI;
      
        d_theta = direction_to_goal-bot_theta;
        while(d_theta < -PI)
          d_theta += 2*PI;
        while(d_theta > PI)
          d_theta -= 2*PI;
        
        float d_theta_abs;
        if( d_theta < 0)
          d_theta_abs = -d_theta;
        else
          d_theta_abs = d_theta;
                
        dist_to_goal = sqrt((bot_x-goal_x)*(bot_x-goal_x) + (bot_y-goal_y)*(bot_y-goal_y))/frac_to_goal;
        
        if(dist_to_goal <= dist_each && d_theta_abs < max_theta) // then this robot moves less than its alloted distance and angle to reach the goal
        {          
          new_config[x_ind] = the_goal[x_ind];
          new_config[y_ind] = the_goal[y_ind];
          
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;
          
          new_config[theta_ind] = direction_to_goal;
        }
        else if(dist_to_goal == 0) // the robot is at the goal, so rotate to the desired orientation 
        {
          if(direction_to_goal > 0)
            direction_to_goal = bot_theta + max_theta; 
          else
            direction_to_goal = bot_theta - max_theta; 
          
          new_config[x_ind] = old_config[x_ind];
          new_config[y_ind] = old_config[y_ind];
          
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI; 
          
          new_config[theta_ind] = direction_to_goal;
        }
        else if(d_theta_abs < max_theta) // the goal is too far, but the angle is ok
        {
          // move alloted distance to the goal              
          new_config[x_ind] = bot_x + dist_each/dist_to_goal * (goal_x-bot_x);
          new_config[y_ind] = bot_y + dist_each/dist_to_goal * (goal_y-bot_y); 
          
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;  
                    
          new_config[theta_ind] = direction_to_goal;
        }
        else if(dist_to_goal <= dist_each) // the angle is too much but the distance is ok
        {
          // move 0 distance at the max angle allowable
          
          if(direction_to_goal > 0)
            direction_to_goal = bot_theta + max_theta; 
          else
            direction_to_goal = bot_theta - max_theta;  
              
          new_config[x_ind] = old_config[x_ind]; // + dist_each/2*cos(direction_to_goal);
          new_config[y_ind] = old_config[y_ind]; // + dist_each/2*sin(direction_to_goal);
          
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;
          
          new_config[theta_ind] = direction_to_goal;    
        }
        else // both angle and distance are too much
        {
          // move this distance at the max angle allowable
          
          direction_to_goal = max_theta+bot_theta; 
          
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;
          
          new_config[x_ind] = old_config[x_ind] + dist_each*cos(direction_to_goal);
          new_config[y_ind] = old_config[y_ind] + dist_each*sin(direction_to_goal);
          
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;
          
          new_config[theta_ind] = direction_to_goal;
        }
      }
    }
  }
  else // want to move in a random direction
  { 
    float heading;
    
    // pick a random distance to move along the direction that is at most the alloted distance (all robots move the same distance)
    dist_each = ((float)rand_int(0, 100000))/100000*dist_each;
    
    if(dist_each > max_dist)
     dist_each = max_dist; 
    
    for(int r = 0; r < num_robots; r++)
    {
      x_ind = r*dims;
      y_ind = x_ind+1;
      theta_ind = y_ind + 1;
      
      // pick a random direction between 0 and 2PI    
      heading = old_config[theta_ind] + PI + ((float)rand_int(0, 100000))/100000*2*max_theta - max_theta; // plus PI because we are searching in the reverse direction that the robots will drive in
      
      while(heading < PI)
        heading += 2*PI;
      while(heading > PI)
        heading -= 2*PI;
      
      new_config[x_ind] = old_config[x_ind] + dist_each*cos(heading);
      new_config[y_ind] = old_config[y_ind] + dist_each*sin(heading);
      
      heading += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
            
      while(heading < PI)
        heading += 2*PI;
      while(heading > PI)
        heading -= 2*PI;
      
      new_config[theta_ind] = heading;
      
      if(new_config[x_ind] < 0)
        new_config[x_ind] = 0;
      else if(new_config[x_ind] > dim_max[0])
        new_config[x_ind] = dim_max[0];
           
      if(new_config[y_ind] < 0)
        new_config[y_ind] = 0;
      else if(new_config[y_ind] > dim_max[1])
        new_config[y_ind] = dim_max[1];
    }
  }    
}

void MultiRobotWorkspace::RandMove2(const vector<float>& old_config, vector<float>& new_config, float safe_dist, float max_dist, float max_theta, float prob_at_goal, const vector<float>& the_goal) // calculates a new configuration point that is whithin max_dist of the old point, but with prob_at_goal moves directly at the goal (often the goal will be the start), safe_dist is the believed safe distance to move, same as above but rand happens on granularity of a robot
{
  int x_ind, y_ind, theta_ind;
  float dist_each = max_dist;

  float goal_x, goal_y, goal_theta, bot_x, bot_y, bot_theta;
  float dist_to_goal;
  float direction_to_goal, d_theta;
  float frac_to_goal = 1;

  for(int r = 0; r < num_robots; r++)
  {      
    x_ind = r*dims;
    y_ind = x_ind+1;
    theta_ind = y_ind + 1;  
      
    //printf(" going into loop %f %f \n", safe_dist, max_dist);
    for(int tries = 0; tries < 100; tries ++) // will break out when a valid point is found for robot r
    {
      float heading;
    
      // pick a random distance to move along the direction that is at most the alloted distance
      dist_each = ((float)rand_int(0, 100000))/100000*max_dist;

      if(((float)rand_int(0, 100000))/100000 < prob_at_goal) // want to move directly at the goal configuration (which is where robots start)
      {
        //printf(" here 111 %f %f %f\n", safe_dist, max_dist, dist_each );  
        goal_x = the_goal[x_ind];
        goal_y = the_goal[y_ind];
        goal_theta = the_goal[theta_ind] + PI;  // plus PI because we are searching in the reverse direction that the robots will drive in
                
        bot_x = old_config[x_ind];
        bot_y = old_config[y_ind];
        bot_theta = old_config[theta_ind] + PI;  // plus PI because we are searching in the reverse direction that the robots will drive in
        
        while(bot_theta < -PI)
          bot_theta += 2*PI;
        while(bot_theta > PI)
          bot_theta -= 2*PI;
        
        direction_to_goal = atan2(goal_y-bot_y,goal_x-bot_x);
      
        while(direction_to_goal < -PI)
          direction_to_goal += 2*PI;
        while(direction_to_goal > PI)
          direction_to_goal -= 2*PI;
      
        d_theta = direction_to_goal-bot_theta;
        while(d_theta < -PI)
          d_theta += 2*PI;
        while(d_theta > PI)
          d_theta -= 2*PI;
        
        float d_theta_abs;
        if( d_theta < 0)
          d_theta_abs = -d_theta;
        else
          d_theta_abs = d_theta;
                
        dist_to_goal = sqrt((bot_x-goal_x)*(bot_x-goal_x) + (bot_y-goal_y)*(bot_y-goal_y))/frac_to_goal;
        
        if(dist_to_goal <= dist_each && d_theta_abs < max_theta) // then this robot moves less than its alloted distance and angle to reach the goal
        {   
          new_config[x_ind] = the_goal[x_ind];
          new_config[y_ind] = the_goal[y_ind];
          
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;
          
          new_config[theta_ind] = direction_to_goal;
        }
        else if(dist_to_goal == 0) // the robot is at the goal, so rotate to the desired orientation 
        {
          if(direction_to_goal > 0)
            direction_to_goal = bot_theta + max_theta; 
          else
            direction_to_goal = bot_theta - max_theta; 
          
          new_config[x_ind] = old_config[x_ind];
          new_config[y_ind] = old_config[y_ind];
          
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI; 
          
          new_config[theta_ind] = direction_to_goal;
        }
        else if(d_theta_abs < max_theta) // the goal is too far, but the angle is ok
        {
          // move alloted distance to the goal              
          new_config[x_ind] = bot_x + dist_each/dist_to_goal * (goal_x-bot_x);
          new_config[y_ind] = bot_y + dist_each/dist_to_goal * (goal_y-bot_y); 
          
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;  
                      
          new_config[theta_ind] = direction_to_goal;
        }
        else if(dist_to_goal <= dist_each) // the angle is too much but the distance is ok
        {
          // move 0 distance at the max angle allowable
          
          if(direction_to_goal > 0)
            direction_to_goal = bot_theta + max_theta; 
          else
            direction_to_goal = bot_theta - max_theta;  
            
          new_config[x_ind] = old_config[x_ind]; // + dist_each/2*cos(direction_to_goal);
          new_config[y_ind] = old_config[y_ind]; // + dist_each/2*sin(direction_to_goal);
           
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;
          
          new_config[theta_ind] = direction_to_goal;    
        }
        else // both angle and distance are too much
        {
          // move this distance at the max angle allowable
          
          direction_to_goal = max_theta+bot_theta; 
          
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;
          
          new_config[x_ind] = old_config[x_ind] + dist_each*cos(direction_to_goal);
          new_config[y_ind] = old_config[y_ind] + dist_each*sin(direction_to_goal);
            
          direction_to_goal += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
          while(direction_to_goal < -PI)
            direction_to_goal += 2*PI;
          while(direction_to_goal > PI)
            direction_to_goal -= 2*PI;
        
          new_config[theta_ind] = direction_to_goal;
        }
      }  
      else // want to move in a random direction
      {
        // pick a random direction between 0 and 2PI    
        heading = old_config[theta_ind] + PI + ((float)rand_int(0, 100000))/100000*2*max_theta - max_theta; // plus PI because we are searching in the reverse direction that the robots will drive in
      
        while(heading < PI)
          heading += 2*PI;
        while(heading > PI)
          heading -= 2*PI;
      
        new_config[x_ind] = old_config[x_ind] + dist_each*cos(heading);
        new_config[y_ind] = old_config[y_ind] + dist_each*sin(heading);
      
        heading += PI; // plus PI because we are searching in the reverse direction that the robots will drive in
            
        while(heading < PI)
          heading += 2*PI;
        while(heading > PI)
          heading -= 2*PI;
        
        new_config[theta_ind] = heading;
      
        if(new_config[x_ind] < 0)
          new_config[x_ind] = 0;
        else if(new_config[x_ind] > dim_max[0])
          new_config[x_ind] = dim_max[0];
           
        if(new_config[y_ind] < 0)
          new_config[y_ind] = 0;
        else if(new_config[y_ind] > dim_max[1])
          new_config[y_ind] = dim_max[1];
      }
      
      if(safe_dist < Scene.resolution)
        break;

      if(ProjectedPointValid(new_config, r) <= 0) // this point has a collision
      {
        //printf(" new point has collision %d \n",r);
        continue;
      }
      
      if(ProjectedEdgeValid(new_config, old_config, r) <= 0) // this edge has a collision
      {  
        //printf(" new edge has collision %d \n",r);  
        continue;
      }
      
      //printf(" found possible point \n");
      
      break; // robot r has not point or edge collisions
    }
  }    
}

void MultiRobotWorkspace::RandMove3(vector<float>& new_config, float prob_at_goal, const vector<float>& the_goal) // calculates a new configuration point that is drawn randomly from the configuration space, but with prob_at_goal a copy of the goal configuration the_goal is chosen
{
  int x_ind, y_ind, theta_ind;
  float bot_theta;
  if(((float)rand_int(0, 100000))/100000 < prob_at_goal) // want to move directly at the goal configuration (which is where robots start)
  {
    new_config = the_goal;
    return;
  }
  else
  {
    for(int r = 0; r < num_robots; r++)
    {      
      x_ind = r*dims;
      y_ind = x_ind+1;
      theta_ind = y_ind + 1;  
      
      new_config[x_ind] = robot_radius[r] + ((float)rand_int(0, 100000))/100000*(dim_max[0] - 2*robot_radius[r]); //robot radius included to give valid vs boundry collisions
      new_config[y_ind] = robot_radius[r] + ((float)rand_int(0, 100000))/100000*(dim_max[1] - 2*robot_radius[r]); //robot radius included to give valid vs boundry collisions
      bot_theta = ((float)rand_int(0, 100000))/100000*(dim_max[2]);
      
      while(bot_theta < -PI)
        bot_theta += 2*PI;
      while(bot_theta > PI)
        bot_theta -= 2*PI;
      new_config[theta_ind] =  bot_theta;
    }
  } 
}

void MultiRobotWorkspace::MoveToward(const vector<float>& old_config_from, const vector<float>& old_config_to, vector<float>& new_config, float move_dist) // new_config is move_dist from old_config_from to old_config_to 
{
  int x_ind, y_ind, theta_ind;
  
  // find lenth of vector from old_config_from to old_config_to
  float sum_up = 0;
  for(int r = 0; r < num_robots; r++)
  {      
    x_ind = r*dims;
    y_ind = x_ind+1;
    sum_up += (old_config_to[x_ind] - old_config_from[x_ind])*(old_config_to[x_ind] - old_config_from[x_ind]);
    sum_up += (old_config_to[y_ind] - old_config_from[y_ind])*(old_config_to[y_ind] - old_config_from[y_ind]);
  }
  
  float vec_length = sqrt(sum_up);

  if(move_dist >= vec_length)
  {
    new_config = old_config_to;
    return;    
  }
  
  vec_length = move_dist/vec_length;
  
  float theta_diff;
  float old_config_from_;
  for(int r = 0; r < num_robots; r++)
  {  
    x_ind = r*dims;
    y_ind = x_ind+1;
    theta_ind = y_ind + 1;  
    
    new_config[x_ind] = old_config_from[x_ind] + (old_config_to[x_ind] - old_config_from[x_ind])*vec_length;
    new_config[y_ind] = old_config_from[y_ind] + (old_config_to[y_ind] - old_config_from[y_ind])*vec_length;
    
    theta_diff = old_config_to[theta_ind] - old_config_from[theta_ind];

    if(fabs(theta_diff) < PI)
    {
      new_config[theta_ind] = old_config_from[theta_ind] + theta_diff*vec_length;   
    }
    else
    {
      if(old_config_from[theta_ind] > old_config_to[theta_ind])
        old_config_from_ =  old_config_from[theta_ind] - 2*PI;  
      else
        old_config_from_ = old_config_from[theta_ind] + 2*PI; 
      
      new_config[theta_ind] = old_config_from_ + (old_config_to[theta_ind] - old_config_from_)*vec_length;
      
      while(new_config[theta_ind] < -PI)
        new_config[theta_ind] += 2*PI;
      while( new_config[theta_ind] > PI)
        new_config[theta_ind] -= 2*PI;
    }    
  } 
}

float MultiRobotWorkspace::PointValid(const vector<float>& P) // checks the point P in the Workspace for validity, returns min distance to collision
{
  // robot locations are stored in P: P[n] P[n+1] is position (x,y) for robot n/2
  
  float robot_1_x, robot_1_y, robot_2_x, robot_2_y, dist, radius_1;
  float min_dist = LARGE;
  
  for(int i = 0; i < num_robots; i++)
  {  
    robot_1_x = P[i*dims];
    robot_1_y = P[i*dims+1];
    radius_1 = robot_radius[i];
    
    // check robot vs robot collisions
    for(int j = i+1; j < num_robots; j++)
    {
      robot_2_x = P[j*dims];
      robot_2_y = P[j*dims+1]; 
        
      //printf("%f, %f \n", sqrt((robot_1_x-robot_2_x)*(robot_1_x-robot_2_x) + (robot_1_y-robot_2_y)*(robot_1_y-robot_2_y)), (radius_1 + robot_radius[j]));
      
      dist = (sqrt((robot_1_x-robot_2_x)*(robot_1_x-robot_2_x) + (robot_1_y-robot_2_y)*(robot_1_y-robot_2_y)) - (radius_1 + robot_radius[j]));
      if(min_dist > dist)
      {
        min_dist = dist;
        if(min_dist <= 0) // early break out if collision
        {
          //printf("robot vs robot \n");     
          return min_dist;
        }
      }
    }
    
    // check robot vs boundry conditions
    dist = robot_1_x-radius_1;
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
      {
        //printf("robot vs boundry 1 \n");     
        return min_dist;
      }
    }
        
    dist = dim_max[0] - (robot_1_x+radius_1);
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
      {
        //printf("robot vs boundry 2 %f \n", dim_max[0]);     
        return min_dist;
      }
    }
     
    dist = robot_1_y-radius_1;
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
      {
        //printf("robot vs boundry 3 \n");     
        return min_dist;
      }
    }
        
    dist = dim_max[1] - (robot_1_y+radius_1);
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
      {
        //printf("robot vs boundry 4 \n");     
        return min_dist;
      }
    }
    
    // check robot vs scene collisions
    dist = Scene.PointSafe(P, i*dims, radius_1);
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
      {
        //printf("robot vs scene\n");     
        return min_dist;
      }
    }
  }
  
  return min_dist;
}

float MultiRobotWorkspace::EdgeValid(const vector<float>& P1, const vector<float>& P2) // checks that points P1 and P2 can be connected, returns min distance to collision
{
  // this assumes all robots move at the same speed
    
  float min_dist = LARGE;  
  int dims_all = num_robots*dims;
  float radius_1;
  float x_1a, y_1a, x_1b, y_1b, x_2a, y_2a, x_2b, y_2b;
  float m_1x, m_2x, m_1y, m_2y, Ax, Ay, Bx, By, t, x1, y1, x2, y2, dist;
  for(int i = 0; i < dims_all; i +=dims)
  {
    radius_1 = robot_radius[i/dims];  
        
    // check robot vs robot collisions
   
    // first edge is    P1[i] to P2[i]  (dim 1 )   and    P1[i+1] to P2[i+1]  (dim 2)
    x_1a = P1[i];
    y_1a = P1[i+1];

    x_1b = P2[i];
    y_1b = P2[i+1];
                  
    for(int j = i+dims; j < dims_all; j +=dims)
    {
      // second edge is P1[j] to P2[j]  (dim 1 )   and    P1[j+1] to P2[j+1]  (dim 2 )
      x_2a = P1[j];
      y_2a = P1[j+1];

      x_2b =  P2[j];
      y_2b = P2[j+1];
        
      // assume time goes from t = 0 to t = 1 and use parametric functions in t, solve for the time t when the two robots are closest to each other

      m_1x = (x_1b-x_1a); // /(1-0)
      m_2x = (x_2b-x_2a); // /(1-0)

      m_1y = (y_1b-y_1a); // /(1-0)
      m_2y = (y_2b-y_2a); // /(1-0)

      Ax = m_1x - m_2x;
      Ay = m_1y - m_2y;

      // if the edges are parallell with the same length and pointing in the same direction, 
      if(Ax == 0 && Ay == 0)
          t = 0;
      else
      {
        Bx = x_1a - x_2a;
        By = y_1a - y_2a;

        t = -(Ax*Bx + Ay*By)/(Ax*Ax + Ay*Ay); // this is the time when the robot are closest to each other
      }
      
      if(t <= 0)  // closest time is before start of edge
      {
        x1 = x_1a;
        y1 = y_1a;

        x2 = x_2a;
        y2 = y_2a;      
      }
      else if(t >= 1) // closest time is after end of edge
      {
        x1 = x_1b;
        y1 = y_1b;

        x2 = x_2b;
        y2 = y_2b;     
      }
      else
      {
        x1 = m_1x*t + x_1a;
        y1 = m_1y*t + y_1a;

        x2 = m_2x*t + x_2a;
        y2 = m_2y*t + y_2a;
      }
      
      dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) - radius_1 - robot_radius[j/dims];
      
      if(min_dist > dist)
      {
        min_dist = dist;
        if(min_dist <= 0) // early break out if collision
          return min_dist;
      }
    } 
    
    // check robot vs boundry collisions, endpoint a
    dist = x_1a-radius_1;
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
        return min_dist;
    }
        
    dist = dim_max[0] - (x_1a+radius_1);
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
        return min_dist;
    }
     
    dist = y_1a-radius_1;
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
        return min_dist;
    }  
        
    dist = dim_max[1] - (y_1a+radius_1);
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
        return min_dist;
    }
    
    // check robot vs boundry conditions, endpoint b
    dist = x_1b-radius_1;
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
        return min_dist;
    }
        
    dist = dim_max[0] - (x_1b+radius_1);
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
        return min_dist;
    }
     
    dist = y_1b-radius_1;
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
        return min_dist;
    }
        
    dist = dim_max[1] - (y_1b+radius_1);
    if(min_dist > dist)
    {
      min_dist = dist;
      if(min_dist <= 0) // early break out if collision
        return min_dist;
    } 
    
    // check robot vs scene collisions 
    if(!Scene.EdgeSafe(P1, P2, i, radius_1)) // the rest of the path segment
      return -radius_1; // early break out if collision
  }
  
  return min_dist;
}


float MultiRobotWorkspace::ProjectedPointValid(const vector<float>& P, int bot) // checks the point P in the Workspace for validity, for the robot bot
{
  // robot locations are stored in P: P[n] P[n+1] is position (x,y) for robot n/2
  
  float robot_1_x, robot_1_y, dist, radius_1;
  float min_dist = LARGE;
  
  int i = bot;
  
  robot_1_x = P[i*dims];
  robot_1_y = P[i*dims+1];
  radius_1 = robot_radius[i];
    
  // check robot vs boundry conditions
  dist = robot_1_x-radius_1;
  if(min_dist > dist)
  {
    min_dist = dist;
    if(min_dist <= 0) // early break out if collision
    {
      //printf("robot vs boundry 1 \n");     
      return min_dist;
    }
  }
        
  dist = dim_max[0] - (robot_1_x+radius_1);
  if(min_dist > dist)
  {
    min_dist = dist;
    if(min_dist <= 0) // early break out if collision
    {
      //printf("robot vs boundry 2 %f \n", dim_max[0]);     
      return min_dist;
    }
  }
     
  dist = robot_1_y-radius_1;
  if(min_dist > dist)
  {
    min_dist = dist;
    if(min_dist <= 0) // early break out if collision
    {
      //printf("robot vs boundry 3 \n");     
      return min_dist;
    }
  }
        
  dist = dim_max[1] - (robot_1_y+radius_1);
  if(min_dist > dist)
  {
    min_dist = dist;
    if(min_dist <= 0) // early break out if collision
    {
      //printf("robot vs boundry 4 \n");     
      return min_dist;
    }
  }
    
  // check robot vs scene collisions
  dist = Scene.PointSafe(P, i*dims, radius_1);
  if(min_dist > dist)
  {
    min_dist = dist;
    if(min_dist <= 0) // early break out if collision
    {
      //printf("robot vs scene\n");     
      return min_dist;
    }
  }
  
  return min_dist;
}

float MultiRobotWorkspace::ProjectedEdgeValid(const vector<float>& P1, const vector<float>& P2, int bot) // checks that points P1 and P2 can be connected, returns min distance to collision, for the robot bot
{
  // this assumes all robots move at the same speed
    
  float min_dist = LARGE;  
  float radius_1 = robot_radius[bot];
  
  int i = bot*dims;

  if(!Scene.EdgeSafe(P1, P2, i, radius_1)) // the rest of the path segment
  {
    //printf(" scene collision %d, [%f %f]  [%f %f] \n", i, P1[i], P1[i+1], P2[i], P2[i+1]);  
    return -radius_1; // early break out if collision
  }
  
  return min_dist;
}



float MultiRobotWorkspace::Dist(const vector<float>& P1, const vector<float>& P2) // returns the distance between P1 and P2 (sum of workspace distance of all robots)
{
  float sum = 0;
  int dims_all = num_robots*dims;
  
  if(DISTANCE_METRIC == 0)
  {
    for(int i = 0; i < dims_all; i +=dims)
      sum += sqrt((P1[i]-P2[i])*(P1[i]-P2[i]) + (P1[i+1]-P2[i+1])*(P1[i+1]-P2[i+1])); // (P1[i+2]-P2[i+2])*(P1[i+2]-P2[i+2] 
  }
  else if(DISTANCE_METRIC == 1)
  {
    float this_len;
    for(int i = 0; i < dims_all; i +=dims)
    {
      this_len = sqrt((P1[i]-P2[i])*(P1[i]-P2[i]) + (P1[i+1]-P2[i+1])*(P1[i+1]-P2[i+1])); // (P1[i+2]-P2[i+2])*(P1[i+2]-P2[i+2]  
      if(this_len > sum)
        sum = this_len;
    } 
  }
  
  return sum;
}

float MultiRobotWorkspace::MaxDist(const vector<float>& P1, const vector<float>& P2) // returns the max distance of any robot between P1 and P2
{
  float max_dist = 0;
  float this_dist;
  int dims_all = num_robots*dims;
  
  if(DISTANCE_METRIC == 0)
  {
    for(int i = 0; i < dims_all; i +=dims)
    {
      this_dist = sqrt((P1[i]-P2[i])*(P1[i]-P2[i]) + (P1[i+1]-P2[i+1])*(P1[i+1]-P2[i+1])); // (P1[i+2]-P2[i+2])*(P1[i+2]-P2[i+2]  
      if(this_dist > max_dist)
        max_dist = this_dist;
    }
  }
  else if(DISTANCE_METRIC == 1)
  {
    for(int i = 0; i < dims_all; i +=dims)
    {
      this_dist = sqrt((P1[i]-P2[i])*(P1[i]-P2[i]) + (P1[i+1]-P2[i+1])*(P1[i+1]-P2[i+1])); // (P1[i+2]-P2[i+2])*(P1[i+2]-P2[i+2]  
      if(this_dist > max_dist)
        max_dist = this_dist;
    } 
  }
  return max_dist;  
}

float MultiRobotWorkspace::AngularDist(const vector<float>& P1, const vector<float>& P2) // returns the angular distance between P1 and P2
{
  float sum = 0;
  int dims_all = num_robots*dims;     
  float this_diff;
  for(int i = 0; i < dims_all; i +=dims)
  {
    this_diff = P1[i+2]-P2[i+2];
    while(this_diff > PI)
      this_diff -= PI;
    while(this_diff < -PI)
      this_diff += PI;   
    
    if(this_diff < 0)
      this_diff *= -1;
    
    sum += this_diff;
  }
  
  return sum;
}

float MultiRobotWorkspace::MaxAngularDist(const vector<float>& P1, const vector<float>& P2) // returns the max angular distance of any robot between P1 and P2
{
  float max_diff = 0;
  int dims_all = num_robots*dims;     
  float this_diff;
  for(int i = 0; i < dims_all; i +=dims)
  {
    this_diff = P1[i+2]-P2[i+2];
    while(this_diff > PI)
      this_diff -= PI;
    while(this_diff < -PI)
      this_diff += PI;   
    
    if(this_diff < 0)
      this_diff *= -1;
    
    if(max_diff < this_diff)
      max_diff = this_diff;
  }
  
  return max_diff;
}
