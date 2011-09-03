NavScene::NavScene()                              // default constructor 
{
  world_dims = 3;  // x, y, theta
  dim_max.resize(world_dims);
  dim_max[0] = 1;
  dim_max[1] = 1;
  dim_max[2] = 2*PI;
  
  startC.resize(world_dims);
  goalC.resize(world_dims);
  for(int i = 0 ; i < world_dims; i++)
  {
    startC[i] = 0;
    goalC[i] = 0;   
  }
 
  num_robots = 1;
  prob_at_goal = .1;
  move_max = 1;
  theta_max = PI;
  resolution = 0;
  angular_resolution = 0;
  
  num_polygons = 0;
  num_spatial_dims = 2;
 
  n_points_ptr = 0;     
  n_edge_ptr = 0;    
  
  translation.resize(world_dims, 0);
}

NavScene::NavScene(const NavScene& S)  // copy constructor
{
  world_dims = S.world_dims;              
  dim_max = S.dim_max;
  startC = S.startC;
  goalC = S.goalC;     
  robot_rad = S.robot_rad;             
  num_robots = S.num_robots;               
  prob_at_goal = S.prob_at_goal;
  move_max = S.move_max;              
  theta_max = S.theta_max;             
  resolution = S.resolution;            
  angular_resolution = S.angular_resolution;  
  
  num_polygons = S.num_polygons;
  polygon_list = S.polygon_list;
  
  num_spatial_dims = S.num_spatial_dims;
  
  PointSafeLookup = S.PointSafeLookup;
  EdgeSafeLookup = S.EdgeSafeLookup;
  
  last_n_points_x = S.last_n_points_x;     
  last_n_points_y = S.last_n_points_y;    
  last_n_points_val = S.last_n_points_val; 
  n_points_ptr = S.n_points_ptr;     
    
  last_n_edges_x1 = S.last_n_edges_x1;
  last_n_edges_y1 = S.last_n_edges_y1;
  last_n_edges_x2 = S.last_n_edges_x2;
  last_n_edges_y2 = S.last_n_edges_y2;
  last_n_edges_val = S.last_n_edges_val;
  n_edge_ptr = S.n_edge_ptr; 
  
  translation = S.translation;
}

NavScene::~NavScene()                 // destructor
{
    
}

void NavScene::PrintSceneInfo()      // prints on command line the info about the scene
{
  printf("Scene Info:\n");
  printf("world_dims: %d\n", world_dims);
  printf("dim limits: ");  
  for(uint i = 0; i < dim_max.size(); i++)
    printf("%f, ",dim_max[i]);
  printf("\n");
  printf("robot_rad: "); 
  for(uint i = 0; i < robot_rad.size(); i++)
    printf("%f, ",robot_rad[i]);
  printf("\n");
  printf("num_robots: %d\n", num_robots);
  printf("start config: \n");  
  for(uint i = 0; i < startC.size(); i++)
  {
    printf("%f, ",startC[i]);
    if(((i+1)/world_dims)*world_dims == i+1)
      printf("\n");
  }
  printf("goal config: \n"); 
  for(uint i = 0; i < goalC.size(); i++)
  {
    printf("%f, ",goalC[i]);
    if(((i+1)/world_dims)*world_dims == i+1)
      printf("\n");
  }
  printf("prob_at_goal: %f\n", prob_at_goal);
  printf("move_max: %f\n", move_max);
  printf("theta_max: %f\n", theta_max);
  printf("resolution: %f\n", resolution);
  printf("angular_resolution: %f\n", angular_resolution);
  
  printf("num_polygons: %d\n", num_polygons);
  printf("num_spatial_dims: %d\n", num_spatial_dims);
}

bool NavScene::LoadFromFile(const char* filename) // loads the scene info from the file
{
  FILE* ifp = fopen(filename,"r");
  if(ifp == NULL)
  {
    printf("cannot read scene file \n");
    return false;   
  }    
  
  // get world dims from from file
  if(fscanf(ifp, "D:%d\n", &world_dims) <= 0)
  {
    printf("problems reading world dims from scene file \n");
    fclose(ifp);
    return false;
  }
  if(world_dims < 1)
  {
    printf("problems: world dims cannot be less than 1 \n"); 
    fclose(ifp);
    return false;
  }
  
  dim_max.resize(world_dims);
  
  int unused_result; // dummy return to make warnings go away

  // get max values for each dimension from file
  float this_value;
  for(int k = 0; k < world_dims; k++)   
  {
    if(fscanf(ifp, "%f, ", &this_value) <= 0) 
    {
      printf("problems reading world dim limits from scene file \n"); 
      fclose(ifp);
      return false;
    }
    dim_max[k] = this_value;
  }
  unused_result = fscanf(ifp, "\n");
  
  // get num_robots from from file
  if(fscanf(ifp, "R:%d\n", &num_robots) <= 0)
  {
    printf("problems reading num_robots from scene file \n");
    fclose(ifp);
    return false;
  }
  if(num_robots < 1)
  {
    printf("problems: num_robots cannot be less than 1 \n"); 
    fclose(ifp);
    return false;
  }
  
  // get robot_rad from from file
  if(fscanf(ifp, "r:%f\n", &this_value) <= 0)
  {
    printf("problems reading robot_rad from scene file \n");
    fclose(ifp);
    return false;
  }
  if(this_value < 0)
  {
    printf("problems: robot_rad cannot be less than 0 \n"); 
    fclose(ifp);
    return false;
  }
  robot_rad.resize(num_robots);
  for(int i = 0; i < num_robots; i++)
    robot_rad[i] = this_value;
            
  // get start configuration from file
  if(fscanf(ifp, "S:\n") < 0)
  {
    printf("problems reading start configuration from scene file \n");
    fclose(ifp);
    return false;
  }
  startC.resize(num_robots*world_dims);
  int l = 0;
  for(int j = 0; j < num_robots; j++)
  {
    for(int k = 0; k < world_dims; k++)   
    {
      if(fscanf(ifp, "%f, ", &this_value) <= 0) 
      {
        printf("problems reading start configuration from scene file (element %d) \n",l); 
        fclose(ifp);
        return false;
      }
      startC[l] = this_value;
      l++;
    }
    unused_result = fscanf(ifp, "\n");
  }
  
  // get goal configuration from file
  if(fscanf(ifp, "G:\n") < 0)
  {
    printf("problems reading start configuration from scene file \n");
    fclose(ifp);
    return false;
  }
  goalC.resize(num_robots*world_dims);
  l = 0;
  for(int j = 0; j < num_robots; j++)
  {
    for(int k = 0; k < world_dims; k++)   
    {
      if(fscanf(ifp, "%f, ", &this_value) <= 0) 
      {
        printf("problems reading start configuration from scene file (element %d) \n",l); 
        fclose(ifp);
        return false;
      }
      goalC[l] = this_value;
      l++;
    }
    unused_result = fscanf(ifp, "\n");
  }
  
  // get prob_at_goal from from file
  if(fscanf(ifp, "p:%f\n", &prob_at_goal) <= 0)
  {
    printf("problems reading prob_at_goal from scene file \n");
    fclose(ifp);
    return false;
  }
  if(prob_at_goal < 0 || prob_at_goal > 1)
  {
    printf("problems: prob_at_goal must be in the range [0,1] \n"); 
    fclose(ifp);
    return false;
  }
  
  // get move_max from from file
  if(fscanf(ifp, "m:%f\n", &move_max) <= 0)
  {
    printf("problems reading move_max from scene file \n");
    fclose(ifp);
    return false;
  }
  if(move_max < 0)
  {
    printf("problems: move_max cannot be less than 0 \n"); 
    fclose(ifp);
    return false;
  }
  
  // get theta_max from from file
  if(fscanf(ifp, "t:%f\n", &theta_max) <= 0)
  {
    printf("problems reading theta_max from scene file \n");
    fclose(ifp);
    return false;
  }
  if(theta_max < 0)
  {
    printf("problems: theta_max cannot be less than 0 \n"); 
    fclose(ifp);
    return false;
  }

  // get resolution from from file
  if(fscanf(ifp, "r:%f\n", &resolution) <= 0)
  {
    printf("problems reading resolution from scene file \n");
    fclose(ifp);
    return false;
  }
  if(resolution < 0)
  {
    printf("problems: resolution cannot be less than 0 \n"); 
    fclose(ifp);
    return false;
  }
  
  // get angular_resolution from from file
  if(fscanf(ifp, "a:%f\n", &angular_resolution) <= 0)
  {
    printf("problems reading angular_resolution from scene file \n");
    fclose(ifp);
    return false;
  }
  if(angular_resolution < 0)
  {
    printf("problems: angular_resolution cannot be less than 0 \n"); 
    fclose(ifp);
    return false;
  }
  
  // get num_polygons from from file
  if(fscanf(ifp, "P:%d\n", &num_polygons) <= 0)
  {
    printf("problems reading world dims from scene file \n");
    fclose(ifp);
    return false;
  }
  
  if(num_polygons < 0)
  {
    printf("problems: num_polygons cannot be less than 0 \n"); 
    fclose(ifp);
    return false;
  }
  else if(num_polygons > 0)
  {
    // get number of spatial workspace dims from from file
    if(fscanf(ifp, "n:%d\n", &num_spatial_dims) <= 0)
    {
      printf("problems reading num spatial world dims from scene file \n");
      fclose(ifp);
      return false;
    }
    if(num_spatial_dims < 1)
    {
      printf("problems: num spatial world dims than 1 \n"); 
      fclose(ifp);
      return false;
    }
  
    int this_points;
    polygon_list.resize(num_polygons);
    for(int i = 0; i < num_polygons; i++)
    {
      // get this polygon's num points from from file
      if(fscanf(ifp, "p:%d\n", &this_points) <= 0)
      {
        printf("problems reading world dims from scene file \n");
        fclose(ifp);
        return false;
      }
      if(this_points < 0)
      {
        printf("problems: a polygon's number of points cannot be less than 0 \n"); 
        fclose(ifp);
        return false;
      }  
      
      polygon_list[i].resize(this_points);
    
      for(int j = 0; j < this_points; j++)
      {
        polygon_list[i][j].resize(num_spatial_dims);
        for(int k = 0; k < num_spatial_dims; k++)   
        {
          if(fscanf(ifp, "%f, ", &this_value) <= 0) 
          {
            printf("problems reading polygon from scene file (element %d) \n",j); 
            fclose(ifp);
            return false;
          }
          polygon_list[i][j][k] = this_value;
          l++;
        }
        unused_result = fscanf(ifp, "\n");
      }
    }
    
    // allocate point lookup table
    int x_num_points = (int)(2*dim_max[0]/resolution+1);
    int y_num_points = (int)(2*dim_max[1]/resolution+1);
  
    PointSafeLookup.resize(x_num_points);
    for(int i = 0; i < x_num_points; i++)
      PointSafeLookup[i].assign(y_num_points, -1);
  
    if(using_edge_lookup_table == 1)
    {
      // allocate edge lookup table
      EdgeSafeLookup.resize(x_num_points);
      for(int i1 = 0; i1 < x_num_points; i1++)
      {
        EdgeSafeLookup[i1].resize(y_num_points);
        for(int j1 = 0; j1 < y_num_points; j1++)
        {
          EdgeSafeLookup[i1][j1].resize(x_num_points);
          for(int i2 = 0; i2 < x_num_points; i2++)    
            EdgeSafeLookup[i1][j1][i2].assign(y_num_points, -1);
        }
      }
    }
    
    if(add_points_to_messages == 1)
    {
      // allocate structures to hold last n points
        
      last_n_points_x.assign(num_points_per_file, -1);     
      last_n_points_y.assign(num_points_per_file, -1);    
      last_n_points_val.assign(num_points_per_file, 0); 
      n_points_ptr = 0;     
    
      last_n_edges_x1.assign(num_edges_per_file, -1);
      last_n_edges_y1.assign(num_edges_per_file, -1);
      last_n_edges_x2.assign(num_edges_per_file, -1);
      last_n_edges_y2.assign(num_edges_per_file, -1);
      last_n_edges_val.assign(num_edges_per_file, 0);
      n_edge_ptr = 0; 
    }
  }
  else // num_polygons == 0
  {
      
      
  }
  
  
  fclose(ifp);
  printf("sucessfully read file \n");

  return true;
}

bool NavScene::LoadMapFromFile(const char* filename) // loads only the map portion of a file
{
  FILE* ifp = fopen(filename,"r");
  if(ifp == NULL)
  {
    printf("cannot read scene file \n");
    return false;   
  }    
  
  float this_value;
  
  //get first line from file
  char line_from_file[500];
  vector<float> this_translation;
  
  if(fscanf(ifp, "%s\n", line_from_file) <= 0)
  {
    printf("problems reading first line of scene file \n");
    fclose(ifp);
    return false;   
  }

  if( line_from_file[0] == 'P' )   // there is no translation
  {
    // get num_polygons from from file
    if(sscanf(line_from_file, "P:%d\n", &num_polygons) <= 0)
    {
      printf("problems reading world dims from scene file \n");
      fclose(ifp);
      return false;
    }    
  }    
  else if( line_from_file[0] == 'T' ) // there is translation
  {   
    // get translation along each dimension
    int ind = 2;
    float temp;
     
    while(sscanf((char*)(line_from_file+(size_t)ind), "%f", &temp) > 0)
    {
      this_translation.push_back(temp);
      while(line_from_file[ind] != ';' && line_from_file[ind] != ',')
        ind++;
      ind++;
    }
     
    // get num_polygons from from file
    if(fscanf(ifp, "P:%d\n", &num_polygons) <= 0)
    {
      printf("problems reading world dims from scene file \n");
      fclose(ifp);
      return false;
    }
  }
  else
  {
    printf("problems reading world dims or translation from scene file \n");
    fclose(ifp);
    return false;
  } 
   
  
  if(num_polygons < 0)
  {
    printf("problems: num_polygons cannot be less than 0 \n"); 
    fclose(ifp);
    return false;
  }
  else if(num_polygons > 0)
  {
    // get number of spatial workspace dims from from file
    if(fscanf(ifp, "n:%d\n", &num_spatial_dims) <= 0)
    {
      printf("problems reading num spatial world dims from scene file \n");
      fclose(ifp);
      return false;
    }
    if(num_spatial_dims < 1)
    {
      printf("problems: num spatial world dims than 1 \n"); 
      fclose(ifp);
      return false;
    }
  
    while(this_translation.size() < (uint)num_spatial_dims)
      this_translation.push_back(0);    
    
    printf("Translation: ");
    for(int i = 0; i < num_spatial_dims; i++)
      printf("%f, ", this_translation[i]);
    printf("\n");
    
    // this is the translation between map_cu and the smaller planning space for multi_robot_planner_cu (Assuming already set by this point);
    while(translation.size() < (uint)num_spatial_dims)
      translation.push_back(0);    
    
    int this_points;
    polygon_list.resize(num_polygons);
    for(int i = 0; i < num_polygons; i++)
    {
      // get this polygon's num points from from file
      if(fscanf(ifp, "p:%d\n", &this_points) <= 0)
      {
        printf("problems reading world dims from scene file \n");
        fclose(ifp);
        return false;
      }
      if(this_points < 0)
      {
        printf("problems: a polygon's number of points cannot be less than 0 \n"); 
        fclose(ifp);
        return false;
      }  
      
      polygon_list[i].resize(this_points);
    
      for(int j = 0; j < this_points; j++)
      {
        polygon_list[i][j].resize(num_spatial_dims);
        for(int k = 0; k < num_spatial_dims; k++)   
        {
          if(fscanf(ifp, "%f, ", &this_value) <= 0) 
          {
            printf("problems reading polygon from file (element %d) \n",j); 
            fclose(ifp);
            return false;
          }
          polygon_list[i][j][k] = this_value + this_translation[k] + translation[k];
        }
        int unused_result;
        unused_result = fscanf(ifp, "\n");  // unused_result makes warning go away
      }
    }
    
    // allocate point lookup table
    int x_num_points = (int)(2*dim_max[0]/resolution+1);
    int y_num_points = (int)(2*dim_max[1]/resolution+1);
  
    PointSafeLookup.resize(x_num_points);
    for(int i = 0; i < x_num_points; i++)
      PointSafeLookup[i].assign(y_num_points, -1);
  
    if(using_edge_lookup_table == 1)
    {
      // allocate edge lookup table
      EdgeSafeLookup.resize(x_num_points);
      for(int i1 = 0; i1 < x_num_points; i1++)
      {
        EdgeSafeLookup[i1].resize(y_num_points);
        for(int j1 = 0; j1 < y_num_points; j1++)
        {
          EdgeSafeLookup[i1][j1].resize(x_num_points);
          for(int i2 = 0; i2 < x_num_points; i2++)    
            EdgeSafeLookup[i1][j1][i2].assign(y_num_points, -1);
        }
      }
    }
    
    if(add_points_to_messages == 1)
    {
      // allocate structures to hold last n points
        
      last_n_points_x.assign(num_points_per_file, -1);     
      last_n_points_y.assign(num_points_per_file, -1);    
      last_n_points_val.assign(num_points_per_file, 0); 
      n_points_ptr = 0;     
    
      last_n_edges_x1.assign(num_edges_per_file, -1);
      last_n_edges_y1.assign(num_edges_per_file, -1);
      last_n_edges_x2.assign(num_edges_per_file, -1);
      last_n_edges_y2.assign(num_edges_per_file, -1);
      last_n_edges_val.assign(num_edges_per_file, 0);
      n_edge_ptr = 0; 
    }
  }
  else // num_polygons == 0
  {
      
      
  }
  
  
  fclose(ifp);
  printf("sucessfully read file \n");

  return true;
}

bool NavScene::LoadFromGlobals(GlobalVariables& G) // loads the scene info from the global variables
{    
  // get world dims
  world_dims = 3; // x, y, theta
  dim_max.resize(world_dims);
   
  // figure out the transform between the space in G and the space that will be used for planning  
  // find min and mx x and y
  float min_x = LARGE;
  float min_y = LARGE;
  float max_x = -LARGE;
  float max_y = -LARGE;  
    
  if(!G.found_single_robot_solution) // for a single robot we need to plan in the entire area
  {
    if(G.default_map_x_size <= 0 || G.default_map_y_size <= 0)
    {
      printf("WARNING: map size unset, set with paramiter server\n");
    }

    min_x = 0;
    min_y = 0;
    max_x = G.default_map_x_size;
    max_y = G.default_map_y_size;
  }
  else // for multi robot only plan in a sub area
  {
    for(int i = 0; i < G.team_size; i++)
    {
      if(min_x > G.start_coords[i][0])
        min_x = G.start_coords[i][0];
      if(min_x > G.goal_coords[i][0])
        min_x = G.goal_coords[i][0]; 
    
      if(min_y > G.start_coords[i][1])
        min_y = G.start_coords[i][1];
      if(min_y > G.goal_coords[i][1])
        min_y = G.goal_coords[i][1];
    
      if(max_x < G.start_coords[i][0])
        max_x = G.start_coords[i][0];
      if(max_x < G.goal_coords[i][0])
        max_x = G.goal_coords[i][0]; 
    
      if(max_y < G.start_coords[i][1])
        max_y = G.start_coords[i][1];
      if(max_y < G.goal_coords[i][1])
        max_y = G.goal_coords[i][1];    
    }
    
    // we want to add a frame of free space around the configuration (obstacles handled later)
    min_x -= G.planning_border_width;
    max_x += G.planning_border_width;
    min_y -= G.planning_border_width;
    max_y += G.planning_border_width;
  }

  float min_theta = 0;
  float max_theta = 2*PI; 
  
  // we will translate everything by subtracting min_x and min_y
  translation[0] = -min_x;
  translation[1] = -min_y;
  translation[2] = 0;
  
  // get max values for each dimension (in post translated space)
  dim_max[0] = max_x - min_x;
  dim_max[1] = max_y - min_y;
  dim_max[2] = max_theta - min_theta;

  // store info about planning bounding area in the global data structure
  G.team_bound_area_min.resize(3);
  G.team_bound_area_min[0] = min_x;
  G.team_bound_area_min[1] = min_y;
  G.team_bound_area_min[2] = 0;
  
  G.team_bound_area_size.resize(3);
  G.team_bound_area_size[0] = dim_max[0];
  G.team_bound_area_size[1] = dim_max[1];
  G.team_bound_area_size[2] = dim_max[2];
  
  // get num_robots
  num_robots = G.team_size;
  if(num_robots < 1)
  {
    printf("problems: num_robots cannot be less than 1 \n"); 
    return false;
  }
  
  // get robot_rad, assuming all the same, can modify this to be sent in start-up messages if we ever need it to be different
  robot_rad.resize(num_robots);
  for(int i = 0; i < num_robots; i++)
  {
    robot_rad[i] = G.robot_radius;
    if(robot_rad[i] < 0)
    {
      printf("problems: robot_rad cannot be less than 0 \n"); 
      return false;
    }
  }
  
  // get start configuration
  startC.resize(num_robots*world_dims);
  int j = 0;
  for(int i = 0; i < num_robots; i++)
  {
    // get to next valid robot   
    while(G.have_info[j] == 0)
      j++;  
      
    startC[i*world_dims] = G.start_coords[j][0] - min_x;
    startC[i*world_dims+1] = G.start_coords[j][1] - min_y;
    startC[i*world_dims+2] = G.start_coords[j][2]; 
    j++; 
  }
   
  printf("adjusted start: \n");
  print_float_vector(startC);
  
  // get goal configuration
  goalC.resize(num_robots*world_dims);
  j = 0;
  for(int i = 0; i < num_robots; i++)
  {
    // get to next valid robot   
    while(G.have_info[j] == 0)
      j++;  
      
    goalC[i*world_dims] = G.goal_coords[j][0] - min_x ;
    goalC[i*world_dims+1] = G.goal_coords[j][1] - min_y;
    goalC[i*world_dims+2] = G.goal_coords[j][2]; 
    j++; 
  }
  
  printf("adjusted goal: \n");
  print_float_vector(startC);
  
  // get prob_at_goal
  prob_at_goal = G.prob_at_goal;
  if(prob_at_goal < 0 || prob_at_goal > 1)
  {
    printf("problems: prob_at_goal must be in the range [0,1] \n"); 
    return false;
  }
  
  // get move_max
  move_max = G.move_max;
  if(move_max < 0)
  {
    printf("problems: move_max cannot be less than 0 \n"); 
    return false;
  }
  
  // get theta_max
  theta_max = G.theta_max;
  if(theta_max < 0)
  {
    printf("problems: theta_max cannot be less than 0 \n"); 
    return false;
  }

  // get resolution
  resolution = G.resolution;
  if(resolution < 0)
  {
    printf("problems: resolution cannot be less than 0 \n"); 
    return false;
  }
  
  // get angular_resolution
  angular_resolution = G.angular_resolution;
  if(angular_resolution < 0)
  {
    printf("problems: angular_resolution cannot be less than 0 \n"); 
    return false;
  }
  
  num_polygons = 0;
  
  return true;
}

void NavScene::DrawObstacles()        // draws the obstacles in the obstacle list
{
  glPushMatrix(); 
  glTranslatef(-1, -1, 0);
  
  float map_rad; 
  if(dim_max[0] > dim_max[1])
    map_rad = 2/dim_max[0];
  else
    map_rad = 2/dim_max[1];
      
  glScaled(map_rad,map_rad,1); 
  glColor3f(.5, .5, .5);
  int this_size;
  for(int i = 0; i < num_polygons; i++)
  {
    glBegin(GL_LINE_STRIP); 
    this_size = polygon_list[i].size();
    for(int j = 0; j < this_size; j++)
      glVertex2f(polygon_list[i][j][0], polygon_list[i][j][1]);
    glVertex2f(polygon_list[i][0][0], polygon_list[i][0][1]);
    glEnd(); 
  }
  glPopMatrix();
}

void NavScene::DrawPointSafeLookup()  // draws the lookup table values
{
  if(PointSafeLookup.size() <= 0)
    return;
    
  glPushMatrix(); 
  glTranslatef(-1, -1, 0);

  float map_rad; 
  if(dim_max[0] > dim_max[1])
    map_rad = 2/dim_max[0];
  else
    map_rad = 2/dim_max[1];
      
  glScaled(map_rad,map_rad,1); 
 
  // first pass, find max dist in table
  float max_dist = -1;
  for(uint i = 0; i < PointSafeLookup.size()-1; i++)
    for(uint j = 0; j < PointSafeLookup[i].size()-1; j++)
      if(PointSafeLookup[i][j] > max_dist)
        max_dist = PointSafeLookup[i][j];
  
  float x, y, val;
  glBegin(GL_POINTS);
  for(uint i = 0; i < PointSafeLookup.size()-1; i++)
  {
    x = ((float)i)*resolution/2;
  
    for(uint j = 0; j < PointSafeLookup[i].size()-1; j++)
    {
      y = ((float)j)*resolution/2;
    
      val = PointSafeLookup[i][j];
      
      if(val == -1)
          continue;
      else if(val == 0)
        glColor3f(1, 1, 1);
      else 
      {
          glColor3f(1-val/max_dist, val/max_dist, 0);
      }
      
      glVertex2f(x, y);
    }
  }
  glEnd(); 
  glPopMatrix();
}

void NavScene::DrawEdgeSafeLookup()  // draws the lookup table values
{
  if(PointSafeLookup.size() <= 0)
    return;
    
  glPushMatrix(); 
  glTranslatef(-1, -1, 0);

  float map_rad; 
  if(dim_max[0] > dim_max[1])
    map_rad = 2/dim_max[0];
  else
    map_rad = 2/dim_max[1];
      
  glScaled(map_rad,map_rad,1); 
  
  float x, y, x2, y2;
  glBegin(GL_LINES);

  for(uint i = 0; i < EdgeSafeLookup.size()-1; i++)
  {
    x = ((float)i)*resolution/2;
  
    for(uint j = 0; j < EdgeSafeLookup[i].size()-1; j++)
    {
      y = ((float)j)*resolution/2;
    
      for(uint k = 0; k < EdgeSafeLookup[i][j].size()-1; k++)
      {
        x2 = ((float)k)*resolution/2;
      
        for(uint l = 0; l < EdgeSafeLookup[i][j][k].size()-1; l++)
        {
          y2 = ((float)l)*resolution/2;
      
          if(EdgeSafeLookup[i][j][k][l] != 1)
              continue;

          glColor3f(((float)rand_int(0, 100000))/100000, ((float)rand_int(0, 100000))/100000, ((float)rand_int(0, 100000))/100000);
          
          glVertex2f(x, y);
          glVertex2f(x2, y2);
        }
      }
    }
  }
  glEnd(); 
  glPopMatrix();
}
 
float NavScene::PointSafe(const vector<float>& point, int index, float the_robot_rad) // this checks if a point is safe in the environment, where the points' coords start at index in vectors, it returns the minimum distance to an obstacle
{
  clock_t t1 = clock(); 
    
  if(num_polygons == 0)
     return LARGE;
  pctr_all++;         
  float r_x, r_y, o_x1, o_x2, o_y1, o_y2, dist_temp;
  float min_dist = LARGE;
  int this_edge_num;
  
  r_x = point[index];
  r_y = point[index+1];
  
  // check if this data is already in the lookup table
  int lookup_x = (int)(2*r_x/resolution+.5); // + .5 causes rounding instad of truncation
  int lookup_y = (int)(2*r_y/resolution+.5);
  float lookup_value = PointSafeLookup[lookup_x][lookup_y];
  if(lookup_value != -1) // then it is in the lookup table
  {
    //x_temp_vec.push_back(((float)lookup_x)*resolution);
    //y_temp_vec.push_back(((float)lookup_y)*resolution); 
      
    clock_t t2 = clock();    
    // printf("lookup: %f \n", difftime_clock(t2, t1));
    
    lookup_sum += difftime_clock(t2, t1);
    n_lookup++;

    return lookup_value;
  }
      
  // otherwise we need to do collision detection
  pctr++;
  for(int i = 0; i < num_polygons; i++) // each polynomial
  {
    this_edge_num = polygon_list[i].size();
    for(int j = 0; j < this_edge_num; j++) // each edge
    {        
      if(j == 0)   
      {   
        o_x1 = polygon_list[i][this_edge_num-1][0];
        o_x2 = polygon_list[i][0][0];
        o_y1 = polygon_list[i][this_edge_num-1][1];
        o_y2 = polygon_list[i][0][1];  
      }
      else
      {
        o_x1 = polygon_list[i][j-1][0];
        o_x2 = polygon_list[i][j][0];
        o_y1 = polygon_list[i][j-1][1];
        o_y2 = polygon_list[i][j][1]; 
      }
     
      dist_temp = line_dist_to_point(o_x1, o_y1, o_x2, o_y2, r_x, r_y) - the_robot_rad;
      if(dist_temp < min_dist)
        min_dist = dist_temp;
      
      dist_temp = line_dist_to_point(o_x1, o_y1, o_x2, o_y2, r_x, r_y) - the_robot_rad; 
      if(dist_temp < min_dist)
        min_dist = dist_temp;
      
      if(min_dist <= 0)
      {
        //x_temp_vec.push_back(r_x);
        //y_temp_vec.push_back(r_y);  
          
        // save info in the lookup table
        PointSafeLookup[lookup_x][lookup_y] = 0;
        
        if(add_points_to_messages == 1 && n_points_ptr < num_points_per_file)
        {
          // save info in last n points structure
          if(n_points_ptr < num_points_per_file)
          last_n_points_x[n_points_ptr] = lookup_x;     
          last_n_points_y[n_points_ptr] = lookup_y;    
          last_n_points_val[n_points_ptr] = 0; 
          n_points_ptr++;
        }
        
        
        clock_t t2 = clock();    
        // printf("collision: %f \n", difftime_clock(t2, t1)*10000);
        

        out_collision += difftime_clock(t2, t1);
        n_collision++;
        
        
        return min_dist;  // early break out if there is a collision
      }
    }    
  }    
  // save info in the lookup table
  PointSafeLookup[lookup_x][lookup_y] = min_dist;
  
  if(add_points_to_messages == 1 && n_points_ptr < num_points_per_file)
  {
    // save info in last n points structure
    last_n_points_x[n_points_ptr] = lookup_x;     
    last_n_points_y[n_points_ptr] = lookup_y;    
    last_n_points_val[n_points_ptr] = min_dist; 
    n_points_ptr++;
  }
  
  clock_t t2 = clock();    
  //printf("out: %f \n", difftime_clock(t2, t1)*10000);

  out_sum += difftime_clock(t2, t1);
  n_out++;
  
  return min_dist; 
}

bool NavScene::EdgeSafe(const vector<float>& point1, const vector<float>& point2, int index, float the_robot_rad) // this checks if an edge is safe in the environment, where the points' coords start at index in vectors
{
  #ifdef save_time_data
  total_collision_checks++;
  #endif   
    
  clock_t t1 = clock(); 
  if(num_polygons == 0)
     return true;
  ectr_all++;
  float r_x1, r_x2, r_y1, r_y2, o_x1, o_x2, o_y1, o_y2, Mr_top, Mr_bottom, Mo_top, Mo_bottom, Mr, Mo;
  float x = LARGE;
  float y = LARGE;
  int this_edge_num;
  
  r_x1 = point1[index];
  r_x2 = point2[index];
  r_y1 = point1[index+1];
  r_y2 = point2[index+1];
  
  int lookup_x1 = -1;
  int lookup_y1 = -1;
  int lookup_x2 = -1;
  int lookup_y2 = -1;
    
  if(using_edge_lookup_table == 1)
  {
    // check if this data is already in the lookup table
    lookup_x1 = (int)(2*r_x1/resolution+.5); // + .5 causes rounding instad of truncation
    lookup_y1 = (int)(2*r_y1/resolution+.5);
    lookup_x2 = (int)(2*r_x2/resolution+.5); // + .5 causes rounding instad of truncation
    lookup_y2 = (int)(2*r_y2/resolution+.5);
    float lookup_value = EdgeSafeLookup[lookup_x1][lookup_y1][lookup_x2][lookup_y2];
    if(lookup_value != -1) // then it is in the lookup table
    {
      //x_temp_vec.push_back(((float)lookup_x)*resolution);
      //y_temp_vec.push_back(((float)lookup_y)*resolution); 
      
      
      clock_t t2 = clock();    
      // printf("lookup: %f \n", difftime_clock(t2, t1));
    
      elookup_sum += difftime_clock(t2, t1);
      en_lookup++;  
      
      if(lookup_value == 0)
        return false;
      else// lookup_value == 1
        return true;
    }
  }
  
  // otherwise we need to do collision detection
  ectr++;
  Mr_top = r_y2 - r_y1;
  Mr_bottom = r_x2 - r_x1;
  
  if(add_points_to_messages == 1 && n_edge_ptr < num_edges_per_file)
  {
    // save info in last n edges structure
    last_n_edges_x1[n_edge_ptr] = lookup_x1;
    last_n_edges_y1[n_edge_ptr] = lookup_y1;
    last_n_edges_x2[n_edge_ptr] = lookup_x2;
    last_n_edges_y2[n_edge_ptr] = lookup_y2;
  }
            
  for(int i = 0; i < num_polygons; i++) // each polygon
  {
    this_edge_num = polygon_list[i].size();
    for(int j = 0; j < this_edge_num; j++) // each edge
    {    
      if(j == 0)   
      {   
        o_x1 = polygon_list[i][this_edge_num-1][0];
        o_x2 = polygon_list[i][0][0];
        o_y1 = polygon_list[i][this_edge_num-1][1];
        o_y2 = polygon_list[i][0][1];  
      }
      else
      {
        o_x1 = polygon_list[i][j-1][0];
        o_x2 = polygon_list[i][j][0];
        o_y1 = polygon_list[i][j-1][1];
        o_y2 = polygon_list[i][j][1]; 
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
            if(using_edge_lookup_table == 1)  
            {
              EdgeSafeLookup[lookup_x1][lookup_y1][lookup_x2][lookup_y2] = 0;
              EdgeSafeLookup[lookup_x2][lookup_y2][lookup_x1][lookup_y1] = 0;
            }
            
            if(add_points_to_messages == 1 && n_edge_ptr < num_edges_per_file)
            {
              // save info in last n edges structure
              last_n_edges_val[n_edge_ptr] = 0;  
              n_edge_ptr++;
            }
            
            
            clock_t t2 = clock();    
            // printf("collision: %f \n", difftime_clock(t2, t1)*10000);
            eout_collision += difftime_clock(t2, t1);
            en_collision++;
            
            //printf("sceen 1 \n");
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
        //x_temp_vec.push_back(x);
        //y_temp_vec.push_back(y); 
          
        if(using_edge_lookup_table == 1)
        {
          EdgeSafeLookup[lookup_x1][lookup_y1][lookup_x2][lookup_y2] = 0;
          EdgeSafeLookup[lookup_x2][lookup_y2][lookup_x1][lookup_y1] = 0;
        }
        
        if(add_points_to_messages == 1 && n_edge_ptr < num_edges_per_file)
        {
          // save info in last n edges structure
          last_n_edges_val[n_edge_ptr] = 0;  
          n_edge_ptr++;
        }
        
        clock_t t2 = clock();    
        // printf("collision: %f \n", difftime_clock(t2, t1)*10000);
        
        eout_collision += difftime_clock(t2, t1);
        en_collision++;
        
        //printf("sceen 2 \n");
        return false; 
      }
     
      // the lines do not intersect, but it is still possible that they are too close together
      // need 4 tests, one each for each end point tested against the other line
      
      
      /* assuming the path segment ends have already been checked against the obstacle
       * we can ignor the firt two tests
      float dist_temp = line_dist_to_point(o_x1, o_y1, o_x2, o_y2, r_x1, r_y1); 
      if(dist_temp < the_robot_rad)
      {
       // x_temp_vec.push_back(r_x1);
       // y_temp_vec.push_back(r_y1);  
        return false;
      }
     
      dist_temp = line_dist_to_point(o_x1, o_y1, o_x2, o_y2, r_x2, r_y2); 
      if(dist_temp < the_robot_rad)
      {
        //x_temp_vec.push_back(r_x2);
        //y_temp_vec.push_back(r_y2);  
        return false;
      } */
      
      float dist_temp = line_dist_to_point(r_x1, r_y1, r_x2, r_y2, o_x1, o_y1); 
      if(dist_temp < the_robot_rad)
      {
        if(using_edge_lookup_table == 1)
        {
          EdgeSafeLookup[lookup_x1][lookup_y1][lookup_x2][lookup_y2] = 0;
          EdgeSafeLookup[lookup_x2][lookup_y2][lookup_x1][lookup_y1] = 0;
        }
        
        if(add_points_to_messages == 1 && n_edge_ptr < num_edges_per_file)
        {
          // save info in last n edges structure
          last_n_edges_val[n_edge_ptr] = 0;  
          n_edge_ptr++;
        }
        
        
        clock_t t2 = clock();    
        // printf("collision: %f \n", difftime_clock(t2, t1)*10000);
        

        eout_collision += difftime_clock(t2, t1);
        en_collision++;
        
        //printf("sceen 3 \n");
        return false;
      }
      
      dist_temp = line_dist_to_point(r_x1, r_y1, r_x2, r_y2, o_x2, o_y2); 
      if(dist_temp < the_robot_rad)
      {
        if(using_edge_lookup_table == 1)  
        {
          EdgeSafeLookup[lookup_x1][lookup_y1][lookup_x2][lookup_y2] = 0;
          EdgeSafeLookup[lookup_x2][lookup_y2][lookup_x1][lookup_y1] = 0;
        }
        
        if(add_points_to_messages == 1 && n_edge_ptr < num_edges_per_file)
        {
          // save info in last n edges structure
          last_n_edges_val[n_edge_ptr] = 0;  
          n_edge_ptr++;
        }
        
        
        clock_t t2 = clock();    
        // printf("collision: %f \n", difftime_clock(t2, t1)*10000);
        
        eout_collision += difftime_clock(t2, t1);
        en_collision++;
        
        //printf("sceen 4 \n");
        return false;
      }
    }    
  }    
  
  if(using_edge_lookup_table == 1)
  {
    EdgeSafeLookup[lookup_x1][lookup_y1][lookup_x2][lookup_y2] = 1;
    EdgeSafeLookup[lookup_x2][lookup_y2][lookup_x1][lookup_y1] = 1;
  }
  
  if(add_points_to_messages == 1 && n_edge_ptr < num_edges_per_file)
  {
    // save info in last n edges structure
    last_n_edges_val[n_edge_ptr] = 1;  
    n_edge_ptr++;
  }
  
  
  clock_t t2 = clock();    
  // printf("collision: %f \n", difftime_clock(t2, t1)*10000);
        

  eout_sum += difftime_clock(t2, t1);
  en_out++;
  
  return true; 
}

bool NavScene::GetPointsFromFile(FILE* ifp) // this reads point data from the file
{
  int num_points = 0;
  int max_points; // the max possible, we may have less
  
  if(fscanf(ifp, "P:%d\n", &max_points) <= 0)
  {
    // no points here or corrupt file
    return false;   
  }
    
  int x, y;
  float val;
  while(true) // will break out when done
  {
    if(fscanf(ifp, "%d ", &x)  <= 0)
    {
      printf("problems, but still got %d points from file \n",num_points);
      return false;
    }
    if(x == -1)
    {
      //printf(" got all points \n");    
      break; 
    }
    
    if(fscanf(ifp, "%d ", &y)  <= 0)
    {
      printf("problems, but still got %d points from file \n",num_points);
      return false;
    }
        
    if(fscanf(ifp, "%f ", &val)  <= 0)
    {
      printf("problems, but still got %d points from file \n",num_points);
      return false;
    }
      
    PointSafeLookup[x][y] = val;
    num_points++;
  }
  
  num_points = 0;
  
  if(fscanf(ifp, "\nE:%d\n", &max_points) <= 0)
  {
    // no points here or corrupt file
    return false;   
  }
    
  int x2, y2;
  while(true) // will break out when done
  {
    if(fscanf(ifp, "%d ", &x)  <= 0)
    {
      printf("problems, but still got %d edges from file \n",num_points);
      return false;
    }
    if(x == -1)
    {
      //printf(" got all edges \n");    
      break; 
    }   
    
    if(fscanf(ifp, "%d ", &y)  <= 0)
    {
      printf("problems, but still got %d edges from file \n",num_points);
      return false;
    }
       
    if(fscanf(ifp, "%d ", &x2)  <= 0)
    {
      printf("problems, but still got %d edges from file \n",num_points);
      return false; 
    }
        
    if(fscanf(ifp, "%d ", &y2)  <= 0)
    {
      printf("problems, but still got %d edges from file \n",num_points);
      return false;
    }
    
    if(fscanf(ifp, "%f ", &val)  <= 0)
    {
      printf("problems, but still got %d edges from file \n",num_points);
      return false;
    }
      
    EdgeSafeLookup[x][y][x2][y2] = val;
    EdgeSafeLookup[x2][y2][x][y] = val;
    num_points++;
  }
  
  return true;
}

bool NavScene::SendPointsToFile(FILE* ofp)  // this writes point data to the file
{
  if(num_polygons == 0 || add_points_to_messages == 0) // none to send or we are not sending points
    return true;
            
  //printf("sending points to file \n");
  
  
  fprintf(ofp, "P:%d\n",n_points_ptr);  
  
  for(int i = 0; i < n_points_ptr; i++)
  { 
    fprintf(ofp, "%d %d %f ",last_n_points_x[i], last_n_points_y[i], last_n_points_val[i]);
  }
  fprintf(ofp, "-1 \n");
  n_points_ptr = 0;
          
  fprintf(ofp, "E:%d\n",n_edge_ptr);  
  for(int i = 0; i < n_edge_ptr; i++)
  {
    if(last_n_points_x[i] == -1)
    break;
    
    fprintf(ofp, "%d %d %d %d %f ",last_n_edges_x1[i], last_n_edges_y1[i], last_n_edges_x2[i], last_n_edges_y2[i], last_n_points_val[i]);
  }
  fprintf(ofp, "-1 \n");
  n_edge_ptr = 0;
  return true;      
}
