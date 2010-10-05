Cspace::Cspace() // default constructor    
{
  dims = 0;
  num_points = 0;
  start_ind = -1;
  best_total_path_length = LARGE;
  chop_tree = false;
  num_valid_points = 0;
}

Cspace::Cspace(vector<float> the_start, vector<float> the_goal, int dimensions) // constructor
{
  Populate(the_start, the_goal, dimensions);
}

Cspace::Cspace(const Cspace& C)   // copy constructor
{
  dims = C.dims;
  num_points = C.num_points;
          
  start.resize(dims);
  for(int i = 0; i < dims; i++)
    start[i] = C.start[i];
  
  goal.resize(dims);
  for(int i = 0; i < dims; i++)
    goal[i] = C.goal[i];
  
  ValidConfigs.resize(num_points);
  for(int i = 0; i < num_points; i++)
    for(int j = 0; j < dims; j++)
      ValidConfigs[i][j] = C.ValidConfigs[i][j];
  
  Neighbors.resize(num_points);
  for(int i = 0; i < num_points; i++)
  {
    int size_temp = Neighbors[i].size();
    for(int j = 0; j < size_temp; j++)
      Neighbors[i][j] = C.Neighbors[i][j];
  }
  
  SafeDistance.resize(num_points);
  for(int i = 0; i < num_points; i++)
    SafeDistance[i] = C.SafeDistance[i];  
  
  DistToGoal.resize(num_points);
  for(int i = 0; i < num_points; i++)
    DistToGoal[i] = C.DistToGoal[i]; 
  
  start_ind = C.start_ind;
  
  best_total_path_length = C.best_total_path_length;
  chop_tree = C.chop_tree;
  // might want to copy the workspace W also
  
  num_valid_points = C.num_valid_points;
  ValidInds.resize(num_valid_points);
  for(int i = 0; i < num_valid_points; i++)
    ValidInds[i] = C.ValidInds[i]; 
  
  ValidIndsInd.resize(num_points);
  for(int i = 0; i < num_points; i++)
    ValidIndsInd[i] = C.ValidIndsInd[i]; 
  
  MinDistToStart.resize(num_points);
  for(int i = 0; i < num_points; i++)
    MinDistToStart[i] = C.MinDistToStart[i];
  
  #ifdef use_kd_tree 
  printf("warning copy of KD_Tree not implimented !!!!!!!!!!!!!!!!!!!!!\n");
  #endif
}

Cspace::~Cspace() // destructor
{  
  #ifdef use_kd_tree 
  if(T!= NULL)
    delete T;        
  #endif
}

void Cspace::Populate(vector<float> the_start, vector<float> the_goal, int dimensions) // populates or re-populates the structure
{
  dims = dimensions;
  num_points = 0;
  
  // check to make sure both start and goal are valid configs
  if(W.PointValid(the_start) < 0)
    printf("error: invalid start configuration, %f \n", W.PointValid(the_start));

  float goal_dist = W.PointValid(the_goal);
  if(goal_dist < 0)
    printf("error: invalid goal configuration, %f \n",goal_dist);

  // save start and goal
  start.resize(dims);
  for(int i = 0; i < dims; i++)
    start[i] = the_start[i];
  
  goal.resize(dims);  
  for(int i = 0; i < dims; i++)
    goal[i] = the_goal[i];
  
  // add goal to tree
  ValidConfigs.resize(0);
  ValidConfigs.push_back(the_goal); 
  
  Neighbors.resize(1);
  Neighbors[0].push_back(0);
  SafeDistance.push_back(goal_dist);
  DistToGoal.push_back(0);
  MinDistToStart.push_back(W.Dist(the_goal,the_start));
  
  num_points = 1;
  start_ind = -1;
  best_total_path_length = LARGE;
  chop_tree = false;
    
  num_valid_points = 1;
  ValidInds.push_back(0);
  ValidIndsInd.push_back(0);
  
  #ifdef use_kd_tree 
  if(T!= NULL)
    delete T;        
  T = new KD_Tree;
  
  vector<float> V2Temp;
  redude_3_to_2(the_goal, V2Temp);
  T->insertNode(0, V2Temp);
  #endif
}

void Cspace::ChopTree()  // removes all nodes but root
{
  // add goal to tree
  ValidConfigs.resize(0);
  ValidConfigs.push_back(goal); 
  
  Neighbors.resize(0);
  Neighbors.resize(1);
  Neighbors[0].push_back(0);
  
  float goal_dist = W.PointValid(goal);
  SafeDistance.resize(0);
  SafeDistance.push_back(goal_dist);
  
  DistToGoal.resize(0);
  DistToGoal.push_back(0);
  
  MinDistToStart.resize(0);
  MinDistToStart.push_back(W.Dist(goal,start));
  
  num_points = 1;
  start_ind = -1;
  
  num_valid_points = 1;
  
  ValidInds.resize(0);
  ValidInds.push_back(0);
  
  ValidIndsInd.resize(0);
  ValidIndsInd.push_back(0); 
  
  #ifdef use_kd_tree 
  if(T!= NULL)
    delete T;        
  T = new KD_Tree;
  vector<float> V2Temp;
  redude_3_to_2(goal, V2Temp);
  T->insertNode(0, V2Temp);
  #endif
          
  printf("chopped tree \n");
}

int Cspace::AddNeighbor(const vector<float>& new_configuration, int neighbor_index, float safe_dist, float dist_to_goal, float est_dist_to_start)  // this adds the new point in configuration space as a neighbor of the point refered to by neighbor_index, returns the new index of the new configuration
{ 
  #ifdef save_time_data
    // record time stats
    clock_t now_time = clock();
    time_stats.push_back(difftime_clock(now_time,start_time));
    nodes_in_tree_stats.push_back(Cspc.num_valid_points);
    best_path_len_stats.push_back(Cspc.best_total_path_length);
    collision_checks_stats.push_back(total_collision_checks);
  #endif   
    
  if(num_valid_points < num_points) // then there is already space to insert
  { 
    //TreeOK();
      
    int new_point_index =  ValidInds[num_valid_points];
    //printf("new_point_index: %d \n", new_point_index);
    
    ValidConfigs[new_point_index] = new_configuration;
    
    Neighbors[neighbor_index].push_back(new_point_index);    
    
    //printf("neighbor_index: %d \n", neighbor_index);
    
    Neighbors[new_point_index].resize(1);
    Neighbors[new_point_index][0] = neighbor_index;
    
    SafeDistance[new_point_index] = safe_dist;    
    DistToGoal[new_point_index] = dist_to_goal;
    MinDistToStart[new_point_index] = est_dist_to_start;
            
    ValidInds[num_valid_points] = new_point_index;
    ValidIndsInd[new_point_index] = num_valid_points;
    num_valid_points++;  
    
    //TreeOK();
    
    if(uni_tree_build == 1)
    {
      MultAgSln.NodeID[MultAgSln.agent_id].push_back(new_point_index);
      MultAgSln.LastItAdded[MultAgSln.agent_id].push_back(0);
      MultAgSln.NodeIDInda[new_point_index] = MultAgSln.agent_id;
      MultAgSln.NodeIDIndb[new_point_index] = MultAgSln.total_nodes_added;
      MultAgSln.total_nodes_added++;

      //printf("added normal1: [%d %d]:%d\n", MultAgSln.NodeIDInda[new_point_index], 
      //                                      MultAgSln.NodeIDIndb[new_point_index],
      //                                      MultAgSln.NodeID[MultAgSln.NodeIDInda[new_point_index]][MultAgSln.NodeIDIndb[new_point_index]]);
    }
    
    #ifdef use_kd_tree
    vector<float> V2Temp;
    redude_3_to_2(new_configuration, V2Temp);
    T->insertNode(new_point_index, V2Temp);
    #endif
            
    return new_point_index;
  }
  else // num_valid_points == num_points
  { 
    ValidConfigs.push_back(new_configuration);
    Neighbors[neighbor_index].push_back(num_points);    
    vector<int> temp(1,neighbor_index);
    Neighbors.push_back(temp);
    SafeDistance.push_back(safe_dist);    
    DistToGoal.push_back(dist_to_goal);
    MinDistToStart.push_back(est_dist_to_start);
            
    ValidInds.push_back(num_points);
    ValidIndsInd.push_back(num_valid_points);
    
    if(uni_tree_build == 1)
    {
      MultAgSln.NodeID[MultAgSln.agent_id].push_back(num_points);
      MultAgSln.LastItAdded[MultAgSln.agent_id].push_back(0);
      MultAgSln.NodeIDInda.push_back(MultAgSln.agent_id);
      MultAgSln.NodeIDIndb.push_back(MultAgSln.total_nodes_added);
      
     // printf("added normal2: [%d %d]:%d\n", MultAgSln.NodeIDInda[num_points],
     //                                       MultAgSln.NodeIDIndb[num_points], 
     //                                       MultAgSln.NodeID[MultAgSln.NodeIDInda[num_points]][MultAgSln.NodeIDIndb[num_points]]);

      MultAgSln.total_nodes_added++;
    }
    
    num_points++;
    num_valid_points++;  
  
    #ifdef use_kd_tree
    vector<float> V2Temp;
    redude_3_to_2(new_configuration, V2Temp);
    T->insertNode(num_valid_points-1, V2Temp);
    #endif
            
    return num_valid_points-1;
  }
}

void Cspace::RemoveInd(int ind)                  // soft removes the ind
{  
    
  //printf("removing: %d\n", ind);
    
  if(ValidIndsInd[ind] >= num_valid_points) // alerady removed
  {
   return;
  }
  
  num_valid_points--;  
  
  if(num_valid_points <= 1)
  {
    num_valid_points = 1;
    return;
  }

  if(Neighbors[ind].size() > 0)
  {
    // need to remove this ind from its parent's list
    int parent_ind = Neighbors[ind][0];
    
    if(parent_ind >= 0)
    {
      int parent_n_number = Neighbors[parent_ind].size();
      if(parent_n_number > 1)
      {
        for(int i = 1; i < parent_n_number; i++)
        {
          if(Neighbors[parent_ind][i] == ind)
          {
            // swap with last element and then remove
            
            Neighbors[parent_ind][i] = Neighbors[parent_ind][parent_n_number-1];
            Neighbors[parent_ind].pop_back();  
            break;
          } 
        } 
      }
    }   
  }
  
  int ValidInds_index1 = ValidIndsInd[ind];  
  
  if(ValidInds_index1 < 0)
    printf(" problems 1, %d  %d\n",ValidInds_index1,ind);
  
  int ValidInds_index2 = num_valid_points; 

  if(ValidInds_index2 < 0)
    printf(" problems 2, %d  %d\n",ValidInds_index2, num_valid_points);
  
  int temp = ValidInds[ValidInds_index2];
  ValidInds[ValidInds_index2] = ValidInds[ValidInds_index1];
  ValidInds[ValidInds_index1] = temp;

  ValidIndsInd[ind] = ValidInds_index2;
  ValidIndsInd[temp] = ValidInds_index1;  
  
  Neighbors[ind].resize(0);
  DistToGoal[ind] = LARGE;
  
  if(uni_tree_build == 1)
  {   
    int key_a_removed = MultAgSln.NodeIDInda[ind];  
    int key_b_removed = MultAgSln.NodeIDIndb[ind];
    
    if(key_a_removed >= 0)
    {
      MultAgSln.NodeIDInda[ind] = -2;
      MultAgSln.NodeIDIndb[ind] = -2;
    
      MultAgSln.NodeID[key_a_removed][key_b_removed] = -2;
    }
  } 
}

void Cspace::RemoveAllDescendants(int ind)       // soft remove all descendants of ind and ind, mostly helps with next function
{    
  if(ind < 0)  // was already removed
    return;
    
  if(Neighbors[ind].size() > 1) // it has descendents to remove
  {         
    int num_neighbors = Neighbors[ind].size();
    for(int i = 1; i < num_neighbors; i++)
      RemoveAllDescendants(Neighbors[ind][i]);
  }
  
  if(ind != 0)  
    RemoveInd(ind);
}

int Cspace::PruneTreeFromInd(int ind)           // starting at ind, this follows backpointers until it finds the earliest node that takes more than the min path length to reach the goal, then it prunes it and all of its descendents and returns its parent.
{
  if(DistToGoal[ind]+MinDistToStart[ind] <= best_total_path_length)
    return ind; // can't prune because the dist from this node is less than or equal to the min path length found so far
      
  int this_ind = ind; 
  int next_ind = Neighbors[this_ind][0]; 
  while(DistToGoal[next_ind]+MinDistToStart[next_ind] > best_total_path_length)
  {
    this_ind = next_ind;
    if(this_ind == 0)
      break; // all the way to the end
    
    next_ind = Neighbors[this_ind][0]; 
  }
    
  // now this_ind is the earliest node on this branch that requires more than the min distance to reach the goal
  // so remove it and all of its descendants (note this is a soft remove)
  RemoveAllDescendants(this_ind);
  
  return next_ind;
}

int Cspace::ClosestNeighborOfTo(int neighbor_index, const vector<float>& new_configuration, float& closest_distance) // returns the closest neighbor of neighbor_index to the new_configuration, puts distance in closest_distance
{
  closest_distance = LARGE;
  int closest_neighbor = -1;
  int num_neighbors = Neighbors[neighbor_index].size();
  float this_dist;
  
  for(int i = 1; i < num_neighbors; i++)
  {
    this_dist = W.Dist(ValidConfigs[Neighbors[neighbor_index][i]], new_configuration);
    
    if(this_dist < closest_distance)
    {
      closest_distance = this_dist;
      closest_neighbor = Neighbors[neighbor_index][i];
    }
  }
  return closest_neighbor;
}

int Cspace::ClosestNeighborOfToWithinAngle(int neighbor_index, const vector<float>& new_configuration, float angle, float& closest_distance) // returns the closest neighbor of neighbor_index to the new_configuration that is withing angle of ne_configuration, puts distance in closest_distance
{
  closest_distance = LARGE;
  int closest_neighbor = -1;
  int num_neighbors = Neighbors[neighbor_index].size();
  float this_dist;
  float this_angular_dist;
  
  for(int i = 1; i < num_neighbors; i++)
  {
    this_dist = W.Dist(ValidConfigs[Neighbors[neighbor_index][i]], new_configuration);
    this_angular_dist = W.AngularDist(ValidConfigs[Neighbors[neighbor_index][i]], new_configuration);
    
    if(this_dist < closest_distance && this_angular_dist < angle)
    {
      closest_distance = this_dist;
      closest_neighbor = Neighbors[neighbor_index][i];
    }
  }
  return closest_neighbor;
}

int Cspace::PropogateCost(int i_index) // recursively propogates new cost to goal data from i_index to its descendants, returns the index of start node (if any exist) with shortest path, else returns -1 
{
  int ind_best_goal_n = -1;
  float best_goal_dist = LARGE;
  
  // if i_index is at start
  if(W.MaxDist(ValidConfigs[i_index], start) < SMALL)
  {        
    ind_best_goal_n = i_index;
    best_goal_dist = DistToGoal[i_index];
  }
  
  //printf("ns: %d \n", Neighbors[i_index].size());
  for(uint i = 1; i < Neighbors[i_index].size(); i++)
  {
    int this_ind = Neighbors[i_index][i];
    
    if(Neighbors[this_ind][0] != i_index || (Neighbors[this_ind][0] == this_ind && 0 != this_ind))
    {
      printf("problems %d \n", i_index);
      getchar();
    }
    
    DistToGoal[this_ind] = DistToGoal[i_index] + W.Dist(ValidConfigs[i_index], ValidConfigs[this_ind]);
    
    //printf("dist to goal: %f \n", DistToGoal[this_ind] );
    
    // propogate cost to neighbors
    int this_goal_n = PropogateCost(this_ind);
    
    if(this_goal_n > 0)
    {
      if(DistToGoal[this_goal_n] < best_goal_dist)   
      {    
        best_goal_dist = DistToGoal[this_goal_n];
        ind_best_goal_n = this_goal_n; 
      }
    }
  }
  //printf("++ g \n");
  //printf("++ g %d \n", ind_best_goal_n);
  return ind_best_goal_n;
}

bool Cspace::BuildTree(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution)  // this builds or continues to build the search tree. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal, in jumps no larger than move_max, returns true when it finds a better path
{  
  int best_node_to_start = 0;
  float best_dist_to_start = W.Dist(start,goal);
  
  float this_dist_to_obstacle_point = SafeDistance[0];
  float this_dist_to_obstacle_edge;
  float this_dist_to_start = best_dist_to_start;
  vector<float> new_configuration(dims, 0);
  
  clock_t now_t = clock();
  // while steps and time left
  while(steps != 0 && difftime_clock(now_t, start_t) < clock_to_plan) // this is the anytime loop 
  {
    // note: break out when nodes are close enough to attach to the start
      
    int inner_loop_closest_node_to_start = 0;
    float inner_loop_closest_dist_to_start = W.Dist(start,goal);

    while(dist_threshold < inner_loop_closest_dist_to_start) // this loop tries to find a single solution
    {   
      if(steps == 0 || difftime_clock(now_t, start_t) >= clock_to_plan) // no steps left or no time left        
        break;
      steps--;
    
      // randomly pick a node to expand from
      int this_rand = rand_int(0,num_valid_points-1);
      int neighbor_index = ValidInds[this_rand]; // even distribution
      
      //printf("neighbor_index first %d  %d \n", this_rand, neighbor_index);
      
      int closest_neighbor;
      float closest_distance;
      do
      { 
        // find earliest node that still has a chance of getting a better path, prune all descendance of that node that do not
        neighbor_index = PruneTreeFromInd(neighbor_index);
      
        // move in a random direction from that node a distance that is guarenteed to be safe, and that is less than max_move
        W.RandMove2(ValidConfigs[neighbor_index], new_configuration, SafeDistance[neighbor_index], move_max, theta_max, prob_at_goal, start);
        
        // check to make sure that it is 1/2 resolution away from all other neighbors of neighbor_index
        //closest_neighbor = ClosestNeighborOfTo(neighbor_index, new_configuration, closest_distance);
        closest_neighbor = ClosestNeighborOfToWithinAngle(neighbor_index, new_configuration, angular_resolution*.5, closest_distance); // same as above, but only looks for neighbors that are within angle of new_configuration
        
        if(closest_distance < resolution*.5) // it is not far enough away, so try expanding from the node it is too close to
        {
            neighbor_index = closest_neighbor;
        }
        
        now_t = clock();
      }while(closest_distance < resolution*.5 && difftime_clock(now_t, start_t) < clock_to_plan);
      
      /* 
      printf("got a possible new config: \n");
      print_float_vector(new_configuration); 
      getchar();
      */
        
      
      // test the new configuration for validity (endpoint)
      this_dist_to_obstacle_point = W.PointValid(new_configuration);  
      if(this_dist_to_obstacle_point <= 0) // not a valid point
      {
        now_t = clock(); 
        continue;
      }
      
      // test the new configuration for validity (edge)
      this_dist_to_obstacle_edge = W.EdgeValid(ValidConfigs[neighbor_index], new_configuration);  
      if(this_dist_to_obstacle_edge > 0) // this is a valid point
      {
        //TreeOK(); 
         
        float this_dist_to_goal = DistToGoal[neighbor_index] + W.Dist(ValidConfigs[neighbor_index], new_configuration);
        this_dist_to_start = W.Dist(new_configuration, start);
        
        if(this_dist_to_goal + this_dist_to_start < best_total_path_length)  // it is possible that a path could be found from here that is better than the best solution we currently have
        {
          // add the new configuration to the tree
          int last_ind_added = AddNeighbor(new_configuration, neighbor_index, this_dist_to_obstacle_point, this_dist_to_goal, this_dist_to_start);
          if(this_dist_to_start < inner_loop_closest_dist_to_start)
          {
            //printf(" this dist to start : %f \n",this_dist_to_start);
            inner_loop_closest_dist_to_start = this_dist_to_start;
            inner_loop_closest_node_to_start = last_ind_added;
          } 
        }
      }
      
      //print_float_vector(new_configuration);  
      now_t = clock();    
    }
    
    if(dist_threshold >= inner_loop_closest_dist_to_start) 
    {
      // we have a new solution, compare it to old solutions  
      float this_total_path_length = DistToGoal[inner_loop_closest_node_to_start]+W.Dist(ValidConfigs[inner_loop_closest_node_to_start], start);
      
      if(this_total_path_length < best_total_path_length)  // if the total path length of the new solution is less than that of the previous best
      {
        best_dist_to_start = inner_loop_closest_dist_to_start;
        best_node_to_start = inner_loop_closest_node_to_start;
        best_total_path_length = this_total_path_length;
        
        float start_safe_dist = W.PointValid(start);
        start_ind = AddNeighbor(start, best_node_to_start, start_safe_dist, this_total_path_length, 0); 
            
        if(steps < 0)
          printf("found a better path, length: %f (%d nodes in tree)\n", best_total_path_length, ValidConfigs.size());
        else
          printf("found a better path with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
      
        //TreeOK();
        
        return true;
      } 
    }
    
    now_t = clock(); 
  }
  
  //printf("exited build_tree without finding a better path \n");
  return false;
}


bool Cspace::BuildTreeV2(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution)  // this builds or continues to build the search tree. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal, in jumps no larger than move_max, returns true when it finds a better path, this version uses pruning when possible, A*-like re-linking when possible, and is otherwise based on an RRT
{
  // while steps and time left  
  bool not_added_start = true;
  
  while(not_added_start) // this loop tries to find a single solution, note: break out when the last added configureation is the start node (guarenteed to get one p % of the time based on experimental setup params)
  {   
    vector<float> new_configuration(dims, 0);
       
    clock_t now_t = clock();

    if(steps == 0 || difftime_clock(now_t, start_t) >= clock_to_plan) // no steps left or no time left      
    {
      break;
    }
    steps--;   
 
    if(attempt_random_path_improve == 1 && num_valid_points > 2)
    {    
      // pick a random node already in the tree, see if it can be rerouted through another node instead of its parent at less cost
      int ind_of_rn = ValidInds[rand_int(1,num_valid_points-1)];
      float rn_dist_to_goal = DistToGoal[ind_of_rn];
      float best_new_parent = -1;
      float best_new_dist_to_goal = rn_dist_to_goal;
              
      //printf("trying to improve: %d \n", ind_of_rn);
        
      int i = 0;
      while(i < num_valid_points)// && difftime_clock(now_t, start_t) < clock_to_plan)
      {   
        int i_index = ValidInds[i];
        float dist_through_i_index_to_goal = DistToGoal[i_index] + W.Dist(ValidConfigs[i_index], ValidConfigs[ind_of_rn]);
        
        if(best_new_dist_to_goal > dist_through_i_index_to_goal + SMALL) //it is better to go through the new node (if there are also no obstacles)
        { 
          // COMMENTED OUT BECAUSE WE ARE NOT WORRYING ABOUT ANGLES
          // need to check that angle is also ok
          //if(W.MaxAngularDist(new_configuration, ValidConfigs[i_index]) <= theta_max)
          //{

            // check to make sure this does not lead to collisions
            if(W.EdgeValid(ValidConfigs[i_index], ValidConfigs[ind_of_rn]) > 0)
            {  
              // no collisions, so save as best found so far  
              best_new_parent = i_index;
              best_new_dist_to_goal = dist_through_i_index_to_goal;         
            }
          //}
        }
      
        i++;
      }
      if(best_new_parent != -1)
      {        
        //printf("improvement for %d found !!\n", ind_of_rn);
        
        // if we are here then there is a better parrent for rn, so rerout
        int old_parent = Neighbors[ind_of_rn][0];
        Neighbors[ind_of_rn][0] = best_new_parent;
              
        // remove rn from neighbor list of old parent
        for(uint n = 1; n < Neighbors[old_parent].size(); n++)
        {
          if(Neighbors[old_parent][n] == ind_of_rn)
          {
            Neighbors[old_parent][n] = Neighbors[old_parent][Neighbors[old_parent].size()-1];
            Neighbors[old_parent].pop_back();
          } 
        }
   
        // add i_index to neighbor list of new parent
        Neighbors[best_new_parent].push_back(ind_of_rn);

        // update cost to goal info
        DistToGoal[ind_of_rn] = best_new_dist_to_goal; 
        
        // need to propogate better cost to goal info to all desendents, and during this we need to figure out if this leads to a better complete path
        int ind_of_start_node = PropogateCost(ind_of_rn);

        //TreeConsistant();
      
        if(ind_of_start_node > 0) // a start node was updated, therefore a better path may have been found
        {         
          //printf("a start node was updated \n");
                   
          // check if the updated start node represents an even better shortest path then what we know
          if(DistToGoal[ind_of_start_node] < best_total_path_length)
          {
            // the path to the updated start node is better than the old best solution
            not_added_start = false;
      
            start_ind = ind_of_start_node; // saves in class space
            best_total_path_length = DistToGoal[ind_of_start_node];          
 
            if(steps < 0)
            {
              printf("found a better path via shortcut, length: %f (%d nodes in tree)\n", best_total_path_length, num_valid_points);
            }
            else
            {
              printf("found a better path via shortcut with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
            }
 
            #ifdef using_smoothing        
            GreedyPathSmooth(start_ind);
            #endif 
            
            // remove nodes that can no longer help
            pruneTree();
                    
            //printf("tree 11 \n");
            //TreeOK();
            return true;  
          } 
        }
      }
    }
    
    // randomly pick a new configuration  
    W.RandMove3(new_configuration, prob_at_goal, start);
      
    // test the new configuration to make sure it could possible lead to a better solution given goal and start
    if(best_total_path_length < W.Dist(goal, new_configuration) + W.Dist(new_configuration, start))
    {
      // it cannot lead to a better solution
      now_t = clock(); 
      continue;  
    }
    
    // test the new configuration for validity (endpoint)
    float this_dist_to_obstacle_point = W.PointValid(new_configuration);  
             
    if(this_dist_to_obstacle_point <= 0) // not a valid point
    { 
      now_t = clock(); 
      continue;
    }
    
    // go through all nodes and find the node that helps the new one connect back to goal in the least ammount of distance (where there is a valid edge between them)
    int i = 0;
    bool break_again = false;
    float closest_dist_to_goal_so_far = LARGE;
    int closest_neighbor_to_goal_so_far = -1;
    float this_dist_to_goal;
    float this_dist_to_start = W.Dist(new_configuration, start);

    while(i < num_valid_points) // && difftime_clock(now_t, start_t) < clock_to_plan) // this loop attempts to find the best node in the tree to connect the new node to
    {    
      int i_index = ValidInds[i];
         
      // on the way through, check if the new node is within resolution/2 of any old nodes, if so, break and then continue on the next outer loop (pick a new new node to try)
      if(W.MaxDist(ValidConfigs[i_index], new_configuration) < resolution*0.5 && (W.Dist(new_configuration, start) > SMALL)) // don't want in here if new node is the goal
      {
        // COMMENTED OUT BECAUSE NOT WORRYING ABOUT ANGLES
        //if(W.AngularDist(ValidConfigs[i_index], new_configuration) < angular_resolution*0.5)
        //{
          
          //printf("within resolution \n");  
          break_again = true; 
          now_t = clock();
          break;
            
        //}
      }
      this_dist_to_goal = W.Dist(ValidConfigs[i_index], new_configuration) + DistToGoal[i_index];
        
      // if this could be the new best neighbor of the new node found so far
      if(this_dist_to_goal < closest_dist_to_goal_so_far)    //**** someday may want to include angular distance in here also somehow
      {    
        // check it is possible that a path could be found from here that is better than the best solution we currently have
        if(this_dist_to_goal + this_dist_to_start < best_total_path_length) 
        {
          // check if the two points can actually be connected         
          if(W.EdgeValid(ValidConfigs[i_index], new_configuration) > 0) // this is a valid connection
          {
             // it is the best neighbor found so far
            closest_dist_to_goal_so_far = this_dist_to_goal;
            closest_neighbor_to_goal_so_far = i_index;
          }
        }
      }
      i++;
      now_t = clock();
    }
    if(break_again)
    {
      // continuing because the new node was too close to an old node already in the graph
      continue;
    }
       
    if(closest_neighbor_to_goal_so_far < 0) // then no valid neighbors could be found
    {
      continue;
    }

    // now we know closest_neighbor_to_goal_so_far is the best old node to attach the new one to
    int last_ind_added;
    
    // add the new configuration to the tree   
    if(this_dist_to_start < SMALL)
    {
      last_ind_added = AddNeighbor(start, closest_neighbor_to_goal_so_far, this_dist_to_obstacle_point, closest_dist_to_goal_so_far, 0); 

      //printf("added start node to tree\n"); // recall that search happens backward from goal to start
      not_added_start = false;
    }
    else
    {
      last_ind_added = AddNeighbor(new_configuration, closest_neighbor_to_goal_so_far, this_dist_to_obstacle_point, closest_dist_to_goal_so_far, this_dist_to_start);
    } 

    // if we are here than a new node was just added at index last_ind_added
    
    float dist_from_new_node_to_goal = DistToGoal[last_ind_added];  
    if(not_added_start) // the node just added was not the start node
    {   
      // go through all nodes again, this time check if rerouting other nodes through the new node will help the other nodes get to the goal quicker

      i = 0;
      while(i < num_valid_points)// && difftime_clock(now_t, start_t) < clock_to_plan)
      {   
        int i_index = ValidInds[i];
        float dist_through_new_node_to_goal = dist_from_new_node_to_goal + W.Dist(new_configuration, ValidConfigs[i_index]);
        
        if(DistToGoal[i_index] > dist_through_new_node_to_goal) //it is better to go through the new node (if there are also no obstacles)
        { 
          // COMMENTED OUT BECAUSE WE ARE NOT WORRYING ABOUT ANGLES
          // need to check that angle is also ok
          //if(W.MaxAngularDist(new_configuration, ValidConfigs[i_index]) <= theta_max)
          //{

            // check to make sure that this does not lead to collisions
            if(W.EdgeValid(ValidConfigs[last_ind_added], ValidConfigs[i_index]) > 0)
            {      
              // no collision, so reroute through new node
              int old_parent = Neighbors[i_index][0];
              Neighbors[i_index][0] = last_ind_added;
              
              // remove i_index from neighbor list of old parent
              for(uint n = 1; n < Neighbors[old_parent].size(); n++)
              {
                if(Neighbors[old_parent][n] == i_index)
                {
                  Neighbors[old_parent][n] = Neighbors[old_parent][Neighbors[old_parent].size()-1];
                  Neighbors[old_parent].pop_back();
                } 
              }
   
              // add i_index to neighbor list of new node
              Neighbors[last_ind_added].push_back(i_index);

              // update cost to goal info
              DistToGoal[i_index] = dist_through_new_node_to_goal; 
            }
        //}
        }

        i++;
      }
      // need to propogate better cost to goal info to all desendents, and during this we need to figure out if this leads to a better complete path
      int ind_of_start_node = PropogateCost(last_ind_added);

      //TreeConsistant();
      
      if(ind_of_start_node > 0) // a start node was updated, therefore a better path may have been found
      {         
        //printf("a start node was updated \n");
                   
        // check if the updated start node represents an even better shortest path then what we know
        if(DistToGoal[ind_of_start_node] < best_total_path_length)
        {
          // the path to the updated start node is better than the old best solution
          not_added_start = false;
      
          start_ind = ind_of_start_node; // saves in class space
          best_total_path_length = DistToGoal[ind_of_start_node];          
 
          if(steps < 0)
          {
            //prinbest_dist_to_obstacle_edge_so_fartf("found a better path, length: %f (%d nodes in tree)\n", best_total_path_length, ValidConfigs.size());
            printf("found a better path via update, length: %f (%d nodes in tree)\n", best_total_path_length, num_valid_points);
          }
          else
          {
            printf("found a better path via update with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
          }
 
          #ifdef using_smoothing        
          GreedyPathSmooth(start_ind);
          #endif 
          
          // remove nodes that can no longer help
          pruneTree();
                  
          //printf("tree 11 \n");
          //TreeOK();
          return true;  
        } 
      }
    }
    else // the node just added was a start node
    {
      //printf("a start node was added \n"); 
         
      // check if the new start node represents an even better shortest path then what we know
      if(dist_from_new_node_to_goal < best_total_path_length)
      {
        // the path to the new start node is better than the old best solution
        not_added_start = false;
                    
        start_ind = last_ind_added; // saves in class space
        best_total_path_length = dist_from_new_node_to_goal;
              
        if(steps < 0)
          //printf("found a better path, length: %f (%d nodes in tree)\n", best_total_path_length, ValidConfigs.size());
          printf("found a better path, length: %f (%d nodes in tree) %u\n", best_total_path_length, num_valid_points, the_seed);
        else
          printf("found a better path with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
      
        #ifdef using_smoothing        
        GreedyPathSmooth(start_ind);
        #endif 
        
        // remove nodes that can no longer help
        pruneTree();
                
        //TreeOK();

        return true;
      }
    }
    now_t = clock(); 
  } // end of loop to find a single new and better solution
  
  return false;
}

bool Cspace::BuildTreeV3(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution)  // same as above, but attempts to connect goal on timeout
{
  // try to find a random solution, 
  bool found_solution = BuildTreeV2(start_t, clock_to_plan, steps, prob_at_goal, move_max, theta_max, resolution, angular_resolution);
  
  // try to connect goal
  int temp_steps = 1; // because prob = 0 that goal is added below
  clock_t temp_time = clock();
  bool connected_goal = BuildTreeV2(temp_time, clock_to_plan, temp_steps, 1, move_max, theta_max, resolution, angular_resolution);

  //if(found_solution)
  //  printf("found_solution \n");
  
  //  if(connected_goal)
  //  printf("connected_goal \n");
  
  if(found_solution || connected_goal)
    return true;
  
  return false;
}

bool Cspace::BuildTreeV4(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution)  // use fast kt for the first tree, then treev2
{
  // try to find a random solution, 
  bool found_solution;  
   
  if(best_total_path_length == LARGE)
  {
    found_solution = BuildRRTFast(start_t, clock_to_plan, steps, prob_at_goal, move_max, theta_max, resolution, angular_resolution);
    if(found_solution)
    {
      GreedyPathSmooth(start_ind);

      // remove nodes that can no longer help
      pruneTree();
    }
  }  
  else
    found_solution = BuildTreeV2(start_t, clock_to_plan, steps, prob_at_goal, move_max, theta_max, resolution, angular_resolution);
  
  
  return found_solution;
}


bool Cspace::BuildTreeV2rho(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution)  // this builds or continues to build the search tree. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal, in jumps no larger than move_max, returns true when it finds a better path, this version uses pruning when possible, A*-like re-linking when possible, and is otherwise based on an R, this tries to expand to the tree to a new node that is rand_move_dist toward the random node from the closest node
{
  // while steps and time left  
  bool not_added_start = true;
  
  while(not_added_start) // this loop tries to find a single solution, note: break out when the last added configureation is the start node (guarenteed to get one p % of the time based on experimental setup params)
  {   
    vector<float> new_configuration(dims, 0);
       
    clock_t now_t = clock();

    if(steps == 0 || difftime_clock(now_t, start_t) >= clock_to_plan) // no steps left or no time left      
    {
      break;
    }
    steps--;   

    // randomly pick a new configuration (completely randomly)
    W.RandMove3(new_configuration, prob_at_goal, start); // more like my version of the shortest path rrt than above
   
    float this_dist_to_obstacle_point;
    
    // go through all nodes and find the node that helps the new one connect back to goal in the least ammount of distance
    int i = 0;
    bool break_again = false;
    float closest_dist_to_goal_so_far = LARGE;
    int closest_neighbor_to_goal_so_far = -1;
    float this_dist_to_goal;
    float this_dist_to_start = W.Dist(new_configuration, start);
    
    while(i < num_valid_points) // && difftime_clock(now_t, start_t) < clock_to_plan) // this loop attempts to find the best node in the tree to connect the new node to
    {    
      int i_index = ValidInds[i];
         
      // on the way through, check if the new node is within resolution/2 of any old nodes, if so, break and then continue on the next outer loop (pick a new new node to try)
      if(W.MaxDist(ValidConfigs[i_index], new_configuration) < resolution*0.5 && (W.Dist(new_configuration, start) > SMALL)) // don't want in here if new node is the goal
      {
        // COMMENTED OUT BECAUSE NOT WORRYING ABOUT ANGLES
        //if(W.AngularDist(ValidConfigs[i_index], new_configuration) < angular_resolution*0.5)
        //{
          
          //printf("within resolution \n");  
          break_again = true; 
          now_t = clock();
          break;
            
        //}
      }
      this_dist_to_goal = W.Dist(ValidConfigs[i_index], new_configuration) + DistToGoal[i_index];
        
      // if this could be the new best neighbor of the new node found so far
      if(this_dist_to_goal < closest_dist_to_goal_so_far)    //**** someday may want to include angular distance in here also somehow
      {    
        // check it is possible that a path could be found from here that is better than the best solution we currently have
        if(this_dist_to_goal + this_dist_to_start < best_total_path_length) 
        {
          if(W.EdgeValid(ValidConfigs[i_index], new_configuration) > 0)
          {
            // it is the best neighbor found so far
            closest_dist_to_goal_so_far = this_dist_to_goal;
            closest_neighbor_to_goal_so_far = i_index;
          }
        }
      }
      i++;
      now_t = clock();
    }
    if(break_again)
    {
      // continuing because the new node was too close to an old node already in the graph
      continue;
    }
       
    if(closest_neighbor_to_goal_so_far < 0) // then no valid neighbors could be found
    {
      continue;
    }

    
    // now we know closest_neighbor_to_node_so_far is the best old node to move toward
    
    
    // make a new configuration that is rand_move_dist from best neighbor toward new_configuration
    vector<float> new_configuration2(dims, 0);
    W.MoveToward(ValidConfigs[closest_neighbor_to_goal_so_far], new_configuration, new_configuration2, rand_move_dist); // new_config is move_dist from old_config_from to old_config_to 

    float dist_to_node2 = W.Dist(ValidConfigs[closest_neighbor_to_goal_so_far], new_configuration2);
    
    // test the new configuration to make sure it could possibly lead to a better solution given goal and start
    if(best_total_path_length < W.Dist(goal, new_configuration2) + W.Dist(new_configuration2, start))
    {
      // it cannot lead to a better solution
      now_t = clock(); 
      continue;  
    }
    
    // test the new configuration for validity (endpoint)
    this_dist_to_obstacle_point = W.PointValid(new_configuration2);  
    
    if(this_dist_to_obstacle_point <= 0) // not a valid point
    { 
      now_t = clock(); 
      continue;
    }
    
    // test the new configuration for validity (edge)
    if(W.EdgeValid(ValidConfigs[closest_neighbor_to_goal_so_far], new_configuration2) <= 0)
    { 
      now_t = clock(); 
      continue;
    }
    
    
    this_dist_to_start = W.Dist(new_configuration2, start); 
    int last_ind_added;
    
    // add the new configuration to the tree   
    if(this_dist_to_start < SMALL)
    {
      last_ind_added = AddNeighbor(start, closest_neighbor_to_goal_so_far, this_dist_to_obstacle_point, dist_to_node2+DistToGoal[closest_neighbor_to_goal_so_far], 0); 

      //printf("added start node to tree\n"); // recall that search happens backward from goal to start
      not_added_start = false;
    }
    else
    {
      last_ind_added = AddNeighbor(new_configuration2, closest_neighbor_to_goal_so_far, this_dist_to_obstacle_point, dist_to_node2+DistToGoal[closest_neighbor_to_goal_so_far], this_dist_to_start);
    } 

    // if we are here than a new node was just added at index last_ind_added   
    float dist_from_new_node_to_goal = DistToGoal[last_ind_added];  
    if(not_added_start) // the node just added was not the start node
    {   
      // go through all nodes again, this time check if rerouting other nodes through the new node will help the other nodes get to the goal quicker
      i = 0;
      while(i < num_valid_points)// && difftime_clock(now_t, start_t) < clock_to_plan)
      {   
        int i_index = ValidInds[i];
        float dist_through_new_node_to_goal = dist_from_new_node_to_goal + W.Dist(new_configuration2, ValidConfigs[i_index]);
        
        if(DistToGoal[i_index] > dist_through_new_node_to_goal) //it is better to go through the new node (if there are also no obstacles)
        { 
          // COMMENTED OUT BECAUSE WE ARE NOT WORRYING ABOUT ANGLES
          // need to check that angle is also ok
          //if(W.MaxAngularDist(new_configuration2, ValidConfigs[i_index]) <= theta_max)
          //{

            // check to make sure that this does not lead to collisions
            if(W.EdgeValid(ValidConfigs[last_ind_added], ValidConfigs[i_index]) > 0)
            {      
              // no collision, so reroute through new node
              int old_parent = Neighbors[i_index][0];
              Neighbors[i_index][0] = last_ind_added;
              
              // remove i_index from neighbor list of old parent
              for(uint n = 1; n < Neighbors[old_parent].size(); n++)
              {
                if(Neighbors[old_parent][n] == i_index)
                {
                  Neighbors[old_parent][n] = Neighbors[old_parent][Neighbors[old_parent].size()-1];
                  Neighbors[old_parent].pop_back();
                } 
              }
   
              // add i_index to neighbor list of new node
              Neighbors[last_ind_added].push_back(i_index);

              // update cost to goal info
              DistToGoal[i_index] = dist_through_new_node_to_goal; 
            }
        //}
        }

        i++;
      }
      // need to propogate better cost to goal info to all desendents, and during this we need to figure out if this leads to a better complete path
      int ind_of_start_node = PropogateCost(last_ind_added);

      //TreeConsistant();
      
      if(ind_of_start_node > 0) // a start node was updated, therefore a better path may have been found
      {         
        //printf("a start node was updated \n");
                   
        // check if the updated start node represents an even better shortest path then what we know
        if(DistToGoal[ind_of_start_node] < best_total_path_length)
        {
          // the path to the updated start node is better than the old best solution
          not_added_start = false;
      
          start_ind = ind_of_start_node; // saves in class space
          best_total_path_length = DistToGoal[ind_of_start_node];          
 
          if(steps < 0)
          {
            //printf("found a better path, length: %f (%d nodes in tree)\n", best_total_path_length, ValidConfigs.size());
            printf("found a better path via update, length: %f (%d nodes in tree)\n", best_total_path_length, num_valid_points);
          }
          else
          {
            printf("found a better path via update with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
          }
 
          #ifdef using_smoothing        
          GreedyPathSmooth(start_ind);
          #endif 
          
          // remove nodes that can no longer help
          pruneTree();
                  
          //TreeOK();
          return true;  
        } 
      }
       
      if(attempt_random_path_improve == 1 && num_valid_points > 2)
      {    
        // pick a random node already in the tree, see if it can be rerouted through another node instead of its parent at less cost
        int ind_of_rn = ValidInds[rand_int(1,num_valid_points-1)];
        float rn_dist_to_goal = DistToGoal[ind_of_rn];
        float best_new_parent = -1;
        float best_new_dist_to_goal = rn_dist_to_goal;
              
        //printf("trying to improve: %d \n", ind_of_rn);
        
        i = 0;
        while(i < num_valid_points)// && difftime_clock(now_t, start_t) < clock_to_plan)
        {   
          int i_index = ValidInds[i];
          float dist_through_i_index_to_goal = DistToGoal[i_index] + W.Dist(ValidConfigs[i_index], ValidConfigs[ind_of_rn]);
        
          if(best_new_dist_to_goal > dist_through_i_index_to_goal + SMALL) //it is better to go through the new node (if there are also no obstacles)
          { 
            // COMMENTED OUT BECAUSE WE ARE NOT WORRYING ABOUT ANGLES
            // need to check that angle is also ok
            //if(W.MaxAngularDist(new_configuration2, ValidConfigs[i_index]) <= theta_max)
            //{

              // check to make sure this does not lead to collisions
              if(W.EdgeValid(ValidConfigs[i_index], ValidConfigs[ind_of_rn]) > 0)
              {  
                // no collisions, so save as best found so far  
                best_new_parent = i_index;
                best_new_dist_to_goal = dist_through_i_index_to_goal;
          
              }
            //}
          }
      
          i++;
        }
        if(best_new_parent == -1)
        {
          //printf("no improvement for %d found\n", ind_of_rn);
          continue;
        }
        
        //printf("improvement for %d found !!\n", ind_of_rn);
        
        // if we are here then there is a better parrent for rn, so rerout
        int old_parent = Neighbors[ind_of_rn][0];
        Neighbors[ind_of_rn][0] = best_new_parent;
              
        // remove rn from neighbor list of old parent
        for(uint n = 1; n < Neighbors[old_parent].size(); n++)
        {
          if(Neighbors[old_parent][n] == ind_of_rn)
          {
            Neighbors[old_parent][n] = Neighbors[old_parent][Neighbors[old_parent].size()-1];
            Neighbors[old_parent].pop_back();
          } 
        }
   
        // add i_index to neighbor list of new parent
        Neighbors[best_new_parent].push_back(ind_of_rn);

        // update cost to goal info
        DistToGoal[ind_of_rn] = best_new_dist_to_goal; 
        
        
       
        // need to propogate better cost to goal info to all desendents, and during this we need to figure out if this leads to a better complete path
        ind_of_start_node = PropogateCost(ind_of_rn);

        //TreeConsistant();
      
        if(ind_of_start_node > 0) // a start node was updated, therefore a better path may have been found
        {         
          //printf("a start node was updated \n");
                   
          // check if the updated start node represents an even better shortest path then what we know
          if(DistToGoal[ind_of_start_node] < best_total_path_length)
          {
            // the path to the updated start node is better than the old best solution
            not_added_start = false;
      
            start_ind = ind_of_start_node; // saves in class space
            best_total_path_length = DistToGoal[ind_of_start_node];          
 
            if(steps < 0)
            {
              //printf("found a better path, length: %f (%d nodes in tree)\n", best_total_path_length, ValidConfigs.size());
              printf("found a better path via shortcut, length: %f (%d nodes in tree)\n", best_total_path_length, num_valid_points);
            }
            else
            {
              printf("found a better path via shortcut with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
            }
 
            #ifdef using_smoothing        
            GreedyPathSmooth(start_ind);
            #endif 
            
            // remove nodes that can no longer help
            pruneTree();
                    
            //TreeOK();
            return true;  
          } 
        }
      }   
    }
    else // the node just added was a start node
    {
      //printf("a start node was added \n"); 
         
      // check if the new start node represents an even better shortest path then what we know
      if(dist_from_new_node_to_goal < best_total_path_length)
      {
        // the path to the new start node is better than the old best solution
        not_added_start = false;
                    
        start_ind = last_ind_added; // saves in class space
        best_total_path_length = dist_from_new_node_to_goal;
              
        if(steps < 0)
          //printf("found a better path, length: %f (%d nodes in tree)\n", best_total_path_length, ValidConfigs.size());
          printf("found a better path, length: %f (%d nodes in tree) %u\n", best_total_path_length, num_valid_points, the_seed);
        else
          printf("found a better path with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
      
        //TreeOK();
        
        #ifdef using_smoothing        
        GreedyPathSmooth(start_ind);
        #endif 
        
        // remove nodes that can no longer help
        pruneTree();
                
        return true;
      }
    }
    now_t = clock(); 
  } // end of loop to find a single new and better solution
  
  //printf("exited build_tree without finding a better path \n");

  return false;
}

bool Cspace::BuildTreeV3rho(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution)  // same as above, but attempts to connect goal on timeout
{
  // try to find a random solution, 
  bool found_solution = BuildTreeV2rho(start_t, clock_to_plan, steps, prob_at_goal, move_max, theta_max, resolution, angular_resolution);
  
  // try to connect goal
  int temp_steps = 1; // because prob = 0 that goal is added below
  clock_t temp_time = clock();
  bool connected_goal = BuildTreeV2rho(temp_time, clock_to_plan, temp_steps, 1, move_max, theta_max, resolution, angular_resolution);

  //if(found_solution)
  //  printf("found_solution \n");
  
  //  if(connected_goal)
  //  printf("connected_goal \n");
  
  if(found_solution || connected_goal)
    return true;
  
  return false;
}

bool Cspace::BuildRRT(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution)  // this builds or continues to build the search tree using an RRT. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal
{
  clock_t now_t = clock();
  
  if(chop_tree || difftime_clock(now_t, last_chop_t) >= chop_time_limit) // if want to chop tree
  {
    if(chop_tree)  
      chop_time_limit = chop_time_limit_mult*difftime_clock(now_t, last_chop_t);
    else
      chop_time_limit *= chop_time_limit_mult_b;
        
    // remove all nodes but root
    ChopTree();
    chop_tree = false;
    
    last_chop_t = clock();
  }
  
  // while steps and time left  
  bool not_added_start = true;
  
  while(not_added_start) // this loop tries to find a single solution, note: break out when the last added configureation is the start node (guarenteed to get one p % of the time based on experimental setup params)
  {   
    vector<float> new_configuration(dims, 0);
       
    clock_t now_t = clock();

    if(steps == 0 || difftime_clock(now_t, start_t) >= clock_to_plan) // no steps left or no time left      
    {
      break;
    }
    steps--;   
 
    // randomly pick a new configuration (completely randomly)
    W.RandMove3(new_configuration, prob_at_goal, start); // pick random point in space
    
    if(best_total_path_length != LARGE) // if not first tree
    {
      // test the new configuration to make sure it could possible lead to a better solution given goal and start
      if(best_total_path_length*rrt_improvement_ratio < W.Dist(goal, new_configuration) + W.Dist(new_configuration, start))
      {
        // it cannot lead to a better solution
        now_t = clock(); 
        continue;  
      }      
    }
    
    // test the new configuration for validity (endpoint)
    float this_dist_to_obstacle_point = W.PointValid(new_configuration);  
             
    if(this_dist_to_obstacle_point <= 0) // not a valid point
    { 
      now_t = clock(); 
      continue;
    }
    
    // go through all nodes and find the node that connects to the new one in the least ammount of distance
    int i = 0;
    bool break_again = false;
    float closest_dist_to_node_so_far = LARGE;
    int closest_neighbor_to_node_so_far = -1;
    float this_dist_to_node;
    float this_dist_to_start = W.Dist(new_configuration, start);
    
    while(i < num_valid_points) // && difftime_clock(now_t, start_t) < clock_to_plan) // this loop attempts to find the best node in the tree to connect the new node to
    {    
      int i_index = ValidInds[i];
      
      // on the way through, check if the new node is within resolution/2 of any old nodes, if so, break and then continue on the next outer loop (pick a new new node to try)
      if(W.MaxDist(ValidConfigs[i_index], new_configuration) < resolution*0.5 && (W.Dist(new_configuration, start) > SMALL)) // don't want in here if new node is the goal
      {
        // COMMENTED OUT BECAUSE NOT WORRYING ABOUT ANGLES
        //if(W.AngularDist(ValidConfigs[i_index], new_configuration) < angular_resolution*0.5)
        //{
          
          //printf("within resolution \n");  
          break_again = true; 
          now_t = clock();
          break;
            
        //}
      }
      this_dist_to_node = W.Dist(ValidConfigs[i_index], new_configuration);
        
      // if this could be the new best neighbor of the new node found so far
      if(this_dist_to_node < closest_dist_to_node_so_far)    //**** someday may want to include angular distance in here also somehow
      {  
        if(best_total_path_length != LARGE) // if not first tree
        {
          // make sure the new point could possibly lead to a better solution based on dist to goal through neighbor and new dist to neighbor
          if(DistToGoal[i_index] + this_dist_to_node + this_dist_to_start < best_total_path_length*rrt_improvement_ratio)  // it is possible that a path could be found from here that is better than the best solution we currently have
          {
            // check if the two points can actually be connected         
            if(W.EdgeValid(ValidConfigs[i_index], new_configuration) > 0) // this is a valid connection
            {
              // it is the best neighbor found so far
              closest_dist_to_node_so_far = this_dist_to_node;
              closest_neighbor_to_node_so_far = i_index;
            }
          }
        }
        else // first tree
        {
          // check if the two points can actually be connected         
          if(W.EdgeValid(ValidConfigs[i_index], new_configuration) > 0) // this is a valid connection
          {
            // it is the best neighbor found so far
            closest_dist_to_node_so_far = this_dist_to_node;
            closest_neighbor_to_node_so_far = i_index;
          }
        }
      }
      i++;
      now_t = clock();
    }
    if(break_again)
    {
      // continuing because the new node was too close to an old node already in the graph
      continue;
    }
       
    if(closest_neighbor_to_node_so_far < 0) // then no valid neighbors could be found
    {
      continue;
    }
    
    // now we know closest_neighbor_to_node_so_far is the best old node to attach the new one to
    int last_ind_added;
       
    // add the new configuration to the tree   
    if(this_dist_to_start < SMALL)
    {
      last_ind_added = AddNeighbor(start, closest_neighbor_to_node_so_far, this_dist_to_obstacle_point, closest_dist_to_node_so_far+DistToGoal[closest_neighbor_to_node_so_far], 0); 

      //printf("added start node to tree\n"); // recall that search happens backward from goal to start
      not_added_start = false;
    }
    else
    {
      last_ind_added = AddNeighbor(new_configuration, closest_neighbor_to_node_so_far, this_dist_to_obstacle_point, closest_dist_to_node_so_far+DistToGoal[closest_neighbor_to_node_so_far], this_dist_to_start);
    } 
    
    float dist_from_new_node_to_goal = DistToGoal[last_ind_added];  
    if(not_added_start) // the node just added was not the start node
    {   
      // do nothing
    }
    else // the node just added was a start node
    {
      //printf("a start node was added \n"); 
         
      // check if the new start node represents an even better shortest path then what we know
      if(dist_from_new_node_to_goal < best_total_path_length*rrt_improvement_ratio)
      {
        // the path to the new start node is better than the old best solution
        not_added_start = false;
                    
        start_ind = last_ind_added; // saves in class space
        best_total_path_length = dist_from_new_node_to_goal;
              
        if(steps < 0)
          //printf("found a better path, length: %f (%d nodes in tree)\n", best_total_path_length, ValidConfigs.size());
          printf("found a better path, length: %f (%d nodes in tree) %u\n", best_total_path_length, num_valid_points, the_seed);
        else
          printf("found a better path with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
      
        #ifdef using_smoothing        
        GreedyPathSmooth(start_ind);
        #endif 
        
        //TreeOK();
                
        chop_tree = true; // remember to chop tree next time
        return true;
      }
    }
    now_t = clock(); 
  } // end of loop to find a single new and better solution
  
  //printf("exited build_tree without finding a better path \n");

  return false;
}

bool Cspace::BuildRRTRho(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution)  // this builds or continues to build the search tree using an RRT. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal, after a random node is chosen and the closest tree node found, this tries to expand to the tree to a new node that is rand_move_dist toward the random node from the closest node
{
  clock_t now_t = clock();
  
  if(chop_tree || difftime_clock(now_t, last_chop_t) >= chop_time_limit) // if want to chop tree
  {
    if(chop_tree)  
      chop_time_limit = chop_time_limit_mult*difftime_clock(now_t, last_chop_t);
    else
      chop_time_limit *= chop_time_limit_mult_b;
    
    // remove all nodes but root
    ChopTree();
    chop_tree = false;
    
    last_chop_t = clock();
  }
  
  // while steps and time left  
  bool not_added_start = true;
  
  while(not_added_start) // this loop tries to find a single solution, note: break out when the last added configureation is the start node (guarenteed to get one p % of the time based on experimental setup params)
  {   
    vector<float> new_configuration(dims, 0);
       
    clock_t now_t = clock();

    if(steps == 0 || difftime_clock(now_t, start_t) >= clock_to_plan) // no steps left or no time left      
    {
      break;
    }
    steps--;   
 
    // randomly pick a new configuration (completely randomly)
    W.RandMove3(new_configuration, prob_at_goal, start); // pick random point in space
    
    
    float this_dist_to_obstacle_point;  

//     // in furgeson paper this was done, I think its kind of funny becaue  what we should care about is if the node to extend to is valid, not the random node that is is on the way toward
//     if(best_total_path_length != LARGE) // if not first tree
//     {
//       // test the new configuration to make sure it could possible lead to a better solution given goal and start
//       if(best_total_path_length*rrt_improvement_ratio < W.Dist(goal, new_configuration) + W.Dist(new_configuration, start))
//       {
//         // it cannot lead to a better solution
//         now_t = clock(); 
//         continue;  
//       }      
//     }
//     
//     // test the new configuration for validity (endpoint)
//     float this_dist_to_obstacle_point = W.PointValid(new_configuration);  
//              
//     if(this_dist_to_obstacle_point <= 0) // not a valid point
//     { 
//       now_t = clock(); 
//       continue;
//     }
    
    
    // go through all nodes and find the node that connects to the new one in the least ammount of distance
    int i = 0;
    float closest_dist_to_node_so_far = LARGE;
    int closest_neighbor_to_node_so_far = -1;
    float this_dist_to_node;
    float this_dist_to_start = W.Dist(new_configuration, start);
    
    #ifdef use_kd_tree      
    float temp_dist_ = LARGE;
    vector<float> V2Temp;
    redude_3_to_2(new_configuration, V2Temp);
    KD_Node* nn = T->findNearest(V2Temp, T->Root, temp_dist_, 0);
   
    if(nn == NULL)
      continue;
    
    this_dist_to_node = W.Dist(ValidConfigs[nn->ind], new_configuration);
    closest_dist_to_node_so_far = this_dist_to_node; // kind of messy, but don't want to break things
    closest_neighbor_to_node_so_far = nn->ind;
        
    #else       
    while(i < num_valid_points)  // this loop attempts to find the best node in the tree to connect the new node to
    {    
      int i_index = ValidInds[i];
      
      this_dist_to_node = W.Dist(ValidConfigs[i_index], new_configuration);
        
      // if this could be the new best neighbor of the new node found so far
      if(this_dist_to_node < closest_dist_to_node_so_far)    //**** someday may want to include angular distance in here also somehow
      {  
        closest_dist_to_node_so_far = this_dist_to_node;
        closest_neighbor_to_node_so_far = i_index;
      }
      i++;
      now_t = clock();
    }
    #endif
   
    if(closest_neighbor_to_node_so_far < 0) // then no valid neighbors could be found
    {
      continue;
    }

    if(best_total_path_length != LARGE) // if not first tree
    {
      // make sure the new point could possibly lead to a better solution based on dist to goal through neighbor and new dist to neighbor
      if(DistToGoal[closest_neighbor_to_node_so_far] + closest_dist_to_node_so_far + this_dist_to_start > best_total_path_length*rrt_improvement_ratio)  // it is not possible that a path could be found from here that is better than the best solution we currently have
        continue;
    }
    
    
    // now we know closest_neighbor_to_node_so_far is the best old node to move toward

    //check if the new node is within resolution/2 of the old node
    if(closest_dist_to_node_so_far < resolution*0.5 && (W.Dist(new_configuration, start) > SMALL)) // don't want in here if new node is the goal
    {
      continue;
    }

    // make a new configuration that is rand_move_dist from best neighbor toward new_configuration
    vector<float> new_configuration2(dims, 0);
    W.MoveToward(ValidConfigs[closest_neighbor_to_node_so_far], new_configuration, new_configuration2, rand_move_dist); // new_config is move_dist from old_config_from to old_config_to 

    float dist_to_node2 = W.Dist(ValidConfigs[closest_neighbor_to_node_so_far], new_configuration2);
    
    
    if(best_total_path_length != LARGE) // if not first tree
    {
      // test the new configuration to make sure it could possible lead to a better solution given goal and start
      if(best_total_path_length*rrt_improvement_ratio < W.Dist(goal, new_configuration2) + W.Dist(new_configuration2, start))
      {
        // it cannot lead to a better solution
        now_t = clock(); 
        continue;  
      }      
    }
    
    // test the new configuration for validity (endpoint)
    this_dist_to_obstacle_point = W.PointValid(new_configuration2);  
             
    if(this_dist_to_obstacle_point <= 0) // not a valid point
    { 
      now_t = clock(); 
      continue;
    }
    
    // test the new configuration for validity (edge)
    if(W.EdgeValid(ValidConfigs[closest_neighbor_to_node_so_far], new_configuration2) <= 0)
    { 
      now_t = clock(); 
      continue;
    }
    
    this_dist_to_start = W.Dist(new_configuration2, start); 
    int last_ind_added; 
    
    // add the new configuration to the tree   
    if(this_dist_to_start < SMALL)
    {
      last_ind_added = AddNeighbor(start, closest_neighbor_to_node_so_far, this_dist_to_obstacle_point, dist_to_node2+DistToGoal[closest_neighbor_to_node_so_far], 0); 
      
      //printf("added start node to tree\n"); // recall that search happens backward from goal to start
      not_added_start = false;
    }
    else
    {
      last_ind_added = AddNeighbor(new_configuration2, closest_neighbor_to_node_so_far, this_dist_to_obstacle_point, dist_to_node2+DistToGoal[closest_neighbor_to_node_so_far], this_dist_to_start); 
    } 
    
    float dist_from_new_node_to_goal = DistToGoal[last_ind_added];  
    if(not_added_start) // the node just added was not the start node
    {   
      // do nothing
    }
    else // the node just added was a start node
    {
      //printf("a start node was added \n"); 
         
      // check if the new start node represents an even better shortest path then what we know
      if(dist_from_new_node_to_goal < best_total_path_length*rrt_improvement_ratio)
      {
        // the path to the new start node is better than the old best solution
        not_added_start = false;
                    
        start_ind = last_ind_added; // saves in class space
        best_total_path_length = dist_from_new_node_to_goal;
              
        if(steps < 0)
          //printf("found a better path, length: %f (%d nodes in tree)\n", best_total_path_length, ValidConfigs.size());
          printf("found a better path, length: %f (%d nodes in tree) %u\n", best_total_path_length, num_valid_points, the_seed);
        else
          printf("found a better path with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
      
        //TreeOK();
        
        #ifdef using_smoothing        
        GreedyPathSmooth(start_ind);
        #endif 
                
        chop_tree = true; // remember to chop tree next time
                
        return true;
      }
    }
    now_t = clock(); 
  } // end of loop to find a single new and better solution
  
  //printf("exited build_tree without finding a better path \n");

  return false;
}    

bool Cspace::BuildRRTFast(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution)  //Same as build RRT but collisions are checked after closest node is found    
{
  clock_t now_t = clock();
  
  if(chop_tree || difftime_clock(now_t, last_chop_t) >= chop_time_limit) // if want to chop tree
  {
    if(chop_tree)  
      chop_time_limit = chop_time_limit_mult*difftime_clock(now_t, last_chop_t);  
    else
      chop_time_limit *= chop_time_limit_mult_b;
    
    // remove all nodes but root
    ChopTree();
    chop_tree = false;
    
    last_chop_t = clock();
  }
  
  // while steps and time left  
  bool not_added_start = true;
  
  while(not_added_start) // this loop tries to find a single solution, note: break out when the last added configureation is the start node (guarenteed to get one p % of the time based on experimental setup params)
  {   
    vector<float> new_configuration(dims, 0);
       
    clock_t now_t = clock();

    if(steps == 0 || difftime_clock(now_t, start_t) >= clock_to_plan) // no steps left or no time left      
    {
      break;
    }
    steps--;   

    // randomly pick a new configuration (completely randomly)
    W.RandMove3(new_configuration, prob_at_goal, start); // pick random point in space
    
    
    float this_dist_to_obstacle_point;  

    // go through all nodes and find the node that connects to the new one in the least ammount of distance
    int i = 0;
    float closest_dist_to_node_so_far = LARGE;
    int closest_neighbor_to_node_so_far = -1;
    float this_dist_to_node;
    float this_dist_to_start = W.Dist(new_configuration, start);
    
    #ifdef use_kd_tree      
    float temp_dist_ = LARGE;
    vector<float> V2Temp;
    redude_3_to_2(new_configuration, V2Temp);
    KD_Node* nn = T->findNearest(V2Temp, T->Root, temp_dist_, 0);
   
    if(nn == NULL)
      continue;
    
    this_dist_to_node = W.Dist(ValidConfigs[nn->ind], new_configuration);
    closest_dist_to_node_so_far = this_dist_to_node; // kind of messy, but don't want to break things
    closest_neighbor_to_node_so_far = nn->ind;
        
    #else       
    while(i < num_valid_points)  // this loop attempts to find the best node in the tree to connect the new node to
    {    
      int i_index = ValidInds[i];
         
      this_dist_to_node = W.Dist(ValidConfigs[i_index], new_configuration);
        
      // if this could be the new best neighbor of the new node found so far
      if(this_dist_to_node < closest_dist_to_node_so_far)    //**** someday may want to include angular distance in here also somehow
      {  
        closest_dist_to_node_so_far = this_dist_to_node;
        closest_neighbor_to_node_so_far = i_index;
      }
      i++;
      now_t = clock();
    }
    #endif
       
    if(closest_neighbor_to_node_so_far < 0) // then no valid neighbors could be found
    {
      continue;
    }
    
    if(best_total_path_length != LARGE) // if not first tree
    {
      // make sure the new point could possibly lead to a better solution based on dist to goal through neighbor and new dist to neighbor
      if(DistToGoal[closest_neighbor_to_node_so_far] + closest_dist_to_node_so_far + this_dist_to_start > best_total_path_length*rrt_improvement_ratio)  // it is not possible that a path could be found from here that is better than the best solution we currently have
        continue;
    }
    
    // now we know closest_neighbor_to_node_so_far is the best old node to move toward

    float dist_to_node2 = W.Dist(ValidConfigs[closest_neighbor_to_node_so_far], new_configuration);
    
    
    if(best_total_path_length != LARGE) // if not first tree
    {
      // test the new configuration to make sure it could possible lead to a better solution given goal and start
      if(best_total_path_length*rrt_improvement_ratio < W.Dist(goal, new_configuration) + W.Dist(new_configuration, start))
      {
        // it cannot lead to a better solution
        now_t = clock(); 
        continue;  
      }      
    }
    
    // test the new configuration for validity (endpoint)
    this_dist_to_obstacle_point = W.PointValid(new_configuration);  
             
    if(this_dist_to_obstacle_point <= 0) // not a valid point
    { 
      now_t = clock(); 
      continue;
    }

    // test the new configuration for validity (edge)
    if(W.EdgeValid(ValidConfigs[closest_neighbor_to_node_so_far], new_configuration) <= 0)
    { 
      now_t = clock(); 
      continue;
    }
    
    this_dist_to_start = W.Dist(new_configuration, start); 
    int last_ind_added; 
    
    // add the new configuration to the tree   
    if(this_dist_to_start < SMALL)
    {
      last_ind_added = AddNeighbor(start, closest_neighbor_to_node_so_far, this_dist_to_obstacle_point, dist_to_node2+DistToGoal[closest_neighbor_to_node_so_far], 0); 
      
      //printf("added start node to tree\n"); // recall that search happens backward from goal to start
      not_added_start = false;
    }
    else
    {
      last_ind_added = AddNeighbor(new_configuration, closest_neighbor_to_node_so_far, this_dist_to_obstacle_point, dist_to_node2+DistToGoal[closest_neighbor_to_node_so_far], this_dist_to_start);
    } 
    
    float dist_from_new_node_to_goal = DistToGoal[last_ind_added];  
    if(not_added_start) // the node just added was not the start node
    {   
      // do nothing
    }
    else // the node just added was a start node
    {
      //printf("a start node was added \n"); 
         
      // check if the new start node represents an even better shortest path then what we know
      if(dist_from_new_node_to_goal < best_total_path_length*rrt_improvement_ratio)
      {
        // the path to the new start node is better than the old best solution
        not_added_start = false;
                    
        start_ind = last_ind_added; // saves in class space
        best_total_path_length = dist_from_new_node_to_goal;
              
        if(steps < 0)
          //printf("found a better path, length: %f (%d nodes in tree)\n", best_total_path_length, ValidConfigs.size());
          printf("found a better path, length: %f (%d nodes in tree) %u\n", best_total_path_length, num_valid_points, the_seed);
        else
          printf("found a better path with %d iterations left, length: %f (%d nodes in tree)\n", steps, best_total_path_length, ValidConfigs.size());
      
        //TreeOK();
        
        #ifdef using_smoothing        
        GreedyPathSmooth(start_ind);
        #endif 
                
        chop_tree = true; // remember to chop tree next time
                
        return true;
      }
    }
    now_t = clock(); 
  } // end of loop to find a single new and better solution
  
  //printf("exited build_tree without finding a better path \n");

  return false;
} 

void Cspace::pruneTree() // go through the list, prune any nodes that can no longer lead to better any-time solutions 
{   
  int i = 0;
  while(i < num_valid_points) // && difftime_clock(now_t, start_t) < clock_to_plan) // this loop attempts to find the best node in the tree to connect the new node to
  {             
    int i_index = ValidInds[i];
          
    //printf("%d %d\n", i, num_valid_points);
      
    while((DistToGoal[i_index] + W.Dist(ValidConfigs[i_index], start)) > best_total_path_length )
    {   
      //printf("removing point %d\n", i_index); 
      RemoveAllDescendants(i_index);
           
      if(i >= num_valid_points || i == 0)
      {
        break;
      }
      i_index = ValidInds[i];
    }
    if(i == num_valid_points)
    {
      break;
    }
    i++;
  }
}

bool Cspace::GreedyPathSmooth(int start_smooth_ind) // starting at start_smooth_ind this attempts to greedily decrease the length of the path by shortcutting to nodes further along, returns true if path has been shortened
{
  bool found_shorter_path = false;
          
  // backtrack from start_smooth_ind to goal and store nodes in a list 
    
  vector<float> original_path_sequence(1, start_smooth_ind);
    
  int ind = start_smooth_ind;
  while(ind > 0)
  {
    ind = Neighbors[ind][0];
    original_path_sequence.push_back(ind);
  }
          
  if(ind != 0)
  {
     printf("problems smoothing, didn't find goal \n");
     getchar();
  }
  
  int front_list_ind = 0;  // this will change as we move through the list
  int front_list_node;
  int back_list_ind = original_path_sequence.size() - 1; // this will keep this value as we move through the list
  int list_node;
  int last_updated_cost_node = -1;
  
  // move from front_list_ind through list toward back_list_end
  while(front_list_ind < back_list_ind)
  {
    front_list_node = original_path_sequence[front_list_ind];
    
    // move through list from back_list_ind toward front_list_ind
    ind = back_list_ind;
    while(ind > front_list_ind+1)
    {
      list_node = original_path_sequence[ind];  
      
      // check if a valid edge exists between front_list_node and list_node
      if(W.EdgeValid(ValidConfigs[front_list_node], ValidConfigs[list_node]) > 0) // this is a valid connection
      { 
        // reroute front_list_node to have list_node as its parent
        
        int old_parent = Neighbors[front_list_node][0];
        Neighbors[front_list_node][0] = list_node;
              
        // remove front_list_node from neighbor list of old parent
        for(uint n = 1; n < Neighbors[old_parent].size(); n++)
        {
          if(Neighbors[old_parent][n] == front_list_node)
          {
            Neighbors[old_parent][n] = Neighbors[old_parent][Neighbors[old_parent].size()-1];
            Neighbors[old_parent].pop_back();
          } 
        }
   
        // add front_list_node to neighbor list of list_node
        Neighbors[list_node].push_back(front_list_node);

        // update cost to goal info
        DistToGoal[front_list_node] = DistToGoal[list_node] + W.Dist(ValidConfigs[front_list_node], ValidConfigs[list_node]);
          
        // remember the node closest to the goal in the search tree that has had its cost updated during the smoothing process 
        last_updated_cost_node = front_list_node;
          
        // set front_list_ind to ind and reset ind to back_list_ind
        front_list_ind = ind-1; // -1 because gets incrimented below
        found_shorter_path = true;
        break;
      }  
       
      ind--;
    }
    front_list_ind++; 
  }      
  
  if(found_shorter_path)
  {
    // need to update costs (in the rest of this if statement ind referes to ind in ValidCongfigs[])
      
    // need to propogate better cost to goal info to all desendents
    int ind_of_start_node = PropogateCost(last_updated_cost_node);

    //TreeConsistant();
      
    if(ind_of_start_node > 0) // a start node was updated, therefore a better path may have been found (this should always happen)
    {         
      //printf("a start node was updated \n");
                   
      // check if the updated start node represents an even better shortest path then what we know
      if(DistToGoal[ind_of_start_node] < best_total_path_length)
      {
        // the path to the updated start node is better than the old best solution
      
        start_ind = ind_of_start_node; // saves in class space
        best_total_path_length = DistToGoal[ind_of_start_node];          
 
        printf("found a better path via smooth, length: %f (%d nodes in tree)\n", best_total_path_length, num_valid_points);
        
        //printf("tree 11 \n");
        //TreeOK();
      } 
    }   
  }
  return found_shorter_path;
}

float Cspace::PathLength(int node_index)        // returns the length from the node at node_index to the root of the tree
{
  float total_len = 0;
  int this_node = node_index;
  int next_node = Neighbors[this_node][0];
  
  while(next_node > 0)
  {
    total_len += W.Dist(ValidConfigs[this_node],ValidConfigs[next_node]);
    this_node = next_node;
    next_node = Neighbors[this_node][0]; 
  }
  total_len += W.Dist(ValidConfigs[this_node],ValidConfigs[next_node]);
  
  return total_len;
}


bool Cspace::TreeOK()                           // returns true if the tree is OK
{
 int this_valid_point;
 for(int i = 0; i < num_valid_points; i++)
 {
   this_valid_point = ValidInds[i];
   
   if(ValidIndsInd[this_valid_point] != i)
   {
     printf("ValidIndsInds[this_valid_point] != i :: %d != %d , %d \n", ValidIndsInd[this_valid_point], i, this_valid_point);
     getchar();
     return false;
   }
           
   for(uint j = 0; j < Neighbors[this_valid_point].size(); j++)
   {
     if(Neighbors[this_valid_point][j] == this_valid_point && this_valid_point != 0)
     {
       printf("Neighbors[this_valid_point][j] == this_valid_point, %d",this_valid_point);
       getchar();
       return false;
     }    
   }
 }
 return true;
}

bool Cspace::TreeTriangle()  // returns true if the tree obeys triangle inequality between node->parent->grandparent
{
  for(int i = 0; i < num_valid_points; i++)
  {
    int ind = ValidInds[i];     
    
    if(ind < 1)
        continue; // its the goal or invalid
    
    if(Neighbors[ind][0] < 1)
        continue; // its parent is the goal or invalid
    
    int parent_ind = Neighbors[ind][0];
    int grand_parent_ind = Neighbors[parent_ind][0];
    
    if(W.Dist(ValidConfigs[ind], ValidConfigs[grand_parent_ind]) <  W.Dist(ValidConfigs[ind], ValidConfigs[parent_ind]) + W.Dist(ValidConfigs[parent_ind], ValidConfigs[grand_parent_ind])) // this does not obey triangle inequality
    {
      // make sure there is an obstacle between ind and grand_parent  
      if(W.EdgeValid(ValidConfigs[ind], ValidConfigs[grand_parent_ind]) > 0) // this is a valid connection, which shouldn't happen
      {
        printf("does not obey triangle inequality !!! [%d -> %d -> %d] \n", ind, parent_ind, grand_parent_ind);
        getchar();
        return false;
      }
    }
  }
  return true;
    
}

bool Cspace::TreeConsistant()  // returns true if the tree is consistant (i.e. parent->goal + dist_to_parent = node->goal)
{
  for(int i = 1; i < num_valid_points; i++)
  {
    int ind = ValidInds[i];     
    
    int parent_ind = Neighbors[ind][0];
    
    if(fabs(W.Dist(ValidConfigs[ind], ValidConfigs[parent_ind]) + DistToGoal[parent_ind] - DistToGoal[ind]) > SMALL) // this does not obey dist to goal stuff
    {
      printf("does not obey goal distance !!! [%d -> %d]:: %f != %f + %f = %f ::: %f\n", ind, parent_ind, DistToGoal[ind], W.Dist(ValidConfigs[ind], ValidConfigs[parent_ind]), DistToGoal[parent_ind], W.Dist(ValidConfigs[ind], ValidConfigs[parent_ind]) + DistToGoal[parent_ind], fabs(W.Dist(ValidConfigs[ind], ValidConfigs[parent_ind]) + DistToGoal[parent_ind] - DistToGoal[ind]));
      getchar();
      return false;
    }
  }
  return true;     
}

bool Cspace::TreeLocalOptimalOK() // returns true if all grand-children nodes cannot be reached due to obstacles
{
  for(int i = 0; i < num_valid_points; i++)
  {
    int ind = ValidInds[i];     
    
    if(ind < 1)
        continue; // its the goal or invalid
    
    if(Neighbors[ind][0] < 1)
        continue; // its parent is the goal or invalid
    
    int parent_ind = Neighbors[ind][0];
    int grand_parent_ind = Neighbors[parent_ind][0];
    
    if(W.EdgeValid(ValidConfigs[ind], ValidConfigs[grand_parent_ind]) > 0) // this is a valid connection, which shouldn't happen
    {
      printf("unncessarily long route !!! [%d -> %d -> %d] \n", ind, parent_ind, grand_parent_ind);
      getchar();
      return false;
    }
  }
  return true;
}

//int tree_animate = 0;
void Cspace::DrawTree() // draws the search tree by calling the workspace draw edge function 
{
  //TreeConsistant();
  
  //if(tree_animate > num_valid_points)
  //  tree_animate = num_valid_points;
  for(int i = 1; i < num_valid_points; i++)
  //for(int i = 1; i < tree_animate; i++)
  {  
      
    int i_index = ValidInds[i];
    //int size_temp2 = Neighbors[i_index].size();
    //for(int j = 1; j < size_temp2; j++)
    //  W.DrawEdge(ValidConfigs[i_index], ValidConfigs[Neighbors[i_index][j]]);
    
    W.DrawEdge(ValidConfigs[i_index], ValidConfigs[Neighbors[i_index][0]]);
    
    /*
    printf("%d: %f =? %f + %f : -> %d ... ", i_index, Cspc.DistToGoal[i_index], Cspc.DistToGoal[Cspc.Neighbors[i_index][0]], Cspc.W.Dist(Cspc.ValidConfigs[i_index], Cspc.ValidConfigs[Cspc.Neighbors[i_index][0]]), Cspc.Neighbors[i_index][0]); 
    if(Neighbors[i_index][0] == 0)
      printf("goal is parent");   
    else if(W.EdgeValid(ValidConfigs[i_index], ValidConfigs[Neighbors[Neighbors[i_index][0]][0]]) > 0)
       printf("no edge to grand parent (%d), %d", Neighbors[Neighbors[i_index][0]][0], Neighbors[i_index][0]);
    else
       printf("edge to grand parent (%d), %d", Neighbors[Neighbors[i_index][0]][0], Neighbors[i_index][0]);      
    printf("\n");
    */
  } 
  //tree_animate ++;
}

void Cspace::DrawPath(bool draw_robots) // draws the path from the start configuration to the goal configuration, if draw_robots is true, then the robot is also shown
{
  int next_ind = start_ind;
  
  if(start_ind < 0)  // start was not found, so there is no path
    return;
  
  do
  {
    W.DrawEdge(ValidConfigs[next_ind], ValidConfigs[Neighbors[next_ind][0]]);
    
    if(draw_robots)
      W.Draw(ValidConfigs[next_ind], NULL);
        
    next_ind = Neighbors[next_ind][0];
  }
  while(next_ind != 0);
  
  if(draw_robots)
    W.Draw(ValidConfigs[next_ind], NULL);
      
}

void Cspace::RoughAnimate(bool draw_paths)      // animates the movement from start to goal, if draw_paths == true, then it draws the paths
{ 
  if(start_ind < 0)  // start was not found, so there is no path
    return;

  if(next_ind_animate < 0) // initialize global
    next_ind_animate = start_ind;   

  // draw this config
  W.DrawGoal(ValidConfigs[0], NULL);
  W.Draw(ValidConfigs[next_ind_animate], NULL);
  
  // draw paths up to this point if we are supposed to
  if(draw_paths && start_ind != next_ind_animate)
  {
    int next_ind = start_ind;  
    do
    {
      W.DrawEdge(ValidConfigs[next_ind], ValidConfigs[Neighbors[next_ind][0]]);  
      next_ind = Neighbors[next_ind][0];
    }
    while(next_ind != next_ind_animate && next_ind != 0);      
  }
  
  if(next_ind_animate == 0)
    next_ind_animate--;    
  else
    next_ind_animate = Neighbors[next_ind_animate][0];  
}
