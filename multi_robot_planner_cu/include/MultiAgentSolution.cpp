MultiAgentSolution::MultiAgentSolution() // default constructor
{
   num_agents = 1;
   agent_id = 0;  
   best_solution_length = -1;
   best_solution_agent = -1;
   moving = false; 
   total_nodes_added = 0;
   current_it = 0;
   dims_per_robot = 2;
   
   Gbls = NULL;
}

MultiAgentSolution::MultiAgentSolution(int the_num_agents, int the_agent_id)  // constructor
{
  Populate(the_num_agents,the_agent_id, 2);
}

MultiAgentSolution::MultiAgentSolution(const MultiAgentSolution& M)   // copy constructor
{
   num_agents = M.num_agents;
   agent_id = M.agent_id;
      
   dims_per_robot = M.dims_per_robot;
   
   best_solution_length = M.best_solution_length;
   BestSolution = M.BestSolution;
   
   best_solution_agent =  M.best_solution_agent;
   
   Votes.resize(num_agents);
   for(int i = 0; i < num_agents; i++)
     Votes[i] = M.Votes[i];
   
   FinalSolutionSent.resize(num_agents);
   for(int i = 0; i < num_agents; i++)
     FinalSolutionSent[i] = M.FinalSolutionSent[i];
   
   moving = M.moving; 
   in_msg_ctr = M.in_msg_ctr;
   out_msg_ctr = M.out_msg_ctr;
   
   message_send_attempts = M.message_send_attempts;
   messages_sent_to_us = M.messages_sent_to_us;
   messages_recieved_by_us = M.messages_recieved_by_us;
   
   if(uni_tree_build == 1)
   {
     NodeID = M.NodeID;
     NodeIDInda = M.NodeIDInda; 
     NodeIDIndb = M.NodeIDIndb;
     current_it = M.current_it;
     LastItAdded = M.LastItAdded;
   }
   
   Gbls = M.Gbls;
}

MultiAgentSolution::~MultiAgentSolution()  // destructor 
{
    
}

void MultiAgentSolution::Populate(int the_num_agents, int the_agent_id, int this_dims_per_robot)  // populates or re-populates the structure
{
   num_agents = the_num_agents;
   agent_id = the_agent_id;
   
   BestSolution.resize(0);
   best_solution_length = -1;
   best_solution_agent = -1;
   
   Votes.resize(0);
   Votes.resize(the_num_agents, -1); 
   Votes[agent_id] = agent_id;
   
   FinalSolutionSent.resize(0);
   FinalSolutionSent.resize(the_num_agents, 0);
   
   moving = false;  
   in_msg_ctr.resize(0);
   in_msg_ctr.resize(the_num_agents);
   out_msg_ctr.resize(0);
   out_msg_ctr.resize(the_num_agents);
   
   message_send_attempts.resize(the_num_agents);   // not resized to 0 first since we want this to last after a re-populate
   messages_sent_to_us.resize(the_num_agents);     // not resized to 0 first since we want this to last after a re-populate
   messages_recieved_by_us.resize(the_num_agents); // not resized to 0 first since we want this to last after a re-populate
   
   if(uni_tree_build == 1)
   {
     NodeID.resize(0);
     NodeID.resize(the_num_agents);
   
     total_nodes_added = 1;
   
     for(int i = 0; i < the_num_agents; i++)
     {
       NodeID[i].resize(0);
       NodeID[i].push_back(0);
     }
     
     NodeIDInda.resize(0);
     NodeIDInda.push_back(0);
     NodeIDIndb.resize(0);
     NodeIDIndb.push_back(0);
     
     current_it = 0;
     
     LastItAdded.resize(0);
     LastItAdded.resize(the_num_agents);
     for(int i = 0; i < the_num_agents; i++)
       LastItAdded[i].push_back(0);
   }
   
   dims_per_robot = this_dims_per_robot;
   
   Gbls = NULL;
}

void MultiAgentSolution::Populate(int the_num_agents, int the_agent_id, GlobalVariables* G, int this_dims_per_robot)  // populates or re-populates the structure
{
   Populate(the_num_agents, the_agent_id, this_dims_per_robot);
   Gbls = G;
}

void MultiAgentSolution::ExtractSolution(Cspace& C)  // this extracts a solution from the Cspace C, and updates things approperiatly if the solution is better than the best solution found so far.
{
  float this_solution_length = C.DistToGoal[C.start_ind];
  
  if(this_solution_length < best_solution_length || best_solution_length == -1)
  {
    if(C.Neighbors.size() < 1) // the solution has only one point
    {
      return; 
    } 
    
    // this is the best solution so far, so we'll extract it and save it
 
    // first we need to calculate how long the solution is
    int ind = C.start_ind;
    int neighbor_ind = C.Neighbors[ind][0];
    int i = 2;

    while(neighbor_ind > 0)
    {
      ind = neighbor_ind;
      neighbor_ind = C.Neighbors[ind][0];
      
      //printf("1 neighbor_ind: %d   ind: %d\n", neighbor_ind, ind);
      
      if(neighbor_ind == ind)
         getchar();
      
      i++; 
    }
    
    BestSolution.resize(i);
    
    // now we actually put the solution in 
    ind = C.start_ind;
    neighbor_ind = C.Neighbors[ind][0];
    
    i = 0;
    BestSolution[i] = C.ValidConfigs[ind];
    i++;
    BestSolution[i] = C.ValidConfigs[neighbor_ind];
    i++;
    
    while(neighbor_ind > 0)
    {
      ind = neighbor_ind;
      neighbor_ind = C.Neighbors[ind][0];
      
      BestSolution[i] = C.ValidConfigs[neighbor_ind];
      i++;
    }
    
    best_solution_length = this_solution_length;
    
    best_solution_agent = agent_id;
    Votes[agent_id] = best_solution_agent;
  }
}

void MultiAgentSolution::AddSolution(Cspace& C)  // this adds the best solution to the Cspace C
{     
  if(add_best_path_to_this_c_space == 0) // don't want to add other agent's solution to this c_space  
  {
    if(C.best_total_path_length > best_solution_length) // still prune based on best solution length
      C.best_total_path_length = best_solution_length;
    return;
  }
  
  // note that the goal is already added 
  int i = BestSolution.size()-2;  
  
  float this_dist_to_obstacle = C.W.PointValid(BestSolution[i]);  
  float this_dist_to_goal = C.W.Dist(C.goal,BestSolution[i]);
  float this_est_dist_to_start = C.W.Dist(BestSolution[i], C.start);
  int last_added_ind = C.AddNeighbor(BestSolution[i], 0, this_dist_to_obstacle, this_dist_to_goal, this_est_dist_to_start);
  float last_dist_to_obstacle;
  float dist_to_obstacle;
  i--;
  
  for(; i >= 0; i--)
  {
    // add the next closest node to the goal
    last_dist_to_obstacle = this_dist_to_obstacle;
    this_dist_to_obstacle = C.W.PointValid(BestSolution[i]);
    
    if(last_dist_to_obstacle < this_dist_to_obstacle)
      dist_to_obstacle = last_dist_to_obstacle;
    else
      dist_to_obstacle = this_dist_to_obstacle;
            
    this_dist_to_goal = this_dist_to_goal + C.W.Dist(BestSolution[i+1],BestSolution[i]);
    this_est_dist_to_start = C.W.Dist(BestSolution[i], C.start);

    last_added_ind = C.AddNeighbor(BestSolution[i], last_added_ind, dist_to_obstacle, this_dist_to_goal, this_est_dist_to_start);  
  } 
  
  if(C.DistToGoal[last_added_ind] < C.best_total_path_length) // this solution is better than the others in the tree
  {
    C.start_ind = last_added_ind; // holds the index of the start (goal index is 0)   
    C.best_total_path_length = C.DistToGoal[last_added_ind];
    
    if(C.best_total_path_length < 0)
    {
      // the stuff in this if statement is mostly for error checking and debugging  
      printf("attampting to add a bogus path with length %f !!!!!!!!!!!!!!!!!!!!!!!!! \n", C.best_total_path_length);
      float this_sum = 0;
      for(uint ii = 0; ii < BestSolution.size(); ii++)
      {
        for(uint jj = 0; jj < BestSolution[ii].size(); jj++)
          printf("%f ,", BestSolution[ii][jj]);
        printf("\n");
        
        if(ii+1 < BestSolution.size())
          this_sum += C.W.Dist(BestSolution[ii+1],BestSolution[ii]);
      }
      printf(" again length = %f \n\n", this_sum);
      
      int this_n = 0;
      this_sum = 0;
      while(C.Neighbors[this_n].size() > 1)
      {
        for(uint jj = 0; jj < C.ValidConfigs[this_n].size(); jj++)
          printf("%f ,", C.ValidConfigs[this_n][jj]);
        printf("\n");
          
        int back = C.Neighbors[this_n].size()-1;
        int next_n = C.Neighbors[this_n][back];
        this_sum += C.W.Dist(C.ValidConfigs[this_n],C.ValidConfigs[next_n]);
        this_n = next_n;
      }
      
      for(uint jj = 0; jj < C.ValidConfigs[this_n].size(); jj++)
        printf("%f ,", C.ValidConfigs[this_n][jj]);
      printf("\n");
        
      printf(" again length = %f \n\n", this_sum);
      
      getchar();  
    } 
  }
}


void MultiAgentSolution::DrawPath(Workspace& W, bool draw_robots)             // draws the path, if draw_robots is true, then the robot is also shown
{
  if(best_solution_length == -1)  // no solution, so there is no path
    return;
   
  int num_points = BestSolution.size();
  
  if(next_ind_animate >= num_points)
    next_ind_animate = num_points-1;
  
  for(int i = 0; i < num_points-1; i++)
  {
    W.DrawEdge(BestSolution[i], BestSolution[i+1]);  
    
    if(draw_robots)
      W.Draw(BestSolution[i], NULL);
  }
  
  if(draw_robots)
    W.Draw(BestSolution[num_points-1], NULL);
}

void MultiAgentSolution::RoughAnimate(Workspace& W, bool draw_paths)  // animates the movement along best solution from start to goal, if draw_paths == true, then it draws the paths
{
  if(best_solution_length == -1)  // no solution, so there is no path
    return;
 
  if(next_ind_animate < 0) // initialize global
    next_ind_animate = 0; 
  
  int num_points = BestSolution.size();
  
  if(next_ind_animate >= num_points)
    next_ind_animate = num_points-1;
  
  // draw this config
  W.DrawGoal(BestSolution[num_points-1], NULL);
  W.Draw(BestSolution[next_ind_animate], NULL);
 
  // draw paths up to this point if we are supposed to
  if(draw_paths)
  {
    for(int i = 0; i < next_ind_animate; i++)
      W.DrawEdge(BestSolution[i], BestSolution[i+1]);  
  }
  
  next_ind_animate++;  
}

bool MultiAgentSolution::GetMessages(const vector<float>& start_config, const vector<float>& goal_config)  // checks for incomming messages, and updates things accordingly, returns true if a better path was found in the message, also makes sure that they use start and goal configs
{   
  bool found_path_in_file = false;
   
  // check for messages from every other agent (even those not yet in team)

  int unused_result; // dummy return to make warning go away

  for(int i = 0; i < Gbls->number_of_agents; i++)
  {
    if(i == Gbls->agent_number)
      continue;  
   
    // look at entire remaining queue

    while(true) // will break out when done
    {
      char this_file[100];
      sprintf(this_file, "%s/%d_to_%d_%d.txt", message_dir, i, agent_id, in_msg_ctr[i]);
    
      FILE* ifp = fopen(this_file,"r");
      if(ifp == NULL)
      {
        //printf("cannot read message file \n");
        break;   
      }   
      //printf("in file %s\n",this_file);
      
      if(Gbls->master_reset)
      {
        fclose(ifp);
        remove(this_file);

        printf("dropping message due to reset \n" );
        continue;
      }

      //printf("parsing file message from %d \n", i);
      
      float file_best_solution_length;
      int file_best_solution_agent;
      vector<int> file_votes(num_agents,0);
      vector<int> file_FinalSolutionSent(num_agents,0);
      int file_num_points;  
      int file_dimensions;  
      vector<int> file_DimensionMapping;
      vector<float> file_DimensionOffset;
      vector<vector<float> > file_solution;
      int file_move_flag;  
      int message_num;
      int senders_planning_iteration;
      
      // get solution length from file
      if(fscanf(ifp, "l:%f,%d\n", &file_best_solution_length, &senders_planning_iteration) <= 0)
      {
        // this may still contain workspace points
        if(Scene.GetPointsFromFile(ifp))
        {
          fclose(ifp);
          
          in_msg_ctr[i]++;
          
          // get rid of message file
          remove(this_file);
        }
        else
        {
          fclose(ifp);
        }
        //printf("break 1 \n");
        break;
      }
      //printf("solution from file: %f \n",file_best_solution_length);
    
            
      if(senders_planning_iteration < Gbls->planning_iteration[i])
      {
        fclose(ifp);
        remove(this_file);
        printf("break 2: ag:%d, %d < %d   \n",agent_id, senders_planning_iteration, Gbls->planning_iteration[i] );
        break;
      }
      
      // get agent that created solution in file
      if(fscanf(ifp, "a:%d\n", &file_best_solution_agent) <= 0)
      {
        // problems reading data
        fclose(ifp);
        printf("break 3 \n");
        break;
      }
      else if(file_best_solution_agent < 0 || file_best_solution_agent >= num_agents)
      {
        // problems reading data (invalid agent id from file)
        fclose(ifp);
        printf("break 4 \n");
        break;
      }
      //printf("corresponding agent: %d \n",file_best_solution_agent);

      //get the list of agents that support the path in the file
      if(fscanf(ifp, "s:") < 0)
      {
        // problems reading data
        fclose(ifp);
        printf("break 5 \n");
        break;
      } 
      //printf("supporting agents: ");
      int this_s;
      while(fscanf(ifp, "%d,", &this_s) > 0)
      {
        //printf("%d, ", this_s);
        if(this_s < 0 || this_s >= num_agents)
        {
          // this could happen if we get a message from an agent working on a different problem
          //printf("problems reading data (invalid agent id from file) \n");
          printf("break 6 \n");
          break;
        }
        file_votes[this_s] = 1;  
      }
      //printf("\n");
     
      unused_result = fscanf(ifp, "f:\n");
      
      // get list of flags indicating final solution
      for(int j = 0; j < num_agents; j++)
      {
        if(fscanf(ifp, "%d, ", &this_s) <= 0) 
        { 
          printf("problems reading data (not enough agent solution flags) \n");
          break;
        }  
        
        file_FinalSolutionSent[j] = this_s;
      }
      unused_result = fscanf(ifp, "\n");
      //printf("\n"); 
    
//       ////////////////  for debugging  
//       printf("recieved:  >>>");
//       for(uint j = 0; j < num_agents; j++)
//         printf("%d, ", file_FinalSolutionSent[j]);
//       printf("  <<<\n");
//       ////////////////
      
      
      // get number of points in the path 
      if(fscanf(ifp, "p:%d\n", &file_num_points) <= 0)
      {
        // problems reading data
        fclose(ifp);
        printf("break 7 \n");
        break;
      }
      else if(file_num_points <= 0)
      {
        // problems reading data (invalid number of solution points)
        fclose(ifp);
        printf("break 8 \n");
        break;
      }
      //printf("number of points in solution: %d \n",file_num_points);
    
      // get number of dimensions in the path 
      int offset_dims;
      if(fscanf(ifp, "d:%d,%d\n", &file_dimensions, &offset_dims) <= 0)
      {
        // problems reading data
        fclose(ifp);
        printf("break 9 \n");
        break;
      }
      else if(file_dimensions <= 0)
      {
        // problems reading data (invalid number of solution dimensions)
        fclose(ifp);
        printf("break 10 \n");
        break;
      }
      //printf("number of dimensions solution: %d \n",file_dimensions);
    
      
      // get offset of each dimesion vs the global coordinates
      file_DimensionOffset.resize(file_dimensions,0);
      bool got_complete_offset = true;
      float this_f;
      for(int j = 0; j < offset_dims; j++)
      {
        if(fscanf(ifp, "%f, ", &this_f) <= 0) 
        {
          printf("problems reading data (not enough dimension offset flags) \n");
          got_complete_offset = false;
          break;
        }  

        //printf("%d, ", this_f);
        file_DimensionOffset[j] = this_f;
      }
      unused_result = fscanf(ifp, "\n");
      //printf("\n"); 
     
      if(!got_complete_offset)
      {
        fclose(ifp);
        printf("break 11 \n");
        break;
      }
      
      // get the mapping for the order of dimesions vs global robot ids
      bool got_complete_mapping = true;
      //printf("recovering the following mapping : "); 
      int solution_num_robots = -1; 
      
      if(fscanf(ifp, "m:%d\n", &solution_num_robots) < 1)
      {
        printf("problems reading data (no mapping num) \n");  
        fclose(ifp);
        printf("break 12 \n");
        break;  
      }
      //printf("%d :", solution_num_robots);
      
      file_DimensionMapping.resize(solution_num_robots,-1);
      
      for(int j = 0; j < solution_num_robots; j++)
      {
        if(fscanf(ifp, "%d, ", &this_s) <= 0) 
        {
          printf("problems reading data (not enough mapping flags) \n");
          got_complete_mapping = false;
          printf("break 13 \n");
          break;
        }  

        //printf("%d, ", this_s);
        file_DimensionMapping[j] = this_s;
      }
      unused_result = fscanf(ifp, "\n");
      //printf("\n"); 
     
      if(!got_complete_mapping)
      {
        fclose(ifp);
        printf("break 14 \n");
        break;
      }
      
      
      bool same_group = true;
      bool successfull_path_get = true;
      vector<int> temp_mapping(file_dimensions, -1);
      if(solution_num_robots == Gbls->team_size)
      {
        // calculate which local dimensions the message dimensions coorisponds to
        for(int k = 0; k < file_dimensions; k++) 
        {
          int message_robot_id = k/dims_per_robot;                    // which robot this represnets in the message
          int global_robot_id = file_DimensionMapping[message_robot_id]; // that robot's global id
        
          if(!Gbls->InTeam[global_robot_id])  // then a robot in message's solution is not in this agent's group
            same_group = false;
        
          if(!same_group)
          {
            //printf("break 15 \n");
            break;
          }
          
          int local_robot_id = Gbls->local_ID[global_robot_id];          // that robot's local id on this agent
          
          temp_mapping[k] = (local_robot_id*dims_per_robot) + (k - (message_robot_id*dims_per_robot)); // the dimension in the solution this goes into
        
          if(temp_mapping[k] > file_dimensions || temp_mapping[k] < 0)
          {
            successfull_path_get = false;
            
            printf(" here ::::::::: %d %d \n", temp_mapping[k], file_dimensions);
          }
        }
        if(!successfull_path_get)
        {
          printf("---ignoring message for a different problem (maps to higher dimension than we have)\n"); 
          fclose(ifp);

          printf("master reset in multiagentsolution because of higer mapping problem \n");
          Gbls->planning_iteration[Gbls->agent_number]++;
          Gbls->master_reset = true;

          break; 
        }
      }
      else
      {
        // different groups since they are not the same size
        same_group = false;
      }
         
      if(!same_group)
      { 
         // don't need any more info about this solution
        fclose(ifp);
        remove(this_file);
        //printf("continue 1 \n");
        continue;      
      }
      
      //if we are here then message is in the same group as this agent

      //extract the path;
      file_solution.resize(file_num_points);
      //printf("the path: \n");
      float this_value;
      for(int j = 0; j < file_num_points && successfull_path_get; j++)
      {   
        file_solution[j].resize(file_dimensions);
        for(int k = 0; k < file_dimensions; k++)   
        {
          if(fscanf(ifp, "%f, ", &this_value) <= 0) 
          {
            // problems reading data (not enough path dimensions)
            successfull_path_get = false;
            printf("break 17 \n");
            break;
          }
              
          //printf(" %d %d %d\n", j, k, temp_mapping[k]);
          file_solution[j][temp_mapping[k]] = this_value;
          //printf("%f, ", file_solution[j][k]); 
        }
        unused_result = fscanf(ifp, "\n");
        //printf("\n");   
      }
      
      if(!successfull_path_get)
      {
        fclose(ifp);
        printf("break 18 \n");
        break;
      }   
          
      
      // make sure that this message is for this problem  
      // check start
      if(! equal_float_vector(file_solution[0], start_config, change_plase_thresh))
      {
        printf("---ignoring message for a different problem (the start is different)\n");   
        
        print_float_vector(file_solution[0]);
        print_float_vector(start_config);
         
        fclose(ifp);
        printf("break 19 \n");
        break;
      }
      
      // check goal
      if(! equal_float_vector(file_solution[file_solution.size()-1], goal_config, change_plase_thresh))
      {
        printf("---ignoring message for a different problem (the goal is different)\n");   
        
        print_float_vector(file_solution[file_solution.size()-1]);
        print_float_vector(goal_config);
        
        fclose(ifp);
        break;
      }
      
      // get remaining planning time from file
      float remaining_planning_time;
      if(fscanf(ifp, "r:%f\n", &remaining_planning_time) <= 0)
      {      
        // problems reading data
        fclose(ifp);
        printf("could not get planning time reamaining \n");
        break;
      }     
      
      if(remaining_planning_time < Gbls->planning_time_remaining[i])
      {
        Gbls->planning_time_remaining[i] = remaining_planning_time;
        timeval temp_time;
        gettimeofday(&temp_time, NULL);
        Gbls->last_update_time[i] = temp_time;
      }
      
      // get file move flag from file
      if(fscanf(ifp, "m:%d\n", &file_move_flag) <= 0)
      {      
        // problems reading data
        printf("problems reading moving data (have you added the time to start to the old message stuff in MultiAgentSolution yet? \n");
        fclose(ifp);
        break;
      }
      if(file_move_flag != 0 && file_move_flag != 1)
      {
        // problems reading data (invalid file move_flag)
        fclose(ifp);
        printf("break 20 \n");
        break;
      }
      //printf("file move_flag: %d \n",file_move_flag);
   
      // get message number
      if(fscanf(ifp, "s:%d\n", &message_num) <= 0)
      {      
        // problems reading data
        fclose(ifp);
        printf("break 21 \n");
        break;
      }
      
      // check for workspace points
      Scene.GetPointsFromFile(ifp); 
      
      // extract nodes from file
      if(uni_tree_build == 1)
      {
        printf("this has not yet been changed to use 1 level of abstraction for dimensions, it will probably not work \n");  
          
        int num_nodes_in_file;
        if(fscanf(ifp, "n:%d\n", &num_nodes_in_file) > 0)
        {
          if(num_nodes_in_file > 0)
          {
            vector<int> node_key_a(num_nodes_in_file,0);
            vector<int> node_key_b(num_nodes_in_file,0);
            vector<int> parent_key_a(num_nodes_in_file,0);
            vector<int> parent_key_b(num_nodes_in_file,0);
            vector<vector<float> > the_coords(num_nodes_in_file); 
            
            int this_node_key_a;
            int this_node_key_b;
            int this_parent_key_a;
            int this_parent_key_b;
            float this_cord;
            vector<float> this_point(file_dimensions,0);

            vector<int> max_key_b(num_agents,-1); // remembers max key b for each agent
            
            // extract the data from the file
            bool problems_getting_nodes = false;
            for(int fn = 0; fn < num_nodes_in_file && !problems_getting_nodes; fn++)
            {
              if(fscanf(ifp, "%d, %d, %d, %d, ", &this_node_key_a, &this_node_key_b, &this_parent_key_a, &this_parent_key_b) < 4)  // get global node id and parrent id
              {
                printf(" problems getting nodes \n");
                problems_getting_nodes = true;
                break;
              }
              
              if(this_node_key_a < 0 || this_node_key_b < 0 || this_parent_key_a < 0 || this_parent_key_b < 0)
              {
                printf("problems reading nodes from file, key is negative \n");
                getchar();
              }
              node_key_a[fn] = this_node_key_a;
              node_key_b[fn] = this_node_key_b;
              parent_key_a[fn] = this_parent_key_a;
              parent_key_b[fn] = this_parent_key_b;
              
              // remember max key b
              if(max_key_b[this_node_key_a] < this_node_key_b)
                max_key_b[this_node_key_a] = this_node_key_b;    
              if(max_key_b[this_parent_key_a] < this_parent_key_b)
                max_key_b[this_parent_key_a] = this_parent_key_b;    
             
              for(int d = 0; d < file_dimensions; d++) // add configuration coordinates
              {
                if(fscanf(ifp, "%f, ",&this_cord) <= 0)
                {
                  problems_getting_nodes = true; 
                  break;
                }
                this_point[d] = this_cord;
              }
              unused_result = fscanf(ifp, "\n");    
              the_coords[fn] = this_point;
            }

            // add the data to the c space;
            if(!problems_getting_nodes)
            {
              vector<bool> already_existed(num_nodes_in_file); // remembers which nodes already existed
                
              
              //for(int c = 0; c < num_nodes_in_file; c++)
              //  printf("[%d %d] -> [%d %d]\n", node_key_a[c], node_key_b[c], parent_key_a[c], parent_key_b[c]);

              // pass 1, make sure nodes exist, make them if they do not
              for(int c = 0; c < num_nodes_in_file; c++)
                already_existed[c] = ConfirmExistanceGlobalIdNode(node_key_a[c], node_key_b[c], the_coords[c]);
              
              //for(int c = 0; c < num_nodes_in_file; c++)
              //  printf("[%d %d]:%d -> [%d %d]:%d\n", node_key_a[c], node_key_b[c], NodeID[node_key_a[c]][node_key_b[c]], parent_key_a[c], parent_key_b[c],NodeID[parent_key_a[c]][parent_key_b[c]]);
              
              
              // pass 2, link new nodes together
              for(int c = 0; c < num_nodes_in_file; c++)
              {
                if(already_existed[c])
                  continue;
                LinkGlobalIdNodes(node_key_a[c], node_key_b[c], parent_key_a[c], parent_key_b[c]);
              }  
            
              // pass 3, update distince info of nodes
              for(int c = 0; c < num_nodes_in_file; c++)
              {
                if(already_existed[c])
                  continue;
                UpdateDistanceInfoGlobalIdNode(node_key_a[c], node_key_b[c]);
              }
            }
          }
        }
      }
      
      fclose(ifp);
      
      // if we are here than we successfully recieved a message
    
      in_msg_ctr[i]++;
      messages_sent_to_us[i] = (float)message_num;
      messages_recieved_by_us[i] = (float)(in_msg_ctr[i]);
         
      //printf("recieved a message, moving_flag = %d, overall_prob:%f per %d sent\n",file_move_flag, messages_sent_to_us[i], message_num);
      
      // get rid of message file
      remove(this_file);

      // check if we have everybody's final solution
      for(int j = 0; j < num_agents; j++)
      {
        if(file_FinalSolutionSent[j] == 1)  
          FinalSolutionSent[j] = 1;
      }
      
      bool have_everybodys_final_solution = true;
      for(int j = 0; j < num_agents; j++)
      {
        if(!Gbls->InTeam[j])
          continue;
        if(FinalSolutionSent[j] == 0)
        {
          have_everybodys_final_solution = false;  
          //printf(" missing agent %d final solution %d %d %d %d %d %d\n", j, FinalSolutionSent[0], FinalSolutionSent[1], FinalSolutionSent[2], FinalSolutionSent[3], FinalSolutionSent[4], FinalSolutionSent[5]);
          //%printf(" file %d final solution %d %d %d %d %d %d\n", j, file_FinalSolutionSent[0], file_FinalSolutionSent[1], file_FinalSolutionSent[2], file_FinalSolutionSent[3], file_FinalSolutionSent[4], file_FinalSolutionSent[5]);
          
          break;
        }
      }
    
      if(have_everybodys_final_solution) // we have everybody's final solution, so use the best one
      { 
        moving = true;
        if(best_solution_length <= file_best_solution_length && best_solution_length != -1)
        {
          // the solution we already have is better than or equal to the one in the message (and we already have a solution)
          //printf("recieved a worse or equal path in a message\n");        
        }
        else // the solution in the file is the best so far
        {
          printf("recieved a better path in a message, length %f created by agent #%d (moving = %d)\n", best_solution_length, file_best_solution_agent,file_move_flag);  

          best_solution_length = file_best_solution_length;
          best_solution_agent = file_best_solution_agent;
        
          BestSolution.resize(file_num_points);
          for(int j = 0; j < file_num_points; j++)
            BestSolution[j] = file_solution[j];  
    
          found_path_in_file = true; 
        }
            
        // regardless, we need to record the robots that the message came from
        for(int j = 0; j < num_agents; j++)
        {
          if(!Gbls->InTeam[j])
            continue;
          if(file_votes[j] == 1)
            Votes[j] = best_solution_agent;
        }
        Votes[agent_id] = best_solution_agent;  
        //printf("have everybody's solution\n");  
      }
    
      if(mode == 0 || mode == 2) // pss or baseline
      {
        if(file_best_solution_agent == agent_id) // this is a message naming this agent as the best agent
        {
          if(best_solution_agent != agent_id)  // this agent no longer believes that itself is the best agent, so it must have recieved a better path from a different agent
          {
            //printf("continue 0 \n");    
            continue;
          }
      
          // we want to see who else supports this agent
          for(int j = 0; j < num_agents; j++)
          {
            if(!Gbls->InTeam[j])
              continue;
                        
            if(file_votes[j] == 1)
              Votes[j] = agent_id;
          }
          Votes[agent_id] = agent_id;
          
          continue;
        }
        else if(best_solution_length <= file_best_solution_length && best_solution_length != -1)
        {
          // the solution we already have is better than or equal to the one in the message (and we already have a solution)
           
          if(!moving && file_move_flag == 1)
          {
            // we are not moving and the agent we recieved the message from is, so even though we have a better solution we'll use their path
          }
          else if(moving && file_move_flag == 1)
          {
            // we are moving, the sender is moving, and for some reason we are not using the same solution 
            if(best_solution_agent < file_best_solution_agent)
            {
              // the id of the agent that created our solution is lower, so we do not use the solution in the message
              //printf("continue 2 \n");
              continue ; 
            }
          }
          else
          {
            // we do not use the solution in the message
            //printf("continue 3 \n");
            continue;     
          }
        }
    
        if(best_solution_length > file_best_solution_length)
        {
          printf("recieved a better path in a message, length %f created by agent #%d (moving = %d)\n", best_solution_length, file_best_solution_agent,file_move_flag);  
        }
        else
        {
          // printf("recieved a worse or equal path in a message, but decided to use it anyway\n");
        }

        best_solution_length = file_best_solution_length;
        best_solution_agent = file_best_solution_agent;
    
        for(int j = 0; j < num_agents; j++)
        {
          if(!Gbls->InTeam[j])
            continue;
            
          if(file_votes[j] == 1)
            Votes[j] = best_solution_agent;
        }
        Votes[agent_id] = best_solution_agent;
      
        BestSolution.resize(file_num_points);
        for(int j = 0; j < file_num_points; j++)
          BestSolution[j] = file_solution[j];

        if(file_move_flag == 1 || moving)
          moving = true;
        else
          moving = false;   
    
        found_path_in_file = true;
      }  
    }
    
    if(Gbls->master_reset)
      break;
  }
  
  return found_path_in_file;
}


void MultiAgentSolution::SendMessage(float send_prob) // sends a message containing the current best solution, with probability send_prob
{    
  //printf("%d %d, %d %d\n", pctr, ectr, pctr_all, ectr_all);  
  pctr = 0;
  ectr = 0;
  pctr_all = 0;
  ectr_all = 0;
  
  if(BestSolution.size() <= 0) // | BestSolution[0].size() <= 0)
  {
    //printf(" solution has 0 points \n");     
    if(add_points_to_messages == 1)
    {
      // just send a message with valid workspace points in it  
      // send messages to every other agent but this one
      for(int i = 0; i < num_agents; i++)
      {
        if(i == agent_id)
          continue;
              
        message_send_attempts[i]++;
              
        // send with send send_prob
        float rand_float = ((float)rand_int(0,1000000))/((float)1000000); // rand float between 0 and 1
      
        if(rand_float > send_prob) 
          continue;
        char this_file[100];
        sprintf(this_file, "%s/%d_to_%d_%d.txt", message_dir, agent_id, i, out_msg_ctr[i]);    
      
        FILE* ofp = fopen(this_file,"w");  
        Scene.SendPointsToFile(ofp);
        fclose(ofp);
        out_msg_ctr[i]++;
      }
    }
    return;
  }
  
  // send messages to every other agent but this one
  for(int i = 0; i < num_agents; i++)
  {
    if(i == agent_id)
      continue;
      
    message_send_attempts[i]++;
    
    // send with send send_prob
    float rand_float = ((float)rand_int(0,1000000))/((float)1000000); // rand float between 0 and 1
    if(rand_float > send_prob) 
      continue;
   
    char this_file[100];
    sprintf(this_file, "%s/%d_to_%d_%d.txt", message_dir, agent_id, i, out_msg_ctr[i]);    
    
    FILE* ofp = fopen(this_file,"w");
    if(ofp == NULL) // problem opening file
    {
      printf("cannot open message file for writing\n");
      continue;
    }
    
    // best path length (i.e. cost)
    fprintf(ofp, "l:%f\n",best_solution_length);
       
    // best path agent id
    fprintf(ofp, "a:%d\n",best_solution_agent);
      
    // all robots that support the best path known to this robot (including this robot)
    fprintf(ofp, "s:");
    for(int j = 0; j < num_agents; j++)
    {
      if(Votes[j] == best_solution_agent)
        fprintf(ofp, "%d,",j);   
    }
    fprintf(ofp, "\nf:\n");   
    
    for(int j = 0; j < num_agents; j++)
    {
      fprintf(ofp, "%d, ", FinalSolutionSent[j]);   
    }
    fprintf(ofp, "\n");  
    
    // best path
    int num_points = BestSolution.size();
    int num_dims = BestSolution[0].size();
    fprintf(ofp, "p:%d\n", num_points);
    fprintf(ofp, "d:%d\n", num_dims);
    for(int j = 0; j < num_points; j++)
    {
      for(int k = 0; k < num_dims; k++) 
        fprintf(ofp, "%f, ", BestSolution[j][k]);
      fprintf(ofp, "\n");
    }      
    
    // move flag
    if(moving)
      fprintf(ofp, "m:1\n");
    else
      fprintf(ofp, "m:0\n");
      
    // message stats
    fprintf(ofp, "s:%d\n",(message_send_attempts[i]));
    
    if(add_points_to_messages == 1)
      Scene.SendPointsToFile(ofp);
    
    if(uni_tree_build == 1)
    {
      // put up to num_nodes_per_file into the file   
          
      current_it++;  
        
      // note that we put in all of their descendents so that the entire path back to tree root is available
      // first pass, find the nodes we want to add and remember their coords
      vector<int> node_key_a;
      vector<int> node_key_b;
      vector<int> parent_key_a;
      vector<int> parent_key_b;
      vector<vector<float> > the_coords;
      int node_a_k;
      int node_b_k;
      int parent_a_k;
      int parent_b_k;
      int node_ind;
      int parent_ind;
      
      int m = 0;
      for(int n = total_nodes_added-1; n >= 1; n--) // most likely will break out of here 
      {
        if(m >= num_nodes_per_file) // we have all we can put into the file
          break;
        
        node_a_k = agent_id;
        node_b_k = n;
        if(LastItAdded[node_a_k][node_b_k] == current_it) // this node and its ancesters have already been added
          continue;
        node_ind = NodeID[node_a_k][node_b_k];
        if(node_ind == 0) // we are at the root
          break;
        else if(node_ind == -1) // node is unalocated (should never happen)
        {
          printf(" unalocated?\n");
          getchar();
          continue;
        }
        else if(node_ind == -2) // node has been removed (should never happen)
        {
          // printf(" should never happen?\n");
          continue;
        }
          
        parent_ind = Cspc.Neighbors[node_ind][0];
        parent_a_k = NodeIDInda[parent_ind];
        parent_b_k = NodeIDIndb[parent_ind];
        
        node_key_a.push_back(node_a_k);
        node_key_b.push_back(node_b_k);
        parent_key_a.push_back(parent_a_k);
        parent_key_b.push_back(parent_b_k);
        the_coords.push_back(Cspc.ValidConfigs[node_ind]);
        
        LastItAdded[node_a_k][node_b_k] = current_it;
        m++;   
                
        // this loop is where we make sure that the whole path to the root is remembered
        while(true) // will break out when done
        { 
          if(LastItAdded[parent_a_k][parent_b_k]  == current_it)  // all ancesters have already been added
            break;
          if(parent_ind == 0)  // we are at the root
            break;
          
          node_a_k = parent_a_k;
          node_b_k = parent_b_k;
          node_ind = NodeID[node_a_k][node_b_k];
          if(node_ind < 0) // node has been removed (should never happen)
            break;
          
          parent_ind = Cspc.Neighbors[node_ind][0];
          parent_a_k = NodeIDInda[parent_ind];
          parent_b_k = NodeIDIndb[parent_ind];
        
          node_key_a.push_back(node_a_k);
          node_key_b.push_back(node_b_k);
          parent_key_a.push_back(parent_a_k);
          parent_key_b.push_back(parent_b_k);
          the_coords.push_back(Cspc.ValidConfigs[node_ind]);
        
          LastItAdded[node_a_k][node_b_k] = current_it;
          m++; 
        }         
      }  
      
      // now actually write the data to the file;
      fprintf(ofp, "n:%d\n", m);
      
      int size_of_c = the_coords[0].size();
      for(int c = 0; c < m; c++)
      {
        fprintf(ofp, "%d, %d, %d, %d, ", node_key_a[c], node_key_b[c], parent_key_a[c], parent_key_b[c]);  // add global node id and parrent id
          
        for(int d = 0; d < size_of_c; d++) // add configuration coordinates
          fprintf(ofp, "%f, ", the_coords[c][d]); 
        fprintf(ofp, "\n");
      }
      
      //printf(" out 7\n");
    }
    
    fclose(ofp);
    out_msg_ctr[i]++;
  }
}

void  MultiAgentSolution::SendMessageUDP(float send_prob)   // while above function just uses a file, this uses UDP
{    

  //printf("%d %d, %d %d\n", pctr, ectr, pctr_all, ectr_all);  
  pctr = 0;
  ectr = 0;
  pctr_all = 0;
  ectr_all = 0;
    
  
//   if(BestSolution.size() <= 0) // | BestSolution[0].size() <= 0)
//   {
//     //printf(" solution has 0 points \n");
//       
//     // just send a message with valid workspace points in it  
//     // send messages to every other agent but this one
//     for(int i = 0; i < num_agents; i++)
//     {
//       if(i == agent_id)
//         continue;
//       
//       // send with send send_prob
//       float rand_float = ((float)rand_int(0,1000000))/((float)1000000); // rand float between 0 and 1
//       
//       if(rand_float > send_prob) 
//         continue;
//       char this_file[100];
//       sprintf(this_file, "%s/%d_to_%d_%d.txt", message_dir, agent_id, i, out_msg_ctr[i]);    
//       
//       if(add_points_to_messages == 1)
//       {
//           
//        printf("warning: trying to use a feature that has not been implimented yet \n");
//            
//         //FILE* ofp = fopen(this_file,"w");  
//         //Scene.SendPointsToFile(ofp);
//         //fclose(ofp);
//         //out_msg_ctr[i]++;
//       }
//     }
//     return;
//   }
  
  // send messages to every other agent but this one
  for(int i = 0; i < num_agents; i++)
  {
    if(i == agent_id)
      continue;
      
    // send with send send_prob
    float rand_float = ((float)rand_int(0,1000000))/((float)1000000); // rand float between 0 and 1
    if(rand_float > send_prob) 
      continue;
   
    char out_buffer[max_message_size];
    char temp_buffer[max_message_size];
    int buffer_len = max_message_size;
    
    // add header data about robot start and goal locations
    int sp = Gbls->populate_buffer_with_data(out_buffer); // sp points to the current position in the string

    // add single robot path data      
    // NOTE, moving toward only using single robot path data to combine teams, so must always send it
    //if(Gbls->an_agent_needs_this_single_path_iteration)
    //{
      //printf("here 6 \n");
      sp += Gbls->populate_buffer_with_single_robot_paths((char*)((size_t)out_buffer + (size_t)sp));
      Gbls->an_agent_needs_this_single_path_iteration = false; // still keep this, since it is true and may be usefull
    //}

    if(BestSolution.size() < 1)
    {
      //printf(" sending here ! \n");
      out_buffer[sp] = 4; // this signals that all this message contained was the prefered robot path 
      sp++;

      Gbls->hard_broadcast((void *)out_buffer, sizeof(char) * sp);  // note, changed to hard broadcast when we started doing dynamic team sizes
      return;
    }

    sprintf(temp_buffer,"2%d",agent_id);
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len);
    
    // best path length (i.e. cost)
    sprintf(temp_buffer,"l:%f,%d\n",best_solution_length, Gbls->planning_iteration[agent_id]);
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len);
    
       
    // best path agent id
    sprintf(temp_buffer, "a:%d\n",best_solution_agent);
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    
    // all robots that support the best path known to this robot (including this robot)
    sprintf(temp_buffer,"s:");
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    
    for(int j = 0; j < num_agents; j++)
    {
      if(Votes[j] == best_solution_agent)
      {
        sprintf(temp_buffer, "%d,",j);   
        string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
      }
    }
    sprintf(temp_buffer, "\nf:\n"); 
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
     
    for(int j = 0; j < num_agents; j++)
    {
      sprintf(temp_buffer, "%d, ",FinalSolutionSent[j]); 
      string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    }
    sprintf(temp_buffer,"\n");  
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len);  
    
    
//     // just for debugging    
//     printf("sending:  xx>");
//     for(int j = 0; j < num_agents; j++)
//       printf("%d, ", FinalSolutionSent[j]);
//     printf("  <xx\n");
    
    
    // best path (meta data)
    int num_points = BestSolution.size();       
    int num_dims = BestSolution[0].size();
    sprintf(temp_buffer, "p:%d\n", num_points);\
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    
    sprintf(temp_buffer, "d:%d,%d\n", num_dims, (int)Gbls->team_bound_area_min.size());
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    
    // add the dimensional offset to the file, the dth value in the file denotes the world coordiante that coorisponds to local coordiante 0
    //printf("adding the following offset: ");
    for(uint j = 0; j < Gbls->team_bound_area_min.size(); j++)
    {
      sprintf(temp_buffer, "%f, ",Gbls->team_bound_area_min[j]); 
      string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
     // printf("%f, ",Gbls->team_bound_area_min[j]);
    }
    sprintf(temp_buffer,"\n");  
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    //printf("\n");
    
    // add the dimensional mapping to the file, the nth value in the file denotes which global robot the nth dimension group represents
    sprintf(temp_buffer, "m:%d\n", Gbls->team_size); 
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    //printf("adding the following mapping: ");
    for(int j = 0; j < Gbls->team_size; j++)
    {
      sprintf(temp_buffer, "%d, ",Gbls->global_ID[j]); 
      string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
     // printf("%d, ",Gbls->global_ID[j]);
    }
    sprintf(temp_buffer,"\n");  
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    //printf("\n");
    
    // best path (itself)
    for(int j = 0; j < num_points; j++)
    {
      for(int k = 0; k < num_dims; k++) 
      {
        sprintf(temp_buffer, "%f, ", BestSolution[j][k]);
        string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
      }
      sprintf(temp_buffer, "\n");
      string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    }      
    
    // time left for planning
    sprintf(temp_buffer, "r:%f\n", Gbls->calculate_time_left_for_planning());
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    
    // move flag
    if(moving)
    {
      sprintf(temp_buffer, "m:1\n");
      string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    }
    else
    {
      sprintf(temp_buffer, "m:0\n");
      string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
    }
    
    
    // message stats
    sprintf(temp_buffer, "s:%d\n",(message_send_attempts[i]));
    string_printf_s(sp, out_buffer, temp_buffer, buffer_len);
    
    if(add_points_to_messages == 1)
    {
      printf("warning: trying to use a feature that has not been implimented yet \n");  
      //Scene.SendPointsToFile(ofp);
    }
    
    if(uni_tree_build == 1)
    {
      //printf(" in 7 \n");  
        
      // put up to num_nodes_per_file into the file   
          
      current_it++;  
        
      // note that we put in all of their descendents so that the entire path back to tree root is available
      // first pass, find the nodes we want to add and remember their coords
      vector<int> node_key_a;
      vector<int> node_key_b;
      vector<int> parent_key_a;
      vector<int> parent_key_b;
      vector<vector<float> > the_coords;
      int node_a_k;
      int node_b_k;
      int parent_a_k;
      int parent_b_k;
      int node_ind;
      int parent_ind;
      
      int m = 0;
      for(int n = total_nodes_added-1; n >= 1; n--) // most likely will break out of here 
      {
        if(m >= num_nodes_per_file) // we have all we can put into the file
          break;
        
        node_a_k = agent_id;
        node_b_k = n;
        if(LastItAdded[node_a_k][node_b_k] == current_it) // this node and its ancesters have already been added
          continue;
        node_ind = NodeID[node_a_k][node_b_k];
        if(node_ind == 0) // we are at the root
          break;
        else if(node_ind == -1) // node is unalocated (should never happen)
        {
          printf(" unalocated?\n");
          getchar();
          continue;
        }
        else if(node_ind == -2) // node has been removed (should never happen)
        {
          // printf(" should never happen?\n");
          continue;
        }
          
        parent_ind = Cspc.Neighbors[node_ind][0];
        parent_a_k = NodeIDInda[parent_ind];
        parent_b_k = NodeIDIndb[parent_ind];
        
        node_key_a.push_back(node_a_k);
        node_key_b.push_back(node_b_k);
        parent_key_a.push_back(parent_a_k);
        parent_key_b.push_back(parent_b_k);
        the_coords.push_back(Cspc.ValidConfigs[node_ind]);
        
        LastItAdded[node_a_k][node_b_k] = current_it;
        m++;   
                
        // this loop is where we make sure that the whole path to the root is remembered
        while(true) // will break out when done
        { 
          if(LastItAdded[parent_a_k][parent_b_k]  == current_it)  // all ancesters have already been added
            break;
          if(parent_ind == 0)  // we are at the root
            break;
          
          node_a_k = parent_a_k;
          node_b_k = parent_b_k;
          node_ind = NodeID[node_a_k][node_b_k];
          if(node_ind < 0) // node has been removed (should never happen)
            break;
          
          parent_ind = Cspc.Neighbors[node_ind][0];
          parent_a_k = NodeIDInda[parent_ind];
          parent_b_k = NodeIDIndb[parent_ind];
        
          node_key_a.push_back(node_a_k);
          node_key_b.push_back(node_b_k);
          parent_key_a.push_back(parent_a_k);
          parent_key_b.push_back(parent_b_k);
          the_coords.push_back(Cspc.ValidConfigs[node_ind]);
        
          LastItAdded[node_a_k][node_b_k] = current_it;
          m++; 
        }         
      }  
      
      // now actually write the data to the file;
      sprintf(temp_buffer, "n:%d\n", m);
      string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
      
      int size_of_c = the_coords[0].size();
      for(int c = 0; c < m; c++)
      {
        sprintf(temp_buffer, "%d, %d, %d, %d, ", node_key_a[c], node_key_b[c], parent_key_a[c], parent_key_b[c]);  // add global node id and parrent id
        string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
        
        
        for(int d = 0; d < size_of_c; d++) // add configuration coordinates
        {
          sprintf(temp_buffer, "%f, ", the_coords[c][d]); 
          string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
        }
        sprintf(temp_buffer, "\n");
        string_printf_s(sp, out_buffer, temp_buffer, buffer_len); 
      }
      
      //printf(" out 7\n");
    }

    Gbls->hard_broadcast(out_buffer, sizeof(out_buffer));  // note, changed to hard broadcast when we started doing dynamic team sizes
    out_msg_ctr[i]++;
  }
}

bool MultiAgentSolution::StartMoving()   // returns true if this agent can start moving
{
  if(moving)     // if we are already moving
    return true;
  
  if(mode == 0) // pss
  {
    // check if all agents vote for our path (given best info we have)
    for(int i = 0; i < Gbls->team_size; i++)
    {    
      if(Votes[Gbls->global_ID[i]] != agent_id)
        return false;
    }
  }
  else if(mode == 1) // ss
  {
    // check if all agents vote for any path
    for(int i = 0; i < num_agents; i++)
      if(Votes[i] == -1)
        return false;   
  }
  
  // all agents are currently voting for us, given best information
  moving = true;  // set that we are now moving
  
  printf("moving quick start \n");
  return true;
}

bool MultiAgentSolution::ConfirmExistanceGlobalIdNode(int node_key_a, int node_key_b, const vector<float> the_coords) // checks the existance of NodeId[node_key_a][node_key_b] and adds it with the_coords if it does not exist (only of relivance when uni_tree_build == 1), returns true if nede already existed before this was called
{
  //printf("confirming existance \n");
  if(node_key_a == agent_id)
  {
    if((int)NodeID[agent_id].size() < node_key_b + 1)
    {
      printf("problems: this NodeId is too small \n");
      getchar();
    }
    return true;
  }
    
  // resize NodeID[node_key_a] so it can hold node_key_b things
  if((int)NodeID[node_key_a].size() < node_key_b + 1)
  {
    NodeID[node_key_a].resize(node_key_b + 1, -1);
    LastItAdded[node_key_a].resize(node_key_b + 1, 0);
  }
  else if(NodeID[node_key_a][node_key_b] >= 0)
  {
    // verrify that the coords are the same
    int this_ind = NodeID[node_key_a][node_key_b];
    int num_coords = the_coords.size();
    bool the_same = true;
    for(int i = 0; i < num_coords; i++)
    {
      if(the_coords[i] != Cspc.ValidConfigs[this_ind][i])
      {
        the_same = false;
        break;
      }  
    }
      
    if(the_same)
    {
      //printf("done confirming existance \n");
      return true;
    }
    else
    {
      printf("problems: node configs not equal : [%d %d] -> %d: %f %f \n", node_key_a, node_key_b, NodeID[node_key_a][node_key_b], the_coords[0], Cspc.ValidConfigs[this_ind][0]);     
     // getchar();
    }
  }
  
  if(Cspc.num_valid_points < Cspc.num_points) // then there is already space to insert
  {
    //TreeOK();
      
    int new_point_index =  Cspc.ValidInds[Cspc.num_valid_points];
    //printf("new_point_index: %d \n", new_point_index);
    
    Cspc.ValidConfigs[new_point_index] = the_coords;
            
    Cspc.ValidInds[Cspc.num_valid_points] = new_point_index;
    Cspc.ValidIndsInd[new_point_index] = Cspc.num_valid_points;
    Cspc.num_valid_points++;  
    
    
    //Cspc.SafeDistance[new_point_index] = Cspc.W.PointValid(Cspc.ValidConfigs[new_point_index]);    // Note: SafeDistance is not used any more
    Cspc.MinDistToStart[new_point_index] = Cspc.W.Dist(Cspc.ValidConfigs[new_point_index], Cspc.start);
    Cspc.DistToGoal[new_point_index] = LARGE; // this must be reset using UpdateDistanceInfoGlobalIdNode()
    
    //TreeOK();

    NodeID[node_key_a][node_key_b] = new_point_index;
    NodeIDInda[new_point_index] = node_key_a;
    NodeIDIndb[new_point_index] = node_key_b;
    
    
    //printf("added 1: [%d %d]:%d\n", NodeIDInda[new_point_index], 
    //                                NodeIDIndb[new_point_index], 
    //                                NodeID[NodeIDInda[new_point_index]][NodeIDIndb[new_point_index]]);
    
  }
  else // num_valid_points == num_points
  {              
    Cspc.ValidConfigs.push_back(the_coords);

    vector<int> temp;
    Cspc.Neighbors.push_back(temp);
    
    Cspc.SafeDistance.push_back(-1);    // Note: SafeDistance is not used any more
    Cspc.MinDistToStart.push_back(Cspc.W.Dist(Cspc.ValidConfigs[Cspc.num_points], Cspc.start));  // must be populated using following function
    Cspc.DistToGoal.push_back(LARGE);   // this must be reset using UpdateDistanceInfoGlobalIdNode()
    
    Cspc.ValidInds.push_back(Cspc.num_points);
    Cspc.ValidIndsInd.push_back(Cspc.num_valid_points);
    
    NodeID[node_key_a][node_key_b] = Cspc.num_points;
    NodeIDInda.push_back(node_key_a);
    NodeIDIndb.push_back(node_key_b);
    
    //printf("sizes: %d %d %d \n", Cspc.ValidIndsInd.size(), NodeIDInda.size(), NodeIDIndb.size());
    
    //printf("added 2: [%d %d]:%d\n", NodeIDInda[Cspc.num_points], 
    //                                NodeIDIndb[Cspc.num_points], 
    //                                NodeID[NodeIDInda[Cspc.num_points]][NodeIDIndb[Cspc.num_points]]);
    
    Cspc.num_points++;
    Cspc.num_valid_points++;  
  }
   // printf("done confirming existance \n");
  return false;
}
  

void MultiAgentSolution::LinkGlobalIdNodes(int node_key_a, int node_key_b, int parent_key_a, int parent_key_b)  // assuming that NodeId[node_key_a][node_key_b] and NodeId[parent_key_a][parent_key_b] exist, this links them as you would expect (only of relivance when uni_tree_build == 1)
{
  // printf("linking global nodes \n");  
    
  if(node_key_a < 0 || node_key_b < 0)
  {
    // node to insert has been removed   
    printf("node to insert has been removed \n");
    getchar();
    return;
  }
    
  int new_point_index = NodeID[node_key_a][node_key_b];
          
  if(new_point_index < 0)
  {
    printf("problems, index is < 0 \n");
    getchar();
    return;
  }
  
  if(parent_key_a < 0 || parent_key_b < 0)
  {
    // parent has been removed   
    Cspc.RemoveAllDescendants(new_point_index); 
    //printf("parent of node to insert has been removed 1 \n");
    //getchar();
    return;
  }
  
  int parent_index = NodeID[parent_key_a][parent_key_b];
  
  if(parent_index < 0) // parent has been removed
  {
    Cspc.RemoveAllDescendants(new_point_index); 
    //printf("parent of node to insert has been removed 2 \n");
    //getchar();
    return;
  }   
      
  if(Cspc.Neighbors[new_point_index].size() == 0)
    Cspc.Neighbors[new_point_index].resize(1);
  else if(Cspc.Neighbors[new_point_index][0] != parent_index)
  {
    Cspc.Neighbors[new_point_index].resize(1);
    if(Cspc.Neighbors[new_point_index][0] != -1)
    {
      printf("changing parents %d %d\n", Cspc.Neighbors[new_point_index][0], parent_index);
    
      printf("[%d %d]:%d , [%d %d]:%d\n",  NodeIDInda[new_point_index], 
                                         NodeIDIndb[new_point_index],
                                         NodeID[NodeIDInda[new_point_index]][NodeIDIndb[new_point_index]],
                                         NodeIDInda[parent_index], 
                                         NodeIDIndb[parent_index],
                                         NodeID[NodeIDInda[parent_index]][NodeIDIndb[parent_index]]);
            
      getchar();
    }
  }
  else
  {
    int parrent_size = Cspc.Neighbors[parent_index].size();
    bool already_a_neighbor = false;
    for(int i = 0; i < parrent_size; i++)
    {
      if(Cspc.Neighbors[parent_index][i] == new_point_index) // the node is already a neighbor
      {
        already_a_neighbor = true;
        break;
      }
    }
    if(already_a_neighbor)
      return;  
  }
  
  if(Cspc.Neighbors[parent_index].size() < 1)
  {
    //printf("size of %d neighbors list is < 1\n",parent_index);
    //getchar();
    Cspc.Neighbors[parent_index].push_back(-1);  
  }
      
  Cspc.Neighbors[parent_index].push_back(new_point_index);    
  Cspc.Neighbors[new_point_index][0] = parent_index;
  //  printf("done global nodes \n");  
}


void MultiAgentSolution::UpdateDistanceInfoGlobalIdNode(int node_key_a, int node_key_b) // updates distance info about NodeId[node_key_a][node_key_b]
{   
  if(node_key_a < 0 || node_key_b < 0)
  {
    printf("should not be here, node has been removed \n");
    getchar();
    return;
  }
  
  int point_index = NodeID[node_key_a][node_key_b];  
  int parent_index;
  if(point_index < 0)
  {
    //printf("problems: point_index < 0\n"); // this node has been removed
    //getchar();
    return;
  }
  if(point_index == 0)
  {
    // the root
    Cspc.DistToGoal[point_index] = 0;
    return;
  }
  
  // need to find first ansesestor that has actual valid distance info
  vector<int> back_trace;
  
  if(Cspc.Neighbors[point_index].size() < 1)
  {
    printf("neighbor list size < 1 \n");
    getchar();
  }
  
  parent_index = Cspc.Neighbors[point_index][0];
  if(parent_index < 0 || NodeIDInda[parent_index] < 0 || NodeIDIndb[parent_index] < 0)
  {
    // this node has been removed, so remove its descendents through point_index
    Cspc.RemoveAllDescendants(point_index);
    return;
  }
  back_trace.push_back(point_index);
  
  while(Cspc.DistToGoal[parent_index] >= LARGE && parent_index != 0)
  {
    point_index = parent_index;
    parent_index = Cspc.Neighbors[point_index][0];
    
    back_trace.push_back(point_index);
    
    if(parent_index < 0 || NodeIDInda[parent_index] < 0 || NodeIDIndb[parent_index] < 0)
    {
      // this node has been removed, so remove its descendents through point_index
      Cspc.RemoveAllDescendants(point_index);
      return;
    }
  }
    
  // now the parent_index node has valid dist to goal data (or it is the root, in which case it also does)
  int depth = back_trace.size()-1;
  for(int d = depth; d >= 0; d--)
  {
    point_index = back_trace[d];
    parent_index = Cspc.Neighbors[point_index][0];
    
    Cspc.DistToGoal[point_index] = Cspc.DistToGoal[parent_index] + Cspc.W.Dist(Cspc.ValidConfigs[point_index],Cspc.ValidConfigs[parent_index]);  
  }
}

bool MultiAgentSolution::GreedyPathSmooth()              // greedily smooths BestSolution
{
  bool found_shorter_path = false;
          
  // backtrack from start_smooth_ind to goal and store nodes in a list 
    
  vector<vector<float> > NewBestSolution;
          
  int front_list_ind = 0;  // this will change as we move through the list
  int back_list_ind = BestSolution.size() - 1; // this will keep this value as we move through the list
  
  // move from front_list_ind through list toward back_list_end
  while(front_list_ind < back_list_ind)
  {
    NewBestSolution.push_back(BestSolution[front_list_ind]);  
      
    // move through list from back_list_ind toward front_list_ind
    int ind = back_list_ind;
    while(ind > front_list_ind+1)
    {
      // check if a valid edge exists between front_list_ind and ind
      if(Cspc.W.EdgeValid(BestSolution[front_list_ind], BestSolution[ind]) > 0) // this is a valid connection
      { 
        // reroute front_list_node to have list_node as its parent
          
        // set front_list_ind to ind and reset ind to back_list_ind
        front_list_ind = ind-1; // -1 because gets incrimented below
        found_shorter_path = true;
        break;
      }  
       
      ind--;
    }
    front_list_ind++; 
  }
  
  NewBestSolution.push_back(BestSolution[back_list_ind]); 
  
  //printf("front_list: \n");
  //print_float_vector(NewBestSolution[0]);
  
  //printf("back_list: \n");
  //print_float_vector(NewBestSolution[NewBestSolution.size()-1]);
  
  if(found_shorter_path)
  {
    // calculate total cost of new path
    float new_dist = 0;
    for(uint i = 0; i < NewBestSolution.size() - 1; i++)
      new_dist += Cspc.W.Dist(NewBestSolution[i], NewBestSolution[i+1]);
        
    if(new_dist < best_solution_length)
    {    
      // store best solution        
      BestSolution = NewBestSolution;
      best_solution_length = new_dist;
      
      Cspc.best_total_path_length = new_dist;
              
      printf("found better path via smooth (multi-robot-space): %f \n", best_solution_length); 
      
      #ifdef save_time_data
      // record time stats
      clock_t now_time = clock();
      time_stats.push_back(difftime_clock(now_time,start_time));
      nodes_in_tree_stats.push_back(Cspc.num_valid_points);
      best_path_len_stats.push_back(Cspc.best_total_path_length);
      collision_checks_stats.push_back(total_collision_checks);
      #endif 
      
    }
  }
  return found_shorter_path;
}

float MultiAgentSolution::OverallMessageStats()          // returns the probability this robot recieved a message that was sent by another robot
{
  float overall_attempts = 0;
  float overall_recieved = 0;
  for(int i = 0; i < num_agents; i++)
  {
    if(i == agent_id)
      continue;
    if(messages_sent_to_us[i] == 0)
      continue;
    if(messages_sent_to_us[i] < messages_recieved_by_us[i]) // problems
      continue;
    
    overall_attempts += messages_sent_to_us[i];
    overall_recieved += messages_recieved_by_us[i];           
  }    
  printf("overall probability : %f \n", overall_recieved/overall_attempts);
  //getchar();
  
  if(overall_attempts == 0)
    return -1;
  return (overall_recieved/overall_attempts);
}
