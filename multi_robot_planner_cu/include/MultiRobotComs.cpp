GlobalVariables::GlobalVariables()  
{
}

GlobalVariables::GlobalVariables(int num_of_agents)  
{
  Populate(num_of_agents);
}

void error(const char *msg)
{
    perror(msg);
}

GlobalVariables::~GlobalVariables()                             // destructor
{
  if(other_IP_strings != NULL)
  {
    for(int i = 0; i < number_of_agents; i++) 
    {
      if(other_IP_strings[i] != NULL)
        free(other_IP_strings[i]);
    }
    free(other_IP_strings);
  }
}

void GlobalVariables::Populate(int num_of_agents)  
{
  number_of_agents = num_of_agents;
  min_team_size = 1;
  team_size = 0;
  
  InPorts.resize(number_of_agents);
  OutPorts.resize(number_of_agents);
  InTeam.resize(number_of_agents, false);
  local_ID.resize(number_of_agents, -1);
  planning_iteration.resize(number_of_agents,0);
  nav_state.resize(number_of_agents,0);
  nav_state_iteration.resize(number_of_agents,-1);
  sub_start_and_goal_iteration.resize(number_of_agents, -1);

  vector<float> pose_temp(2, LARGE); 
  sub_start_coords.resize(number_of_agents, pose_temp);
  sub_goal_coords.resize(number_of_agents, pose_temp);
  last_known_pose.resize(number_of_agents, pose_temp);
  pose_iteration.resize(number_of_agents, -1);


  int min_port = 55000;
  for(int i = 0; i < number_of_agents; i++)
  {
    InPorts[i] = min_port; 
    min_port ++;
  }
  for(int i = 0; i < number_of_agents; i++)
  {
    OutPorts[i] = min_port; 
    min_port ++;
  }

  MasterInPort = 57001;
  MasterOutPort = 57002;

  have_info.resize(0);
  have_info.resize(number_of_agents, 0);   // gets set to 1 when we get an agent's sub_start and sub_goal info
  agent_ready.resize(0);
  agent_ready.resize(number_of_agents, 0); // gets set to 1 when an agent is ready to plan
  other_addresses.resize(number_of_agents);

  if(other_IP_strings == NULL)
  {
    other_IP_strings = (char**)malloc(number_of_agents*sizeof(char*));
  
    for(int i = 0; i < number_of_agents; i++)
      other_IP_strings[i] = (char*)malloc(256*sizeof(char));   
  }  

  agent_number = -1;
  
  start_coords.resize(number_of_agents);
  goal_coords.resize(number_of_agents);
  
  planning_time_remaining.resize(number_of_agents, LARGE);
  last_update_time.resize(number_of_agents);
  last_path_conflict_check_time.resize(number_of_agents);

  printf("resetting planning start time\n");
  gettimeofday(&start_time_of_planning, NULL);  
  min_clock_to_plan = 10; // reset later

  last_known_dist.resize(number_of_agents, LARGE);
  last_known_time.resize(number_of_agents, clock());
  
  sync_message_wait_time = 1;
  message_wait_time = 1;
  
  robot_radius = 1;
  prob_at_goal = .8;
  move_max = 1;
  theta_max = 2*PI;
  resolution = .01;
  angular_resolution = .3;
  planning_border_width = 1;
  
  master_reset = true;
  printf("master reset in populate\n");
  kill_master = false;
  done_planning = false;
  
  start_time = clock();
  MAgSln = NULL;

  found_single_robot_solution = false;
  single_robot_solution.resize(0);

  other_robots_single_solutions.resize(number_of_agents);
  planning_iteration_single_solutions.resize(number_of_agents, -1);

  have_calculated_start_and_goal = false;
  use_sub_sg = false;

  revert_to_single_robot_path = false;

  default_map_x_size = -1;
  default_map_y_size = -1;

  sender_Ad_Hoc_running = false;
  listener_active = false;
}

// resets globals for a new planning cycle
void GlobalVariables::Reset()  
{
  // at this point master_reset is true because globals may be changing and being reset, it will only be set to false lower down in this loop,
  // which signals to other threads that it is safe to continue
  // in case it is not already true, we'll make it true
  master_reset = true;

  // waiting here until other threads are safe (not using Globals)
  while(sender_Ad_Hoc_running || listener_active)
  {
    usleep(100000); // sleep for 1/10 sec
  }

  done_planning = false;

  have_info.resize(0);
  have_info.resize(Globals.number_of_agents, 0);   // gets set to 1 when we get an agent's info
        
  agent_ready.resize(0); 
  agent_ready.resize(Globals.number_of_agents, 0); // gets set to 1 when we get an agent's info
    
  have_info[0] = 1;
  agent_ready[0] = 1;
          
  last_update_time.resize(0);
  last_path_conflict_check_time.resize(0);
  timeval temp_time;
  gettimeofday(&temp_time, NULL);
  last_update_time.resize(number_of_agents, temp_time);
  last_path_conflict_check_time.resize(number_of_agents, temp_time);

  start_time_of_planning = temp_time;
  min_clock_to_plan = min_clock_to_plan;

  planning_time_remaining.resize(0);
  planning_time_remaining.resize(Globals.number_of_agents, LARGE);
    
  MAgSln = NULL;

  have_calculated_start_and_goal = false;
 
  start_coords.resize(0);
  start_coords.resize(Globals.number_of_agents);
  goal_coords.resize(0);
  goal_coords.resize(Globals.number_of_agents);


  master_reset = false;  // now set master_reset to false indicating that "perminate" robot data in globals is stable
                         // this should be the only place in the code where master_reset is set to false
}


// sets up global address data for the agent with ag_id using the IP_string
bool GlobalVariables::set_up_agent_address(int ag_id, const char* IP_string)
{
  strcpy(other_IP_strings[ag_id], IP_string);
                     
  // set up address stuff for this robot       
  struct hostent *hp_this = gethostbyname(IP_string);
          
  if (hp_this==0) 
  {
    error("Unknown host");
    return false;
  }

  // populate master_address structure
  other_addresses[ag_id].sin_family = AF_INET;
          
  bcopy((char *)hp_this->h_addr, (char *)&other_addresses[ag_id].sin_addr, hp_this->h_length); // copy in address
  other_addresses[ag_id].sin_port = htons(InPorts[ag_id]);       
  have_info[ag_id] = 1;   
  
  return true;
}


// returns true if we have data for all members of the team
bool GlobalVariables::have_all_team_start_and_goal_data()
{
  for(int i = 0; i < team_size; i++)
  {         
    if(have_info[i] == 0)
      return false;
  }
  return true;
}




// returns true if all agents in the team are ready to plan
bool GlobalVariables::all_team_ready_to_plan()
{
  for(int i = 0; i < team_size; i++)
  {
    if(agent_ready[i] == 0)
      return false;
  }
  return true;
}

void GlobalVariables::broadcast(void* buffer, size_t buffer_size) // sends data to all robots we have info from
{
  for(int i = 0; i < number_of_agents; i++)
  {
    if(i == agent_number)
      continue;
    if(!InTeam[i])
      continue;
    if(have_info[local_ID[i]] == 0)
      continue;
    
    
    int sent_size = sendto(my_out_sock, buffer, buffer_size, 0, (struct sockaddr *)&other_addresses[i], sizeof(struct sockaddr_in));  // send this agent's IP to the master
    if(sent_size < 0) 
      error("Problems sending data 2a");
  } 
}

void GlobalVariables::hard_broadcast(void* buffer, size_t buffer_size) // sends data to all robots we think may exist, regardless of if we have info from them
{
  //printf("buffer: %s \n", buffer);
  for(int i = 0; i < number_of_agents; i++)
  {
    if(i == agent_number)
      continue;
    
    int sent_size = sendto(my_out_sock, buffer, buffer_size, 0, (struct sockaddr *)&other_addresses[i], sizeof(struct sockaddr_in));  // send this agent's IP to the master
    if(sent_size < 0) 
      error("Problems sending data 2b");
  } 
}

// puts data about all robots the sender knows about into the buffer, returns the number of chars that it required
int GlobalVariables::populate_buffer_with_all_robot_data(char* buffer)
{
  // update pose
  last_known_pose[agent_number][0] = robot_pose->x;
  last_known_pose[agent_number][1] = robot_pose->y;
  pose_iteration[agent_number]++;

  int index = 0;

  buffer[index] = 'S';    // data about the sender
  index++;

  // normal data
  index += add_int_to_buffer(                agent_number,                                        (void*)((size_t)buffer + (size_t)index));
  index += add_int_to_buffer(                planning_iteration[agent_number],                    (void*)((size_t)buffer + (size_t)index));
  index += add_int_to_buffer(                nav_state_iteration[agent_number],                   (void*)((size_t)buffer + (size_t)index));
  index += add_int_to_buffer(                nav_state[agent_number],                             (void*)((size_t)buffer + (size_t)index));
  index += add_int_to_buffer(                pose_iteration[agent_number],                        (void*)((size_t)buffer + (size_t)index));
  index += add_1d_float_vector_to_buffer(    last_known_pose[agent_number],                       (void*)((size_t)buffer + (size_t)index));
  index += add_int_to_buffer(                planning_iteration_single_solutions[agent_number],   (void*)((size_t)buffer + (size_t)index));
  index += add_2d_float_vector_to_buffer(    single_robot_solution,                               (void*)((size_t)buffer + (size_t)index));
  index += add_int_to_buffer(                sub_start_and_goal_iteration[agent_number],          (void*)((size_t)buffer + (size_t)index));
  index += add_1d_float_vector_to_buffer(    sub_start_coords[agent_number],                      (void*)((size_t)buffer + (size_t)index));
  index += add_1d_float_vector_to_buffer(    sub_goal_coords[agent_number],                       (void*)((size_t)buffer + (size_t)index));

  // extra data about sender's team
  index += add_int_to_buffer(                team_size,                                           (void*)((size_t)buffer + (size_t)index));
  index += add_1d_int_vector_to_buffer(      global_ID,                                           (void*)((size_t)buffer + (size_t)index));
  index += add_1d_int_vector_to_buffer(      agent_ready,                                         (void*)((size_t)buffer + (size_t)index));
  index += add_1d_int_vector_to_buffer(      agent_moving,                                        (void*)((size_t)buffer + (size_t)index));
  index += add_1d_float_vector_to_buffer(    team_bound_area_min,                                 (void*)((size_t)buffer + (size_t)index));
  index += add_1d_float_vector_to_buffer(    team_bound_area_size,                                (void*)((size_t)buffer + (size_t)index));

  // add all other robot data this agent knows about
  for(int lcl_id = 1; lcl_id < team_size; lcl_id++)
  {
    int glbl_id = global_ID[lcl_id];

    buffer[index] = 'R'; // data about another robot
    index++;

    //normal data
    index += add_int_to_buffer(               glbl_id,                                           (void*)((size_t)buffer + (size_t)index));
    index += add_int_to_buffer(               planning_iteration[glbl_id],                       (void*)((size_t)buffer + (size_t)index));
    index += add_int_to_buffer(               nav_state_iteration[glbl_id],                      (void*)((size_t)buffer + (size_t)index));
    index += add_int_to_buffer(               nav_state[glbl_id],                                (void*)((size_t)buffer + (size_t)index));
    index += add_int_to_buffer(               pose_iteration[glbl_id],                           (void*)((size_t)buffer + (size_t)index));
    index += add_1d_float_vector_to_buffer(   last_known_pose[glbl_id],                          (void*)((size_t)buffer + (size_t)index));
    index += add_int_to_buffer(               planning_iteration_single_solutions[glbl_id],      (void*)((size_t)buffer + (size_t)index));
    index += add_2d_float_vector_to_buffer(   other_robots_single_solutions[glbl_id],            (void*)((size_t)buffer + (size_t)index));
    index += add_int_to_buffer(               sub_start_and_goal_iteration[glbl_id],             (void*)((size_t)buffer + (size_t)index));
    index += add_1d_float_vector_to_buffer(   sub_start_coords[glbl_id],                         (void*)((size_t)buffer + (size_t)index));
    index += add_1d_float_vector_to_buffer(   sub_goal_coords[glbl_id],                          (void*)((size_t)buffer + (size_t)index));
  }

  buffer[index] = 'Z'; // end of robot message data
  index++;

  return index;
}


bool GlobalVariables::recover_all_robot_data_from_buffer(char* buffer, int &index) // gets robot data out of the buffer, updates index, returns true if the planning iteration changes due to what was in the buffer
{
  bool need_to_join_teams = false;
  bool planning_iteration_increase = false;

  // holds normal data
  int ag_gbl_id;
  int pln_itr;
  int nav_st_it;
  int nav_st;
  int pose_it;
  vector<float> last_pose;
  int planning_it_single_sln;
  vector<vector<float> >  single_sln;
  int sub_s_and_g_it;
  vector<float> sub_s;
  vector<float> sub_g;   
   
  // holds extra data about sender's team
  int senders_team_size;
  vector<int> senders_team_global_ID;
  vector<int> senders_team_agent_ready;
  vector<int> senders_team_agent_moving;
  vector<float> senders_team_bound_area_min;
  vector<float> senders_team_bound_area_size;


  if(buffer[index] == 'S') // contains sender data
  {
    index++;

    // normal data
    index += extract_int_from_buffer(               ag_gbl_id,                    (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               pln_itr,                      (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               nav_st_it,                    (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               nav_st,                       (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               pose_it,                      (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_float_vector_from_buffer(   last_pose,                    (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               planning_it_single_sln,       (void*)((size_t)buffer + (size_t)index));
    index += extract_2d_float_vector_from_buffer(   single_sln,                   (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               sub_s_and_g_it,               (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_float_vector_from_buffer(   sub_s,                        (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_float_vector_from_buffer(   sub_g,                        (void*)((size_t)buffer + (size_t)index));

    // extra data about sender's team
    index += extract_int_from_buffer(               senders_team_size,            (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_int_vector_from_buffer(     senders_team_global_ID,       (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_int_vector_from_buffer(     senders_team_agent_ready,     (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_int_vector_from_buffer(     senders_team_agent_moving,    (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_float_vector_from_buffer(   senders_team_bound_area_min,  (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_float_vector_from_buffer(   senders_team_bound_area_size, (void*)((size_t)buffer + (size_t)index));


    bool overlap = false;

    // maintainance of this agent vs the sender 
    if(InTeam[ag_gbl_id] && pln_itr >= planning_iteration[agent_number]) // the sender is in our team already, and its planning iterate is the same or more
    {
      // check if the sender has added new members to its team that are not in our team
      for(int i = 0; i < senders_team_size; i++)
      {    
        if(!InTeam[senders_team_global_ID[i]]) // there is a member of the sender's team that is not in our team
        {
          need_to_join_teams = true;
        }
      }
    }
    else if(!InTeam[ag_gbl_id]) // the sender is not in our team already
    {
      // check if sender's team needs to be combined with our team
      if(JOIN_ON_OVERLAPPING_AREAS)
      {
        // check if message planning bounds intersects with this agent's team's planning bounds
        if(team_bound_area_min.size() > 1 && team_bound_area_size.size() > 1)
        {
          overlap = quads_overlap(senders_team_bound_area_min[0], senders_team_bound_area_size[0], 
                                  senders_team_bound_area_min[1], senders_team_bound_area_size[1], 
                                  team_bound_area_min[0], team_bound_area_size[0], 
                                  team_bound_area_min[1], team_bound_area_size[1]);

          if(overlap)              // need to join due to overlap
          {

            float dist_to_sender = euclid_dist(last_pose, last_known_pose[agent_number]);
            if(dist_to_sender > path_conflict_combine_dist)
            {
              printf("Planning area overlaps with an agent not yet in our team, but it is too far away\n");  
            }
            else
            { 
              printf("Planning area overlaps with an agent not yet in our team, and it is near\n");
              need_to_join_teams = true;
            }
          }
        }
      }

      // check if this agent thinks we are in its team
      bool team_includes_this_ag = false;
      for(int i = 0; i < senders_team_size; i++)
      {    
        if(senders_team_global_ID[i] == agent_number)
        {
          team_includes_this_ag = true;
          break;
        }
      }

      if(team_includes_this_ag && pln_itr > planning_iteration[ag_gbl_id])   
      {
        // need to join because the sending agent thinks we are in its team, but we currently don't think so, 
        // and this is a new planning iteration for the sender (needed since we may have been in an old team but are not any more)
        printf("Recieved a message from an agent that has added us to their team\n");
        need_to_join_teams = true;
      }
    }

    // if we need to join teams, then join them here
    if(need_to_join_teams)
    {
      printf("----------------------- need to join teams (due to message) -------------------------\n");

      for(int i = 0; i < senders_team_size; i++)
      {   
        int temp_ag = senders_team_global_ID[i];

        if(!InTeam[temp_ag])
        {
          InTeam[temp_ag] = true;
          local_ID[temp_ag] = team_size;
          global_ID.push_back(temp_ag);
          team_size++; 
        }
      }

      //  increase our planning iteration
      if(overlap || team_size > senders_team_size)  // second case handles the case where the sender still needs to know about members of our team
      {
        // active combine

        // increase this robot's planning iteration and makes sure it is larger than sender's planning iteration too
        if(pln_itr > planning_iteration[agent_number])
        {
          planning_iteration[agent_number] = pln_itr + 1;
        }
        else 
        {
          planning_iteration[agent_number]++;
        } 
      }
      else 
      {
        // passive combine
        planning_iteration[agent_number] = pln_itr;
      }

      planning_iteration_increase = true;
    }

    // update state information about sender
    if(pln_itr > planning_iteration[ag_gbl_id]) // new planning iteration for this robot
    {
      planning_iteration[ag_gbl_id] = pln_itr;
    }
    else if(pln_itr > planning_iteration[ag_gbl_id] && nav_st_it > nav_state_iteration[ag_gbl_id]) // new planning state for this robot
    {
      nav_state_iteration[ag_gbl_id] = nav_st_it;
      nav_state[ag_gbl_id] = nav_st;
    }

    if(pose_it > pose_iteration[ag_gbl_id]) // new pose iteration
    {
      pose_iteration[ag_gbl_id] = pose_it;
      last_known_pose[ag_gbl_id] = last_pose;
    }

    if(planning_it_single_sln >= planning_iteration_single_solutions[ag_gbl_id]) // new single robot plan iteration
    {
      planning_iteration_single_solutions[ag_gbl_id] = planning_it_single_sln;
      other_robots_single_solutions[ag_gbl_id] = single_sln;
    }

    if(sub_s_and_g_it > sub_start_and_goal_iteration[ag_gbl_id])  // new start and goal iteration
    {
      sub_start_and_goal_iteration[ag_gbl_id] = sub_s_and_g_it;
      sub_start_coords[ag_gbl_id] = sub_s;
      sub_goal_coords[ag_gbl_id] = sub_g;

      if(InTeam[ag_gbl_id])
      {
        int local_an_id = local_ID[ag_gbl_id];

        if(start_coords[local_an_id].size() < 3)
          start_coords[local_an_id].resize(3); 
        start_coords[local_an_id][0] = sub_s[0];
        start_coords[local_an_id][1] = sub_s[1];
        start_coords[local_an_id][2] = 0;
        
        if(goal_coords[local_an_id].size() < 3)
          goal_coords[local_an_id].resize(3);
        goal_coords[local_an_id][0] = sub_g[0];
        goal_coords[local_an_id][1] = sub_g[1];
        goal_coords[local_an_id][2] = 0;
      
        have_info[local_an_id] = 1;     
        
        printf("recieved new data directly from %d: \n", ag_gbl_id);
        printf("start: [%f %f %f] \n", start_coords[local_an_id][0], start_coords[local_an_id][1], start_coords[local_an_id][2]);
        printf("goal:  [%f %f %f] \n", goal_coords[local_an_id][0], goal_coords[local_an_id][1], goal_coords[local_an_id][2]);
     }
    }
  }

  while(buffer[index] == 'R') // contains other robot data
  {
    index++;
    
    // normal data
    index += extract_int_from_buffer(               ag_gbl_id,                    (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               pln_itr,                      (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               nav_st_it,                    (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               nav_st,                       (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               pose_it,                      (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_float_vector_from_buffer(   last_pose,                    (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               planning_it_single_sln,       (void*)((size_t)buffer + (size_t)index));
    index += extract_2d_float_vector_from_buffer(   single_sln,                   (void*)((size_t)buffer + (size_t)index));
    index += extract_int_from_buffer(               sub_s_and_g_it,               (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_float_vector_from_buffer(   sub_s,                        (void*)((size_t)buffer + (size_t)index));
    index += extract_1d_float_vector_from_buffer(   sub_g,                        (void*)((size_t)buffer + (size_t)index));

    if(ag_gbl_id == agent_number)  // this is data about the recieving robot
    {
      continue;
    }

    // update state information about the other robot
    if(pln_itr > planning_iteration[ag_gbl_id]) // new planning iteration for this robot
    {
      planning_iteration[ag_gbl_id] = pln_itr;
    }
    else if(pln_itr > planning_iteration[ag_gbl_id] && nav_st_it > nav_state_iteration[ag_gbl_id]) // new planning state for this robot
    {
      nav_state_iteration[ag_gbl_id] = nav_st_it;
      nav_state[ag_gbl_id] = nav_st;
    }

    if(pose_it > pose_iteration[ag_gbl_id]) // new pose iteration
    {
      pose_iteration[ag_gbl_id] = pose_it;
      last_known_pose[ag_gbl_id] = last_pose;
    }

    if(planning_it_single_sln >= planning_iteration_single_solutions[ag_gbl_id]) // new single robot plan iteration
    {
      planning_iteration_single_solutions[ag_gbl_id] = planning_it_single_sln;
      other_robots_single_solutions[ag_gbl_id] = single_sln;
    }

    if(sub_s_and_g_it > sub_start_and_goal_iteration[ag_gbl_id])  // new start and goal iteration
    {
      sub_start_and_goal_iteration[ag_gbl_id] = sub_s_and_g_it;
      sub_start_coords[ag_gbl_id] = sub_s;
      sub_goal_coords[ag_gbl_id] = sub_g;

      if(InTeam[ag_gbl_id])
      {
        int local_an_id = local_ID[ag_gbl_id];

        if(start_coords[local_an_id].size() < 3)
          start_coords[local_an_id].resize(3); 
        start_coords[local_an_id][0] = sub_s[0];
        start_coords[local_an_id][1] = sub_s[1];
        start_coords[local_an_id][2] = 0;
        
        if(goal_coords[local_an_id].size() < 3)
          goal_coords[local_an_id].resize(3);
        goal_coords[local_an_id][0] = sub_g[0];
        goal_coords[local_an_id][1] = sub_g[1];
        goal_coords[local_an_id][2] = 0;
      
        have_info[local_an_id] = 1;     
        
        printf("recieved new data from %d: \n", ag_gbl_id);
        printf("start: [%f %f %f] \n", start_coords[local_an_id][0], start_coords[local_an_id][1], start_coords[local_an_id][2]);
        printf("goal:  [%f %f %f] \n", goal_coords[local_an_id][0], goal_coords[local_an_id][1], goal_coords[local_an_id][2]);
      }
    }
  }

  if(buffer[index] == 'Z') // done with robot data
  {
    index++;
  }


  // now check if any members in our team have increased their planning iteration above our planning iteration
  for(int i = 1; i < team_size; i++)
  {    
    if(planning_iteration[global_ID[i]] >  planning_iteration[agent_number])
    {
      planning_iteration[agent_number] = planning_iteration[global_ID[i]];
      planning_iteration_increase = true;
    }
  }

  return planning_iteration_increase;
}


// checks if this team needs to be joined with another, if so then it joins them and returns true
bool GlobalVariables::JoinedTeams()
{
  bool joined_teams = false;

  // if we are not joining teams based on overlapping planning areas, then we need to periodically 
  // check if any other_robots_single_solutions conflict with our single robot solution
  if(!JOIN_ON_OVERLAPPING_AREAS)
  {
    if(found_single_robot_solution) //we have a solution
    {
      // check for conflicts vs robots not in our team
      timeval time_now;  
      gettimeofday(&time_now, NULL);
      for(int temp_ag = 0; temp_ag < number_of_agents; temp_ag++)
      {
        if(InTeam[temp_ag]) // already in our team
        {
          continue;
        }

        bool need_to_join_teams = false;
        if(difftime_timeval(time_now, last_path_conflict_check_time[temp_ag]) > .1)  // only check vs each agent every .1 secs
        { 
          if(other_robots_single_solutions[temp_ag].size() < 1) // make sure we have temp_ag's solution
            continue;

          vector<float> A_conflict; // dummy data holder 
          vector<float> B_conflict; // dummy data holder
          float time_resolution = .05;

          //printf("__________________ checking for conflicts vs agent %d ____________\n", temp_ag);

          float this_dist = euclid_dist(last_known_pose[temp_ag], last_known_pose[agent_number]);
          if(find_first_time_conflict_points(other_robots_single_solutions[temp_ag], single_robot_solution, robot_radius, time_resolution, A_conflict, B_conflict)) 
          {
            // the paths conflicts, calculate distance to that agent's last known point based on the first point in either path
            printf("agent %d conflicts with me (%d)\n",temp_ag, agent_number);

            if(this_dist > path_conflict_combine_dist)
            {
              printf("Path conflicts with an agent not yet in our team\n");
              printf("... but it is too far away to care about right now (%f)\n", this_dist);
            }
            else
            {
              printf("Path conflicts with an agent not yet in our team\n");
              need_to_join_teams = true;
            }
          }

          if(this_dist < combine_dist)
          {
             printf("join based on being too close right now (%f)\n", this_dist);
             need_to_join_teams = true;
          }

          last_path_conflict_check_time[temp_ag] = time_now;
        }
    
        if(need_to_join_teams)     // for overlap or because we were in their team, based on what was found above
        {
          printf("----------------------- need to join teams -------------------------\n");
   
          InTeam[temp_ag] = true;
          local_ID[temp_ag] = team_size;
          global_ID.push_back(temp_ag);
          team_size++;
           
          // active combine, so make our planning iteration larger (and larger than the new member's) 
          if(planning_iteration[temp_ag] > planning_iteration[agent_number])
          {
            planning_iteration[agent_number] = planning_iteration[temp_ag]+1;
          }
          else
          {
            planning_iteration[agent_number]++;
          }

          joined_teams = true;        
        }
      }
    }
  }
  return joined_teams;
}


float GlobalVariables::calculate_time_left_for_planning()  // based on info from all agents, this returns the time that remains for planning
{
  float time_for_planning_remaining = LARGE;      
  timeval time_now;  
  gettimeofday(&time_now, NULL);

  float time_elapsed_since_last = difftime_timeval(time_now, last_update_time[agent_number]);

  if(time_elapsed_since_last < .01) // if not much time has elapsed since the last update, then just return based on that
    return planning_time_remaining[agent_number] - time_elapsed_since_last;

  // otherwize update remaining time for all agents

  // find minimum planning time left considering all agents
  for(int i = 0; i < number_of_agents; i++)
  {
    if(planning_time_remaining[i] == LARGE) // no planning time data from this agent yet
      continue;
 
    if(i == agent_number)
    {
      float time_elapsed_since_planning_started = difftime_timeval(time_now, start_time_of_planning);
      planning_time_remaining[i] = min_clock_to_plan - time_elapsed_since_planning_started;
      last_update_time[i] = time_now;
    }
    else
    {
      planning_time_remaining[i] -= time_elapsed_since_last;
      last_update_time[i] = time_now;
    }

    if(planning_time_remaining[i] < time_for_planning_remaining)
      time_for_planning_remaining = planning_time_remaining[i];
  }
  
  // reset this agent's time left for planning to be equal to the (minimum) ammount of time left for planning by anybody
  planning_time_remaining[agent_number] = time_for_planning_remaining;
  
  return time_for_planning_remaining;
}


bool GlobalVariables::have_all_team_single_paths()         // returns true if we have all team members current single paths, else false
{
  // make sure data exists to check
  if(global_ID.size()< 1 || planning_iteration_single_solutions.size() < 1)
    return false;

  for(int tm = 1; tm < team_size; tm ++)
  {
    int glbl_id = global_ID[tm];

    if(planning_iteration_single_solutions[glbl_id] != planning_iteration[agent_number])
    {
      return false;
    }
  }

  return true;
}

void GlobalVariables::output_state_data()
{
  for(int i = 0; i < number_of_agents; i++)
  {
    if(!InTeam[i])
    {
      printf("(%d.%d %d %d %d), ", planning_iteration[i], nav_state_iteration[i], pose_iteration[i], sub_start_and_goal_iteration[i], planning_iteration_single_solutions[i]);
    }
    else if(i == agent_number)
    {
      printf("[%d.%d %d %d %d], ", planning_iteration[i], nav_state_iteration[i], pose_iteration[i], sub_start_and_goal_iteration[i], planning_iteration_single_solutions[i]);
    }
    else
    {
      printf("<%d.%d %d %d %d>, ", planning_iteration[i], nav_state_iteration[i], pose_iteration[i], sub_start_and_goal_iteration[i], planning_iteration_single_solutions[i]);
    }
  }
  printf("\n");
}


void output_pulse(clock_t & last_listener_pulse, float pulse_time, const char* str)
{
  clock_t now_time_pulse = clock();
  if(difftime_clock(now_time_pulse, last_listener_pulse) > pulse_time)
  {
    last_listener_pulse = now_time_pulse;

    printf("%s", str);
  }
}

// this thread always listens for incomming messages from other robots
void *Robot_Listner_Ad_Hoc(void * inG)
{
  GlobalVariables* G = (GlobalVariables*)inG; 
 
  while(G->master_reset)
  {
    // if master is resetting, then wait here while globals are reset
    G->listener_active = false;
    sleep(1);                        /// !!!!!!!!!!!!!!!!!!!! make shorter
  }

  G->listener_active = true;
 
  struct sockaddr_in my_address, senders_address;
  int my_address_length, senders_address_length;
  int in_socket;
  int message_length;
  int in_port;
    
  in_port = G->InPorts[G->agent_number];    

  printf("listener: ad-hoc listener thread\n"); 
  
  // create socket
  in_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if(in_socket < 0)  // failed to open socket
    error("listener: Problems opening socket\n");
  
  // clear all memory of my_address structure
  my_address_length = sizeof(my_address);
  memset(&my_address, NULL, my_address_length); 
   
  // populate my_address structure
  my_address.sin_family = AF_INET;
  my_address.sin_addr.s_addr = INADDR_ANY; // ip address of this machine
  my_address.sin_port = htons(in_port);  // htons() converts 'number' to proper network byte order
  
  // bind in_socket with my_address
  if(bind(in_socket, (struct sockaddr *)&my_address, my_address_length)<0) 
    error("listener: problems binding in_socket");
  
  senders_address_length = sizeof(struct sockaddr_in);  // get the memory size of a sockaddr_in struct 
  char planning_message_buffer[max_message_size];
  //clock_t last_listener_pulse = clock();
  //float pulse_time = 1;

  while(!G->kill_master)
  {
    while(G->master_reset)
    {
      printf("(listener) waiting until master reset is done \n");
      sleep(1);         // MAKE SHORTER !!!!!!!!!!!!!!!!!!
    }

    while(!G->kill_master && !G->master_reset) // this thread is responsible for reading in data from other processes
    {  
      G->listener_active = true;

      printf("listener: waiting for messages\n");

      memset(&planning_message_buffer,'\0',sizeof(planning_message_buffer)); 
      message_length = recvfrom(in_socket, planning_message_buffer, sizeof(planning_message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks until a message is recieved

      int message_ptr = 0;
      if(message_length < 0)
      { 
        printf("listener: had problems getting a message \n");
        continue;
      }

      if(planning_message_buffer[message_ptr] == 'S') // message contains robot data
      {
        if(G->recover_all_robot_data_from_buffer(planning_message_buffer, message_ptr))
        {
          G->master_reset = true;   
          printf("master reset due to incriment from message \n");
          continue;
        }
        if(G->JoinedTeams())
        {
          G->master_reset = true;   
          printf("master reset due to team join \n");
          continue;
        }
      }

      if(planning_message_buffer[message_ptr] == '2') // it has planning data in it;
      {
        //printf("contains planning data \n");
        message_ptr++;
        int agent_sending;

        if(sscanf(&(planning_message_buffer[message_ptr]),"%d", &agent_sending) < 1)
          agent_sending = -1;        
        else if( agent_sending < 10)       
          message_ptr++;
        else if( agent_sending < 100)       
          message_ptr += 2;
        else if( agent_sending < 1000)       
          message_ptr += 3;
        else
          printf("listener: error: agent id >= 1000 \n");
     
        //printf("recieved message from agent %d:\n%c\n", agent_sending, &(planning_message_buffer[message_ptr]));
        if(agent_sending < 0)
        {
           // don't know who sent the message, so ignore 
           //printf("s:%d (b)\n", agent_sending);
        }
        else if((MultiAgentSolution*)G->MAgSln == NULL )  // this case keeps the next check from exploding, especially after a master reset
        {
          // can also get in here after a rever to single robot path, in which case we do not need to worry about path-planning messages

          //printf("(listener) waiting while things reset \n"); 
        }
        else if(G->InTeam[agent_sending] && agent_sending < (int)(((MultiAgentSolution*)G->MAgSln)->in_msg_ctr.size())) // last case checks for when messages are recieved before MultAgSln is populated
        { 
          // put the message into a file             
          // printf("((MultiAgentSolution*)G->MAgSln)->in_msg_ctr[agent_sending]: %d\n", ((MultiAgentSolution*)G->MAgSln)->in_msg_ctr[agent_sending]);
   
          //printf("filing message from %d +++++++++++++++++++++++++++++++++++++\n", agent_sending);  
              
          char this_file[100];
          sprintf(this_file, "%s/%d_to_%d_%d.txt", message_dir, agent_sending, G->agent_number, ((MultiAgentSolution*)(G->MAgSln))->in_msg_ctr[agent_sending]);
          
          //printf("attempting to open: %s \n",this_file);
       
          FILE* ofp = fopen(this_file,"w");
          //               printf("here 33.3 \n");
          if(ofp == NULL) // problem opening file
          {
            printf("listener: cannot open message file for writing\n");
            continue;
          }

          fprintf(ofp, "%s\n", &(planning_message_buffer[message_ptr]));
          fclose(ofp);
          
          if(G->local_ID[agent_sending] != -1)
          {
            // mark that this agent is planning
            G->agent_ready[G->local_ID[agent_sending]] = 1;
          }
        }
    
      }
      else if(planning_message_buffer[message_ptr] == 3) // it has a kill message in it
      {
        G->kill_master = true;
      }
      else if(planning_message_buffer[message_ptr] == 4) // only had robot data
      {
        //printf("recieved prefered path exchange message\n");
      }
      else if(planning_message_buffer[message_ptr] != '\0')   
      {
        if(G->master_reset)
        {
          // during a master reset not all of a message will be read
          continue;
        }
        printf("listener: recieved unknown message type --\n%s\n",  &(planning_message_buffer[message_ptr]));    
      }
    }
  }
  
  // this thread terminates
  return NULL;
}

// this thread sends this robot's data to other robots durring the start up sync phase, then terminates
// also terminates on master_reset = true
void *Robot_Data_Sync_Sender_Ad_Hoc(void * inG)
{
  // =================================== exchange prefered path phase and calculate start/goal phase (1) ===================================

  GlobalVariables* G = (GlobalVariables*)inG;  
  char buffer[max_message_size];    
   
  G->sender_Ad_Hoc_running = true;
  printf("startup sender: ad-hoc sender start-up thread \n");  

  // create an outgoing socket
  G->my_out_sock = socket(AF_INET, SOCK_DGRAM, 0); 
  if(G->my_out_sock < 0)        // failed to create socket
    error("startup sender: problems creating socket");

  // wait until we have calculated the start and goal
  while(!G->have_calculated_start_and_goal && !G->master_reset && !G->revert_to_single_robot_path)
  {
    printf("startup sender: waiting to calculate this agents start and goal\n");

    int index = G->populate_buffer_with_all_robot_data(buffer);
    buffer[index] = 4; // this signals that all this message contained was robot data
    index++;

    G->hard_broadcast((void *)buffer, sizeof(char) * index);

    usleep(G->sync_message_wait_time*1000000);
  }

  if(G->master_reset)
  {
    printf("startup sender: sender thread exiting due to master reset 0 \n");
    G->sender_Ad_Hoc_running = false;
    return NULL;
  }
  
  if(G->revert_to_single_robot_path)
  {
    printf("startup sender: sender thread exiting due to revert to single robot path 0\n");
    G->sender_Ad_Hoc_running = false;
    return NULL;
  }



  // =========================================== exchange start/goal phase (2) ===========================================

  while(!G->have_all_team_start_and_goal_data() && !G->master_reset && !G->revert_to_single_robot_path) // until we have the teams's start/goal data
  { 
    printf("startup sender: waiting for agent start and goal coords\n"); 
  
    int index = G->populate_buffer_with_all_robot_data(buffer);
    buffer[index] = 4; // this signals that all this message contained was robot data
    index++;

    G->hard_broadcast((void *)buffer, sizeof(char) * index);

    usleep(G->sync_message_wait_time*1000000);
  }

  if(G->master_reset)
  {
    printf("startup sender: sender thread exiting due to master reset 1 \n");
    G->sender_Ad_Hoc_running = false;
    return NULL;
  }
  
  if(G->revert_to_single_robot_path)
  {
    printf("startup sender: sender thread exiting due to revert to single robot path 1\n");
    G->sender_Ad_Hoc_running = false;
    return NULL;
  }

  printf("startup sender: we have min number of start and goal locations to start planning\n");  


  // =========================================== planning phase (3) ===========================================

  // now we start path planning, but this thread still keeps broadcasting the data until all robot in our team are also planning
  while(!G->all_team_ready_to_plan() && !G->master_reset && !G->revert_to_single_robot_path) // until the rest of the team is ready to plan
  {       
    printf("startup sender: waiting until all of team is ready to plan\n");

    int index = G->populate_buffer_with_all_robot_data(buffer);
    buffer[index] = 4; // this signals that all this message contained was robot data
    index++;

    G->hard_broadcast((void *)buffer, sizeof(char) * index);

    usleep(G->sync_message_wait_time*1000000);
  }
  
  if(G->master_reset)
  {
    printf("(startup sender) sender thread exiting due to master reset 2 \n");
    G->sender_Ad_Hoc_running = false;
    return NULL;
  }
  
  if(G->revert_to_single_robot_path)
  {
    printf("(startup sender) sender thread exiting due to revert to single robot path 2\n");
    G->sender_Ad_Hoc_running = false;
    return NULL;
  }

  // now we know the team is all planning, so we exit this thread (note that robot data continues to be sent along with plannng messages in master thread)
  printf("(startup sender) all members of team are planning, exiting ad-hoc sender startup thread \n"); 
  G->sender_Ad_Hoc_running = false;
  return NULL;
} 
