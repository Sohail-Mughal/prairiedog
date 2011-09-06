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
  have_info.resize(number_of_agents, 0);   // gets set to 1 when we get an agent's info
  agent_ready.resize(0);
  agent_ready.resize(number_of_agents, 0); // gets set to 1 when we get an agent's info
  other_addresses.resize(number_of_agents);

  if(other_IP_strings == NULL)
  {
    other_IP_strings = (char**)malloc(number_of_agents*sizeof(char*));
  
    for(int i = 0; i < number_of_agents; i++)
      other_IP_strings[i] = (char*)malloc(256*sizeof(char));   
  }  
  not_planning_yet = true;   
  
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
  an_agent_needs_this_single_path_iteration = false;

  revert_to_single_robot_path = false;

  Globals.default_map_x_size = -1;
  Globals.default_map_y_size = -1;
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

// returns true if we have all other agent's address data
bool GlobalVariables::have_all_agent_addresses()
{
  for(int i = 0; i < number_of_agents; i++)
  {
    if(i == agent_number)
      continue;
         
    if(have_info[i] == 0)
      return false;
  }
            
  return true;  
}

// returns true if we have min number of agent's address data
bool GlobalVariables::have_min_agent_data()
{
  int num = 1;
  for(int i = 0; i < number_of_agents; i++)
  {
    if(i == agent_number)
      continue;
         
    if(have_info[i] != 0)
      num++;
  }
           
  //printf(" have info about %d robots , min : %d \n", num, min_number_of_agents);
  
  if(num >= min_team_size)
    return true;
  
  return false;
}

// returns true if we have data for all members of the team
bool GlobalVariables::have_all_team_data()
{
  for(int i = 0; i < team_size; i++)
  {         
    if(have_info[i] == 0)
      return false;
  }
  return true;
}

// returns true if all agents are ready to plan
bool GlobalVariables::all_agents_ready_to_plan()
{
  for(int i = 0; i < number_of_agents; i++)
  {      
    if(agent_ready[i] == 0)
      return false;
  }
            
  return true;  
}

// returns true if min agents are ready to plan
bool GlobalVariables::min_agents_ready_to_plan()
{
  int num = 1;
  for(int i = 0; i < number_of_agents; i++)
  {
    if(i == agent_number)
      continue;
         
    if(agent_ready[i] != 0)
      num++;
  }
           
  if(num >= min_team_size)
    return true;
  
  return false;
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


// returns true if all agents are moving
bool GlobalVariables::all_agents_moving()
{
  for(int i = 0; i < number_of_agents; i++)
  {      
    if(agent_moving[i] == 0)
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

void GlobalVariables::populate_buffer_with_ips(char* buffer) // puts everybody's ip into a buffer
{
  sprintf(buffer,"%d: ",number_of_agents);  // '\0' removed from end because of warnings
  for(int i = 0; i < number_of_agents; i++)
  {
     strcat(buffer, other_IP_strings[i]);
     strcat(buffer, " ");
  }   
}

void GlobalVariables::recover_ips_from_buffer(char* buffer) // gets everybody's ip out of the buffer
{
  char new_buffer[max_message_size];  
  char single_ip[max_message_size];
  int num;
    
  sscanf(buffer, "%d", &num);
  
  int index = 1;
  
  for(int i = 0; i < num && i < number_of_agents; i++)
  {
    while(buffer[index] != ' ' && buffer[index] != '\0')
     index++;
    index++;
    strcpy(new_buffer, &(buffer[index]));
    sscanf(new_buffer, "%s ", single_ip);
    set_up_agent_address(i, single_ip);
    
    printf("position:%d %s \n", i, other_IP_strings[i]);
  }
}

int GlobalVariables::populate_buffer_with_single_robot_paths(char* buffer) // puts single robot paths in buffer, returns the number of chars that it required
{
  //printf("populating buffer with paths \n");
  int index = 0;

  buffer[index] = 'V';
  index++;

  index += add_int_to_buffer(agent_number, (void*)((size_t)buffer + (size_t)index));

  index += add_int_to_buffer(planning_iteration_single_solutions[agent_number], (void*)((size_t)buffer + (size_t)index));

  index += add_2d_vector_to_buffer(single_robot_solution, (void*)((size_t)buffer + (size_t)index));

  // add all other agents prefered paths that this agent knows about
  for(int lcl_id = 1; lcl_id < team_size; lcl_id++)
  {
    int glbl_id = global_ID[lcl_id];

    buffer[index] = 'V';
    index++;

    index += add_int_to_buffer(glbl_id, (void*)((size_t)buffer + (size_t)index));

    index += add_int_to_buffer(planning_iteration_single_solutions[glbl_id], (void*)((size_t)buffer + (size_t)index));

    index += add_2d_vector_to_buffer(other_robots_single_solutions[glbl_id], (void*)((size_t)buffer + (size_t)index));
  }
  return index;
}

int GlobalVariables::populate_buffer_with_data(char* buffer) // puts this agents ip, start, and goal positions into the buffer, returns the index of '\0' end of the message
{
  char temp[2000];
  sprintf(buffer, "A %d,%d,%f,%f\n", agent_number, planning_iteration[agent_number], robot_pose->x, robot_pose->y);  // message type 'A' from this agent, who is at this pose
  
  //printf("populating buffer \n");
  
  char temptemp[500];
  
  // add the team members
  sprintf(temp, "T: %d\n", team_size);   
  
  for(int i = 0; i < team_size; i++)
  {
    sprintf(temptemp,"%d,", global_ID[i]);  
    strcat(temp, temptemp);
  }
   
  sprintf(temptemp,"\n");  
  strcat(temp, temptemp);
  strcat(buffer, temp);  
  
  // add who is ready to plan
  int num_ready = 0;
  for(int i = 0; i < team_size; i++)
  {
    if(agent_ready[i] == 1)
      num_ready++;
  }
  
  if(num_ready > 0)
  {
    sprintf(temp, "R: %d\n", num_ready);   
  
    for(int i = 0; i < team_size; i++)
    {
      if(agent_ready[i] == 1)
      {
        sprintf(temptemp,"%d,", global_ID[i]);  
        strcat(temp, temptemp);
      }
    }
   
    sprintf(temptemp,"\n");  
    strcat(temp, temptemp);
    strcat(buffer, temp);  
  }
     
  if(team_bound_area_min.size() > 0)
  {
    // add this team's planning bounds
    sprintf(temp,"B: %f %f %f %f %f %f\n", team_bound_area_min[0], team_bound_area_min[1], team_bound_area_min[2], team_bound_area_size[0], team_bound_area_size[1], team_bound_area_size[2]);   
    strcat(buffer, temp);
  }
  
  // for each robot that is in our team that we have info about
  for(int i = 0; i < number_of_agents; i++)
  {
    if(!InTeam[i])
      continue;

    if(have_info[local_ID[i]] == 0)
      continue;
    
    int local_id_temp = local_ID[i];
    
    // agent id, start and goal
    sprintf(temp,"%d %d %f %f %f %f %f %f\n", i, planning_iteration[i], start_coords[local_id_temp][0], start_coords[local_id_temp][1], start_coords[local_id_temp][2], goal_coords[local_id_temp][0], goal_coords[local_id_temp][1], goal_coords[local_id_temp][2]); 
    //printf("adding: %d %d %f %f %f %f %f %f\n", i, planning_iteration[i], start_coords[local_id_temp][0], start_coords[local_id_temp][1], start_coords[local_id_temp][2], goal_coords[local_id_temp][0], goal_coords[local_id_temp][1], goal_coords[local_id_temp][2]);
    strcat(buffer, temp);
  }

  sprintf(temp,"A");   
  strcat(buffer, temp);
  
  //printf("SENDING --------->\n%s\n<---------\n", buffer); 
  int index = 0;  

  // add this agent's prefered path
  //if(found_single_robot_solution)
  //{ 
  //  while(buffer[index] != '\0')
  //    index++;
  //
  // // add single robot prefered paths to buffer
  // printf("here 1 \n");
  // index += populate_buffer_with_single_robot_paths((char*)((size_t)buffer + (size_t)index));
  //
  //}
  //else
  //{
    while(buffer[index] != '\0')
      index++;
  //}

  return index;
}

bool GlobalVariables::recover_data_from_buffer(char* buffer, int &index, vector<int> & robots_in_senders_team) // gets an agents ip, start, and goal position out of the buffer, returns true if we get a message from another team that should be joined with our team, index holds the index directly after data
{  
  index = 0;
  
  if(master_reset)
    return false;
    
  bool need_to_join_teams = false;      
  bool overlap = false;
  if(buffer[0] == 'A') // message type 'A'  
  { 
    int sending_agent = -1;
    int senders_planning_iteration = -1;
    int an_id, ag_pln_it;
    float sx, sy, st, gx, gy, gt;
    float sender_pose_x, sender_pose_y;
    int num = 0;
    bool team_includes_this_ag = false;
    
    // get sending agent id
    if(sscanf(buffer,"A %d,%d,%f,%f\n", &sending_agent,&senders_planning_iteration, &sender_pose_x, &sender_pose_y) < 4)
      return false;

    float d_x = robot_pose->x - sender_pose_x;
    float d_y = robot_pose->y - sender_pose_y;     
    float dist_to_sender = sqrt((d_x*d_x) + (d_y*d_y));   
    last_known_dist[sending_agent] = dist_to_sender;
    last_known_time[sending_agent] = clock();
    
    //printf("parsing header from %d \n", sending_agent);

    
    if(senders_planning_iteration < planning_iteration[sending_agent]) // this message is for an old problem
    {
      printf("old problem --- %d %d\n", senders_planning_iteration, planning_iteration[sending_agent]);
      return false;
    }
    
    // printf("recovering message: %s \n", buffer);
    
    // get to start of next line
    while(buffer[index] != '\n' && buffer[index] != '\0') 
      index++;
    if(buffer[index] == '\0')
      return false;
    index++;

    if(buffer[index] == 'T')
    {
      // find out who is in the sending robot's team 
      int num_in;
      if(sscanf(&(buffer[index]),"T: %d\n", &num_in) < 1)
      {
        printf("problem reading agent in team\n");
        return false;  
      } 
      
      // get to start of next line
      while(buffer[index] != '\n' && buffer[index] != '\0') 
        index++;
      if(buffer[index] == '\0')
        return false;
      index++;
      
      robots_in_senders_team.resize(num_in);
      //printf(" agents in sender's team : ");
      int temp_ag;
      for(int j = 0; j < num_in; j++)
      {
        if(sscanf(&(buffer[index]),"%d,", &temp_ag) < 1)
        {
          printf("problems agent ready to plan\n");
          return false;  
        }   
        
        if(temp_ag == agent_number) // this agent is in the sending robots team
        {
          team_includes_this_ag = true;
          //printf("!->");
        }
        //printf("%d, ", temp_ag);
        robots_in_senders_team[j] = temp_ag; 
        
        while(buffer[index] != ',' && buffer[index] != '\n' && buffer[index] != '\0')
          index++;
        
        if(buffer[index] == ',')
          index++;
      }
      //printf("\n");
      if(buffer[index] != '\n' && buffer[index] != '\0')
        printf("problems, end of line not where expected \n");
      
      index++;   
    }

    if(buffer[index] == 'R')
    {
      // find out who is ready to plan  
      int num_ready;
      if(sscanf(&(buffer[index]),"R: %d\n", &num_ready) < 1)
      {
        printf("problem reading number ready to plan\n");
        return false;  
      } 
      
      // get to start of next line
      while(buffer[index] != '\n' && buffer[index] != '\0') 
        index++;
      if(buffer[index] == '\0')
        return false;
      index++;
      
      //printf("agents in sender's team that are ready to plan: ");
      int temp_ag;
      while(buffer[index] != '\n' && buffer[index] != '\0')
      {
        if(sscanf(&(buffer[index]),"%d,", &temp_ag) < 1)
        {
          printf("problems agent ready to plan\n");
          return false;  
        }   
          
        //printf("%d", temp_ag);
        if(local_ID[temp_ag] >= 0 && local_ID[temp_ag] < team_size) 
          agent_ready[local_ID[temp_ag]] = 1;
        
        while(buffer[index] != ',' && buffer[index] != '\n' && buffer[index] != '\0')
          index++;
        
        if(buffer[index] == ',')
          index++;
      }
      //printf("\n");
      index++;   
    }
        
    if(buffer[index] == 'B')
    {
      // get message planning area bounds
      if(sscanf(&(buffer[index]),"B: %f %f %f %f %f %f\n", &sx, &sy, &st, &gx, &gy, &gt) < 6)
      {
        printf("problem reading planning bounds from message\n");
        return false;  
      }
      
      //printf("sending agent's message bounds : %f %f %f %f %f %f\n", sx, sy, st, gx, gy, gt);
    }
    else
      index--; // hack to make next buffer read a little easier in case planning area bounds not sent    

    // if the sending agent is not in our team, check if it should be
    if(!InTeam[sending_agent] && JOIN_ON_OVERLAPPING_AREAS)
    {
      // check if message planning bounds intersects with this agent's team's planning bounds
      if(team_bound_area_min.size() > 1 && team_bound_area_size.size() > 1)
        overlap = quads_overlap(sx, gx, sy, gy, team_bound_area_min[0], team_bound_area_size[0], team_bound_area_min[1], team_bound_area_size[1]);

      if(overlap)              // need to join due to overlap
      {
        printf("Planning area overlaps with an agent not yet in our team\n");
        need_to_join_teams = true;

        if(dist_to_sender > combine_dist )
        {
          printf("... but it is too far away to care about right now\n");
          overlap = false;   
          need_to_join_teams = false;
        }
      }
    }

    if(!InTeam[sending_agent] && team_includes_this_ag && senders_planning_iteration > planning_iteration[sending_agent])   // need to join because the sending agent thinks we are in its team, but we currently don't think so, and this is a new planning iteration for the sender (needed since we may have been in an old team but are not any more)
    {
      printf("Recieved a message from an agent that has added us to their team\n");
      need_to_join_teams = true;
    }
        
    while(true) // break out when done
    {
      // get to start of next line
      while(buffer[index] != '\n' && buffer[index] != '\0') 
        index++;
      if(buffer[index] == '\0' || buffer[index] == 'A')
        break;
      index++;
      if(buffer[index] == '\0' || buffer[index] == 'A')
        break;
      
      // read this data
      if(sscanf(&(buffer[index]),"%d %d %f %f %f %f %f %f\n", &an_id, &ag_pln_it, &sx, &sy, &st, &gx, &gy, &gt) < 8) 
        continue;
      //printf("read data: %d %d %f %f %f %f %f %f\n", an_id, ag_pln_it, sx, sy, st, gx, gy, gt);   
      
      num++;
      
      int local_an_id = local_ID[an_id];
      
      //printf("local_an_id:%d \n", local_an_id);
      if(local_an_id != -1)
      {  
        if(InTeam[an_id])// new data
        {       
          bool need_to_add_info = false;  
            
          if(have_info[local_an_id] == 0)
          {
            need_to_add_info = true;
            planning_iteration[an_id] = ag_pln_it; 
          }
          else if(ag_pln_it >= planning_iteration[an_id]) //we already had info, but this is potentially new info
          {
            planning_iteration[an_id] = ag_pln_it; 
            
            vector<float> temp_vec(3);
            temp_vec[0] = sx;
            temp_vec[1] = sy;
            temp_vec[2] = st;
            
            if(!equal_float_vector(temp_vec, start_coords[local_an_id], change_plase_thresh)) // start is different
              need_to_add_info = true; 
            
            temp_vec[0] = gx;
            temp_vec[1] = gy;
            temp_vec[2] = gt;
            
            if(!equal_float_vector(temp_vec, goal_coords[local_an_id], change_plase_thresh)) // goal is different
              need_to_add_info = true; 
            
            #ifdef drop_old_robots_from_teams
            if(difftime_clock(clock(), last_known_time[an_id]) > drop_time || last_known_dist[an_id] > drop_dist)
            {
              // either the drop distance or time has been reached, so drop this agent from our team
              need_to_add_info = false; 
            }
            #endif
                    
            if(need_to_add_info) // start or goal of one of the agents has changed (and the robots are still close enough to matter), so it is a new planning problem, update our planning iteration
            {
              planning_iteration[agent_number]++;
              master_reset = true;
            }
          } 
          
          if(need_to_add_info && !master_reset)
          {
            if(start_coords[local_an_id].size() < 3)
              start_coords[local_an_id].resize(3); 
            start_coords[local_an_id][0] = sx;
            start_coords[local_an_id][1] = sy;
            start_coords[local_an_id][2] = st;
        
            if(goal_coords[local_an_id].size() < 3)
              goal_coords[local_an_id].resize(3);
            goal_coords[local_an_id][0] = gx;
            goal_coords[local_an_id][1] = gy;
            goal_coords[local_an_id][2] = gt;
      
            have_info[local_an_id] = 1;     
        
            printf("recieved new data from %d: \n", an_id);
            printf("start: [%f %f %f] \n", start_coords[local_an_id][0], start_coords[local_an_id][1], start_coords[local_an_id][2]);
            printf("goal:  [%f %f %f] \n", goal_coords[local_an_id][0], goal_coords[local_an_id][1], goal_coords[local_an_id][2]);
            //getchar();
          }
        }
      }
    }
    
    if(!InTeam[sending_agent])
    {
      // nothing goes here
      
    }
    #ifdef drop_old_robots_from_teams
    else // sending agent is in our team
    {   
      if(done_planning) // (i.e. not in the middle of calculating a solution)
      {  
        if(dist_to_sender > drop_dist )  // the two agents are far enough away that they are no longer in the same team
        { 
          need_to_join_teams = false;    
        }
      }
    }
    #endif
  }
  else
    printf("asked to parse unknown message type \n");
   
  return need_to_join_teams;
}

void GlobalVariables::tell_master_we_are_moving(void * inG) // tells the master that this robot is moving
{  
  GlobalVariables* G = (GlobalVariables*)inG;  
  struct hostent *hp;
  char buffer[256];
  int message_size;        
  struct sockaddr_in master_address;
  int master_address_length;   

  // create an outgoing socket
  G->my_out_sock = socket(AF_INET, SOCK_DGRAM, 0); 
  if(G->my_out_sock < 0)        // failed to create socket
    error("problems creating socket");
   
  // set up hostent structure with master's data
  hp = gethostbyname(G->master_IP);
  if(hp==0) 
    error("Unknown host");

  // populate master_address structure
  master_address.sin_family = AF_INET;
  bcopy((char *)hp->h_addr, (char *)&master_address.sin_addr, hp->h_length); // copy in address of master
  master_address.sin_port = htons(G->MasterInPort);   // htons() converts 'number' to proper network byte order
  
  master_address_length = sizeof(struct sockaddr_in);  // get the memory size of a sockaddr_in struct
  
  sprintf(buffer, "%d 2 %s\n", G->agent_number, G->my_IP);
  message_size = sendto(G->my_out_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&master_address, master_address_length);  // send info to master

  if(message_size < 0) 
    error("Problems sending data");
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

  if(global_ID.size()< 1 || planning_iteration_single_solutions.size() < 1)
    return false;

  //printf("%d[%d]", global_ID[0], planning_iteration_single_solutions[global_ID[0]]);

  for(int tm = 1; tm < team_size; tm ++)
  {
    int glbl_id = global_ID[tm];

    //printf("%d[%d]", glbl_id, planning_iteration_single_solutions[glbl_id]);

    if(planning_iteration_single_solutions[glbl_id] < planning_iteration_single_solutions[agent_number])
    {
      //printf(" f \n");
      return false;
    }
  }

  //printf(" t \n");
  return true;
}

/* ------------------------- the actual threads ------------------------ */

// void *Master_Listner(void * inG)
// {
//   GlobalVariables* G = (GlobalVariables*)inG;  
//   struct sockaddr_in my_address, senders_address;
//   int my_address_length, senders_address_length;
//   char message_buffer[max_message_size];
//   int message_length;
//   int in_port;
//     
//   if(G->agent_number < 0)
//     in_port = G->MasterInPort;
//   else
//     in_port = G->InPorts[G->agent_number];    
// 
//   printf("Master Listener \n");   
//   
//   // create socket
//   G->my_in_sock = socket(AF_INET, SOCK_DGRAM, 0);
//   if(G->my_in_sock < 0)  // failed to open socket
//   error("Problems opening socket\n");
// 
//   // clear all memory of my_address structure
//   my_address_length = sizeof(my_address);
//   memset(&my_address, NULL, my_address_length); 
//    
//   // populate my_address structure
//   my_address.sin_family = AF_INET;
//   my_address.sin_addr.s_addr = INADDR_ANY; // ip address of this machine
//   my_address.sin_port = htons(in_port);  // htons() converts 'number' to proper network byte order
// 
// 
//   // bind in_socket with my_address
//   if(bind(G->my_in_sock, (struct sockaddr *)&my_address, my_address_length)<0) 
//     error("problems binding in_socket");
// 
//   senders_address_length = sizeof(struct sockaddr_in);  // get the memory size of a sockaddr_in struct  
//         
//   while(!G->have_all_agent_addresses()) // until have all
//   {
//     printf("waiting for data from robots \n"); 
//     memset(&message_buffer,'\0',sizeof(message_buffer)); 
//     message_length = recvfrom(G->my_in_sock, message_buffer, sizeof(message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks untill a message is recieved
//         
//     if(message_length < 0) 
//       printf("had problems getting a message \n");
//     else
//     {   
//       char IP[256];
//       int sending_agent; 
//       int ready; 
//       sscanf(message_buffer, "%d %d %s",&sending_agent, &ready, IP);
//           
//       if(sending_agent < 0)
//         continue;
//           
//       if(ready == 1)
//         G->agent_ready[sending_agent] = 1;
//           
//       if(G->have_info[sending_agent] == 1)
//         continue;
// 
//       if(G->set_up_agent_address(sending_agent, IP)) // then we have data from a new agent
//         printf("Received data from robot: %d %s\n", sending_agent, G->other_IP_strings[sending_agent]);
//     }
//   }
//       
//   printf("have all agent's IP data \n");
//       
//   while(!G->all_agents_ready_to_plan())
//   {
//     char IP[256];
//     int sending_agent; 
//     int ready; 
//     
//     memset(&message_buffer,'\0',sizeof(message_buffer)); 
//     message_length = recvfrom(G->my_in_sock, message_buffer, sizeof(message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks untill a message is recieved
//     sscanf(message_buffer, "%d %d %s",&sending_agent, &ready, IP);
//           
//     if(sending_agent < 0)
//       continue;
//           
//     if(ready == 1)
//       G->agent_ready[sending_agent] = 1;   
//   }
//       
//   printf("all agent's are ready to plan \n");
//   // now all agents are ready to plan, and will be moving shortly
//   
//   while(!G->all_agents_moving())
//   {  
//     char IP[256];
//     int sending_agent; 
//     int ready; 
//     
//     printf("waiting for agents to start moving \n"); 
//     sleep(1);
//     
//     memset(&message_buffer,'\0',sizeof(message_buffer)); 
//     message_length = recvfrom(G->my_in_sock, message_buffer, sizeof(message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks untill a message is recieved
//     sscanf(message_buffer, "%d %d %s",&sending_agent, &ready, IP);
//           
//     if(sending_agent < 0)
//       continue;
//           
//     if(ready == 2)
//       G->agent_moving[sending_agent] = 1;   
//   }
//   
//   // now all agents are moving, this thread terminates
//   return NULL;
// }
// 
// 
// void *Master_Sender(void * inG)
// {
//   GlobalVariables* G = (GlobalVariables*)inG;  
//   char buffer[max_message_size]; 
//  
//   printf("Master Sender \n");
//   
//   // create an outgoing socket
//   G->my_out_sock = socket(AF_INET, SOCK_DGRAM, 0); 
//   if(G->my_out_sock < 0)        // failed to create socket
//     error("problems creating socket");
//   
//   while(!G->have_all_agent_addresses()) // wait until we get ip data from every robot;
//   { 
//     printf("waiting for addresses \n");
//     sleep(1);
//   }
//   while(!G->all_agents_ready_to_plan())
//   {
//     // send IP data about all agents to all agents  
//     printf("distributing addresses \n");
//       
//     G->populate_buffer_with_ips(buffer);
//     
//     printf("sending: %s \n", buffer);
//     G->broadcast(buffer, max_message_size);
//     sleep(1);  
//   }
//   
//   printf("Sender: agents ready to plan\n");
//   
//   for(int i = 0; i < 20; i++)  // send 20 messages to every saying they can start moving
//   {
//     buffer[0] = 1; // plan flag  
//     buffer[1] = '\0';
//     G->broadcast(buffer, max_message_size);  
//   }
//   
//   printf("agents should now be planning \n");
// 
//   while(!G->all_agents_moving())
//   {
//     printf("waiting for agents to start moving \n"); 
//     sleep(1);
//   }
//   
//   // now all agents are moving, so we send kill messages to them, and then kill the master
//   
//   for(int i = 0; i < 20; i++)  // send 20 messages to kill every robot
//   {
//     buffer[0] = 3; // kill flag  
//     buffer[1] = '\0';
//     G->broadcast(buffer, 1024);  
//   }
//   
//   G->kill_master = true;
//   
//   // this thread terminates
//   return NULL;
// }

// void *Robot_Listner(void * inG)
// {
//   GlobalVariables* G = (GlobalVariables*)inG;  
//   struct sockaddr_in my_address, senders_address;
//   int my_address_length, senders_address_length;
//   char message_buffer[max_message_size];
//   int in_socket;
//   int message_length;
//   int in_port;
//     
//   if(G->agent_number < 0)
//     in_port = G->MasterInPort;
//   else
//     in_port = G->InPorts[G->agent_number];    
// 
//   printf("Robot Listener \n"); 
//     
//     
//   // create socket
//   in_socket = socket(AF_INET, SOCK_DGRAM, 0);
//   if(in_socket < 0)  // failed to open socket
//     error("Problems opening socket\n");
// 
//   // clear all memory of my_address structure
//   my_address_length = sizeof(my_address);
//   memset(&my_address, NULL, my_address_length); 
//    
//   // populate my_address structure
//   my_address.sin_family = AF_INET;
//   my_address.sin_addr.s_addr = INADDR_ANY; // ip address of this machine
//   my_address.sin_port = htons(in_port);  // htons() converts 'number' to proper network byte order
// 
// 
//   // bind in_socket with my_address
//   if(bind(in_socket, (struct sockaddr *)&my_address, my_address_length)<0) 
//     error("problems binding in_socket");
// 
//   senders_address_length = sizeof(struct sockaddr_in);  // get the memory size of a sockaddr_in struct 
//       
//   while(!G->have_all_agent_addresses()) // forever
//   {
//     printf("waiting for data from master \n"); 
//     memset(&message_buffer,'\0',sizeof(message_buffer)); 
//     message_length = recvfrom(in_socket, message_buffer, sizeof(message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks until a message is recieved
//     if(message_length < 0) 
//       printf("had problems getting a message \n");
//     else  // the sending computers info is in senders_address
//     {   
//       printf("Received data from master: %s\n", message_buffer);
//       G->recover_ips_from_buffer(message_buffer);
//     }
//   }  
//    
//   while(G->non_planning_yet)
//   {
//     printf("waiting to start planning\n");  
//     
//     memset(&message_buffer,'\0',sizeof(message_buffer)); 
//     message_length = recvfrom(in_socket, message_buffer, sizeof(message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks until a message is recieved
//     if(message_length < 0) 
//       printf("had problems getting a message \n");
//     else  // the sending computers info is in senders_address
//     {   
//       printf("Received data: %d\n", (int)message_buffer[0]);
//       
//       if(message_buffer[0] == 1) // we can start moving
//         G->non_planning_yet = false; 
//     }
//   }
//   
//   printf("starting to plan \n");
//   
//   char planning_message_buffer[max_message_size];
//           
//   while(!G->kill_master) // this thread is now responsible for reading in data from other processes
//   {  
//     memset(&planning_message_buffer,'\0',sizeof(planning_message_buffer)); 
//     message_length = recvfrom(in_socket, planning_message_buffer, sizeof(planning_message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks until a message is recieved
//     if(message_length < 0) 
//       printf("had problems getting a message \n");
//     else  // the sending computers info is in senders_address
//     {   
//       //printf("Received data: %d\n", (int)planning_message_buffer[0]);
//       
//       if(planning_message_buffer[0] == 2) // it has planning data in it;
//       {
//         int agent_sending = (int)planning_message_buffer[1];  
//         if(agent_sending >= 0 && agent_sending < G->number_of_agents)
//         { 
//           // put the message into a file  
//           //printf("%s\n", &(planning_message_buffer[2]));
//         
//           char this_file[100];
//           if((MultiAgentSolution*)G->MAgSln != NULL)
//           {
//             sprintf(this_file, "%s/%d_to_%d_%d.txt", message_dir, agent_sending, G->agent_number, ((MultiAgentSolution*)G->MAgSln)->in_msg_ctr[agent_sending]);
//           
//             //printf("attempting to open: %s \n",this_file);
//           
//             FILE* ofp = fopen(this_file,"w");
//             if(ofp == NULL) // problem opening file
//             {
//               printf("cannot open message file for writing\n");
//               continue;
//             }
//           
//             fprintf(ofp, "%s\n", &(planning_message_buffer[2]));
//             fclose(ofp);
//           }
//         }
//       }
//       else if(planning_message_buffer[0] == 3) // it has a kill message in it
//       {
//         G->kill_master = true;
//       }
//     }     
//   }
//   // this thread terminates
//   return NULL;
// }

// void *Robot_Sender(void * inG)
// {
//   GlobalVariables* G = (GlobalVariables*)inG;  
//   struct hostent *hp;
//   char buffer[256];
//   int message_size;      
//    
//   printf("Robot Sender \n");   
//         
//   struct sockaddr_in master_address;
//   int master_address_length;   
// 
//   // create an outgoing socket
//   G->my_out_sock = socket(AF_INET, SOCK_DGRAM, 0); 
//   if(G->my_out_sock < 0)        // failed to create socket
//     error("problems creating socket");
//    
//   // set up hostent structure with master's data
//   hp = gethostbyname(G->master_IP);
//   if(hp==0) 
//     error("Unknown host");
// 
//   // populate master_address structure
//   master_address.sin_family = AF_INET;
//   bcopy((char *)hp->h_addr, (char *)&master_address.sin_addr, hp->h_length); // copy in address of master
//   master_address.sin_port = htons(G->MasterInPort);   // htons() converts 'number' to proper network byte order
//   
//   master_address_length = sizeof(struct sockaddr_in);  // get the memory size of a sockaddr_in struct
//   
//   while(!G->have_all_agent_addresses()) // until we have the other robot's data, send our data to the server
//   { 
//     printf("waiting for agent adresses \n");  
//       
//     memset(&buffer,'\0',sizeof(buffer)); 
//     sprintf(buffer, "%d 0 %s\n", G->agent_number, G->my_IP);
//     message_size = sendto(G->my_out_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&master_address, sizeof(struct sockaddr_in));  // send this agent's IP to the master
// 
//     if(message_size < 0) 
//       error("Problems sending data 1");
//      sleep(1);        
//   }
//   printf("have all agent adresses \n");  
//    
//   // once we have all other robot's id, we send that info to the agent, but keep sending our info as well
//   while(G->non_planning_yet)
//   {
//     printf("waiting to plan \n");    
//       
//     sprintf(buffer, "%d 1 %s\n", G->agent_number, G->my_IP);
//     message_size = sendto(G->my_out_sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&master_address, master_address_length);  // send info to master
// 
//     if(message_size < 0) 
//       error("Problems sending data");
//     sleep(1);          
//   }
//   printf("planning \n"); 
//   
//   // now we can start path planning, this thread terminates   
//   return NULL;
// } 

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
  struct sockaddr_in my_address, senders_address;
  int my_address_length, senders_address_length;
//   char message_buffer[max_message_size];
  int in_socket;
  int message_length;
  int in_port;
    
  in_port = G->InPorts[G->agent_number];    

  printf("(listener) ad-hoc listener thread\n"); 
  
  // create socket
  in_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if(in_socket < 0)  // failed to open socket
    error("(listener) Problems opening socket\n");
  
  // clear all memory of my_address structure
  my_address_length = sizeof(my_address);
  memset(&my_address, NULL, my_address_length); 
   
  // populate my_address structure
  my_address.sin_family = AF_INET;
  my_address.sin_addr.s_addr = INADDR_ANY; // ip address of this machine
  my_address.sin_port = htons(in_port);  // htons() converts 'number' to proper network byte order
  
  // bind in_socket with my_address
  if(bind(in_socket, (struct sockaddr *)&my_address, my_address_length)<0) 
    error("(listener) problems binding in_socket");
  
  senders_address_length = sizeof(struct sockaddr_in);  // get the memory size of a sockaddr_in struct 

  char planning_message_buffer[max_message_size];
          

  clock_t last_listener_pulse = clock();
  float pulse_time = 1;

  while(!G->kill_master)
  {
    if(G->master_reset)
    {
      output_pulse(last_listener_pulse, pulse_time, "(listener) sleeping due to master_reset \n");

      usleep(100000); // sleep for 1/10 sec
    }

    while(!G->kill_master && !G->master_reset) // this thread is responsible for reading in data from other processes
    {  
      //printf("(listener) pulse\n");

      if(!Globals.have_all_team_single_paths())
      {
        output_pulse(last_listener_pulse, pulse_time, "(listener) listening for single robot paths from other robots \n");
      }
      else if(G->not_planning_yet) // i.e. while we don't have the min number of agent start/goal locations
      {
        printf("(listener) listining for start/goal from other agents \n");   
        printf("(listener) so far, team includes: ");
        for(int i = 0; i < G->team_size; i++)
          printf("%d(%d), ", G->global_ID[i], G->have_info[i]);
        printf("\n"); 

        // check if we have all team data, if so then start planning
        if(G->have_all_team_data())
          G->not_planning_yet = false;
      }
    
      memset(&planning_message_buffer,'\0',sizeof(planning_message_buffer)); 
      message_length = recvfrom(in_socket, planning_message_buffer, sizeof(planning_message_buffer), 0, (struct sockaddr *)&senders_address, (socklen_t *)&senders_address_length);  // blocks until a message is recieved

      int message_ptr = 0;
      if(message_length < 0) 
        printf("(listener) had problems getting a message \n");
      else  // the sending computer's info is in senders_address
      {   
        //printf("(listener) Received data: %c\n", (char)planning_message_buffer[message_ptr]);
        
        bool need_to_join_teams = false;
        vector<int> robots_in_senders_team; // gets populated from buffer data

        if(planning_message_buffer[message_ptr] == 'A') // it has start up message in it
        {
          //printf("-Received start-up data from an agent:\n%s\n", planning_message_buffer);
          need_to_join_teams = G->recover_data_from_buffer(planning_message_buffer, message_ptr, robots_in_senders_team);
        
          //printf("trying to recover data \n");
        
          //message_ptr++;
          while(planning_message_buffer[message_ptr] != 'A' && planning_message_buffer[message_ptr] != '\0')
            message_ptr++;
          if(planning_message_buffer[message_ptr] == 'A')
            message_ptr++;
        }
      
        // extract prefered single robot paths known to sending agent
        bool extracted_at_least_one_path = false;
        while(planning_message_buffer[message_ptr] == 'V')
        {
          message_ptr++;

          int ag_gbl_id;
          message_ptr += extract_int_from_buffer(ag_gbl_id, (void*)((size_t)planning_message_buffer + (size_t)message_ptr));

          int pln_itr;
          message_ptr += extract_int_from_buffer(pln_itr, (void*)((size_t)planning_message_buffer + (size_t)message_ptr));

          //printf("(listener) extract maybe? %d[%d] \n", ag_gbl_id, pln_itr);

          if(ag_gbl_id == G->agent_number) // what the sending agent thinks about this agent
          {
            if(pln_itr < G->planning_iteration_single_solutions[G->agent_number]) // the sending agent is incorrect about this agent's iteration
            {
              //printf("(listener) another agent is incorrect about (my) planning iteration \n");
              G->an_agent_needs_this_single_path_iteration = true;
            }

          }

          if(pln_itr > G->planning_iteration_single_solutions[ag_gbl_id]) // new planning iteration
          {
            if(pln_itr > G->planning_iteration_single_solutions[G->agent_number] && G->InTeam[ag_gbl_id])
            {
               G->planning_iteration_single_solutions[G->agent_number] = pln_itr; 
            }

            G->planning_iteration_single_solutions[ag_gbl_id] = pln_itr;
          }
           
          // if this is current data from a different agent then us 
          if(ag_gbl_id != G->agent_number && pln_itr == G->planning_iteration_single_solutions[ag_gbl_id])
          {
            message_ptr += extract_2d_vector_from_buffer(G->other_robots_single_solutions[ag_gbl_id], (void*)((size_t)planning_message_buffer + (size_t)message_ptr));
            extracted_at_least_one_path = true;
          } 
          else // either data about us or really old data
          {
            // extract to temp to advance in buffer 
            vector<vector<float> > temp;
            message_ptr += extract_2d_vector_from_buffer(temp, (void*)((size_t)planning_message_buffer + (size_t)message_ptr));
          } 
          //printf("did extract? \n");
        }

        // if we are not joining teams based on overlapping planning areas, then we need to periodically 
        // check if any other_robots_single_solutions conflict with our single robot solution
        if(!JOIN_ON_OVERLAPPING_AREAS && extracted_at_least_one_path && robots_in_senders_team.size() > 0)
        {
          if(!G->InTeam[robots_in_senders_team[0]] && G->found_single_robot_solution) // the senders team is different from ours and we have a solution
          {
            // check for conflicts vs robots in the senders team

            timeval time_now;  
            gettimeofday(&time_now, NULL);
            for(uint k = 0; k < robots_in_senders_team.size(); k++)
            {
              int temp_ag = robots_in_senders_team[k];

              if(true) //difftime_timeval(time_now, G->last_path_conflict_check_time[temp_ag]) > .5)  // only check vs each agent every .5 secs
              { 
                if(G->other_robots_single_solutions[temp_ag].size() < 1) // make sure we have temp_ag's solution
                  continue;

                vector<float> A_conflict; // dummy data holder 
                vector<float> B_conflict; // dummy data holder
                float time_resolution = .05;

                printf("__________________ checking for conflicts vs agent %d ____________\n", temp_ag);

                if(find_first_time_conflict_points(G->other_robots_single_solutions[temp_ag], G->single_robot_solution, G->robot_radius, time_resolution, A_conflict, B_conflict)) 
                {
                  // the paths conflicts, calculate distance to that agent's last known point based on the first point in either path

                  if(euclid_dist(G->other_robots_single_solutions[temp_ag][0], G->single_robot_solution[0]) > G->combine_dist)
                  {
                    printf("Path conflicts with an agent not yet in our team\n");
                    printf("... but it is too far away to care about right now\n");
                  }
                  else
                  {
                    printf("Path conflicts with an agent not yet in our team\n");
                    need_to_join_teams = true;
                  }
                }
                G->last_path_conflict_check_time[temp_ag] = time_now;
              }
            }
          }
        }

        if(need_to_join_teams)     // for overlap or because we were in their team, based on what was found above
        {
          printf("----------------------- need to join teams -------------------------\n");

          for(uint k = 0; k < robots_in_senders_team.size(); k++)
          {
            int temp_ag = robots_in_senders_team[k];
              
            if(!G->InTeam[temp_ag])
            {    
              G->InTeam[temp_ag] = true;
              G->local_ID[temp_ag] = G->team_size;
              G->global_ID.push_back(temp_ag);
              G->team_size++;
            } 
          }
      
          G->planning_iteration[G->agent_number]++; // the problem has changed, so update our planning iteration number
      
          G->master_reset = true;   
          continue;
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
            printf("(listener) error: agent id >= 1000 \n");
     
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
              printf("(listener) cannot open message file for writing\n");
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
          //else
            //printf("s:%d (d)  <?   %d \n", agent_sending, (int)(((MultiAgentSolution*)G->MAgSln)->in_msg_ctr.size()));
        }
        else if(planning_message_buffer[message_ptr] == 3) // it has a kill message in it
        {
          G->kill_master = true;
        }
        else if(planning_message_buffer[message_ptr] == 4)
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
          printf("(listener) recieved unknown message type --\n%s\n",  &(planning_message_buffer[message_ptr]));    
        }
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
  GlobalVariables* G = (GlobalVariables*)inG;  
  char buffer[max_message_size];    
   
  printf("(startup sender) ad-hoc sender start-up thread \n");  

  // create an outgoing socket
  G->my_out_sock = socket(AF_INET, SOCK_DGRAM, 0); 
  if(G->my_out_sock < 0)        // failed to create socket
    error("(startup sender) problems creating socket");

  if(G->master_reset)
  {
    printf("(startup sender) sender thread exiting due to master reset -2 \n");
    return NULL;
  }
  
  if(G->revert_to_single_robot_path)
  {
    printf("(startup sender) sender thread exiting due to revert to single robot path -2\n");
    return NULL;
  }


  // while we don't have all the other robots individual prefered paths
  while(!G->have_all_team_single_paths() && !G->master_reset && !G->revert_to_single_robot_path)
  {
    if(G->team_size <= 1)  // if only 1 robot in team, then can break out here
      break;

    // send all prefered paths we know about

    int index = G->populate_buffer_with_single_robot_paths(buffer);

    buffer[index] = 4; // this signals that all this message contained was the prefered robot path 
    index++;

    G->hard_broadcast((void *)buffer, sizeof(char) * index);

    printf("(startup sender) waiting until we know all team members single paths\n");

    usleep(G->sync_message_wait_time*1000000);

  }

  printf("(startup sender) have all robot's single paths \n");

  if(G->master_reset)
  {
    printf("(startup sender) sender thread exiting due to master reset -1 \n");
    return NULL;
  }
  
  if(G->revert_to_single_robot_path)
  {
    printf("(startup sender) sender thread exiting due to revert to single robot path -1\n");
    return NULL;
  }


  while(!G->have_calculated_start_and_goal && !G->master_reset && !G->revert_to_single_robot_path)
  {
    // wait until we have calculated the start and goal

    if(G->an_agent_needs_this_single_path_iteration) // some agent is incorrect about this agent's single robot path planning iteration
    {
      // send all prefered paths we know about

      int index = G->populate_buffer_with_single_robot_paths(buffer);

      buffer[index] = 4; // this signals that all this message contained was the prefered robot path 
      index++;

      G->hard_broadcast((void *)buffer, sizeof(char) * index);
      G->an_agent_needs_this_single_path_iteration = false;
    }

    printf("(startup sender) waiting to calculate this agents start and goal\n");

    usleep(G->sync_message_wait_time*1000000);

  }


  if(G->master_reset)
  {
    printf("(startup sender) sender thread exiting due to master reset 0 \n");
    return NULL;
  }
  
  if(G->revert_to_single_robot_path)
  {
    printf("(startup sender) sender thread exiting due to revert to single robot path 0\n");
    return NULL;
  }


  while((!G->have_all_team_data() || G->team_size < G->min_team_size) && !G->master_reset && !G->revert_to_single_robot_path) // until we have the min number of the other robot's data
  {   
    //printf("here -0- %d\n", max_message_size);
        
    int final_index = G->populate_buffer_with_data(buffer);
    //if(G->an_agent_needs_this_single_path_iteration)
    //{
      //printf("here 4 \n");
      final_index += G->populate_buffer_with_single_robot_paths((char*)((size_t)buffer + (size_t)final_index));
      G->an_agent_needs_this_single_path_iteration = false;
    //}
    G->hard_broadcast((void *)buffer, sizeof(buffer));
    
    usleep(G->sync_message_wait_time*1000000);
    printf("(startup sender) waiting for agent start and goal coords -(sending data)- \n");
  }
  printf("(startup sender) we have min number of start and goal locations to start planning\n");  
  
  if(G->master_reset)
  {
    printf("(startup sender) sender thread exiting due to master reset 1 \n");
    return NULL;
  }
  
  if(G->revert_to_single_robot_path)
  {
    printf("(startup sender) sender thread exiting due to revert to single robot path 1\n");
    return NULL;
  }

  G->not_planning_yet = false;
  
  // now we start path planning, but this thread still keeps broadcasting the data to agents in our team who are not yet ready to plan
  while(!G->all_team_ready_to_plan() && !G->master_reset && !G->revert_to_single_robot_path) // until the rest of the team is ready to plan
  {       
    printf("here -1-, %d\n", max_message_size);
    int final_index = G->populate_buffer_with_data(buffer);
    if(G->an_agent_needs_this_single_path_iteration)
    {
      //printf("here 5 \n");
      final_index += G->populate_buffer_with_single_robot_paths((char*)((size_t)buffer + (size_t)final_index));
      G->an_agent_needs_this_single_path_iteration = false;
    }
    G->hard_broadcast((void *)buffer, sizeof(buffer));
    
    usleep(G->sync_message_wait_time*1000000);
  }
  
  if(G->master_reset)
  {
    printf("(startup sender) sender thread exiting due to master reset 2 \n");
    return NULL;
  }
  
  if(G->revert_to_single_robot_path)
  {
    printf("(startup sender) sender thread exiting due to revert to single robot path 2\n");
    return NULL;
  }

  // now we know the team is all planning, so we exit this thread
  printf("(startup sender) all members of team are planning, exiting ad-hoc sender startup thread \n"); 
  
  return NULL;
} 
