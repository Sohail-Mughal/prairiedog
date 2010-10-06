/* ----------------------- threading and socket stuff ------------------ */

class GlobalVariables
{
  public:

   GlobalVariables(); 
   GlobalVariables(int num_of_agents);
   ~GlobalVariables();
   void Populate(int num_of_agents);
   
   bool set_up_agent_address(int ag_id, const char* IP_string);
   bool have_all_agent_addresses();
   bool have_min_agent_data();
   bool have_all_team_data();
   bool all_agents_ready_to_plan();
   bool min_agents_ready_to_plan();
   bool all_team_ready_to_plan();
   bool all_agents_moving();
   void broadcast(void* buffer, size_t buffer_size); // sends data to all robots we have info from
   void hard_broadcast(void* buffer, size_t buffer_size); // sends data to all robots we think may exist, regardless of if we have info from them
   
   void populate_buffer_with_ips(char* buffer); // puts everybody's ip into a buffer
   void recover_ips_from_buffer(char* buffer); // gets everybody's ip out of the buffer
   int populate_buffer_with_data(char* buffer); // puts this agents ip, start, and goal positions into the buffer
   void recover_data_from_buffer(char* buffer); // gets an agents ip, start, and goal position out of the buffer
   void tell_master_we_are_moving(void * inG); // tells the master that this robot is moving
   
   float calculate_time_left_for_planning();  // based on info from all agents, this returns the time that remains for planning
   
   vector<int> InPorts;      // indexed using global ID (i.e. agent number)
   vector<int> OutPorts;     // indexed using global ID (i.e. agent number)
   int MasterInPort;
   int MasterOutPort;

   vector<bool> InTeam;       // InTeam[n] = true means that agent n is in this agent's dynamic team, indexed using global ID (i.e. agent number)
   vector<int> local_ID;      // local_ID[n] gives the index associated with robot n on this agent (i.e. 1 level of id abstraction)
   vector<int> global_ID;     // inverse mapping of local_ID
   
   vector<int> have_info;     // have_info[i] gets set to 1 when we get agent i's info (start and goal), indexed using local_ID
   vector<int> agent_ready;   // agent_ready[i] is set to 1 when agent i has enough info to start planning, indexed using local_ID
   vector<int> agent_moving;  // agent_moving[i] set to 1 when i starts moving, indexed using local_ID
   vector<struct sockaddr_in> other_addresses;  // indexed using global ID (i.e. agent number)
   char** other_IP_strings;                     // indexed using global ID (i.e. agent number)
   
   vector<vector<float> > start_coords;  // start_coords[i] holds the start location for robot i, indexed using local_ID
   vector<vector<float> > goal_coords;   // goal_coords[i] holds the goal location for robot i, indexed using local_ID
   
   vector<float> planning_time_remaining; // holds the ammount of planning time remaining for each agent, indexed using local_ID
   vector<clock_t> last_update_time;      // last_update_time[i] holds the last time planning_time_remaining[i] was updated, indexed using local_ID
   
   bool non_planning_yet;
   
   int agent_number;    // this agent's global id
     
   char master_IP[256]; // only used in structured mode (i.e. not ad-hoc)
   char base_IP[256];   // only used in ad-hoc, all ip addresses are then found as base_IP.(agent_id+1)
   char my_IP[256];
   int  my_out_sock;
   int  my_in_sock;
   
   int number_of_agents;      // total number of agents 
   int min_team_size;         // the dynamic team must be this big to start planning
   int team_size;             // the dynamic team currently has this many members
   
   bool kill_master;
   
   float sync_message_wait_time;  // time to wait between sending messages during sync phases
   float message_wait_time;       // time to wait between sending messages during planning
   
   float robot_radius;
   
   float prob_at_goal;        // probability a new edge goes at the goal
   float move_max;            // max move allowed during rrt generation
   float theta_max;           // max rotation allowed durring rrt generation
   float resolution;          // resolution of the rrt
   float angular_resolution;  // angular resolution of the rrt
   float planning_border_width; // this much more space provided around robots for planning
   
   vector<float> team_bound_area_min;  // holds the minimum point in the hyper cube team bounding area  (note, this gets populated when NavSceen is populated)
   vector<float> team_bound_area_size; // holds the distance along each dimension of the hyper cube team bounding area (note, this gets populated when NavSceen is populated)
   
};