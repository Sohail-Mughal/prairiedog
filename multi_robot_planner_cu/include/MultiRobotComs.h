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

   int populate_buffer_with_single_robot_paths(char* buffer); // puts single robot paths in buffer, returns the number of chars that it required

   int populate_buffer_with_data(char* buffer); // puts this agents ip, start, and goal positions into the buffer
   bool recover_data_from_buffer(char* buffer, int &index); // gets an agents ip, start, and goal position out of the buffer, returns true if we get a message from another team that overlaps (just simple quad) with our solution, index holds the index directly after data
   void tell_master_we_are_moving(void * inG); // tells the master that this robot is moving
   
   float calculate_time_left_for_planning();  // based on info from all agents, this returns the time that remains for planning
   
   bool have_all_team_single_paths();         // returns true if we have all team members current single paths, else false

   vector<int> InPorts;      // indexed using global ID (i.e. agent number)
   vector<int> OutPorts;     // indexed using global ID (i.e. agent number)
   int MasterInPort;
   int MasterOutPort;

   vector<bool> InTeam;       // InTeam[n] = true means that agent n is in this agent's dynamic team, indexed using global ID (i.e. agent number)
   vector<int> local_ID;      // local_ID[n] gives the index associated with robot n on this agent (i.e. 1 level of id abstraction)
   vector<int> global_ID;     // inverse mapping of local_ID
   
   vector<int> planning_iteration;   // keeps track of how many times each agent has restarted planning, indexed using global ID (i.e. agent number)
   vector<float> last_known_dist;    // last known distance between this agent's robot and the other robots (global index)
   vector<clock_t> last_known_time;  // the time that last_known_dist was captured (global index)
   
   vector<int> have_info;     // have_info[i] gets set to 1 when we get agent i's info (start and goal), indexed using local_ID
   vector<int> agent_ready;   // agent_ready[i] is set to 1 when agent i has enough info to start planning, indexed using local_ID
   vector<int> agent_moving;  // agent_moving[i] set to 1 when i starts moving, indexed using local_ID
   vector<struct sockaddr_in> other_addresses;  // indexed using global ID (i.e. agent number)
   char** other_IP_strings;                     // indexed using global ID (i.e. agent number)
   
   vector<vector<float> > start_coords;  // start_coords[i] holds the start location for robot i, indexed using local_ID
   vector<vector<float> > goal_coords;   // goal_coords[i] holds the goal location for robot i, indexed using local_ID
   
   vector<float> planning_time_remaining; // holds the ammount of planning time remaining for each agent, indexed using local_ID
   vector<timeval> last_update_time;      // last_update_time[i] holds the last time planning_time_remaining[i] was updated, indexed using local_ID
   timeval start_time_of_planning;
   float min_clock_to_plan;

   bool not_planning_yet; // is true when this agen is not planning
   
   int agent_number;    // this agent's global id
     
   char master_IP[256]; // only used in structured mode (i.e. not ad-hoc)
   char base_IP[256];   // only used in ad-hoc, all ip addresses are then found as base_IP.(agent_id+1)
   char my_IP[256];
   int  my_out_sock;
   int  my_in_sock;
   
   int number_of_agents;      // total number of agents 
   int min_team_size;         // the dynamic team must be this big to start planning
   int team_size;             // the dynamic team currently has this many members
   
   bool kill_master;          // if true then we shut down all threads
   bool master_reset;         // if true then we restart the planning
   bool done_planning;             // true when done_planning (i.e. moving)
   
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
   
   POSE* robot_pose;  // most recent pose of the robot
   
   float combine_dist;   // if paths intetsect, then we must be this close to the robot of the other path to join their team
   float drop_dist;      // after we know a robot is this far away from us, we can drop them from our team (note: combine_dist < drop_dist)
   float drop_time;      // after this long without hearing from a robot we drop it from the team
           
   clock_t start_time; // this gets set at the beginning and then never changed
   void* MAgSln;   // pointer to the multi agent solution

   bool found_single_robot_solution;             // starts false, turns true when single_robot_solution is found
   vector<vector<float> > single_robot_solution; // a single robot solution for this robot from its start to goal, 
                                                 // always updated to incoporate new multi-robot path stuff 

   vector<vector<vector<float> > > other_robots_single_solutions; //  other_robots_single_solutions[i] holds the single robot solution for robot i
                                                                  //  where i is indexed by global ID (i.e. agent number). 

   vector<int> planning_iteration_single_solutions; // holds the associated planning iteration of other_robots_single_solutions 

   bool have_calculated_start_and_goal;  // gets set to true when we calculate this agent's start and goal

   bool use_sub_sg; // gets set to true if we need to use sub_start and sub_goal instead of start and goal

   bool an_agent_needs_this_single_path_iteration; // gets set to true if another agent is incorrect about this robot's single robot path iteration
                                                   // gets set to false after a hard broadcast of that data

   bool revert_to_single_robot_path;       // set to true if no team's single robot paths conflict
};
