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
   bool have_min_agent_addresses();
   bool all_agents_ready_to_plan();
   bool min_agents_ready_to_plan();
   bool all_agents_moving();
   void broadcast(void* buffer, size_t buffer_size); // sends data to all robots we have info from
   void hard_broadcast(void* buffer, size_t buffer_size); // sends data to all robots we think may exist, regardless of if we have info from them
   
   void populate_buffer_with_ips(char* buffer); // puts everybody's ip into a buffer
   void recover_ips_from_buffer(char* buffer); // gets everybody's ip out of the buffer
   int populate_buffer_with_data(char* buffer); // puts this agents ip, start, and goal positions into the buffer
   void recover_data_from_buffer(char* buffer); // gets an agents ip, start, and goal position out of the buffer
   void tell_master_we_are_moving(void * inG); // tells the master that this robot is moving
   
   
   vector<int> InPorts;
   vector<int> OutPorts;
   int MasterInPort;
   int MasterOutPort;

   vector<int> have_info;     // gets set to 1 when we get an agent's info
   vector<int> agent_ready;   // gets set to 1 when we get an agent's info
   vector<int> agent_moving;  // gets set to 1 when we get an agent's info
   vector<struct sockaddr_in> other_addresses;  
   char** other_IP_strings;
   vector<vector<float> > start_coords; // start_coords[i] holds the start location for robot i
   vector<vector<float> > goal_coords;  // goal_coords[i] holds the goal location for robot i
   
   bool non_planning_yet;
   
   int agent_number;
     
   char master_IP[256]; // only used in structured mode (i.e. not ad-hoc)
   char base_IP[256];   // only used in ad-hoc, all ip addresses are then found as base_IP.(agent_id+1)
   char my_IP[256];
   int  my_out_sock;
   int  my_in_sock;
   
   int number_of_agents;
   int min_number_of_agents;  //  min number needed to start planning
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
};