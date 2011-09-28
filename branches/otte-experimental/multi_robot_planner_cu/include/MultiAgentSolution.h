// this keeps track of the distrubuted solution, including communications with the other agents
class MultiAgentSolution
{
  public:
    MultiAgentSolution();                                      // default constructor
    MultiAgentSolution(int the_num_agents, int the_agent_id);  // constructor
    MultiAgentSolution(const MultiAgentSolution& M);           // copy constructor
    ~MultiAgentSolution();                                     // destructor 
    
    void Populate(int the_num_agents, int the_agent_id, int this_dims_per_robot);       // populates or re-populates the structure
    void Populate(int the_num_agents, int the_agent_id, GlobalVariables* G, int this_dims_per_robot);  // populates or re-populates the structure
            
    void ExtractSolution(Cspace& C);                           // this extracts a solution from the Cspace C, and updates things approperiatly if the solution is better than the best solution found so far.
    void AddSolution(Cspace& C);                               // this adds the best solution to the Cspace C
    
    void DrawPath(Workspace& W, bool draw_robots);             // draws the path, if draw_robots is true, then the robot is also shown
    void RoughAnimate(Workspace& W, bool draw_paths);          // animates the movement along best solution from start to goal, if draw_paths == true, then it draws the paths
        
    bool GetMessages(const vector<float>& start_config, const vector<float>& goal_config);  // checks for incomming messages, and updates things accordingly, returns true if a better path was found in the message, also makes sure that they use start and goal configs
    void SendMessageUDP(float send_prob);                      // while above function just uses a file, this uses UDP
            
    bool StartMoving();                                        // returns true if this agent can start moving
    
    bool ConfirmExistanceGlobalIdNode(int node_key_a, int node_key_b, const vector<float> the_coords); // checks the existance of NodeId[node_key_a][node_key_b] and adds it with the_coords if it does not exist (only of relivance when uni_tree_build == 1), returns true if nede already existed before this was called
    void LinkGlobalIdNodes(int node_key_a, int node_key_b, int parent_key_a, int parent_key_b);  // assuming that NodeId[node_key_a][node_key_b] and NodeId[parent_key_a][parent_key_b] exist, this links them as you would expect (only of relivance when uni_tree_build == 1)
    void UpdateDistanceInfoGlobalIdNode(int node_key_a, int node_key_b); // updates distance info about NodeId[node_key_a][node_key_b]
    
    bool GreedyPathSmooth();                                   // greedily smooths BestSolution
    
    float OverallMessageStats();                               // returns the probability this robot recieved a message that was sent by another robot
    
    int num_agents;                       // the number of agents
    int agent_id;                         // the id of this agent (note this is the global id)
    int dims_per_robot;                   // the number of dimensions in the solution per each robot
    
    vector<vector<float> > BestSolution;  // a path of configurations that is the best solution this agent has found/recieved so far
    float best_solution_length;           // the length of the best solution
    int best_solution_agent;              // the agent with the best solution so far
    
    vector<float> Votes;                  // each slot represents the coorisponding agent's vote for who has the best path, indexed using global id
    vector<int> FinalSolutionSent;        // each slot represents the coorisponding agent's status as to if they have sent out their final solution, indexed using global id
    
    bool moving;                          // this starts as false, but switches to true when this robot starts moving. if a message with this as true is recieved, then it means that some robot has started moveng, and the attached path is the one to be used 

    vector<int> in_msg_ctr;               // used to help keep track of message queue   (global id)
    vector<int> out_msg_ctr;              // used to help keep track of message queue   (global id)
      
    vector<int> message_send_attempts;    // stats on how well we send to other robots  (global id)
    vector<float> messages_sent_to_us;    // stats on how well we recieve messages from other robots  (global id)
    vector<float> messages_recieved_by_us;// stats on how well we recieve messages from other robots  (global id)
    
    // the following are only of relivance when uni_tree_build == 1
    int total_nodes_added;
    vector<vector<int> > NodeID;           // holds NodeID[a][b] holds the local index of the node with global id (a,b), that is the bth node inserted by robot a, this is used to keep track of the combinded search tree
    vector<int> NodeIDInda;                // this holds the a value (above) of this configuration spaces node i
    vector<int> NodeIDIndb;                // this holds the b value (above) of this configuration spaces node i

    int current_it;                        // remembers the messaging iteration
    vector<vector<int> > LastItAdded;      // keeps track of if NodeID[a][b] has been added this message iteration
    
    GlobalVariables* Gbls;                 // pointer to the global variable structure;
    
        
    ros::Publisher* obstacles_pub;
};
