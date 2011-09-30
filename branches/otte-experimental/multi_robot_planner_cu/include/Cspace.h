/* ---------------------------- class Cspace --------------------------- */
class Cspace
{
  public:
    Cspace();                    // default constructor 
    Cspace( vector<float>& the_start,  vector<float>& the_goal, int dimensions); // constructor
    Cspace(const Cspace& C);     // copy constructor
    ~Cspace();                   // destructor 
    
    bool Populate(vector<float>& the_start, vector<float>& the_goal, int dimensions); // populates or re-populates the structure, returns true on success
    void ChopTree();                                                                // removes all nodes but root     
    
    int AddNeighbor(const vector<float>& new_configuration, int neighbor_index, float safe_dist, float dist_to_goal, float est_dist_to_start);  // this adds the new point in configuration space as a neighbor of the point refered to by neighbor_index, returns the new index of the new configuration
    void RemoveInd(int ind);                 // soft removes the ind
    void RemoveAllDescendants(int ind);      // soft remove all descendants of ind and ind, mostly helps with next function
    int  PruneTreeFromInd(int ind);          // starting at ind, this follows backpointers until it finds the earliest node that takes more than the min path length to reach the goal, then it prunes it and all of its descendents and returns its parent.
   
    int ClosestNeighborOfTo(int neighbor_index, const vector<float>& new_configuration, float& closest_distance); // returns the closest neighbor of neighbor_index to the new_configuration, puts distance in closest_distance
    int ClosestNeighborOfToWithinAngle(int neighbor_index, const vector<float>& new_configuration, float angle, float& closest_distance); // returns the closest neighbor of neighbor_index to the new_configuration that is withing angle of ne_configuration, puts distance in closest_distance
    
    int PropogateCost(int i_index); // recursively propogates new cost to goal data from i_index to its descendants, returns the index of start node (if any exist) with shortest path, else returns -1 
   
    bool BuildTree(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution);  // this builds or continues to build the search tree. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal, in jumps no larger than move_max, returns true when it finds a better path      
    bool BuildTreeV2(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution, bool config_provided, vector<float>& config);  // this builds or continues to build the search tree. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal, in jumps no larger than move_max, returns true when it finds a better path, this version uses pruning when possible, A*-like re-linking when possible, and is otherwise based on an RRT, note move_max no longer used. if config_provided is true then explicitly uses config instead of anything else
    bool BuildTreeV3(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution);  // same as above, but attempts to connect goal on timeout
    bool BuildTreeV4(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution);  // same as above, but attempts to connect goal on timeout

    bool BuildTreeV2rho(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution);  // this builds or continues to build the search tree. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal, in jumps no larger than move_max, returns true when it finds a better path, this version uses pruning when possible, A*-like re-linking when possible, and is otherwise based on an R, this tries to expand to the tree to a new node that is rand_move_dist toward the random node from the closest node
    bool BuildTreeV3rho(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution);  // same as above, but attempts to connect goal on timeout

    bool BuildRRT(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution);  // this builds or continues to build the search tree using an RRT. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal
    bool BuildRRTRho(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution);  // this builds or continues to build the search tree using an RRT. steps is decremented for each attempt to add a new point to the tree. The tree grows untill either the current time is clock_to_plan seconds past start_t or steps reaches 0 (if steps starts as a negative, then steps is ignored). the search moves at goal with prob_at_goal, after a random node is chosen and the closest tree node found, this tries to expand to the tree to a new node that is rand_move_dist toward the random node from the closest node

    bool BuildRRTFast(clock_t start_t, double clock_to_plan, int& steps, float prob_at_goal, float move_max, float theta_max, float resolution, float angular_resolution);  //Same as build RRT but collisions are checked after closest node is found    
    
    void pruneTree(); // go through the list, prune any nodes that can no longer lead to better any-time solutions 
    bool GreedyPathSmooth(int start_smooth_ind); // starting at start_smooth_ind this attempts to greedily decrease the length of the path by shortcutting to nodes further along, returns true if path has been shortened
    
    bool FindA1DShortCut(int start_smooth_ind, vector<float>&  new_config); // attempts to find a benificial new_config based on combining one dimension of a parent with its child, returns true if it finds one (note returns the best one based on the path from start_smooth_ind

    float PathLength(int node_index);        // returns the length from the node at node_index to the root of the tree
    
    bool TreeOK();                           // returns true if the tree is OK
    bool TreeTriangle();                     // returns true if the tree obeys triangle inequality between node->parent->grandparent
    bool TreeConsistant();                   // returns true if the tree is consistant (i.e. parent->goal + dist_to_parent = node->goal)
    bool TreeLocalOptimalOK();               // returns true if all grand-children nodes cannot be reached due to obstacles
    
    void DrawTree();                         // draws the search tree by calling the workspace draw edge function 
    void DrawPath(bool draw_robots);         // draws the path from the start configuration to the goal configuration, if draw_robots is true, then the robot is also shown
    void RoughAnimate(bool draw_paths);      // animates the movement from start to goal, if draw_paths == true, then it draws the paths
    
    int dims;  // number of dimesions in the Cspace
    vector<float> start;
    vector<float> goal;
    
    int num_points;
    vector<vector <float> >  ValidConfigs;  // each row stores a valid configuration
    vector<vector <int> > Neighbors;        // each row corresponds to a row in ValidConfigs, each column represents the indicies of its neighbors, the first neighbor is a backpointer to the generating node
    vector<float> SafeDistance;             // each row correspinds to a row in ValidConfigs, and represents the minimum distance to obstacles from that point
    vector<float> DistToGoal;               // stores the distance to the goal
    vector<float> MinDistToStart;           // stores the minimum estimated distance to the start
    
    int num_valid_points;                   // the number of points with DistToGoal < best_total_path_length
    vector<int> ValidInds;                  // this stores each index (i.e. into ValidConfigs) of each valid point, but the ordering is random
    vector<int> ValidIndsInd;               // this is num_points long and stores the index in ValidInds that is associated with ValidConfigs
    
    int start_ind; // holds the index of the start (goal index is 0)   
    Workspace W; // the workspace we will be using
    
    float best_total_path_length;           // this holds the best path length that has been found so far
    bool chop_tree;                         // used to remember when we want to chop the tree with rrt and rrtrho
    
    #ifdef use_kd_tree
    KD_Tree* T;     
    #endif
};
