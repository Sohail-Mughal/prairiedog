/*---------------- class MultiRobotWorkspace ----------------------------*/
// this gets extended by class Workspace
class MultiRobotWorkspace
{
  public:
    MultiRobotWorkspace();                              // default constructor 
    MultiRobotWorkspace(int n_robots, float robot_r);   // constructor 
    MultiRobotWorkspace(const MultiRobotWorkspace& W);  // copy constructor
    ~MultiRobotWorkspace();                             // destructor
    
    void Populate(int n_robots, float robot_r);         // populates or re-populates the structure
    void Populate(int n_robots, float robot_r, const vector<float>& dim_max_in); // populates or re-populates the structure

    void Print();                                       // displays info about the class to command line
    void Draw(const vector<float>& config, float* clr); // draws the workspace in the given configuration
    void DrawGoal(const vector<float>& config, float* clr);                     // draws the workspace in the given configuration as a goal
    void DrawEdge(const vector<float>& config1, const vector<float>& config2);  // draws a line(s) in the workspace between the two configurations 

    void RandMove(const vector<float>& old_config, vector<float>& new_config, float safe_dist, float max_dist, float max_theta, float prob_at_goal, const vector<float>& the_goal); // calculates a new configuration point that is whithin max_dist of the old point, but with prob_at_goal moves directly at the goal (often the goal will be the start), safe_dist is the believed safe distance to move
    void RandMove2(const vector<float>& old_config, vector<float>& new_config, float safe_dist, float max_dist, float max_theta, float prob_at_goal, const vector<float>& the_goal); // calculates a new configuration point that is whithin max_dist of the old point, but with prob_at_goal moves directly at the goal (often the goal will be the start), safe_dist is the believed safe distance to move, same as above but rand happens on granularity of a robot
    void RandMove3(vector<float>& new_config, float prob_at_goal, const vector<float>& the_goal); // calculates a new configuration point that is drawn randomly from the configuration space, but with prob_at_goal a copy of the goal configuration the_goal is chosen
    void RandMove4(vector<float>& new_config, float prob_at_goal, const vector<float>& the_goal, const vector<vector<float> >& f_space); // calculates a new configuration point that is drawn randomly from the configuration space, defined by f_space, but with prob_at_goal a copy of the goal configuration the_goal is chosen
    
    void MoveToward(const vector<float>& old_config_from, const vector<float>& old_config_to, vector<float>& new_config, float move_dist); // new_config is move_dist from old_config_from to old_config_to 
    
    float PointValid(const vector<float>& P); // checks the point P in the Workspace for validity, returns min distance to collision
    float EdgeValid(const vector<float>& P1, const vector<float>& P2); // checks that points P1 and P2 can be connected, returns min distance to collision
  
    float ProjectedPointValid(const vector<float>& P, int bot); // checks the point P in the Workspace for validity, for the robot bot
    float ProjectedEdgeValid(const vector<float>& P1, const vector<float>& P2, int bot); // checks that points P1 and P2 can be connected, returns min distance to collision, for the robot bot
     
    float Dist(const vector<float>& P1, const vector<float>& P2); // returns the distance between P1 and P2 (sum of workspace distance of all robots)
    float MaxDist(const vector<float>& P1, const vector<float>& P2); // returns the max distance of any robot between P1 and P2
    float AngularDist(const vector<float>& P1, const vector<float>& P2); // returns the angular distance between P1 and P2
    float MaxAngularDist(const vector<float>& P1, const vector<float>& P2); // returns the max angular distance of any robot between P1 and P2
    
    int num_robots;
    vector<float> robot_radius;
    
    int dims; // NOTE: this is the dimensionality of the workspace, not the configuration space
    vector<float> dim_max; // workspace goes from 0 to this value along each dimension
    
    GlobalVariables* Gbls;
};
