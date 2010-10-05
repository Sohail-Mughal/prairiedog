/*-------------------- class NavScene --------------------------------------*/
class NavScene
{
  public:
    
    NavScene();                              // default constructor 
    NavScene(const NavScene& W);             // copy constructor
    ~NavScene();                             // destructor
    
    void PrintSceneInfo();      // prints on command line the info about the scene
    bool LoadFromFile(const char* filename);   // loads the scene info from the file
    bool LoadMapFromFile(const char* filename); // loads only the map portion of a file

    
    #ifndef not_using_globals
    bool LoadFromGlobals(GlobalVariables& G); // loads the scene info from the global variables
    #endif
            
    void DrawObstacles();        // draws the obstacles in the obstacle list
    void DrawPointSafeLookup();  // draws the lookup table values
    void DrawEdgeSafeLookup();   // draws the lookup table values
    
    float PointSafe(const vector<float>& point, int index, float the_robot_rad); // this checks if a point is safe in the environment, where the points' coords start at index in vectors, it returns the minimum distance to an obstacle
    bool EdgeSafe(const vector<float>& point1, const vector<float>& point2, int index, float the_robot_rad); // this checks if an edge is safe in the environment, where the points' coords start at index in vectors

    bool GetPointsFromFile(FILE* ifp); // this reads point data from the file
    bool SendPointsToFile(FILE* ofp);  // this writes point data to the file
    
    int world_dims;              // number of dimensions in the workspace
    vector<float> dim_max;       // holds the max extents of each dimension
    
    vector<float> robot_rad;     // radius of the robots
    int num_robots;              // number of robots 
    
    vector<float> startC;        // start configuration (1 X (num_robots * world_dims)
    vector<float> goalC;         // goal configuration (1 X (num_robots * world_dims)
    
    float prob_at_goal;          // probability that rrt moves at goal 
    float move_max;              // max move allowed by rrt
    float theta_max;             // max move with respect to angular direction allowd by rrt
    float resolution;            // the search resolution of the world
    float angular_resolution;    // the angular resolution of the world 
    
    int num_polygons;
    vector<vector<vector<float> > > polygon_list; // [a][b][c], a polygon obstacles with b points of c dimensions
    
    // the following are only used when certian flags are set:
    
    vector<vector<float> > PointSafeLookup; // [x/resolution][y/resolution] stores the values from PointSafe when used on point x y
    vector<vector<vector<vector<int> > > > EdgeSafeLookup; // [x1/resolution][y1/resolution][x1/resolution][1y/resolution] stores the values from EdgeSafe when used on edge between  x1 y1 and x2 y2
    
    int num_spatial_dims;        // the number of spatial dims (i.e. not rotational components)

    vector<int> last_n_points_x;     // x index of last n new points checked
    vector<int> last_n_points_y;     // y index of last n new points checked
    vector<float> last_n_points_val; // value of last n new points checked
    int n_points_ptr;                // points to the next index that gets replaced
    
    vector<int> last_n_edges_x1;     // x1 index of last n new edge checked
    vector<int> last_n_edges_y1;     // y1 index of last n new edge checked
    vector<int> last_n_edges_x2;     // x2 index of last n new edge checked
    vector<int> last_n_edges_y2;     // y2 index of last n new edge checked
    vector<int> last_n_edges_val;    // value of last n new points checked
    int n_edge_ptr; 
    
    vector<float> translation;               // the transform between map_cu coords and multi_robot planning coords
};