/* ------------------------ helper function headers -------------------- */
void redude_3_to_2(const vector<float>& V3, vector<float>& V2); // puts the Workspace3 vector int a Workapace2 vector

void populate_int_vector(vector<int>& V, int* A, int s); // populates V with A of size s

void populate_float_vector(vector<float>& V, float* A, int s); // populates V with A of size s

bool equal_float_vector(const vector<float>& A, const vector<float>& B, float tol); // returns true if the vectors are the same size and contain the same elements to within tolerance tol

void draw_circle(float* pos, float rad, float* color); // draws a circle

void draw_x(float* pos, float rad, float* color); // draws an X with rad rad

void draw_arrow(float* pos1, float* pos2, float* color); // draws an arrow from pos1 to pos2

void draw_points(const vector<float>& xs, const vector<float>& ys); // draws points at locations specified in the vectors

int rand_int(int left_bound, int right_bound); // returns a random integer between left_bound and right_bound, inclusive

void print_float_vector(const vector<float>& vec); // prints the vector to the command line

float line_dist_to_point(float ax, float ay, float bx, float by, float cx, float cy); // returns the minimum distance between the line segment [a b] and the point c

void data_dump(const char* directory, float prob_success, float min_clock_to_plan, float phase_two_time, Cspace& C, MultiAgentSolution& M, float actual_clock_to_plan, float total_time); // appends info to a file per agent in directory

void data_dump_dynamic_team(const char* directory, const Cspace& C, const MultiAgentSolution& M, const GlobalVariables& G, POSE* robot_pose, const timeval & first_start_time);

#ifdef save_time_data
void time_data_dump(const char* directory, float prob_success, float min_clock_to_plan);
#endif

float difftime_clock(const clock_t& clock_time_1, const clock_t& clock_time_2); // returns the difference in seconds between the clock time 1 and 2, (2 is assumed earlier than 1)

float difftime_timeval(const timeval& time_1, const timeval& time_2); // returns the difference in seconds between the timeval time 1 and 2, (2 is assumed earlier than 1)

bool string_printf_s(int &sp, char* buffer, char* buffer2, int buffer_len); // this takes the string in buffer 2 and puts it into buffer starting at buffer[sp], it then resets sp to be the new end of the string, returns false if there is not enfough space in buffer

void double_up_points(const vector<vector<float> >& V1, vector<vector<float> >& V2); // this doubles each point from V1 and puts it in V2 (1,2,3 becomes 1,1,2,2,3,3)

void extract_and_translate_solution(vector<vector<float> >& AgentSolution, vector<vector<float> >& MultiSolution, vector<float>& Translation, int agent_id, int dims); // extracts the agent solution if agent_id where dims is the workspace dimensions and Translate holds the inverse ammount to translate along each

void remove_unnecessary_rotation(vector<vector<float> >& MultiSolution); // removes unnecessary rotation, assumes 2D workspace and orientation for each robot

void calculate_rotation(vector<vector<float> >& MultiSolution); // calculates the rotation at each point based on the position of the next point

void calculate_times(vector<float>& Times, vector<vector<float> >& MultiSolution, float mps_target, float rps_target); // calculates the time parametery of the MultiSolution, where mps_target is the target meters per second of the fastest moving robot and rps is the target radians per second of fastest moving robot, note that these should be slightly slower than the max possible values to allow a behind robot to catch up, assumes 2D workspace and orientation for each robot

void verrify_start_angle(vector<vector<float> >& MultiSolution, vector<float>& start_config);  // makes sure start angles are correct

bool quads_overlap(float x1_min, float x1_length, float y1_min, float y1_length, float x2_min, float x2_length, float y2_min, float y2_length ); // returns true if the quads overlap

float PointSafe(const vector<float>& point, int index, float the_robot_rad, const vector<vector<vector<float> > >& obstacle_list); // this checks if a point is safe with respect to the obstacle list, where the points' coords start at index in vectors, it returns the minimum distance to an obstacle

bool EdgeSafe(const vector<float>& point1, const vector<float>& point2, int index, const vector<vector<vector<float> > >& obstacle_list, float the_robot_rad); // this checks if an edge is safe vs obstacle_list, where the points' coords start at index in vectors

bool SolutionSafe(const vector<vector<float> >& solution, const vector<vector<vector<float> > >& obstacle_list, float the_robot_rad, int path_dims); // this if the solution is safe vs obstacle_list, path_dims is the dimensions of 1 robot's path


int add_int_to_buffer(const int &i, void* buffer); // add int to buffer and returns the size in chars
int extract_int_from_buffer(int &i, void* buffer); // extracts int from buffer and places it in i, returns the number of chars that were used

int add_1d_int_vector_to_buffer(const vector<int> &v, void* buffer); // add v to buffer and returns the size in chars
int extract_1d_int_vector_from_buffer(vector<vector<int> > &v, void* buffer); // extracts a 1d vector from buffer and places it in v, returns the number of chars that were used

int add_float_to_buffer(const float &i, void* buffer); // add float to buffer and returns the size in chars
int extract_float_from_buffer(float &i, void* buffer); // extracts float from buffer and places it in i, returns the number of chars that were used

int add_1d_float_vector_to_buffer(const vector<float> &v, void* buffer); // add v to buffer and returns the size in chars
int extract_1d_float_vector_from_buffer(vector<vector<float> > &v, void* buffer); // extracts a 1d vector from buffer and places it in v, returns the number 

int add_2d_float_vector_to_buffer(const vector<vector<float> > &v, void* buffer); // add v to buffer and returns the size in chars
int extract_2d_float_vector_from_buffer(vector<vector<float> > &v, void* buffer); // extracts a 2d vector from buffer and places it in v, returns the number 

// returns the 2D eudlidian distance between two points (using first 2 dims)
float euclid_dist(const vector<float> & A, const vector<float> &B);

// finds the closest point along the edge [edge0, edge1] to the point within resolution, returns false if closest_point is not populated
bool find_closest_point_along_edge(const vector<float> & point, const vector<float> & edge0, const vector<float> & edge1, float resolution, vector<float> & closest_point, float & best_dist_to_point);

// finds the closest point along the path to the point within resolution, returns false if closest_point is not populated
bool find_closest_point_along_path(const vector<float> & point, const vector<vector<float> > & path, float resolution, vector<float> & closest_point, float & best_dist_to_point);

// helps the function below, returns true of the point is within robot_rad of the edge to within resolution
bool edge_and_point_conflict(const vector<float> & point, const vector<vector<float> > & edge, float robot_rad, float resolution);


// used in the function below
bool edge_and_edges_conflict(const vector<vector<float> > & robot_forward_conflict, 
                             const vector<vector<vector<float> > > & obstacle_forward_conflicts,
                             const vector<vector<vector<float> > > & obstacle_backward_conflicts,
                             bool found_obstacle_forward_conflict,
                             bool found_obstacle_backward_conflict,
                             vector<float> & robot_forward_point_conflict, 
                             float robot_rad, float resolution);

// finds the first and last conflict (with respect to robot path) between robot path and obstacle path. Edges are check at resolution, and conflicts take into account robot rad. Returns true if there is a conflict, else false
bool find_conflict_points(const vector<vector<float> > &robot_path, const vector<vector<float> > &obstacle_path, float robot_rad, float resolution, vector<float> &first_r_conflict,  float &dist_to_first_r_conflict,
vector<float> &last_r_conflict, float &dist_to_last_r_conflict,
vector<float> &first_o_conflict, float &dist_to_first_o_conflict,  
vector<float> &last_o_conflict,  float &dist_to_last_o_conflict);

// moves the point that is located on edge with inds [i , i+1] time resolution along the path P
void move_time_res_along_path(const vector<vector<float> > & P, float time_res, int &i, vector<float> & point, bool & at_end_P);

// finds the first conflict (with respect to time) between paths (A and B) accounting for time (unlike the above function),  each path point is assumed [x y time]. time_res is the time granularity to use
bool find_first_time_conflict_points(const vector<vector<float> > &A, const vector<vector<float> > &B, float robot_rad, float time_res, vector<float> &A_conflict, vector<float> &B_conflict);


// find edge that most likely contains point and return the index of that edge or -1 if nothing is found, and also return the point in the path that was the best match
int find_edge_containing_point(const vector<vector<float> > & robot_path, const vector<float> & point, vector<float> & best_point_found);


// given data about a robot's path, start point along that path, and a planning region, this returns the last point along the path in the region
bool calculate_exit_point(const vector<vector<float> > & robot_path, const vector<float> & start_point, float area_min_x, float area_max_x, float  area_min_y, float area_max_y, float resolution, float robot_rad, vector<float> & exit_point);

// returns true if any two points in sub_goals conflict within robot_rad, returns their inds in conflicting_ind_a, conflicting_ind_b
bool any_two_points_conflict(const vector<vector<float> > & sub_goals, const vector<bool> & InTeam, float robot_rad, int & conflicting_ind_a, int & conflicting_ind_b);

// moves the point step along the path, assumes that point is already on path
void move_point_along_path(vector<float> & point, const vector<vector<float> > & path, float step);

// calculates the sub_start and sub goal that this robot should use for multi-robot path planning based on single robot paths, returns true if finds a new sub_start and sub_goal. no_conflicts_between_single_paths gets set to true if there is no conflicts between anybody
bool calculate_sub_goal(const vector<vector<vector<float> > > & robot_paths, const vector<bool> & InTeam, int this_agent_id, float preferred_min_planning_area_side_length, float preferred_max_planning_area_side_length, float robot_rad, float resolution, float time_res, vector<float> & sub_start, vector<float> & sub_goal, bool & no_conflicts_between_single_paths);

