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

#ifdef save_time_data
void time_data_dump(const char* directory, float prob_success, float min_clock_to_plan);
#endif

float difftime_clock(const clock_t& clock_time_1, const clock_t& clock_time_2); // returns the difference in seconds between the clock time 1 and 2, (2 is assumed earlier than 1)

bool string_printf_s(int &sp, char* buffer, char* buffer2, int buffer_len); // this takes the string in buffer 2 and puts it into buffer starting at buffer[sp], it then resets sp to be the new end of the string, returns false if there is not enfough space in buffer

void double_up_points(const vector<vector<float> >& V1, vector<vector<float> >& V2); // this doubles each point from V1 and puts it in V2 (1,2,3 becomes 1,1,2,2,3,3)

void extract_and_translate_solution(vector<vector<float> >& AgentSolution, vector<vector<float> >& MultiSolution, vector<float>& Translation, int agent_id, int dims); // extracts the agent solution if agent_id where dims is the workspace dimensions and Translate holds the inverse ammount to translate along each

void remove_unnecessary_rotation(vector<vector<float> >& MultiSolution); // removes unnecessary rotation, assumes 2D workspace and orientation for each robot

void calculate_rotation(vector<vector<float> >& MultiSolution); // calculates the rotation at each point based on the position of the next point

void calculate_times(vector<float>& Times, vector<vector<float> >& MultiSolution, float mps_target, float rps_target); // calculates the time parametery of the MultiSolution, where mps_target is the target meters per second of the fastest moving robot and rps is the target radians per second of fastest moving robot, note that these should be slightly slower than the max possible values to allow a behind robot to catch up, assumes 2D workspace and orientation for each robot

void verrify_start_angle(vector<vector<float> >& MultiSolution, vector<float>& start_config);  // makes sure start angles are correct

bool quads_overlap(float x1_min, float x1_length, float y1_min, float y1_length, float x2_min, float x2_length, float y2_min, float y2_length ); // returns true if the quads overlap
