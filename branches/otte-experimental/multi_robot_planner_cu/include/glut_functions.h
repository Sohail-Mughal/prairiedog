/* ------------------------- glut functions ---------------------------- */

// colors
float BLUE[] = {0,0,1};
float GREEN[] = {0,1,0};
float RED[] = {1,0,0};
float YELLOW[] = {1,1,0};
float LIGHTBLUE[] = {1,0,1};
float ORANGE[] = {0,1,1};
float WHITE[] = {1,1,1};

float* RAINBOW[] = {LIGHTBLUE, BLUE, GREEN, RED, ORANGE, YELLOW, LIGHTBLUE, BLUE, GREEN, RED, ORANGE, YELLOW, LIGHTBLUE, BLUE, GREEN, RED, ORANGE, YELLOW, LIGHTBLUE, BLUE, GREEN, RED, ORANGE, YELLOW};

// this sets the projection in the current window
void Project();

// default display function, gets called once each loop through glut
void display();

// this routine is called when the window is resized
void reshape(int width,int height);

// this routine is called when nothing else is happening
void idle();