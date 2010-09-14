/* ------------------------- glut functions ---------------------------- */

// this sets the projection in the current window
void Project()
{
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glOrtho(-win_aspect_ratio*scale_factor,+win_aspect_ratio*scale_factor, -scale_factor,+scale_factor, -1,+1);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

// default display function, gets called once each loop through glut
void display()
{      
   if(display_flag < 0)
       return;

   // set up graphics stuff
   Project();

   glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
   glEnable(GL_DEPTH_TEST);
   glDisable (GL_BLEND);
   glCullFace(GL_BACK);  
   glLoadIdentity();
   glPushMatrix();
   glDepthFunc(GL_ALWAYS);  
   glTranslatef(win_pan_x,win_pan_y,0);

   Scene.DrawObstacles();
   //printf("num_valid_points : %d \n", Cspc.num_valid_points);

   //Scene.DrawPointSafeLookup();
   //Scene.DrawEdgeSafeLookup();
 
   if(display_flag == 0)
   {
     // draw funtions go here
     Cspc.W.Draw(Cspc.start, NULL);
     Cspc.W.Draw(Cspc.goal, NULL);
     //Cspc.W.DrawEdge(startc, goalc);
   
     if(!found_path)
     {
       Cspc.DrawTree();
      //Cspc.W.Draw(Cspc.goal, NULL);
     }
     else
     {
       //Cspc.DrawPath(true);
       MultAgSln.DrawPath(Cspc.W, true); 
       display_flag = 1;
     }
   }
   else if(display_flag == 1)
   {
      getchar(); 
       Cspc.DrawTree();
      //Cspc.RoughAnimate(true);    
      //MultAgSln.RoughAnimate(Cspc.W, true);
      display_flag = 2;
   }
   else if(display_flag == 2)
   {
      getchar(); 
      //Cspc.RoughAnimate(true);    
      //if(next_ind_animate < 0)
      //  display_flag = -1;
      
      MultAgSln.RoughAnimate(Cspc.W, true);
      if(next_ind_animate >= (int)MultAgSln.BestSolution.size())
        display_flag = -1;
      
      //Cspc.DrawTree();
   }
   
   // draw the updated scene
   glFlush();
   glutSwapBuffers(); 
}


// this routine is called when the window is resized
void reshape(int width,int height)
{
   win_width = width;
   win_height = height;
   //  Ratio of the width to the height of the window
   win_aspect_ratio = (height>0) ? (double)width/height : 1;
   //  Set the viewport to the entire window
   glViewport(0,0, width, height);
   //  Set projection
   Project();
   
   glutPostRedisplay();   
}

// this routine is called when nothing else is happening
void idle()
{
  if(display_flag > 0)
    glutPostRedisplay(); 
}