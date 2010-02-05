/*  Copyright Michael Otte, University of Colorado, 9-9-2009
 *
 *  This file is part of Mapper_CU.
 *
 *  Mapper_CU is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapper_CU is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Mapper_CU. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <list>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"

#include "sensor_msgs/PointCloud.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"

#include "hokuyo_listener_cu/PointCloudWithOrigin.h"

#define PROBMAP // HITMAP: map remembers all laser scanner obstacle data
                // SIMPLEMAP: map updates to reflect current info from scanner (obstacles and free space)
                // PROBMAP: map calculates prob of obstacle based on last MAP_MEMORY readings about a grid

using namespace std;
        
struct POINT;
typedef struct POINT POINT;

struct MAP;
typedef struct MAP MAP;

// globals that can be reset using parameter server, see main();
float map_y_max = 20;              // (m), map goes from 0 to this in the y direction
float map_x_max = 20;              // (m), map goes from 0 to this in the x direction
float RESOLUTION = .08;            // (m), each map grid spans this much real-world distance
float OBS_PRIOR = 0.2;             // the prior_probability of an obstacle in the laser map
int MAP_MEMORY = 100;              // (last MAP_MEMORY readings of a particular grid used to calculate probability of obstacle there) only used if PROBMAP is defined above
float SCANNER_RANGE = 5.5;         // the range of the scanner in meters;
float BUMPER_COST = 1;             // the probability of obstacle associated with a bumper hit
float robot_radius = .2;           // (m)
bool using_tf = false;             // when set to true, use the tf package
float global_map_x_offset = 0;     // the map is this far off of the world coordinate system in the x direction
float global_map_y_offset = 0;     // the map is this far off of the world coordinate system in the y direction
float global_map_theta_offset = 0; // the map is this far off of the world coordinate system in the rotationally

// global ROS subscriber handles
ros::Subscriber laser_scan_sub;
ros::Subscriber pose_sub;
ros::Subscriber bumper_pose_sub;

// global ROS publisher handles
ros::Publisher map_changes_pub;

// global ROS provide service server handles
ros::ServiceServer get_map_srv;

// globals
MAP* laser_map = NULL;
MAP* bumper_map = NULL;
vector<POINT> laser_map_changes;
vector<POINT> bumper_map_changes;

int HEIGHT = map_y_max/RESOLUTION + 1; //the height of the map in grids
int WIDTH = map_x_max/RESOLUTION + 1;  // the width of the map in grids
float MAP_INC = 1/(float)MAP_MEMORY; // each laser reading is worth this much probability

/* ------------------ stuff for loading images ------------------------- */ 
struct float_array;
typedef struct float_array float_array;


struct float_array
{
    int rows, cols; // size of A
    int temp;       // a flag to help with memory management
    int t;          // a flag to allow for easy transposing
    float** A;
};

float_array* make_float_array(int rows, int cols)
{
    float_array* this_array = (float_array*)calloc(1, sizeof(float_array));

    this_array->rows = rows;
    this_array->cols = cols;
    this_array->temp = 0;
    this_array->t = 0;

    this_array->A = (float**)calloc(rows, sizeof(float*));
    int i;
    for(i = 0; i < rows; i++)
        this_array->A[i] = (float*)calloc(cols, sizeof(float));
    return this_array;
}

// adjusts the array A randomly, where each column's max adjustment is given by +/- the corresponding 
// value in R_max. Note A itself is changed (a new array is not returned)
float_array* adjust_array_randomly(float_array* A, float* R_max)
{
  int i,j;
  float** AA = A->A;
  int rows = A->rows;
  int cols = A->cols;

  for(i = 0; i < rows; i++)
    for(j = 0; j < cols; j++)
      AA[i][j] += R_max[j]*(1 - 2*(float)(rand() % (int)(101))/100); // adds random number on [-R_max ... R_max]

  return A;
}

void destroy_float_array(float_array* this_array)
{
  if(this_array == NULL)
    return;

  int rows = this_array->rows;

  int i;
  for(i = 0; i < rows; i++)
    free(this_array->A[i]);

  free(this_array->A);
  free(this_array);
}

// prints the float array, also destroys it if it is a temp
void print_float_array(float_array* this_array)
{
  int i, j;
  if(this_array == NULL)
  {
    printf("0\n");
    return;
  }

  int rows = this_array->rows;
  int cols = this_array->cols;

  if(this_array->t == 0)
  {
    for(i = 0; i < rows; i++)
    {
      for(j = 0; j < cols; j++)
        printf("%f ",this_array->A[i][j]);
      printf("\n");
    }
  }
  else // it is inverted
  {
    for(j = 0; j < cols; j++)
    {
      for(i = 0; i < rows; i++)
        printf("%f ",this_array->A[i][j]);
      printf("\n");
    }
  }

  if(this_array->temp == 1)
    destroy_float_array(this_array);
}
#define TWOBYTE unsigned short
#define FOURBYTE unsigned long
#define LONG unsigned long

struct Bitmap;
typedef struct Bitmap Bitmap;

struct Image;
typedef struct Image Image;

struct Bitmap
{
  // BMP header stuff
  TWOBYTE BMP_Type;               // 2 bytes
  FOURBYTE BMP_Size;              // 4 bytes
  FOURBYTE BMP_Reserved;          // 4 bytes
  FOURBYTE BMP_Offset;            // 4 bytes

  // DIB header stuff
  FOURBYTE DBI_Size;              // 4 bytes
  LONG DBI_Width;                 // 4 bytes
  LONG DBI_Height;                // 4 bytes
  TWOBYTE DBI_Colorplanes;        // 2 bytes
  TWOBYTE DBI_BitsPerPixel;       // 2 bytes
  FOURBYTE DBI_CompressionMethod; // 4 bytes
  FOURBYTE DBI_ImageSize;         // 4 bytes
  LONG DBI_HorizPixelsPerMeter;   // 4 bytes
  LONG DBI_VertPixelsPerMeter;    // 4 bytes
  FOURBYTE DBI_PaletteSize;       // 4 bytes
  FOURBYTE DBI_ImportantColors;         // 4 bytes

  unsigned long BPP;
  unsigned long width;
  unsigned long height;
  unsigned long size;
  unsigned char* Bitmap_Image;
  unsigned char* palette;
  unsigned int bps;
  unsigned int KompressionFormat;
};

Bitmap* make_Bitmap()
{
  Bitmap* Bmp = (Bitmap*)calloc(1, sizeof(Bitmap));
  Bmp->BPP=0;
  Bmp->width=0;
  Bmp->height=0;
  Bmp->Bitmap_Image = NULL;
  Bmp->palette = NULL;
  Bmp->size=0;
  Bmp->bps=0;
  Bmp->KompressionFormat=0;

  return Bmp;
}

void destroy_Bitmap(Bitmap* Bmp)
{
  if(Bmp == NULL)
    return;

  free(Bmp->Bitmap_Image);

  if(Bmp->palette != NULL)
    free(Bmp->palette);

  free(Bmp);
}

void print_Bitmap_info(Bitmap* B)
{
  if( B == NULL)
	  return;

  printf("BMP Header: \n");
  printf(" Type: %x \n", B->BMP_Type);
  printf(" Size: %u \n", B->BMP_Size);
  printf(" Reserved: %u \n", B->BMP_Reserved);
  printf(" Offset: %u \n", B->BMP_Offset);

  printf("DIB Header: \n");
  printf(" Size: %u \n", B->DBI_Size);
  printf(" Width: %u \n", B->DBI_Width);
  printf(" Height: %u \n", B->DBI_Height);
  printf(" Color Planes: %u \n", B->DBI_Colorplanes);
  printf(" Bits Per Pixel: %u \n", B->DBI_BitsPerPixel);
  printf(" Compression Method: %u \n", B->DBI_CompressionMethod);
  printf(" Image Size: %u \n", B->DBI_ImageSize);
  printf(" Horizontal Pixels Per Meter: %u \n", B->DBI_HorizPixelsPerMeter);
  printf(" Vertical Pixels PEr Meter: %u \n", B->DBI_VertPixelsPerMeter);
  printf(" Palette Size: %u \n", B->DBI_PaletteSize);
  printf(" Important Colors: %u \n", B->DBI_ImportantColors);
}


Bitmap*  load_Bitmap_from_file(const char *filename)
{
  Bitmap* B = make_Bitmap();

  FILE* inf = NULL;
  unsigned int ImageIdx = 0;
  unsigned char* Bitmap_Image = NULL;

  if(!filename)
  {
    printf("can't open file");
    return NULL;
  }
  else
  {
    inf = fopen(filename,"rb");
    if(!inf)
    {
      printf("can't open file \n");
      return NULL;
    }
  }

  // read in the 14 byte BMP header
  fread(&B->BMP_Type,sizeof(TWOBYTE),1,inf);
  fread(&B->BMP_Size,sizeof(FOURBYTE),1,inf);
  fread(&B->BMP_Reserved,sizeof(FOURBYTE),1,inf);
  fread(&B->BMP_Offset,sizeof(FOURBYTE),1,inf);

  if(ferror(inf))
  {
    printf("problem with file header \n");
    fclose(inf);
    return NULL;
  }

  // read in the 40 byte DIB Header, assuming V3 is used
  fread(&B->DBI_Size,sizeof(FOURBYTE),1,inf);
  if(B->DBI_Size != 40)
  {
    printf("cannot open file because it does not use DBI header V3 \n");
    fclose(inf);
    return NULL;
  }
  fread(&B->DBI_Width,sizeof(LONG),1,inf);
  fread(&B->DBI_Height,sizeof(LONG),1,inf);
  fread(&B->DBI_Colorplanes,sizeof(TWOBYTE),1,inf);
  fread(&B->DBI_BitsPerPixel,sizeof(TWOBYTE),1,inf);
  fread(&B->DBI_CompressionMethod,sizeof(FOURBYTE),1,inf);
  fread(&B->DBI_ImageSize,sizeof(FOURBYTE),1,inf);
  fread(&B->DBI_HorizPixelsPerMeter,sizeof(LONG),1,inf);
  fread(&B->DBI_VertPixelsPerMeter,sizeof(LONG),1,inf);
  fread(&B->DBI_PaletteSize,sizeof(FOURBYTE),1,inf);
  fread(&B->DBI_ImportantColors,sizeof(FOURBYTE),1,inf);

  if(ferror(inf))
  {
    printf("problem with info header \n");
    fclose(inf);
    return NULL;
  }

  // extract color palette
  if(B->DBI_BitsPerPixel == 24)
  {
    // don't need to
  }
  else if(B->DBI_BitsPerPixel == 8 || B->DBI_BitsPerPixel == 4 || B->DBI_BitsPerPixel == 1)
  {

    int colors_used = B->DBI_PaletteSize;
    if(colors_used == 0)
      colors_used = (int)pow((float)2,(int)B->DBI_BitsPerPixel);

    //printf(" colors used: %d \n", colors_used);

    unsigned char* color_map = (unsigned char*)calloc(colors_used*4, sizeof(unsigned char));
    fread(color_map,sizeof(unsigned char),colors_used*4,inf);

    B->palette = color_map;
  }
  else
    printf("This number of bits per pixel (%u) not implimented \n", B->DBI_BitsPerPixel);


  fseek(inf,B->BMP_Offset,SEEK_SET);
  if(ferror(inf))
  {
    printf("problem with 'bfOffBits' \n");
    fclose(inf);
    return 0;
  }


  if(B->DBI_ImageSize != 0)
  {
    Bitmap_Image = (unsigned char*)calloc(B->DBI_ImageSize, sizeof(unsigned char));
    fread(Bitmap_Image,B->DBI_ImageSize,1,inf);
  }

  if(B->BMP_Type != 0x4D42)
  {
    printf("problem with magic number \n");
    fclose(inf);
  }

  if(!Bitmap_Image)
  {
    free(Bitmap_Image);
    fclose(inf);
  }

  if(Bitmap_Image==NULL)
    fclose(inf);

  B->Bitmap_Image = Bitmap_Image;

  fclose(inf);

  return B;
}

struct Image
{
  float_array* Red;
  float_array* Green;
  float_array* Blue;
};

Image* make_Image(int rows, int cols)
{
  Image* I = (Image*)calloc(1, sizeof(Image));
  I->Red = make_float_array(rows,cols);
  I->Green = make_float_array(rows,cols);
  I->Blue = make_float_array(rows,cols);

  return I;
}

void destroy_Image(Image* I)
{
  if(I == NULL)
    return;

  destroy_float_array(I->Red);
  destroy_float_array(I->Green);
  destroy_float_array(I->Blue);
  free(I);
}

void print_Image(Image* I)
{
  if(I == NULL)
    return;

  printf("Red: \n");
  print_float_array(I->Red);

  printf("Green: \n");
  print_float_array(I->Green);

  printf("Blue: \n");
  print_float_array(I->Blue);
}

// converts the Bitmap struct into an Image struct
Image* convert_Bitmap_to_double_array(Bitmap* B)
{
  if(B == NULL)
    return NULL;

  if(B->DBI_CompressionMethod != 0)
  {
    printf("unsupported compression type \n");
    return NULL;
  }

  int rows = B->DBI_Height;
  int cols = B->DBI_Width;

  Image* I = make_Image(rows, cols);
  float** Red = I->Red->A;
  float** Blue = I->Blue->A;
  float** Green = I->Green->A;
  unsigned char* Data = B->Bitmap_Image;

  int bits_per_pixel = B->DBI_BitsPerPixel;

  if(bits_per_pixel == 24)
  {
    int i, j, k;
    int col_size_with_pad = 3*cols;
    if(4*(col_size_with_pad/4) != col_size_with_pad)
      col_size_with_pad = 4*(col_size_with_pad/4+1);

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      for(j = 0; j < cols; j++)
      {
        Blue[i][j]  = (float)Data[k]/255;
        k++;
        Green[i][j] = (float)Data[k]/255;
        k++;
        Red[i][j] = (float)Data[k]/255;
        k++;
      }
    }
    return I;
  }
  else if(bits_per_pixel == 8)
  {
    int i, j, k;
    int col_size_with_pad = cols;
    if(4*(col_size_with_pad/4) != col_size_with_pad)
      col_size_with_pad = 4*(col_size_with_pad/4+1);

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      for(j = 0; j < cols; j++)
      {
        Blue[i][j]  = (float)B->palette[Data[k]*4]/255;
        Green[i][j] = (float)B->palette[Data[k]*4+1]/255;
        Red[i][j] = (float)B->palette[Data[k]*4+2]/255;
        k++;
      }
    }
    return I;

  }
  else if(bits_per_pixel == 4)
  {
    int i, j, k;
    int col_size_with_pad = cols;

    if(8*(col_size_with_pad/8) != col_size_with_pad)
      col_size_with_pad = 8*(col_size_with_pad/8+1);

    col_size_with_pad = col_size_with_pad/2;

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      unsigned char this_char = (unsigned char)Data[k];
      int bit_num = -1;
      int this_bit;

      for(j = 0; j < cols; j++)
      {
        bit_num++;
        if(bit_num == 2)
        {
          bit_num = 0;
          k++;
          this_char = Data[k];
        }
        this_bit = this_char/16;
        this_char = this_char << 4;


        Blue[i][j]  = (float)B->palette[this_bit*4]/255;
        Green[i][j] = (float)B->palette[this_bit*4+1]/255;
        Red[i][j] = (float)B->palette[this_bit*4+2]/255;
      }
    }
    return I;

  }
  else if(bits_per_pixel == 1)
  {
    int i, j, k;
    int col_size_with_pad = cols;

    if(32*(col_size_with_pad/32) != col_size_with_pad)
      col_size_with_pad = 32*(col_size_with_pad/32+1);

    col_size_with_pad = col_size_with_pad/8;

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      unsigned char this_char = (unsigned char)Data[k];
      int bit_num = -1;
      int this_bit;

      for(j = 0; j < cols; j++)
      {
        bit_num++;
        if(bit_num == 8)
        {
          bit_num = 0;
          k++;
          this_char = Data[k];
        }
        this_bit = this_char/128;
        this_char = this_char << 1;


        Blue[i][j]  = (float)B->palette[this_bit*4]/255;
        Green[i][j] = (float)B->palette[this_bit*4+1]/255;
        Red[i][j] = (float)B->palette[this_bit*4+2]/255;
      }
    }
    return I;

  }
  else
    printf("This number of bits per pixel not supported \n");

  return NULL;
}
////////// end stuff for loading images


/* -------------------------- POINT -------------------------------------*/
struct POINT
{
    float x;
    float y;
    float z;
};

/*---------------------- MAP --------------------------------------------*/
struct MAP
{
    float** cost;    
    int height;
    int width;
    float resolution;
    
    #if defined(SIMPLEMAP) || defined(PROBMAP)
      int** in_list;
    #endif           
};

// this creates and returns a pointer to a map struct
MAP* make_map(int height, int width, float resolution)
{
  MAP* map = (MAP*)calloc(1, sizeof(MAP));
  map->height = height;
  map->width = width;
  map->resolution = resolution; 
  
  map->cost = (float**)calloc(height, sizeof(float*));
  for (int y = 0; y < height; y++)  
  {
	map->cost[y] = (float*)calloc(width, sizeof(float)); 
    for(int x = 0; x < width; x++) 
      map->cost[y][x] = OBS_PRIOR;
  }

  #if defined(SIMPLEMAP) || defined(PROBMAP)
    map->in_list = (int**)calloc(height, sizeof(int*));
    for (int y = 0; y < height; y++)   
    {
      map->in_list[y] = (int*)calloc(width, sizeof(int));
      for(int x = 0; x < width; x++) 
        map->in_list[y][x] = -1;
    }
  #endif
                 
  return map;    
}


// this allocates all required memory for a map
void destroy_map(MAP* map)
{
  int y;
  if(map != NULL)
  {
    for (y = 0; y < map->height; y++)  
    {
	  if(map->cost[y] != NULL)
        free(map->cost[y]);
    }

    if(map->cost != NULL)
      free(map->cost);
    
    #if defined(SIMPLEMAP) || defined(PROBMAP)
      for (y = 0; y < map->height; y++)  
      {
        if(map->in_list[y] != NULL)
          free(map->in_list[y]);
      }
    
      if(map->in_list != NULL)
        free(map->in_list);
    #endif
                    
    map->height = 0;
    map->width = 0;
    free(map);
  }
}

// prints map on command line
void print_map(MAP* map)
{
  int y, x;

  if(map->cost != NULL)
  {
    printf("\n");  
    for (y = 0; y < map->height; y++)  
    {
      for (x = 0; x < map->width; x++)  
	    printf(" %f", map->cost[y][x]);
      printf("\n");  
    }
    printf("\n");  
  } 
}

// creates a blank map of height, width, and resolution, and cost cst
MAP* load_blank_map(int map_height, int map_width, float map_resolution, float cst)
{
  MAP* map = make_map(map_height, map_width, map_resolution);
     
  for(int i = 0; i < map_height; i++)
    for(int j = 0; j < map_width; j++)
        map->cost[i][j] = cst;   
  return map;
}

// populates the map with costs determined by the bitmap in filename that are larger than thresh
void populateMapFromBitmap(MAP* Map, char* filename, float thresh)
{
  Bitmap* B = load_Bitmap_from_file(filename);
  Image* I = convert_Bitmap_to_double_array(B);

  float** map = Map->cost;
  
  int x, y;
  for (y = 0; y < HEIGHT; ++y)
    for (x = 0; x < WIDTH; ++x)
      if(1-I->Blue->A[y][x] > thresh)
        map[HEIGHT-1-y][x] = 1-I->Blue->A[y][x];  
  
  destroy_Image(I);
  destroy_Bitmap(B);
}

/*---------------------------- ROS tf functions -------------------------*/
void broadcast_robot_tf()
{
 
  static tf::TransformBroadcaster br;  
    
  tf::Transform transform;   
  transform.setOrigin(tf::Vector3(global_map_x_offset, global_map_y_offset, 0));
  transform.setRotation(tf::Quaternion(global_map_theta_offset, 0, 0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world_cu", "/map_cu"));  
}

/*---------------------------- ROS Callbacks ----------------------------*/
void laser_scan_callback(const hokuyo_listener_cu::PointCloudWithOrigin::ConstPtr& msg)
{ 
  if(laser_map == NULL)
      return;
    
  int length = msg->cloud.points.size();
  
  if(length <= 0)
    return;
 
  
  float x_scl = 1/laser_map->resolution;
  float y_scl = 1/laser_map->resolution;
  
  // change map, and remember changes
  
  //zeroth pass transform to grid coords (super annoying bug, can't figure out why this needs to be +1)
  vector<POINT> scl_cloud_points;
  scl_cloud_points.resize(length);
  for(int i = 0; i < length; i++)
  {
    scl_cloud_points[i].x = x_scl*msg->cloud.points[i].x + 1;
    scl_cloud_points[i].y = y_scl*msg->cloud.points[i].y + 1;
  }
      
  #ifdef HITMAP // just add new hits to the map
          
    laser_map_changes.resize(length);
    float xf, yf;
    int x, y;
    int j=0;
    for(int i = 0; i < length; i++)
    {
      xf = scl_cloud_points[i].x;
      yf = scl_cloud_points[i].y;
    
      x = (int)xf;
      y = (int)yf;
    
      float this_cost = 1;
    
      if(x >= 0 && x < laser_map->width && y >= 0 && y < laser_map->height)
      {
        laser_map->cost[y][x] = this_cost;
    
        laser_map_changes[j].x = (float)x;
        laser_map_changes[j].y = (float)y;
        laser_map_changes[j].z = this_cost;
        j++;
      }
    }
    laser_map_changes.resize(j);
    
  #elif defined(SIMPLEMAP) || defined(PROBMAP) // add new hits, but first remove old hits the sensor had to see through in order to see the new hits
           
    float origin_x = x_scl*msg->origin.x;
    float origin_y = y_scl*msg->origin.y; 
          
    // first pass, see the max memory we will need to store all changes
    float min_x = scl_cloud_points[0].x;
    float max_x = scl_cloud_points[0].x;
    float min_y = scl_cloud_points[0].y; 
    float max_y = scl_cloud_points[0].y; 
    for(int i = 0; i < length; i++)
    {
      if(min_x > scl_cloud_points[i].x)
        min_x = scl_cloud_points[i].x;
      else if(max_x < scl_cloud_points[i].x)
        max_x = scl_cloud_points[i].x;
      
      if(min_y > scl_cloud_points[i].y)
        min_y = scl_cloud_points[i].y;
      else if(max_y < scl_cloud_points[i].y)
        max_y = scl_cloud_points[i].y;     
    }
          
    int max_changes = ((int)max_x - (int)min_x + 2)*((int)max_y - (int)min_y + 2);  
    int next_change = laser_map_changes.size(); // remember previous changes that have not been sent out yet
    laser_map_changes.resize(next_change + max_changes);  
          
    // create a small temp map to accumulate changes
    MAP*  tempmap = load_blank_map((int)max_y - (int)min_y + 1, (int)max_x - (int)min_x + 1, RESOLUTION, 0);

    // second pass, calculate empty spaces
    float rise, run, y_start = 0, y_end = 0, x_start = 0, x_end = 0;
    int direction_flag = -1;
    float** cost = laser_map->cost;
    float** tcost = tempmap->cost;
    int map_x_offset = (int)min_x;
    int map_y_offset = (int)min_y;
    for(int i = 0; i < length; i++)
    {     
      rise =  scl_cloud_points[i].y - origin_y;
      run =  scl_cloud_points[i].x - origin_x;
    
      if(rise >= 0 && run >= 0) // work from origin to hit
      {
        y_start = origin_y;
        y_end = scl_cloud_points[i].y;
        x_start = origin_x;
        x_end = scl_cloud_points[i].x;
        
        if(run > rise) // work in x direction
          direction_flag = 0;
        else // work in y direction
          direction_flag = 1;
      }
      else if(rise < 0 && run < 0) // work from hit to origin
      {
        y_end = origin_y;
        y_start = scl_cloud_points[i].y;
        x_end = origin_x;
        x_start = scl_cloud_points[i].x;
        
        rise *= -1;
        run *= -1;
      
        if(run > rise) // work in x direction
          direction_flag = 0;
        else // work in y direction
          direction_flag = 1;  
      }
      else if(rise < 0)
      {
        if(run >= -rise) // work from origin to hit in x direction
        {
          y_start = origin_y;
          y_end = scl_cloud_points[i].y;
          x_start = origin_x;
          x_end = scl_cloud_points[i].x;
      
          direction_flag = 0;
        }
        else // work from hit to origin in y direction
        {
          y_end = origin_y;
          y_start = scl_cloud_points[i].y;
          x_end = origin_x;
          x_start = scl_cloud_points[i].x;
        
          rise *= -1;
          run *= -1;
      
          direction_flag = 1;  
        }
      }
      else if(run < 0)
      {
        if(-run >= rise) // work from hit to origin in x direction
        {  
          y_end = origin_y;
          y_start = scl_cloud_points[i].y;
          x_end = origin_x;
          x_start = scl_cloud_points[i].x;
           
          rise *= -1;
          run *= -1;
        
          direction_flag = 0;
        }
        else // work from origin to hit in y direction
        {
          y_start = origin_y;
          y_end = scl_cloud_points[i].y;
          x_start = origin_x;
          x_end = scl_cloud_points[i].x;
        
          direction_flag = 1;
        }
      }
   
      if(direction_flag == 0) // working in x direction
      {
        float m = rise/run;
        int r;
        int the_test = (int)x_end;
        for(int c = (int)x_start + 2; c < the_test; c++)
        {
          r = (int)(m*(c - x_start) + y_start);
          
          // update positions [r][c-1]  and [r][c] in large map
          tcost[r-map_y_offset][c-1-map_x_offset] -= 1;
          tcost[r-map_y_offset][c-map_x_offset] -= 1;    
        }
      }
      else // work in y direction
      {
        float m = run/rise;
        int c;
        int the_test = (int)y_end;
        for(int r = (int)y_start + 2; r < the_test; r++)
        {
          c = (int)(m*(r - y_start) + x_start);
          
          // update positions [r-1][c] and [r][c] in large map
          tcost[r-1-map_y_offset][c-map_x_offset] -= 1;
          tcost[r-map_y_offset][c-map_x_offset] -= 1;   
        }
      } 
    }
   
    // third pass, remember hits
    for(int i = 0; i < length; i++)
    {

      // only mark hits within range
      float no_scale_dx = msg->cloud.points[i].x-msg->origin.x;
      float no_scale_dy = msg->cloud.points[i].y-msg->origin.y;
      
      if(SCANNER_RANGE <= sqrt(no_scale_dx*no_scale_dx + no_scale_dy*no_scale_dy))
        continue;
              
      #ifdef SIMPLEMAP  
        tcost[(int)(scl_cloud_points[i].y) - map_y_offset][
              (int)(scl_cloud_points[i].x) - map_x_offset] = 1;   
      #elif defined(PROBMAP)
        tcost[(int)(scl_cloud_points[i].y) - map_y_offset][
              (int)(scl_cloud_points[i].x) - map_x_offset] += 1;   
      #endif
    }  

    // fourth pass (first and only pass through the temp map)
    int** in_list = laser_map->in_list;
    int height = laser_map->height;
    int width = laser_map->width;
    float** bumper = bumper_map->cost;
    for(int r = 0; r < tempmap->height; r++)
    {
      for(int c = 0; c < tempmap->width; c++)
      { 
        if(tcost[r][c] != 0)
        {  
          int global_r = r+map_y_offset;
          int global_c = c+map_x_offset;
              
          if(global_r >= 0 && global_r < height && global_c >= 0 && global_c < width)
          {  
            if(in_list[global_r][global_c] == -1) // not already in list
            {  
              if(tcost[r][c] < 0 && cost[global_r][global_c] == 0) // don't need to report this change
                continue;
              if(tcost[r][c] > 0 && cost[global_r][global_c] == 1) // don't need to report this change
                continue; 
              if(bumper[global_r][global_c] != 0) // in bumber hit map, so we don't care what the laser scanner says
                continue;
              
              laser_map_changes[next_change].x = (float)(global_c);
              laser_map_changes[next_change].y = (float)(global_r);
          
              in_list[global_r][global_c] = next_change;
              next_change++;
            }
             
            #ifdef SIMPLEMAP
              if(tcost[r][c] < 0)
              {
                cost[global_r][global_c] = 0;
                laser_map_changes[in_list[global_r][global_c]].z = 0;
              }
              else
              {
                cost[global_r][global_c] = 1; 
                laser_map_changes[in_list[global_r][global_c]].z = 1; 
              }
            #elif defined(PROBMAP)
              float new_cost = cost[global_r][global_c] + MAP_INC*tcost[r][c];
            
              if(new_cost < 0)
              {
                cost[global_r][global_c] = 0;
                laser_map_changes[in_list[global_r][global_c]].z = 0;
              }
              else if(new_cost > 1)
              {
                cost[global_r][global_c] = 1;
                laser_map_changes[in_list[global_r][global_c]].z = 1;
              }
              else
              {
                cost[global_r][global_c] = new_cost;  
                laser_map_changes[in_list[global_r][global_c]].z = new_cost;
              }
            #endif

          }
        }
      }
    }
    laser_map_changes.resize(next_change);
    destroy_map(tempmap);
  
  #endif
  scl_cloud_points.resize(0);       
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
  if(laser_map == NULL)
    return;
      
  // mark grids within the radius of the robot as safe 
  float resolution = laser_map->resolution;
  float pose_x = msg->pose.position.x/resolution;
  float pose_y = msg->pose.position.y/resolution;
  float rob_rad = robot_radius/resolution;
  
  int width = laser_map->width;
  int height = laser_map->height;
  
  int start_r = (int)(pose_y - rob_rad);
  if(start_r < 0)
    start_r = 0;
          
  int end_r = (int)(pose_y + rob_rad + 1);
  if(end_r >= height)
    end_r = height-1;
 
  int start_c = (int)(pose_x - rob_rad);
  if(start_c < 0)
    start_c = 0;
  
  int end_c = (int)(pose_x + rob_rad + 1);
  if(end_c >= width)
    end_c = width-1;
  
  int max_changes = (end_r - start_r)*(end_c - start_c);
  float** cost = laser_map->cost;
  float** b_cost = bumper_map->cost;
  
  #if defined(SIMPLEMAP) || defined(PROBMAP)
  int** in_list = laser_map->in_list;
  int** b_in_list = laser_map->in_list;
  #endif
  
  int next_change = laser_map_changes.size();
  int b_next_change = bumper_map_changes.size();
  
  laser_map_changes.resize(next_change+max_changes);
  bumper_map_changes.resize(b_next_change+max_changes);
  
  float dist;
  bool there_was_a_change = false;
  for(int r = start_r; r <= end_r; r++)
  {
    for(int c = start_c; c <= end_c; c++)   
    {
      float x_dist = ((float)c - .5 - pose_x);
      float y_dist = ((float)r - .5 - pose_y);
      
      dist = sqrt(x_dist*x_dist + y_dist*y_dist);
      
      if(dist < rob_rad)
      {
        #if defined(SIMPLEMAP) || defined(PROBMAP)
        
          there_was_a_change = true;
          if(in_list[r][c] == -1) // not already in list (these get accumulated forever)
          {  
            if(cost[r][c] == 0) // don't need to report this change (this grid is already free of obstacles)
              there_was_a_change = false;
            else
            {
              laser_map_changes[next_change].x = (float)c;
              laser_map_changes[next_change].y = (float)r;
              in_list[r][c] = next_change;
              next_change++;
            }
          }
        
          if(there_was_a_change)
          {
            // change laser_map
            cost[r][c] = 0;
            laser_map_changes[in_list[r][c]].z = 0; 
          }
          
          there_was_a_change = true;
          if(b_in_list[r][c] == -1) // not already in list (these get accumulated forever)
          {  
            if(b_cost[r][c] == 0) // don't need to report this change (this grid is already free of obstacles)
              there_was_a_change = false;
            else
            {
              bumper_map_changes[b_next_change].x = (float)c;
              bumper_map_changes[b_next_change].y = (float)r;
              b_in_list[r][c] = b_next_change;
              b_next_change++;
            }
          }
        
          if(there_was_a_change)
          {
            // change bumper_map
            b_cost[r][c] = 0;
            bumper_map_changes[b_in_list[r][c]].z = 0; 
          }
          
        #elif defined(HITMAP) 
        
          if(cost[r][c] != 0) // only report this change if  grid is not already free of obstacles
            laser_map_changes[next_change].x = (float)c;
            laser_map_changes[next_change].y = (float)r;
            next_change++;   
          
            // change laser_map
            cost[r][c] = 0;
          }
          
          
          if(b_cost[r][c] != 0) // only report this change if  grid is not already free of obstacles
          {
            bumper_map_changes[b_next_change].x = (float)c;
            bumper_map_changes[b_next_change].y = (float)r;
            b_next_change++;   
          
            // change bumper_map
            b_cost[r][c] = 0;
          }     
          
        #endif
      }
    }
  }
  laser_map_changes.resize(next_change);
}

void bumper_pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{ 
  if(bumper_map == NULL)
    return;
  
  float x_scl = 1/laser_map->resolution;
  float y_scl = 1/laser_map->resolution;
  
  // transform bumper points to map coords and remember them
  float x = (int)(x_scl*msg->x + 1); // same + 1 annoying bug as in laser scan callback
  float y = (int)(y_scl*msg->y + 1); // same + 1 annoying bug as in laser scan callback
          
  int list_ind = bumper_map_changes.size(); 
  bumper_map_changes.resize(list_ind+1);
  
  if(x >= 0 && x < laser_map->width && y >= 0 && y < laser_map->height)
  {
    bumper_map->cost[(int)y][(int)x] = BUMPER_COST;
    
    bumper_map_changes[list_ind].x = (float)x;
    bumper_map_changes[list_ind].y = (float)y;
    bumper_map_changes[list_ind].z = BUMPER_COST;
    
    #if defined(SIMPLEMAP) || defined(PROBMAP)
      bumper_map->in_list[(int)y][(int)x] = list_ind;
    #endif
  }
}

bool get_map_callback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &resp)
{
 if(laser_map == NULL || bumper_map == NULL)
   return false;
  
  int width = laser_map->width;
  int height = laser_map->height;
  
  resp.map.header.frame_id = "/map_cu";
  resp.map.info.width = width;
  resp.map.info.height = height;
  resp.map.info.resolution = laser_map->resolution;
           
  resp.map.data.resize(height*width);
          
  float** laser_cost = laser_map->cost;
  float** bumper_cost = bumper_map->cost;
          
  int i = 0;
  for(int r = 0; r < height; r++)
  {
    for(int c = 0; c < width; c++)
    {
      if(bumper_cost[r][c] != 0) // bumper hit
        resp.map.data[i] = (uint8_t)(100*bumper_cost[r][c]);
      else // laser cost
        resp.map.data[i] = (uint8_t)(100*laser_cost[r][c]);
      i++;
    }
  }
  return true;
}


/*---------------------- ROS Publisher Functions ------------------------*/
void publish_map_changes()
{
  int length = laser_map_changes.size();
  int bumper_length = bumper_map_changes.size();
    
  if(length <= 0)
      return;
  
  sensor_msgs::PointCloud msg;  
  
  msg.points.resize(length+bumper_length);

  #if defined(SIMPLEMAP) || defined(PROBMAP)
  int** in_laser_list = laser_map->in_list;
  int** in_bumper_list = bumper_map->in_list;
  #endif
  
  // add laser data to message
  int i;
  for(i = 0; i < length; i++)
  {  
    msg.points[i].x = laser_map_changes[i].x;
    msg.points[i].y = laser_map_changes[i].y;
    msg.points[i].z = laser_map_changes[i].z;
    
    #if defined(SIMPLEMAP) || defined(PROBMAP)
      // mark as not in change list
      in_laser_list[(int)laser_map_changes[i].y][(int)laser_map_changes[i].x] = -1;
    #endif
  } 
  
  // add bumper data to message (bumper overrides laser, hence it is last)
  for(int j = 0; j < bumper_length; j++)
  {
    msg.points[i].x = bumper_map_changes[j].x;
    msg.points[i].y = bumper_map_changes[j].y;
    msg.points[i].z = bumper_map_changes[j].z;
    
    #if defined(SIMPLEMAP) || defined(PROBMAP)
      // mark as not in change list
      in_bumper_list[(int)bumper_map_changes[i].y][(int)bumper_map_changes[i].x] = -1;
    #endif
    
    i++;
  }
  
  map_changes_pub.publish(msg);
 
  laser_map_changes.resize(0);
  bumper_map_changes.resize(0);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "mapper_cu");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
    
  // load globals from parameter server
  double param_input;
  int int_input;
  bool bool_input;
  if(ros::param::get("mapper_cu/map_y_size", param_input)) 
    map_y_max = (float)param_input;                                        // (m), map goes from 0 to this in the y direction
  if(ros::param::get("mapper_cu/map_x_size", param_input)) 
    map_x_max = (float)param_input;                                        // (m), map goes from 0 to this in the x direction
  if(ros::param::get("mapper_cu/map_resolution", param_input)) 
    RESOLUTION = (float)param_input;                                       // (m), each map grid spans this much real-world distance
  if(ros::param::get("mapper_cu/obstacle_prior", param_input)) 
    OBS_PRIOR = (float)param_input;                                        // the prior_probability of an obstacle in the laser map
  if(ros::param::get("mapper_cu/map_attention_span", int_input)) 
    MAP_MEMORY = int_input;                                                // (last MAP_MEMORY readings of a particular grid used to calculate probability of obstacle there) only used if PROBMAP is defined above
  if(ros::param::get("mapper_cu/scanner_range", param_input)) 
    SCANNER_RANGE = (float)param_input;                                    // the range of the scanner in meters;
  if(ros::param::get("mapper_cu/bumper_obstacle_posterior", param_input)) 
    BUMPER_COST = (float)param_input;                                      // the probability of obstacle associated with a bumper hit
  if(ros::param::get("mapper_cu/robot_radius", param_input)) 
    robot_radius = (float)param_input;                                     // (m)
  if(ros::param::get("prairiedog/using_tf", bool_input)) 
    using_tf = bool_input;                                                 // when set to true, use the tf package
  if(ros::param::get("mapper_cu/global_map_x_offset", param_input))
    global_map_x_offset = (float)param_input;                              // the map is this far off of the world coordinate system in the x direction
  if(ros::param::get("mapper_cu/global_map_y_offset", param_input))
    global_map_y_offset = (float)param_input;                              // the map is this far off of the world coordinate system in the y direction
  if(ros::param::get("mapper_cu/global_map_theta_offset", param_input))
    global_map_theta_offset = (float)param_input;                          // the map is this far off of the world coordinate system in the rotationally
  
  HEIGHT = map_y_max/RESOLUTION + 1; //the height of the map in grids
  WIDTH = map_x_max/RESOLUTION + 1;  // the width of the map in grids
  MAP_INC = 1/(float)MAP_MEMORY;     // each laser reading is worth this much probability
  
  // print data about parameters
  if(using_tf)
    printf("using tf\n");
  else
    printf("not using tf\n");
  printf("map offset (x, y, theta): [%f %f %f]\n", global_map_x_offset, global_map_y_offset, global_map_theta_offset);
  printf("map size: [%f %f], resolution:%f, obs_prior:%f, memory_time:%d \n", map_y_max, map_x_max, RESOLUTION, OBS_PRIOR, MAP_MEMORY); 
  printf("bumper cost:%f, scanner range:%f, robot radius:%f \n", BUMPER_COST, SCANNER_RANGE, robot_radius);
  
    
  destroy_map(laser_map);
  destroy_map(bumper_map);
  laser_map = load_blank_map(HEIGHT, WIDTH, RESOLUTION, OBS_PRIOR);
  bumper_map = load_blank_map(HEIGHT, WIDTH, RESOLUTION, 0);
    
  populateMapFromBitmap(bumper_map, "../map.bmp", OBS_PRIOR);
    
  // set up subscribers
  laser_scan_sub = nh.subscribe("/cu/laser_scan_cu", 1, laser_scan_callback);
  pose_sub = nh.subscribe("/cu/pose_cu", 1, pose_callback);
  bumper_pose_sub = nh.subscribe("/cu/bumper_pose_cu", 10, bumper_pose_callback);
    
  // set up publishers
  map_changes_pub = nh.advertise<sensor_msgs::PointCloud>("/cu/map_changes_cu", 1);
    
  // set up service servers
  get_map_srv = nh.advertiseService("/cu/get_map_cu", get_map_callback);

  while (ros::ok()) 
  {
    publish_map_changes();
       
    if(using_tf)
      broadcast_robot_tf();
    
    ros::spinOnce();
    loop_rate.sleep();
  }
    
  laser_scan_sub.shutdown();
  pose_sub.shutdown();
  bumper_pose_sub.shutdown();
  map_changes_pub.shutdown();
  get_map_srv.shutdown();
     
  destroy_map(bumper_map);
  destroy_map(laser_map);
}
