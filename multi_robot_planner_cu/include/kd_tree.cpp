// #include <stdio.h>
// #include <stdlib.h>
// #include <assert.h>
// #include <sys/types.h>
// #include <time.h>
// #include <math.h>
// #include <list>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <sstream>
// #include <iostream>   // file I/O
// #include <fstream>   // file I/O
// #include <vector>
// using namespace std;
// 
// #define LARGE 100000000
 #define KDSMALL 0.00001

class KD_Node
{
  public:
    
    KD_Node();                        // default constructor 
    KD_Node(const KD_Node& N);        // copy constructor
    ~KD_Node();   
    
    vector<float> V;                  // stores the data-point location of this node
    int ind;                          // stores an ind
    
    KD_Node* Parent;                  // parent
    KD_Node* Left;                    // left child
    KD_Node* Right;                   // right child
};


KD_Node::KD_Node()                              // default constructor 
{
  Parent = NULL;
  Left = NULL;
  Right = NULL;
}

KD_Node::KD_Node(const KD_Node& N)              // copy constructor
{
  V = N.V;  
  ind = N.ind;
  
  Parent = N.Parent;
  Left = N.Left;              
  Right = N.Right;
}

KD_Node::~KD_Node()                             // destructor
{     
  if(Left != NULL)
    delete Left;
  
  if(Right != NULL)
    delete Right;
}

class KD_Tree
{
  public:
    
    KD_Tree();                        // default constructor 
    KD_Tree(const KD_Tree& W);       // copy constructor
    ~KD_Tree();   
    
    void insertNode(int i, const vector<float>& datapoint);    // inserts the datapoint associated with ind into the tree 
    void PrintTree(KD_Node* ptr, int level);                   // print tree from ptr
    KD_Node* findNearest(const vector<float>& target, KD_Node* subroot, float& hsrsq, int initial_d);    // returns the nearest neighbor to the target point in the subtree starting at subroot (init to ROOT),  hsrsq is the current hyper_circle_radius squared (the distance betwen current nearest and target)^2 (init to LARGE), initial_d is the initial dimension to check on (init to 0)
    KD_Node* findNearestHelper(const vector<float>& target, KD_Node* subroot, float& hsrsq, int initial_d);    // returns the nearest neighbor to the target point in the subtree starting at subroot (init to ROOT),  hsrsq is the current hyper_circle_radius squared (the distance betwen current nearest and target)^2 (init to LARGE), initial_d is the initial dimension to check on (init to 0) helps the previous function
    KD_Node* findNearestHardWay(const vector<float>& target, KD_Node* subroot, float& hsrsq, int initial_d);    // returns the nearest neighbor to the target point in the subtree starting at subroot (init to ROOT),  hsrsq is the current hyper_circle_radius squared (the distance betwen current nearest and target)^2 (init to LARGE), initial_d is the initial dimension to check on (init to 0)
    bool TreeConsistant(KD_Node* ptr, int initial_d); // checks the subtree for consitancy, normally init prt to ROOT and initial_d to 0
    
    KD_Node* Root; 
};


KD_Tree::KD_Tree()                              // default constructor 
{
  Root = NULL;
}

KD_Tree::KD_Tree(const KD_Tree& N)              // copy constructor
{
  printf("Warning: this not implimented yet (KD_Tree copy constructor) \n");
}

KD_Tree::~KD_Tree()                             // destructor
{
  delete Root;
}

void KD_Tree::insertNode(int i, const vector<float>& datapoint)    // inserts the datapoint associated with ind i into the tree 
{
  // make new node
  KD_Node* new_node = new KD_Node;
  new_node->V = datapoint;
  new_node->ind = i;
  
  if(Root == NULL)
  {
    Root = new_node;
    Root->Parent = NULL;
    Root->Left = NULL;
    Root->Right = NULL;
    return;
  }
    
  // find where to insert
  
  int num_dims = datapoint.size();
  int plane_dim = 0; // the dimension we are splitting on
  
  KD_Node* ptr = Root;
  while(true)  // will break out when done
  {
    // see which side of node to go toward  
      
    if(datapoint[plane_dim] < ptr->V[plane_dim]) // go left
    {
       if(ptr->Left == NULL) // insert on the left of the current ptr
       {
         //printf("insert left: %f < %f\n", datapoint[plane_dim], ptr->V[plane_dim]);
         ptr->Left = new_node;
         new_node->Parent = ptr;
         break;
       }
        
       ptr = ptr->Left;
    }
    else // go right
    {
       if(ptr->Right == NULL) // insert on the right of the current ptr
       {
         //printf("insert right: %f >= %f\n", datapoint[plane_dim], ptr->V[plane_dim]);
         ptr->Right = new_node;
         new_node->Parent = ptr;
         break;    
       }
        
       ptr = ptr->Right;  
    }
      
    plane_dim++;
    if(plane_dim == num_dims)
      plane_dim = 0;   
  }  
  
//   if(!TreeConsistant(Root, 0))
//   {
//     printf("tree is not consistant \n");
//       
//     getchar();
//   }
  
  return;
}


KD_Node* KD_Tree::findNearest(const vector<float>& target, KD_Node* subroot, float& hsrsq, int initial_d)    // returns the nearest neighbor to the target point in the subtree starting at subroot (init to ROOT),  hsrsq is the current hyper_circle_radius squared (the distance betwen current nearest and target)^2 (init to LARGE), initial_d is the initial dimension to check on (init to 0)
{
  if(subroot == NULL)
  {
    return NULL;
  }
  
  KD_Node* NearestNode = NULL;
  KD_Node* ptr = subroot;
  
  // find where to "insert"
  
  int num_dims = target.size();
  int plane_dim = initial_d; // the dimension we are splitting on
  
  while(true)  // will break out when done
  {
    // see which side of node to go toward  
      
    if(target[plane_dim] < ptr->V[plane_dim]) // go left
    {        
      if(ptr->Left == NULL) // "insert" on the left of the current ptr
        break;
         
      ptr = ptr->Left;
    }
    else // go right
    {            
      if(ptr->Right == NULL) // "insert" on the right of the current ptr
        break;
      
      ptr = ptr->Right;  
    }
      
    plane_dim++;
    if(plane_dim == num_dims)
      plane_dim = 0;   
  } 
  
  // ptr is the closest nearest neighbor so far
  NearestNode = ptr;
  
  // calculate hsrsq
  hsrsq = 0;
  for(int d = 0; d < num_dims; d++)
    hsrsq += (target[d] - ptr->V[d])*(target[d] - ptr->V[d]);   
 
  // now go back through the tree and see if we can find any closer, but prune search based on hsrsq
  float hsrsq_temp = hsrsq;
  KD_Node* PotentialNewNearest = findNearestHelper(target, Root, hsrsq_temp, 0);
  
  if(PotentialNewNearest != NULL)
  {
    if(hsrsq_temp > hsrsq)
    {
      printf("hunh??? \n");   
      getchar();
    }
    
    NearestNode = PotentialNewNearest;
    hsrsq = hsrsq_temp;
  }
  
  return NearestNode;
}


KD_Node* KD_Tree::findNearestHelper(const vector<float>& target, KD_Node* subroot, float& hsrsq, int initial_d)    // returns the nearest neighbor to the target point in the subtree starting at subroot (init to ROOT),  hsrsq is the current hyper_circle_radius squared (the distance betwen current nearest and target)^2 (init to LARGE), initial_d is the initial dimension to check on (init to 0) helps the previous function
{
  if(subroot == NULL)
  {
    hsrsq = LARGE;
    return NULL;
  }
  
  KD_Node* BestFound = NULL;  
    
  // calculate dist to splitting plane
  float dist_to_plane = (target[initial_d] - subroot->V[initial_d])*(target[initial_d] - subroot->V[initial_d]);
    
  if(dist_to_plane < hsrsq + KDSMALL) // nodes on other side of plane may be better than hsrsq
  {
    // find nearest node on other side
    int temp_d = initial_d + 1; 
    if(temp_d == subroot->V.size())
      temp_d = 0;
    float temp_hsrsq = hsrsq;
            
    KD_Node* PossibleNearest = NULL;
    
    if(target[initial_d] < subroot->V[initial_d])  // target is on the left of subroot, so need to check on right of subroot
      PossibleNearest = findNearestHelper(target, subroot->Right, temp_hsrsq, temp_d);
    else // target is on the right of subroot, so need to check on left of subroot
      PossibleNearest = findNearestHelper(target, subroot->Left, temp_hsrsq, temp_d);     
     
    if(PossibleNearest != NULL)
    {
      if(temp_hsrsq > hsrsq)
      {
         printf("hmmmmmm 1\n");
         getchar();
      }
       
      // have a new best 
      BestFound = PossibleNearest;
      hsrsq = temp_hsrsq;
    } 
  }
 
  
  // regardless, nodes on this side of splitting plane may still be better
  
  // find nearest node on this side
  int temp_d = initial_d + 1; 
  if(temp_d == subroot->V.size())
    temp_d = 0;
  float temp_hsrsq = hsrsq;
            
  KD_Node* PossibleNearest = NULL;
    
  if(target[initial_d] < subroot->V[initial_d])  // target is on the left of subroot, so need to check on left of subroot
    PossibleNearest = findNearestHelper(target, subroot->Left, temp_hsrsq, temp_d);
  else // target is on the right of subroot, so need to check on right of subroot
    PossibleNearest = findNearestHelper(target, subroot->Right, temp_hsrsq, temp_d);     
     
  if(PossibleNearest != NULL)
  {
    if(temp_hsrsq > hsrsq)
    {
      printf("hmmmmmm 2\n");
      getchar();
    }
       
    // have a new best 
    BestFound = PossibleNearest;
    hsrsq = temp_hsrsq;
  } 
  
  // and subroot may still be better
  
  float this_dist = 0;
  for(int d = 0; d < subroot->V.size(); d++)
    this_dist += (target[d] - subroot->V[d])*(target[d] - subroot->V[d]);  
  
  if(this_dist < hsrsq) // subroot is better
  {
    BestFound = subroot; 
    hsrsq = this_dist; 
  }

  return BestFound;
}

KD_Node* KD_Tree::findNearestHardWay(const vector<float>& target, KD_Node* subroot, float& hsrsq, int initial_d)    // returns the nearest neighbor to the target point in the subtree starting at subroot (init to ROOT),  hsrsq is the current hyper_circle_radius squared (the distance betwen current nearest and target)^2 (init to LARGE), initial_d is the initial dimension to check on (init to 0)
{
  if(subroot == NULL)
  {
    return NULL;
  }

  
  float this_dist = 0;
  for(int d = 0; d < target.size(); d++)
    this_dist += (target[d] - subroot->V[d])*(target[d] - subroot->V[d]); 
  
  float left_dist = LARGE;
  float right_dist = LARGE;
  
  int next_d = initial_d + 1;
  if(next_d == target.size())
    next_d = 0;   
  
  KD_Node* LeftBest = findNearestHardWay(target, subroot->Left, left_dist, next_d);
  KD_Node* RightBest = findNearestHardWay(target, subroot->Right, right_dist, next_d);
  
  if(left_dist <= this_dist && left_dist <= right_dist)
  {
    hsrsq = left_dist;
    return LeftBest;
  }
  else if(right_dist < this_dist)
  {
    hsrsq = right_dist;
    return RightBest;
  }
  else
  {
    hsrsq = this_dist;
    return subroot;  
  }
}
  

void KD_Tree::PrintTree(KD_Node* ptr, int level)
{
    
  for(int i = 0; i < level; i++)
    printf("-");  
    
  if(ptr == NULL)
  {
    printf("|----- \n");
    return; 
  }
  
  for(int i = 0; i < ptr->V.size(); i++)
    printf("%f, ", ptr->V[i]);
  
  printf("    :%d\n", ptr->ind);
  
  PrintTree(ptr->Left, level+1);
  PrintTree(ptr->Right, level+1);
}


bool KD_Tree::TreeConsistant(KD_Node* ptr, int initial_d) // checks the subtree for consitancy, normally init prt to ROOT and initial_d to 0
{
  if(ptr == NULL)
    return true;
    
  if(ptr->Left != NULL)
  {
    if(ptr->Left->V[initial_d] > ptr->V[initial_d])
    {
      printf("Left:%f > Center:%f \n", ptr->Left->V[initial_d], ptr->V[initial_d]);
      return false;
    }
  } 
  
  if(ptr->Right != NULL)
  {
    if(ptr->Right->V[initial_d] < ptr->V[initial_d])
    {
      printf("Right:%f < Center:%f \n", ptr->Left->V[initial_d], ptr->V[initial_d]);
      return false;
    }
  } 
  
  
  int next_d = initial_d + 1;
  if(next_d == ptr->V.size())
    next_d = 0;
      
      
  if(!TreeConsistant(ptr->Left, next_d))
    return false;  
  
  if(!TreeConsistant(ptr->Right, next_d))
    return false;    
  
  return true;
}

// int rand_int(int left_bound, int right_bound) // returns a random integer between left_bound and right_bound, inclusive
// {
//   if(  left_bound == right_bound)
//     return left_bound;
//   return (rand() % (right_bound - left_bound +1)) + left_bound;  
// }
// 
// int main(int argc, char *argv[])
// {
//   time_t seed;
//   time(&seed);
//   srand(seed);
//     
//   int d = 2;
//   int n = 100;
//   
//   KD_Tree T;
//   
//   for(int i = 0; i < n; i++)
//   {
//     vector<float> new_vec(d);
//   
//     for(int j = 0; j < d; j++)
//       new_vec[j] = (float)rand_int(0, 100);
//       
//     T.insertNode(i, new_vec);
//   }
//   
//   
//   printf("\n");
//   
//   T.PrintTree(T.Root, 0);
//   
//   printf("\n");
//   
//   vector<float> target_vec(d,61);
//   
//   float dist = LARGE;
//   KD_Node* nn = T.findNearest(target_vec, T.Root, dist, 0);
//   
//   printf("nearest neighbor ind: %d at dist %f\n", nn->ind, sqrt(dist));
//   
//   return 0;   
// }
