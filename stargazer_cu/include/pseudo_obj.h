#ifndef PSEUDO_OBJ_H
#define PSEUDO_OBJ_H

#define MAX_PSEUDOLITE 100

#include "tinyxml/tinyxml.h"

class Pseudolite
{
  int idarr[MAX_PSEUDOLITE];
  int arrsize;
  float xarr[MAX_PSEUDOLITE];
  float yarr[MAX_PSEUDOLITE];
  float anglearr[MAX_PSEUDOLITE];

public:

  float x;
  float y;
  float angle;
  Pseudolite(const char * pseudo_xml_file);
  bool getPseudoliteById(int id);

};

Pseudolite::Pseudolite(const char * pseudo_xml_file){
  TiXmlDocument doc( pseudo_xml_file );
  bool loadOkay = doc.LoadFile();
  if (loadOkay)
	{
		ROS_INFO("Pseudo-lite XML Load Successful\n");
	  TiXmlElement *parent = doc.RootElement();
    TiXmlElement *child = 0;
    int idx = 0;
    // Read the Pseudo-lite data from the xml file.
    while( child = (TiXmlElement *)(parent->IterateChildren( child )) ){
      const char *myid = child->Attribute("id");
      const char *myx = child->Attribute("x");
      const char *myy = child->Attribute("y");
      const char *myangle = child->Attribute("angle");
      idarr[idx] = atoi(myid);
      xarr[idx] = atof(myx);
      yarr[idx] = atof(myy);
      anglearr[idx] = atof(myangle);
      ROS_INFO("ID = %i, x = %f, y = %f, angle = %f", idarr[idx], xarr[idx], yarr[idx], anglearr[idx]);
      idx++;
    }
    arrsize = idx;
	}
	else
	{
		ROS_INFO("Pseudo-lite XML Load Failed\n");
	}
}

bool Pseudolite::getPseudoliteById(int id){
  int idx = 0;
  bool success = 0;
  for(idx =0;idx<arrsize;idx++)
    {
    if(idarr[idx] == id){
      x = xarr[idx];
      y = yarr[idx];
      angle = anglearr[idx];   
      success = 1;
    } 
  }
  return success;
}

#endif
