#ifndef _SMOKER_H
#define _SMOKER_H

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

#include "XPLMScenery.h"
#include "XPLMInstance.h"
#include "XPLMProcessing.h"

// ----------------------------------------------------------------------------
// Class
// ----------------------------------------------------------------------------

namespace XLABEL {

  class Smoker {
  public:
    std::string path;
    double x;
    double y;
    double z;
    XPLMObjectRef obj;// = XPLMLoadObject(objname)
    XPLMInstanceRef instance;
    float life_time;
    int mode;
    Smoker();
    ~Smoker();
    
    bool load_obj(std::string&);
    void instantiate();
    void deinstantiate();
    void update(float);
    void update(float, float, float, float, float, float, float);
    void set_pos(float x, float y, float z);
  };

  void conversion(float x_plane, float y_plane, float z_plane, // = source location in airplane coordinates.  
		  float phi, float psi, float the, 
		  float local_x, float local_y, float local_z,  //plane's location in the world 
		  float& x_wrl, float& y_wrl, float& z_wrl);
  float smoker_loop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon);
  
}

#endif
