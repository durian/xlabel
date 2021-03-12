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
    void set_pos(float x, float y, float z);
  };
  
  float smoker_loop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon);
  
}

#endif
