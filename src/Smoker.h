#ifndef _OBJECT_H
#define _OBJECT_H

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

#include "XPLMScenery.h"
#include "XPLMInstance.h"

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
    Smoker();
    ~Smoker();
    
    bool load_obj(std::string&);
    void instantiate();
    void deinstantiate();
    void set_pos(float x, float y, float z);
  };

  
}

#endif
