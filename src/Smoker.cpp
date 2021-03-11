#define NOMINMAX
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <ctime>
#include <vector>
#include <cmath> //fabs
#include <algorithm>
#include <iterator>

#include <stdlib.h>

#include "XPLMUtilities.h"
#include "XPLMScenery.h"
#include "XPLMInstance.h"

#include "Smoker.h"
#include "Log.h"
#include "defs.h"

namespace XLABEL {

// ----------------------------------------------------------------------------
// Code
// ----------------------------------------------------------------------------

  Smoker::Smoker() {
#ifdef DBG
    lg.xplm("Smoker() init START.\n");
#endif
    path = "";
    x         = 0.0;
    y         = 0.0;
    z         = 0.0;
    obj       = nullptr;
    instance  = nullptr;
    life_time = 90.0; // seconds
#ifdef DBG
    lg.xplm("Smoker() init END.\n");
#endif
  }

  Smoker::~Smoker() {
    if ( instance != nullptr ) {
      XPLMDestroyInstance( instance );
#ifdef DBG
      lg.xplm("Instance centre destroyed\n");
#endif
    }
    if ( obj != nullptr ) {
      XPLMUnloadObject(obj);
#ifdef DBG
      lg.xplm("Smoker ("+path+") destroyed\n");
#endif
      obj = nullptr;
    }
  }
  
  bool Smoker::load_obj(std::string& p) { 
#ifdef DBG
    lg.xplm("Smoker::load_obj START.\n");
#endif
    obj = XPLMLoadObject(p.c_str());
    if ( obj == nullptr ) {
      lg.xplm( "ERROR LOADING OBJECT "+path+"\n" );
      return false;
    }
    path = p;
#ifdef DBG
    lg.xplm( "OBJECT "+path+"\n" );
#endif
    return true;
  }

  void Smoker::instantiate() {
    if ( ! instance ) {
      const char * drefs[] = { "sim/time/hobbs_time", NULL }; //  can this be a unique dr for each object?!
      instance = XPLMCreateInstance(this->obj, drefs);
      lg.xplm( "Created instance\n" );
    }
  }
  
  void Smoker::deinstantiate() {
    if ( instance ) {
      XPLMDestroyInstance( instance );
      instance = nullptr;
    }
  }

  void Smoker::set_pos(float x, float y, float z) {
    if ( instance ) {
      XPLMDrawInfo_t loc;
      loc.structSize = sizeof( loc );
      loc.x = static_cast<float>(x);
      loc.y = static_cast<float>(y);
      loc.z = static_cast<float>(z);
      loc.pitch   = 0;
      loc.heading = 0;
      loc.roll    = 0;
      static float dr_val = 0;
      char buffer[256];
      sprintf( buffer, "pos %.2f %.2f %.2f \n", x, y, z );
      //lg.xplm(  buffer );
      XPLMInstanceSetPosition(instance, &loc, &dr_val);
      this->x = x;
      this->y = y;
      this->z = z;
    }
  }
  
}
// The End --------
