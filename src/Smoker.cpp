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
#include "XPLMProcessing.h"

#include "Smoker.h"
#include "Log.h"
#include "global.h"
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
    mode      = 0; // OpenGL coordinates, 1=plane coordinates
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
    lg.xplm( "Smoker "+path+"\n" );
#endif
    return true;
  }

  void Smoker::instantiate() {
    if ( ! instance ) {
      //const char * drefs[] = { NULL }; //  can this be a unique dr for each object?!
      const char * drefs[] = { "durian/xlabel/shade", NULL }; //  can this be a unique dr for each object?!
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

  void Smoker::update(float e) {
    life_time -= e;

    if ( instance ) {
      XPLMDrawInfo_t loc;
      loc.structSize = sizeof( loc );
      //static float dr_val = 0; //{0, NULL};

      float *dr_val = new float[1];
      dr_val[0] = 1.0;
      //dr_val[1] = NULL;

 
      char buffer[256];

      if ( mode == 0 ) { // doesn't need to be done continously! FIXME
	loc.x = x;
	loc.y = y;
	loc.z = z;
	loc.pitch   = 0;
	loc.heading = 0;
	loc.roll    = 0;
	sprintf( buffer, "pos %.2f %.2f %.2f \n", x, y, z );
	//lg.xplm(  buffer );
	XPLMInstanceSetPosition(instance, &loc, dr_val);
      } else if ( mode == 1 ) {
	// user's plane position/orientation
	float px = dr_pos_x.get_float(); // updating causes flicker/strange effects?
	float py = dr_pos_y.get_float();
	float pz = dr_pos_z.get_float();
	float phi = dr_plane_phi.get_float() * (M_PI/180); // roll of ac
	float psi = dr_plane_psi.get_float() * (M_PI/180); // heading
	float the = dr_plane_the.get_float() * (M_PI/180); // pitch of ac
	
	float x_wrl;
	float y_wrl;
	float z_wrl;
	
	conversion( this->x, this->y, this->z, // offset from plane 0,0,0
		    phi, psi, the, // plane orientation
		    px, py, pz,  // plane pos
		    x_wrl, y_wrl, z_wrl // result
		    );
	
	loc.x = x_wrl;
	loc.y = y_wrl;
	loc.z = z_wrl;
	loc.pitch   = the; // Do we set those?
	loc.heading = psi;
	loc.roll    = phi;
	sprintf( buffer, "pos %.2f %.2f %.2f \n", x, y, z );
	//lg.xplm(  buffer );
	XPLMInstanceSetPosition(instance, &loc, dr_val);
      }
    }
  }
  
  void Smoker::update(float e, float px, float py, float pz, float phi, float psi, float the) {
    life_time -= e;

    if ( instance ) {
      XPLMDrawInfo_t loc;
      loc.structSize = sizeof( loc );
      static float dr_val = 0;
      char buffer[256];

      if ( mode == 0 ) { // doesn't need to be done continously! FIXME
	loc.x = x;
	loc.y = y;
	loc.z = z;
	loc.pitch   = 0;
	loc.heading = 0;
	loc.roll    = 0;
	sprintf( buffer, "pos %.2f %.2f %.2f \n", x, y, z );
	//lg.xplm(  buffer );
	XPLMInstanceSetPosition(instance, &loc, &dr_val);
      } else if ( mode == 1 ) {
	
	float x_wrl;
	float y_wrl;
	float z_wrl;
	
	conversion( this->x, this->y, this->z, // offset from plane 0,0,0
		    phi, psi, the, // plane orientation
		    px, py, pz,  // plane pos
		    x_wrl, y_wrl, z_wrl // result
		    );
	
	loc.x = x_wrl;
	loc.y = y_wrl;
	loc.z = z_wrl;
	loc.pitch   = the; // Do we set those?
	loc.heading = psi;
	loc.roll    = phi;
	sprintf( buffer, "pos %.2f %.2f %.2f \n", x, y, z );
	//lg.xplm(  buffer );
	XPLMInstanceSetPosition(instance, &loc, &dr_val);
      }
    }
  }
  
  void Smoker::set_pos(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;

    update( 0 );
  }

  // Convert plane coords / opengl coords, to get a point on e.g. engine in
  // plane coords to world/opengl coords
  /*
  // Calculate position from plane coordinates to world coordinates.
  float phi = dr_plane_phi*(M_PI/180);
  float psi = dr_plane_psi*(M_PI/180);
  float the = dr_plane_the*(M_PI/180);

  float x_wrl;
  float y_wrl;
  float z_wrl;

  conversion( G.emit_x[i], G.emit_y[i], G.emit_z[i], //+lvz, 
	      phi, psi, the,
	      dr_plane_lx, dr_plane_ly, dr_plane_lz, // for +lvz we need this static outside loop?
	      x_wrl, y_wrl, z_wrl);
  */
  void conversion(float x_plane, float y_plane, float z_plane, // = source location in airplane coordinates.  
		  float phi, float psi, float the, 
		  float local_x, float local_y, float local_z,  //plane's location in the world 
		  float& x_wrl, float& y_wrl, float& z_wrl) {  // Gets the result
    // OUTPUTS:(x_wrl, y_wrl, z_wrl) = transformed location in world.
    float x_phi = x_plane*cos(phi) + y_plane*sin(phi);
    float y_phi = y_plane*cos(phi) - x_plane*sin(phi);
    float z_phi = z_plane;
    float x_the = x_phi;
    float y_the = y_phi*cos(the) - z_phi*sin(the);
    float z_the = z_phi*cos(the) + y_phi*sin(the);
    x_wrl = x_the*cos(psi) - z_the*sin(psi) + local_x;
    y_wrl = y_the                           + local_y;
    z_wrl = z_the*cos(psi) + x_the*sin(psi) + local_z;
  }


  struct smoker_deleter {
    void operator()(Smoker*& s) { 
      if (s->life_time < 0.0) {
	s->deinstantiate();
	delete s;
	s = nullptr;
      }
    }
  };
  
  float smoker_loop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon) {
    
    (void)inElapsedTimeSinceLastFlightLoop;
    (void)inCounter;
    (void)inRefcon;
    
    double elapsed = inElapsedSinceLastCall;
    
    // Delete old ones
    for_each( smokers.begin(), smokers.end(), smoker_deleter() );
    std::vector<Smoker*>::iterator new_end = remove( smokers.begin(),
						     smokers.end(),
						     static_cast<Smoker*>(NULL)
						     );
    smokers.erase( new_end, smokers.end() );

    float px = dr_pos_x.get_float(); // updating causes flicker/strange effects?
    float py = dr_pos_y.get_float();
    float pz = dr_pos_z.get_float();
    float phi = dr_plane_phi.get_float() * (M_PI/180); // roll of ac
    float psi = dr_plane_psi.get_float() * (M_PI/180); // heading
    float the = dr_plane_the.get_float() * (M_PI/180); // pitch of ac

    for ( auto si = smokers.begin(); si != smokers.end(); si++ ) {
      Smoker *s = *si;
      s->update( elapsed, px, py, pz, phi, psi, the );
    }
    
    return -1;
  } 
  
}
// The End --------
