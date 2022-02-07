#pragma once

#include <stdio.h>
#include <string>
#include <vector>

#include "XPLMDataAccess.h"

namespace XLABEL {
  
  class DRef { 
  public:
    
    XPLMDataRef dref;
    bool        initialised;
    bool        writeable;
    std::string path;
    int         cur_size =  0; // current
    int         max_size = 64; // max memory we reserve
    
    DRef( const std::string &a_path );
    ~DRef();
    virtual bool init();
    int get_max_size() { return max_size; }
    int get_cur_size() { return cur_size; }
    bool is_initialised() { return initialised; }
    
    virtual int   get_int()         { return 0; } // or get_value() ?
    virtual int   get_all()         { return 0; } 
    virtual bool  set_int(int)      { return false; }
    virtual float get_float()       { return 0.0; }
    virtual bool  set_float(float)  { return false; }
    virtual int   get_int(int)      { return 0; } // or get_value() ?
    virtual bool  set_int(int, int) { return 0; } // or get_value() ?
  };

  // --------------------------------------------------------------------------
  // DRefInt
  // --------------------------------------------------------------------------

  class DRefInt : public DRef {
  public:
    DRefInt( const std::string &a_path );

    int  get_int();
    bool set_int(int);
  };

  // --------------------------------------------------------------------------
  // DRefFloat
  // --------------------------------------------------------------------------

  class DRefFloat : public DRef {
  public:
    DRefFloat( const std::string &a_path );
    
    float get_float();
    bool  set_float(float);
  };

  // --------------------------------------------------------------------------
  // DRefDouble
  // --------------------------------------------------------------------------

  class DRefDouble : public DRef {
  public:
    DRefDouble( const std::string &a_path );
    
    double get_double();
    bool   set_double(double);
  };

  // --------------------------------------------------------------------------
  // DRefIntArray
  // --------------------------------------------------------------------------

  class DRefIntArray : public DRef {
  public:
    int* memory; 
    DRefIntArray( const std::string &a_path );
    ~DRefIntArray();
    bool init();
    int  get_memory(int); // use after get_all()
    
    int  get_int(int);
    int  get_all();
    bool set_int(int, int);
  };

  // --------------------------------------------------------------------------
  // DRefFloatArray
  // --------------------------------------------------------------------------

  class DRefFloatArray : public DRef {
  public:
    float* memory; 
    DRefFloatArray( const std::string &a_path );
    bool init();
    float get_memory(int); // use after get_all()
    void* get_memory_adr(); // use after get_all()
    int get_all_ref(float*, int);
	
    float get_float(int);
    int   get_all();
    bool  set_float(float, int);
  };

  // --------------------------------------------------------------------------
  // Higher level access
  // --------------------------------------------------------------------------
  /*  
  //static DRefInt dr_tcas_num_acf{"sim/cockpit2/tcas/indicators/tcas_num_acf"}; 
  static DRefInt dr_override_TCAS{"sim/operation/override/override_TCAS"};
  static DRefInt dr_acf_modeS_id{"sim/aircraft/view/acf_modeS_id"};
  static DRefFloatArray dr_tcas_pos_x{"sim/cockpit2/tcas/targets/position/x"};
  static DRefFloatArray dr_tcas_pos_y{"sim/cockpit2/tcas/targets/position/y"};
  static DRefFloatArray dr_tcas_pos_z{"sim/cockpit2/tcas/targets/position/z"};
  static DRefFloatArray dr_tcas_pos_vx{"sim/cockpit2/tcas/targets/position/vx"};
  static DRefFloatArray dr_tcas_pos_vy{"sim/cockpit2/tcas/targets/position/vy"};
  static DRefFloatArray dr_tcas_pos_vz{"sim/cockpit2/tcas/targets/position/vz"};
  static DRefFloatArray dr_tcas_pos_lat{"sim/cockpit2/tcas/targets/position/lat"};
  static DRefFloatArray dr_tcas_pos_lon{"sim/cockpit2/tcas/targets/position/lon"};
  static DRefFloatArray dr_tcas_pos_ele{"sim/cockpit2/tcas/targets/position/ele"};
  static DRefFloatArray dr_tcas_pos_psi{"sim/cockpit2/tcas/targets/position/psi"};
  static DRefFloatArray dr_tcas_pos_phi{"sim/cockpit2/tcas/targets/position/phi"};
  static DRefIntArray   dr_tcas_modeS{"sim/cockpit2/tcas/targets/modeS_id"};
  bool get_tcas_positions();
  */
}

