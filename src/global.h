#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "XPLMUtilities.h"

#include "dr.h"

// using namespace XLABEL;
namespace XLABEL {

  extern XPLMCommandRef toggle_label_cmd;
  extern bool show_label;
  extern int  label_kind;
  
  extern DRefInt dr_tcas_num_acf;
  extern DRefInt dr_override_TCAS;
  extern DRefInt dr_acf_modeS_id;
  extern DRefFloatArray dr_tcas_pos_x;
  extern DRefFloatArray dr_tcas_pos_y;
  extern DRefFloatArray dr_tcas_pos_z;
  extern DRefFloatArray dr_rel_dist_mtrs;
  extern DRefFloatArray dr_ref_alt_mtrs;
  extern DRefFloatArray dr_vertical_speed;
  extern DRefFloatArray dr_V_msc;
  extern DRefIntArray   dr_tcas_modeS;

  extern DRefFloat dr_pos_x;
  extern DRefFloat dr_pos_y;
  extern DRefFloat dr_pos_z;
  extern DRefDouble dr_pos_latitude;
  extern DRefDouble dr_pos_longitude;
  
  extern DRefFloatArray dr_matrix_wrl;
  extern DRefFloatArray dr_matrix_proj;
  extern DRefInt dr_screen_width;
  extern DRefInt dr_screen_height;

  bool get_tcas_positions();
  void mult_matrix_vec(float dst[4], const float m[16], const float v[4]);

  extern float  nearest_ap_lat; // nearest airport lat, lon
  extern float  nearest_ap_lon;
  extern double nearest_ap_x; // nearest airport opengl
  extern double nearest_ap_y;
  extern double nearest_ap_z;
  extern char id[32];
  extern char name[256];
  extern XPLMProbeRef hProbe;
  extern XPLMProbeInfo_t info;

  void get_nearest_ap(double plane_lat, double plane_lon, float& latitude, float& longitude);
}
#endif
