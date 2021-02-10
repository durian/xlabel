#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "XPLMUtilities.h"

#include <vector>
#include <string>

#include "dr.h"

// using namespace XLABEL;
namespace XLABEL {

  extern XPLMCommandRef toggle_ac_label_cmd;
  extern XPLMCommandRef toggle_ap_label_cmd;
  extern XPLMCommandRef toggle_units_cmd;
  extern bool show_ac_label;
  extern bool show_ap_label;
  extern int  label_kind;
  extern int  units;
  
  extern DRefInt dr_tcas_num_acf;
  extern DRefInt dr_override_TCAS;
  extern DRefInt dr_acf_modeS_id;
  extern DRefFloatArray dr_tcas_pos_x;
  extern DRefFloatArray dr_tcas_pos_y;
  extern DRefFloatArray dr_tcas_pos_z;
  extern DRefFloatArray dr_tcas_pos_ele;
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
  void poi_to_local(double lat, double lon, double& x, double& y, double& z);
  size_t listify(const std::string& s, std::vector<std::string>& v);
  struct poi { 
    float lat;
    float lon;
    float alt;
    int   dst; // have to be closer than n meter distance
    std::string name;
    double x;
    double y;
    double z;
    int    update;
  };
  float dist_between(const poi& lhs, const poi& rhs);
  bool read_pois( const std::string& filename, std::vector<poi>& pois );

  void make_dist_str( float dist_m, char* buffer, int units );
  void make_spd_str( float v_msc, char* buffer, int units );
  void make_alt_str( float alt_m, char* buffer, int units );
}
#endif
