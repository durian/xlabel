#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "XPLMUtilities.h"
#include "XPLMInstance.h"
#include "XPLMPlugin.h"

#include <vector>
#include <string>
#include <map>

#include "Smoker.h"
#include "dr.h"

// using namespace XLABEL;
namespace XLABEL {

  extern void poi_to_local(double lat, double lon, double& x, double& y, double& z);
  
  struct poi { 
    double lat;
    double lon;
    float alt;
    int   dst; // have to be closer than n meter distance
    std::string name;
    double x;
    double y;
    double z;
    int    update;
    std::string geohash;
  };

  struct MyPoi {
    static constexpr size_t DIM = 2;
    
    double lat, lon;
    double alt;            // still available if you need altitude
    int    dst;
    std::string label;
    std::string hash;
    
    MyPoi(double _lat, double _lon, double _alt,
          int _dst, std::string const& _label,
          std::string const& _hash)
      : lat(_lat), lon(_lon), alt(_alt),
        dst(_dst), label(_label), hash(_hash)
    {}
    
    double operator[](size_t i) const {
      switch (i) {
      case 0: return lat;
      case 1: return lon;
      default: throw std::out_of_range("MyPoi::operator[]");
      }
    }
  };


  extern XPLMCommandRef toggle_ac_label_cmd;
  extern XPLMCommandRef toggle_ap_label_cmd;
  extern XPLMCommandRef toggle_ap_smoker_cmd;
  extern XPLMCommandRef toggle_ua_smoke_cmd;
  extern XPLMCommandRef toggle_units_cmd;
  extern XPLMCommandRef toggle_warp_to_next_ai_cmd;
  extern XPLMCommandRef toggle_warp_to_prev_ai_cmd;
  extern XPLMCommandRef toggle_warp_to_closest_ai_cmd;
  extern XPLMCommandRef toggle_warp_forwards_cmd;
  extern bool show_ac_label;
  extern bool show_ap_label;
  extern bool show_ua_smoke;
  extern int  label_kind;
  extern int  units;
  
  extern DRefInt dr_tcas_num_acf;
  extern DRefInt dr_override_TCAS;
  extern DRefInt dr_override_flightcontrol;
  extern DRefInt dr_acf_modeS_id;
  // For the AI aircraft:
  extern DRefFloatArray dr_tcas_pos_x;
  extern DRefFloatArray dr_tcas_pos_y;
  extern DRefFloatArray dr_tcas_pos_z;
  extern DRefFloatArray dr_tcas_pos_ele;
  extern DRefFloatArray dr_tcas_vel_x;
  extern DRefFloatArray dr_tcas_vel_y;
  extern DRefFloatArray dr_tcas_vel_z;
  extern DRefFloatArray dr_tcas_pos_psi;
  extern DRefFloatArray dr_tcas_pos_phi;
  extern DRefFloatArray dr_tcas_pos_the;
  extern DRefFloatArray dr_rel_dist_mtrs;
  extern DRefFloatArray dr_ref_alt_mtrs;
  extern DRefFloatArray dr_vertical_speed;
  extern DRefFloatArray dr_V_msc;
  extern DRefIntArray   dr_tcas_modeS;

  // For the user aircraft:
  extern DRefFloat dr_pos_x;
  extern DRefFloat dr_pos_y;
  extern DRefFloat dr_pos_z;
  extern DRefFloat dr_plane_psi;
  extern DRefFloat dr_plane_the;
  extern DRefFloat dr_plane_phi;
  extern DRefFloat dr_vel_x;
  extern DRefFloat dr_vel_y;
  extern DRefFloat dr_vel_z;
  extern DRefFloatArray dr_plane_q;
  extern DRefFloatArray dr_plugin_ground_center;
  
  extern DRefDouble dr_pos_latitude;
  extern DRefDouble dr_pos_longitude;
  
  extern DRefFloatArray dr_matrix_wrl;
  extern DRefFloatArray dr_matrix_proj;
  extern DRefInt dr_screen_width;
  extern DRefInt dr_screen_height;

  extern DRefInt dr_override_forces;
  extern DRefInt dr_override_wing_forces;
  extern DRefInt dr_override_engine_forces;

  extern DRefFloat dr_fside_aero;
  extern DRefFloat dr_fnrml_aero;
  extern DRefFloat dr_faxil_aero;
  extern DRefFloat dr_L_aero;
  extern DRefFloat dr_M_aero;
  extern DRefFloat dr_N_aero;
  
  // Override with override_engines or override_engine_forces
  extern DRefFloat dr_fside_prop;
  extern DRefFloat dr_fnrml_prop;
  extern DRefFloat dr_faxil_prop;
  extern DRefFloat dr_L_prop;
  extern DRefFloat dr_M_prop;
  extern DRefFloat dr_N_prop;

  // Override with override_forces
  extern DRefFloat dr_L_total;
  extern DRefFloat dr_M_total;
  extern DRefFloat dr_N_total;

  extern DRefFloat dr_pos_P;
  extern DRefFloat dr_pos_Q;
  extern DRefFloat dr_pos_R;
  extern DRefFloat dr_pos_Prad;
  extern DRefFloat dr_pos_Qrad;
  extern DRefFloat dr_pos_Rrad;
  
  extern double plane_prev_lat;
  extern double plane_prev_lon;

  extern int max_shown;

  // for warp
  extern int ai_ac_index;
  extern int last_ai_ac_index;
  extern int warp_distance;
  
  extern std::map< std::string, std::vector<poi> > poimap;
  
  extern Smoker *pss_obj;
  extern std::vector<Smoker*> smokers;
  
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

  extern XPLMDataRef cdr_shade;
  extern XPLMDataRef cdr_warp;
  
  void get_nearest_ap(double plane_lat, double plane_lon, float& latitude, float& longitude);
  void poi_to_local(double lat, double lon, double& x, double& y, double& z);
  size_t listify(const std::string& s, std::vector<std::string>& v);
  float dist_between(const poi& lhs, const poi& rhs);
  double dist_latlon(double lat0, double lon0, double lat1, double lon1);
  bool read_pois_kdt( const std::string& filename, std::vector<MyPoi>& pois );
  bool read_pois( const std::string& filename, std::vector<poi>& pois );
  
  void make_dist_str( float dist_m, char* buffer, int units );
  void make_spd_str( float v_msc, char* buffer, int units );
  void make_alt_str( float alt_m, char* buffer, int units );

  void encode_test();
  void decode_test();
  void neighbour_test();
}
#endif
