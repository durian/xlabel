#include "XPLMUtilities.h"
#include "XPLMNavigation.h"
#include "XPLMScenery.h"
#include "XPLMGraphics.h"

#include "Log.h"
#include "dr.h"
#include "global.h"

namespace XLABEL {

  XPLMCommandRef toggle_label_cmd;
  bool show_label = false;
  int  label_kind = 0;
  
  DRefInt dr_tcas_num_acf{"sim/cockpit2/tcas/indicators/tcas_num_acf"};
  DRefInt dr_override_TCAS{"sim/operation/override/override_TCAS"};
  DRefInt dr_acf_modeS_id{"sim/aircraft/view/acf_modeS_id"};
  DRefFloatArray dr_tcas_pos_x{"sim/cockpit2/tcas/targets/position/x"};
  DRefFloatArray dr_tcas_pos_y{"sim/cockpit2/tcas/targets/position/y"};
  DRefFloatArray dr_tcas_pos_z{"sim/cockpit2/tcas/targets/position/z"};
  
  DRefFloatArray dr_rel_dist_mtrs{"sim/cockpit2/tcas/indicators/relative_distance_mtrs"};
  DRefFloatArray dr_ref_alt_mtrs{"sim/cockpit2/tcas/indicators/relative_altitude_mtrs"};
  DRefFloatArray dr_vertical_speed{"sim/cockpit2/tcas/targets/position/vertical_speed"};
  DRefFloatArray dr_V_msc{"sim/cockpit2/tcas/targets/position/V_msc"};
  DRefIntArray   dr_tcas_modeS{"sim/cockpit2/tcas/targets/modeS_id"};

  DRefFloat dr_pos_x{"sim/flightmodel/position/local_x"};
  DRefFloat dr_pos_y{"sim/flightmodel/position/local_y"};
  DRefFloat dr_pos_z{"sim/flightmodel/position/local_z"};
  DRefDouble dr_pos_latitude{"sim/flightmodel/position/latitude"};
  DRefDouble dr_pos_longitude{"sim/flightmodel/position/longitude"};
  
  DRefFloatArray dr_matrix_wrl{"sim/graphics/view/world_matrix"};
  DRefFloatArray dr_matrix_proj{"sim/graphics/view/projection_matrix_3d"};
  DRefInt dr_screen_width{"sim/graphics/view/window_width"};
  DRefInt dr_screen_height{"sim/graphics/view/window_height"};

  bool get_tcas_positions() {
#ifdef DBG
    lg.xplm( "get_tcas_positions() START\n" );
#endif
    // Number is in dr_tcas_num_acf
    if ( dr_tcas_num_acf.is_initialised() ) {
      //lg.xplm( "dr_tcas_num_acf says "+std::to_string(dr_tcas_num_acf.get_int())
      //       +" "+std::to_string(dr_tcas_num_acf.init())
      //       +"\n" );
    } else {
      lg.xplm( "Cannot read dr_tcas_num_acf\n" );
    }
    
    dr_tcas_pos_x.get_all();
    dr_tcas_pos_y.get_all();
    dr_tcas_pos_z.get_all();

    dr_V_msc.get_all();
    dr_vertical_speed.get_all();
 
    dr_tcas_modeS.get_all();
    
#ifdef DBG
    lg.xplm( "get_tcas_positions() END\n" );
#endif
    return true;
  }

  void mult_matrix_vec(float dst[4], const float m[16], const float v[4]) {
  dst[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8] + v[3] * m[12];
  dst[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9] + v[3] * m[13];
  dst[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10] + v[3] * m[14];
  dst[3] = v[0] * m[3] + v[1] * m[7] + v[2] * m[11] + v[3] * m[15];
}


  float  nearest_ap_lat; // nearest airport lat, lon
  float  nearest_ap_lon;
  double nearest_ap_x; // nearest airport opengl
  double nearest_ap_y;
  double nearest_ap_z;
  char id[32];
  char name[256];
  XPLMProbeRef hProbe = XPLMCreateProbe(xplm_ProbeY);
  XPLMProbeInfo_t info = { 0 };

  void get_nearest_ap(double plane_lat, double plane_lon, float& latitude, float& longitude) {
    // This is slow.
    float lat = static_cast<float>(plane_lat);
    float lon = static_cast<float>(plane_lon);
    
    XPLMNavRef closest_ap = XPLMFindNavAid(
					   NULL, //const char *         inNameFragment,    /* Can be NULL */
					   NULL, //const char *         inIDFragment,    /* Can be NULL */
					   (float*)&lat, 
					   (float*)&lon, //float *              inLon,    /* Can be NULL */
					   NULL, //int *                inFrequency,    /* Can be NULL */
					   xplm_Nav_Airport);
    
    XPLMGetNavAidInfo(closest_ap, NULL, &latitude, &longitude, NULL, NULL, NULL, id, name, NULL);
    //lg.xplm( "NEAREST Plane lat, lon: "+std::to_string(lat)+", "+std::to_string(lon)+"\n");
    //lg.xplm( "NEAREST AP: "+std::string(id)+", "+std::string(name)+", "+
    //	   std::to_string(latitude)+", "+std::to_string(longitude)+"\n");
    
    nearest_ap_lat = latitude;
    nearest_ap_lon = longitude;
    
    XPLMWorldToLocal( nearest_ap_lat, nearest_ap_lon, 0, &nearest_ap_x, &nearest_ap_y, &nearest_ap_z );
    
    info.structSize = sizeof(info);
    // If we have a hit then return Y coordinate
    if ( XPLMProbeTerrainXYZ( hProbe, nearest_ap_x, nearest_ap_y, nearest_ap_z, &info) == xplm_ProbeHitTerrain ) {
      //lg.xplm( "nearest_ap_y="+std::to_string(nearest_ap_y)+", info.locationY="+std::to_string(info.locationY)+"\n" );
      nearest_ap_y = info.locationY;
    }
  }

  void poi_to_local(double lat, double lon, double& x, double& y, double& z) {

    XPLMWorldToLocal( lat, lon, 0, &x, &y, &z );
    
    info.structSize = sizeof(info);
    // If we have a hit then return Y coordinate
    if ( XPLMProbeTerrainXYZ( hProbe, x, y, z, &info) == xplm_ProbeHitTerrain ) {
      //lg.xplm( "nearest_ap_y="+std::to_string(nearest_ap_y)+", info.locationY="+std::to_string(info.locationY)+"\n" );
      y = info.locationY;
    }
  }

}
