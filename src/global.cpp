#include "XPLMUtilities.h"
#include "XPLMNavigation.h"
#include "XPLMScenery.h"
#include "XPLMGraphics.h"
#include "XPLMInstance.h"

#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <ctime>
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#include <random>
#include "cmath"

#include "Log.h"
#include "Smoker.h"
#include "dr.h"
#include "global.h"

namespace XLABEL {

  XPLMCommandRef toggle_ac_label_cmd;
  XPLMCommandRef toggle_ap_label_cmd;
  XPLMCommandRef toggle_ap_smoker_cmd;
  XPLMCommandRef toggle_units_cmd;
  bool show_ac_label = false;
  bool show_ap_label = false;
  int  label_kind = 0;
  int  units = 0; // 0=metric, 1=feet/knots/nm
    
  DRefInt dr_tcas_num_acf{"sim/cockpit2/tcas/indicators/tcas_num_acf"};
  DRefInt dr_override_TCAS{"sim/operation/override/override_TCAS"};
  DRefInt dr_acf_modeS_id{"sim/aircraft/view/acf_modeS_id"};
  DRefFloatArray dr_tcas_pos_x{"sim/cockpit2/tcas/targets/position/x"};
  DRefFloatArray dr_tcas_pos_y{"sim/cockpit2/tcas/targets/position/y"};
  DRefFloatArray dr_tcas_pos_z{"sim/cockpit2/tcas/targets/position/z"};
  DRefFloatArray dr_tcas_pos_ele{"sim/cockpit2/tcas/targets/position/ele"};
  
  DRefFloatArray dr_rel_dist_mtrs{"sim/cockpit2/tcas/indicators/relative_distance_mtrs"};
  DRefFloatArray dr_ref_alt_mtrs{"sim/cockpit2/tcas/indicators/relative_altitude_mtrs"};
  DRefFloatArray dr_vertical_speed{"sim/cockpit2/tcas/targets/position/vertical_speed"};
  DRefFloatArray dr_V_msc{"sim/cockpit2/tcas/targets/position/V_msc"};
  DRefIntArray   dr_tcas_modeS{"sim/cockpit2/tcas/targets/modeS_id"};

  DRefFloat dr_pos_x{"sim/flightmodel/position/local_x"};
  DRefFloat dr_pos_y{"sim/flightmodel/position/local_y"};
  DRefFloat dr_pos_z{"sim/flightmodel/position/local_z"};

  DRefFloat dr_plane_psi{"sim/flightmodel/position/true_psi"}; // just psi in opengl display?
  DRefFloat dr_plane_the{"sim/flightmodel/position/true_theta"};
  DRefFloat dr_plane_phi{"sim/flightmodel/position/phi"};
  
  DRefDouble dr_pos_latitude{"sim/flightmodel/position/latitude"};
  DRefDouble dr_pos_longitude{"sim/flightmodel/position/longitude"};
  
  DRefFloatArray dr_matrix_wrl{"sim/graphics/view/world_matrix"};
  DRefFloatArray dr_matrix_proj{"sim/graphics/view/projection_matrix_3d"};
  DRefInt dr_screen_width{"sim/graphics/view/window_width"};
  DRefInt dr_screen_height{"sim/graphics/view/window_height"};

  double plane_prev_lat = 0.0; // Yes, I know, real coords, but in the ocean.
  double plane_prev_lon = 0.0;

  int max_shown = 28;
  
  Smoker *pss_obj  = nullptr;
  std::vector<Smoker*> smokers;
  
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
    dr_tcas_pos_ele.get_all();
    
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

  float dist_between(const poi& lhs, const poi& rhs) {
    float dx = lhs.x - rhs.x;
    //float dy = lhs.y - rhs.y; // not used
    float dz = lhs.z - rhs.z;
    return sqrt( dx*dx + dz*dz ); // for sorting we can do w/o the sqrt...
  }

  // https://blog.mapbox.com/fast-geodesic-approximations-with-cheap-ruler-106f229ad016
  static double DEGTORAD = 3.1415926536 / 180.0;
  double dist_latlon_off(double lat0, double lon0, double lat1, double lon1) {
    double dy = 12430 * ((lat0-lat1) / 180.0);
    double dx = 24901 * ((lon0-lon1) / 360.0) * cos( ((lat0+lat1)/2.0) * DEGTORAD ); 
    double dist = sqrt( dy*dy + dx*dx );
    return dist;
  }

  double dist_latlon(double lat0, double lon0, double lat1, double lon1) {
  double slat = sin((lat1-lat0) * (double)(M_PI/360));
  double slon = sin((lon1-lon0) * (double)(M_PI/360));
  double aa   = slat*slat + cos(lat0 * (double)(M_PI/180)) * cos(lat1 * (double)(M_PI/180)) * slon * slon;
  return 6378145.0 * 2 * atan2(sqrtf(aa), sqrt(1-aa));
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
  
  // trim from start
  std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
  }
  
  // trim from end
  std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
  }
  
  // trim from both ends
  std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
  }
  
  size_t listify(const std::string& s, std::vector<std::string>& v) {
    char delim = ',';
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
      trim(item);
      v.push_back(item);
    }
    return v.size();
  }

  bool read_pois( const std::string& filename, std::vector<poi>& pois ) {

    std::ifstream file( filename.c_str() );
    if ( ! file ) {
      lg.xplm( "WARNING: can't read pois file.\n" );
      return false;
    }

    std::string a_line;
    char buffer[255];
    int poi_counter = 0;
    while( std::getline( file, a_line )) {
      if ( a_line.length() == 0 ) {
	continue;
      }
      if ( a_line.at(0) == '#' ) {
	continue;
      }
      std::vector<std::string> bits;
      listify( a_line, bits );
      // 56.5, 12.3, 0, ESTA
      if ( bits.size() == 5 ) {
	try {
	  float lat = std::stof(bits[0]);
	  float lon = std::stof(bits[1]);
	  float alt = std::stof(bits[2]);
	  int   dst = int(std::stoi(bits[3]));
	  std::string lbl = bits[4];
	  pois.push_back( poi{ lat, lon, alt, dst, lbl, 0, 0, 0, 1 } );
	  ++poi_counter;
	  //sprintf( buffer, "Added POI %i: %.4f, %.4f, %.4f, %i, %s\n", poi_counter, lat, lon, alt, dst, lbl.c_str() );
	  //lg.xplm( buffer );
	} catch (...) {
	  lg.xplm( "Error in POI file.\n" );
	}
      }
    } // while
    
    sprintf( buffer, "Read POIs %i\n", poi_counter );
    lg.xplm( buffer );
    
    return true;
    
  }

  // dist, speed, alt
  // append?
  void make_dist_str( float dist_m, char* buffer, int units ) {
    if ( units == 0 ) { // metric
      if ( dist_m > 5000.0f ) {
	sprintf( buffer, "%.1f km", dist_m/1000.0 );
      } else {
	sprintf( buffer, "%i m", int(dist_m) );
      }
    } else {
      //float dist_nm = dist_m * 0.000539957;
      float dist_ft = dist_m * 3.28084;
      if ( dist_ft > 12000.0f ) {
	sprintf( buffer, "%.1f nm", dist_ft * 0.000164579); // ft to nm
      } else {
	sprintf( buffer, "%i f", int(dist_ft) );
      }
    }
  }

  void make_spd_str( float v_msc, char* buffer, int units ) {
    sprintf( buffer, "%i kn", int(v_msc*1.943844) );
  }

  void make_alt_str( float alt_m, char* buffer, int units ) {
    if ( units == 0 ) {
      int m = int(alt_m);
      if ( m > 1000 ) {
	m = (m + 5) / 10 * 10;
      }
      sprintf( buffer, "%i m", m );
    } else {
      int ft = int(alt_m * 3.28084);
      if ( ft > 1000 ) {
	ft = (ft + 50) / 100 * 100;
      }
      sprintf( buffer, "%i ft", ft );
    }
  }
  
}
