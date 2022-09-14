//
// Make a "mark here" function while flying?
//
#include "XPLMMap.h"
#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
#include "XPLMNavigation.h"
#include "XPLMScenery.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMInstance.h"
#include <string.h>
#include <stdio.h>

#if APL
#include <OpenGL/OpenGL.h>
#include <OpenGL/glu.h>
#elif IBM
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
//#include "gl/glew.h"
#include <GL/gl.h>
#include <GL/glu.h>
#elif LIN
//#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <math.h>
#include <algorithm>

// Mine
#include "defs.h"
#include "dr.h"
#include "Log.h"
#include "Smoker.h"
#include "global.h"
#include "geohash.h"

#include "toml.h"

using namespace XLABEL;

/*
DRefFloatArray dr_tcas_pos_vx{"sim/cockpit2/tcas/targets/position/vx"};
DRefFloatArray dr_tcas_pos_vy{"sim/cockpit2/tcas/targets/position/vy"};
DRefFloatArray dr_tcas_pos_vz{"sim/cockpit2/tcas/targets/position/vz"};
DRefFloatArray dr_tcas_pos_lat{"sim/cockpit2/tcas/targets/position/lat"};
DRefFloatArray dr_tcas_pos_lon{"sim/cockpit2/tcas/targets/position/lon"};
DRefFloatArray dr_tcas_pos_ele{"sim/cockpit2/tcas/targets/position/ele"};
DRefFloatArray dr_tcas_pos_psi{"sim/cockpit2/tcas/targets/position/psi"};
DRefFloatArray dr_tcas_pos_phi{"sim/cockpit2/tcas/targets/position/phi"};

sim/flightmodel/ground/plugin_ground_center	float[3]	y	meters	Location of a pt on the ground in local coords
*/

std::vector<poi> pois;
float flight_loop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon);

float height(double x, double y, double z) {
  XPLMProbeInfo_t info = { 0 };
  info.structSize = sizeof(info);  
  // If we have a hit then return Y coordinate
  if (XPLMProbeTerrainXYZ( hProbe, x, y, z, &info) == xplm_ProbeHitTerrain) {
    return info.locationY;
  }
  return -1.0;
}

// For warp (special DEG_TO_RAD!)
#define DEG_TO_RAD_2 M_PI / 360.0
typedef struct _Eulers {
  double psi;
  double the;
  double phi;
} Eulers;
typedef struct _Quaternion {
  double w;
  double x;
  double y;
  double z;
} Quaternion;
void EulersToQuaternion(Eulers ypr, Quaternion &q) {
    double spsi = sin(ypr.psi * DEG_TO_RAD_2);
    double sthe = sin(ypr.the * DEG_TO_RAD_2);
    double sphi = sin(ypr.phi * DEG_TO_RAD_2);
    double cpsi = cos(ypr.psi * DEG_TO_RAD_2);
    double cthe = cos(ypr.the * DEG_TO_RAD_2);
    double cphi = cos(ypr.phi * DEG_TO_RAD_2);

    double qw = cphi * cthe * cpsi + sphi * sthe * spsi;
    double qx = sphi * cthe * cpsi - cphi * sthe * spsi;
    double qy = sphi * cthe * spsi + cphi * sthe * cpsi;
    double qz = cphi * cthe * spsi - sphi * sthe * cpsi;

    double e = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

    q.w = qw / e;
    q.x = qx / e;
    q.y = qy / e;
    q.z = qz / e;
}
#define RAD_TO_DEG 180.0 / M_PI
double sgn(double a) {
    if (a < 0.0) return -1.0;
    else return 1.0;
}
void QuaternionToEulers(Quaternion q, Eulers &ypr) {
    double sx = q.x * q.x;
    double sy = q.y * q.y;
    double sz = q.z * q.z;
    double yz = q.y * q.z;
    double wx = q.w * q.x;

    double m00 = 1.0 - 2.0 * (sy + sz);
    double m10 = 2.0 * (q.x * q.y + q.w * q.z);
    double m11 = 1.0 - 2.0 * (sx + sz);
    double m12 = 2.0 * (yz - wx);
    double m20 = 2.0 * (q.x * q.z - q.w * q.y);
    double m21 = 2.0 * (wx + yz);
    double m22 = 1.0 - 2.0 * (sx + sy);

    double s = -m20;
    double c = sqrt(m00 * m00 + m10 * m10);

    ypr.the = atan2(s, c) * RAD_TO_DEG;

    if (c > 0.001) {
      ypr.phi = atan2(m21, m22) * RAD_TO_DEG;
      ypr.psi = atan2(m10, m00) * RAD_TO_DEG;
    } else {
      ypr.phi = 0.0;
      ypr.psi = -sgn(s) * atan2(-m12, m11) * RAD_TO_DEG;
    }
}
// end for warp


// ----

static int DrawCallback1(XPLMDrawingPhase inPhase, int inIsBefore, void * inRefcon) {

  if ( ! show_ac_label ) {
    return 1;
  }
  
  float mv[16], proj[16];
  // Read the model view and projection matrices from this frame
  dr_matrix_wrl.get_all_ref( &mv[0], 16 );
  dr_matrix_proj.get_all_ref( &proj[0], 16 );

  static int printed = 0;
  if ( ! printed ) {
    char buffer[256];
    for( int i = 0; i < 4; i++ ) {
      sprintf( buffer, "m_wrl %.2f %.2f %.2f %.2f\n", mv[0+i*4], mv[1+i*4], mv[2+i*4], mv[3+i*4] ); 
      lg.xplm( std::string(buffer) );
    }
    for( int i = 0; i < 4; i++ ) {
      sprintf( buffer, "proj  %.2f %.2f %.2f %.2f\n", proj[0+i*4], proj[1+i*4], proj[2+i*4], proj[3+i*4] ); 
      lg.xplm( std::string(buffer) );
    }
    /*
    sprintf( buffer, "r %.2f %.2f %.2f\n", r[0], r[1], r[2] ); 
    lg.xplm( std::string(buffer) );
    sprintf( buffer, "u %.2f %.2f %.2f\n", u[0], u[1], u[2] ); 
    lg.xplm( std::string(buffer) );
    */
    printed = 1;
  }

  float acf_eye[4], acf_ndc[4];

  float px = dr_pos_x.get_float();
  float py = dr_pos_y.get_float();
  float pz = dr_pos_z.get_float();

  float acf_wrl[4] = {	
    px,
    py,
    pz,
    1.0f };

  int ai = dr_tcas_num_acf.get_int();
  bool res = get_tcas_positions();
  char buffer[256];
  char dist_buf[32];
  char alt_buf[32];
  char spd_buf[32];

  for ( auto i = 1; i < ai; i++ ) { // skip 0, it is user's plane, only when at a distance?
    float lx  = static_cast<float>(dr_tcas_pos_x.get_memory(i)); // or were these doubles
    float lz  = static_cast<float>(dr_tcas_pos_z.get_memory(i));
    float ly  = static_cast<float>(dr_tcas_pos_y.get_memory(i));
    float ele = static_cast<float>(dr_tcas_pos_ele.get_memory(i));
    
    acf_wrl[0] = lx;
    acf_wrl[1] = ly;
    acf_wrl[2] = lz;
    acf_wrl[3] = 1.0f;

    // Simulate the OpenGL transformation to get screen coordinates.
    mult_matrix_vec(acf_eye, mv, acf_wrl);
    mult_matrix_vec(acf_ndc, proj, acf_eye);

    if ( acf_ndc[3] >= 0 ) {  // < 0 means behind me
      acf_ndc[3] = 1.0f / acf_ndc[3];
      acf_ndc[0] *= acf_ndc[3];
      acf_ndc[1] *= acf_ndc[3];
      acf_ndc[2] *= acf_ndc[3];
      
      float screen_w = (float)dr_screen_width.get_int();
      float screen_h = (float)dr_screen_height.get_int();
      
      float final_x = screen_w * (acf_ndc[0] * 0.5f + 0.5f);
      float final_y = screen_h * (acf_ndc[1] * 0.5f + 0.5f);
      
      float dx = lx - px;
      float dz = lz - pz;
      float dy = ly - py; // positive means I am lower

      float dist = sqrt( dx*dx + dz*dz ); // there is a tcas / relative_distance_mtrs dataref

      if ( dist < 100 * 1000 ) { // only < 100 kms

	char v_spd = '-';
	if ( dr_vertical_speed.get_memory(i) > 0.5 ) {
	  v_spd = '^';
	} else if ( dr_vertical_speed.get_memory(i) < -0.5 ) {
	  v_spd = 'v';
	}
	
	float v_msc = dr_V_msc.get_memory(i); // 1.943844
	
	//sim/cockpit2/tcas/indicators/relative_distance_mtrs     float[64]       y       meters  Distance to each other
	//sim/cockpit2/tcas/indicators/relative_altitude_mtrs     float[64]       y       meters  Relative altitude
	//sim/cockpit2/tcas/targets/position/vertical_speed       float[64]       y       feet/min
	//sim/cockpit2/tcas/targets/position/V_msc                float[64]       n       meter/s total true speed
	//sim/cockpit2/tcas/targets/position/ele                  float[64]	n	  meter	global coordinate
	
	float colWHT[] = { 1.0, 1.0, 1.0 };
	make_dist_str( dist, dist_buf, units );
	make_spd_str( v_msc, spd_buf, units );
	make_alt_str( ele, alt_buf, units );
	sprintf( buffer, "%i | %s | %s | %s | %c", i, dist_buf, spd_buf, alt_buf, v_spd );
	//lg.xplm( std::string(buffer)+"\n" );
	int len = strlen(buffer);
	
	float box_y = final_y + 12 + 10; // We put box above
	float box_x = final_x + 12; // We put box to right
	XPLMDrawTranslucentDarkBox(box_x - 5, box_y + 10, box_x + 6*len + 5, box_y - 8); // slow?
	XPLMDrawString(colWHT, box_x, box_y-1, buffer, NULL, xplmFont_Basic);
	
	// draw the label higer, and a small green line to the final_x and final_y points?
	
	XPLMSetGraphicsState(
			     0 /* no fog */,
			     0 /* 0 texture units */,
			     0 /* no lighting */,
			     0 /* no alpha testing */,
			     1 /* do alpha blend */,
			     1 /* do depth testing */,
			     0 /* no depth writing */
			     );
	glColor3f(0, 1, 0); // green
	static float half_width  = 10;
	static float half_height = 10;
	glBegin(GL_LINE_LOOP);
	{
	  // final_x - 5, final_y + 10, final_x + 6*len + 5, final_y - 8
	  glVertex2f(box_x - 5,         box_y + 10);
	  glVertex2f(box_x + 6*len + 5, box_y + 10);
	  glVertex2f(box_x + 6*len + 5, box_y - 8);
	  glVertex2f(box_x - 5,         box_y -8);
	}
	glEnd();
	// line to object from box
	glBegin(GL_LINE_LOOP);
	{
	  glVertex2f(box_x-5, box_y-8);
	  glVertex2f(final_x, final_y);
	}
	glEnd();
	
      } // dist
    } 
  }
  
  return 1;
}

// Should be deregistered also
int counter = 0;

/*
poi pois[] = {
  poi{ 56.292109, 12.854471, 0, 1000, "ESTA",  0, 0, 0, 0 },
  poi{ 56.053821, 13.530892, 0, 1000, "RH118", 0, 0, 0, 0 },
  poi{ 56.338259, 12.895436, 0, 1000, "KV8",   0, 0, 0, 0 },
  poi{ 56.069841, 13.402676, 0, 1000, "HRD",   0, 0, 0, 0 }, 
};
*/

static int DrawCallback2(XPLMDrawingPhase inPhase, int inIsBefore, void * inRefcon) {
  // Read the ACF's OpengL coordinates
  
  if ( ! show_ap_label ) {
    return 1;
  }
  if ( pois.size() == 0 ) {
    return 1;
  }
  
  float mv[16], proj[16];
  // Read the model view and projection matrices from this frame
  dr_matrix_wrl.get_all_ref( &mv[0], 16 );
  dr_matrix_proj.get_all_ref( &proj[0], 16 );
  float acf_eye[4], acf_ndc[4];

  // user's plane position
  float px = dr_pos_x.get_float();
  float py = dr_pos_y.get_float();
  float pz = dr_pos_z.get_float();
  double uplat = dr_pos_latitude.get_double(); 
  double uplon = dr_pos_longitude.get_double();
  
  double poi_x;
  double poi_y;
  double poi_z;

  char dist_buf[32];
  char alt_buf[32];
  char spd_buf[32];

  // we don't need to sort every frame, once a second is enough...

  /*
  poi p{uplat, uplon, 0, 0, "", px, py, pz }; // hmmm
  std::sort( begin(pois),
	     end(pois),
	     //[p](const poi& lhs, const poi& rhs) { return dist_between(p, lhs) < dist_between(p, rhs); }
	     [p](const poi& lhs, const poi& rhs) {
	       return dist_latlon(p.lat, p.lon, lhs.lat, lhs.lon) < dist_latlon(p.lat, p.lon, rhs.lat, rhs.lon);
	     }
	     );
  
  int c = 10;
  lg.xplm( "---\n" );
  for ( auto& poi : pois) {
    lg.xplm( "Closest to: "+poi.name+", "+std::to_string(poi.dst)+"\n" );
    if ( --c <= 0 ) {
      break;
    }
  }
  */
  
  //poi p0 = pois[0]; // ptr instead
  //float p0dist = dist_between(p, p0);
  //lg.xplm( "Closest to: "+p0.name+", "+std::to_string(p0dist)+"\n" );
    
  // When sorted, we can stop after first one which is too far away?

  // Problem is that due to the nature of the WorldToLocal function, we get fields
  // that are really far away close to the plane.
  // We need to use lat/lon for this.

  auto shown_counter = max_shown;
  /*
  poi p{uplat, uplon, 0, 0, "", px, py, pz }; 
  std::partial_sort( pois.begin(), pois.begin() + max_shown, pois.end(),
		     [p](const poi& lhs, const poi& rhs) {
		       return dist_latlon(p.lat, p.lon, lhs.lat, lhs.lon) < dist_latlon(p.lat, p.lon, rhs.lat, rhs.lon);
		     }
		     );
  */

  float screen_w = (float)dr_screen_width.get_int();
  float screen_h = (float)dr_screen_height.get_int();
  float half_width  = 10;
  float half_height = 10;

  for ( auto& poi : pois) { // break when max_shown are displayed
    
    double plat = poi.lat; //dr_pos_latitude.get_double();
    double plon = poi.lon; //dr_pos_longitude.get_double();
    float latitude;
    float longitude;
    float alt = poi.alt;
    int max_dist = poi.dst;
    if ( poi.update == 1 ) { // For init, and made 1 on scenery reload to trigger recalculation. Could be done there
      poi_to_local(plat, plon, poi_x, poi_y, poi_z); // only needed on scenery switch?!
      poi.x = poi_x;
      poi.y = poi_y;
      poi_y += alt;
      poi.z = poi_z;
      poi.update = 0;
    } else {
      poi_x = poi.x;
      poi_y = poi.y;
      poi_z = poi.z;
    }
    
    float dx = poi_x - px;
    float dz = poi_z - pz;
    float dy = poi_y - py; 
    //float dist = sqrt( dx*dx + dz*dz ); // should be lat/lon
    double latlon_dist = dist_latlon(uplat, uplon, plat, plon); // but not exact.... hmmm
    //lg.xplm( "POI: "+poi.name+", "+std::to_string(latlon_dist)+"\n" );
    
    // doesn't work because the first time pois are not initialised so sort doesn't work.
    if ( latlon_dist >= max_dist ) {  // they can have different distances, so a short one disables a longer one after it. maybe sort on diff between distance and viewdistance?
      //break;//continue; // skip if too far away // break; // we are sorted
      continue;
    }

    float dist = sqrt( dx*dx + dz*dz ); // more exact if locally close
    
    float acf_wrl[4] = {	
      (float)poi_x,
      (float)poi_y,
      (float)poi_z,
      1.0f };
    
    char buffer[256];
    
    // Simulate the OpenGL transformation to get screen coordinates.
    mult_matrix_vec(acf_eye, mv, acf_wrl);
    mult_matrix_vec(acf_ndc, proj, acf_eye);
    
    if ( acf_ndc[3] >= 0 ) {  // < 0 means behind me
      acf_ndc[3] = 1.0f / acf_ndc[3];
      acf_ndc[0] *= acf_ndc[3];
      acf_ndc[1] *= acf_ndc[3];
      acf_ndc[2] *= acf_ndc[3];
            
      float final_x = screen_w * (acf_ndc[0] * 0.5f + 0.5f);
      float final_y = screen_h * (acf_ndc[1] * 0.5f + 0.5f);
      int indent = shown_counter % 3;
      if ( dist > 5000 ) {
	//final_y = screen_h - 34 - (24 * indent); // TEST, puts them on top line. Disabled.
      }
      
      float colWHT[] = { 1.0, 1.0, 1.0 };
      make_dist_str( dist, dist_buf, units );
      sprintf( buffer, "%s | %s", poi.name.c_str(), dist_buf ); // POI | 10 nm 

      int len = strlen(buffer);
      
      float box_y = final_y + 12 + 10; // We put box above
      float box_x = final_x + 12; // We put box to right
      XPLMDrawTranslucentDarkBox(box_x - 5, box_y + 10, box_x + 6*len + 5, box_y - 8);
      XPLMDrawString(colWHT, box_x, box_y-1, buffer, NULL, xplmFont_Basic);

      if ( true ) {
	XPLMSetGraphicsState(
			     0 /* no fog */,
			     0 /* 0 texture units */,
			     0 /* no lighting */,
			     0 /* no alpha testing */,
			     1 /* do alpha blend */,
			     1 /* do depth testing */,
			     0 /* no depth writing */
			     );
	
	glColor3f(0, 0, 1); // blue

	glBegin(GL_LINE_LOOP);
	{
	  // final_x - 5, final_y + 10, final_x + 6*len + 5, final_y - 8
	  glVertex2f(box_x - 5,         box_y + 10);
	  glVertex2f(box_x + 6*len + 5, box_y + 10);
	  glVertex2f(box_x + 6*len + 5, box_y - 8);
	  glVertex2f(box_x - 5,         box_y -8);
	}
	glEnd();
	// line to object from box
	glBegin(GL_LINE_LOOP);
	{
	  glVertex2f(box_x-5, box_y-8);
	  glVertex2f(final_x, final_y);
	}
	glEnd();
      } // true
      
    } // acf_ndc[3] > 0
    
    if ( --shown_counter <= 0 ) {
      break;
    }
    
  } // for
  
  return 1;
}

static void warp_forwards() {

  // sim/flightmodel/ground/plugin_ground_center? aray, float, 3 ?
  // get agl or so
  float ground_y = dr_plugin_ground_center.get_float(1);
  float user_y   = dr_pos_y.get_float();
  float diff_y   = user_y - ground_y;
  
  float user_x  = dr_pos_x.get_float();
  float user_z  = dr_pos_z.get_float();

  float h = height(user_x, user_y, user_z);
  float reference_h = user_y - h;
  //  float h = height(dr_plane_lx, dr_plane_ly, dr_plane_lz); // on ground
  //  reference_h = dr_plane_ly - h;
  //      dr_plane_ly = h + reference_h; // from x-slew
  
  float user_vx = dr_vel_x.get_float();
  float user_vz = dr_vel_z.get_float();

  float vel = sqrt( (user_vx*user_vx) + (user_vz*user_vz) );
  float dir = dr_plane_psi.get_float();

  double s =  sin(dir * (double)(M_PI/180));
  double c = -cos(dir * (double)(M_PI/180));

  float delta_x = s * 1000.0; // 1000m
  float delta_z = c * 1000.0;

  dr_pos_x.set_float( user_x + delta_x );
  dr_pos_z.set_float( user_z + delta_z );

  // adjust agl
  //ground_y = dr_plugin_ground_center.get_float(1); // diff from first measurement?
  //dr_pos_y.set_float( ground_y + diff_y );
  user_x = dr_pos_x.get_float();
  user_z = dr_pos_z.get_float();
  user_y = dr_pos_y.get_float();
  h = height(user_x, user_y, user_z);
  dr_pos_y.set_float( h + reference_h );
}

static void warp_to_closest_ai() {

  int ai = dr_tcas_num_acf.get_int();
#ifdef DBG
  lg.xplm( "warp_to_next_ai():get_tcas_positions() = " + std::to_string(ai) + "\n" );
#endif
  if ( ai < 2 ) {
    return;
  }

  // take closest? Should prolly keep a sorted list? Sort?
  // Or just step through the list!
  int closest_idx   = 1;
  float max_dist    = 1000000;
  float user_x      = dr_pos_x.get_float();
  float user_z      = dr_pos_z.get_float();
  bool  res         = get_tcas_positions();
  for( auto i = 1; i < ai; ++i) {
    float lx   = static_cast<float>(dr_tcas_pos_x.get_memory(i)); 
    float lz   = static_cast<float>(dr_tcas_pos_z.get_memory(i));
    float dist = sqrt( ((user_x-lx)*(user_x-lx)) + ((user_z-lz)*(user_z-lz)) );
    //if ( dist > 500 ) { // if too close, take another
      if ( dist < max_dist ) {
	max_dist = dist;
	closest_idx = i;
      }
      //}
  }

  int i = closest_idx;
  float lx1  = static_cast<float>(dr_tcas_pos_x.get_memory(i)); // or were these doubles
  float lz1  = static_cast<float>(dr_tcas_pos_z.get_memory(i));
  float ly1  = static_cast<float>(dr_tcas_pos_y.get_memory(i));
  float ele1 = static_cast<float>(dr_tcas_pos_ele.get_memory(i));
  
  float vx1  = static_cast<float>(dr_tcas_vel_x.get_memory(i)); // or were these doubles
  float vz1  = static_cast<float>(dr_tcas_vel_z.get_memory(i));
  float vy1  = static_cast<float>(dr_tcas_vel_y.get_memory(i));

#ifdef DBG
  lg.xplm( "warp_to_next_ai():ai x/y/z vel = " + std::to_string(vx1) + ", "
	   + std::to_string(vy1) + ", "
	   + std::to_string(vz1) + ", "
	   + "\n" );
#endif

  float phi1 = static_cast<float>(dr_tcas_pos_phi.get_memory(i)); // or were these doubles
  float psi1 = static_cast<float>(dr_tcas_pos_psi.get_memory(i)); // heading
  float the1 = static_cast<float>(dr_tcas_pos_the.get_memory(i));

  double angle_offset = fmod((psi1 + 180.0), 360);
  double s =  sin(angle_offset * (double)(M_PI/180));
  double c = -cos(angle_offset * (double)(M_PI/180));
  
  double delta_x = warp_distance * s;
  double delta_z = warp_distance * c;
  lx1 += delta_x;
  lz1 += delta_z;

  dr_override_flightcontrol.set_int( 1 );
  
  dr_pos_x.set_float( lx1 );
  dr_pos_z.set_float( lz1 );
  //
  dr_pos_y.set_float( ly1 );

  // velocity (not doing this causes problems... why?)
  // because we need to put the vx/vz right, reclculate it for the new headingm aybe
  angle_offset = psi1; // just swap s and c instead?
  s =  sin(angle_offset * (double)(M_PI/180));
  c = -cos(angle_offset * (double)(M_PI/180));
  float user_vel = sqrt( (dr_vel_x.get_float() * dr_vel_x.get_float()) + (dr_vel_z.get_float() * dr_vel_z.get_float()) );

  // We can also match velocity of the target AI (vx1/vz1)
  //dr_vel_x.set_float( vx1 );
  dr_vel_x.set_float( user_vel * s );
  //dr_vel_z.set_float( vz1 );
  dr_vel_z.set_float( user_vel * c );
  dr_vel_y.set_float( vy1 ); 
  
  // convert AI eulers to Q
  Eulers ypr = {0, 0, 0};
  Quaternion q;
  ypr.psi = psi1;
  ypr.the = the1;
  ypr.phi = phi1;
  EulersToQuaternion(ypr, q); // convert back

  dr_plane_q.set_float( static_cast<float>(q.w), 0 );
  dr_plane_q.set_float( static_cast<float>(q.x), 1 );
  dr_plane_q.set_float( static_cast<float>(q.y), 2 );
  dr_plane_q.set_float( static_cast<float>(q.z), 3 );

  dr_plane_psi.set_float( psi1 );
  dr_plane_phi.set_float( phi1 );
  dr_plane_the.set_float( the1 );

  // Do this to align you aircraft, should reset forces!

  dr_override_forces.set_int( 1 );
  dr_L_total.set_float( 0.0 );
  dr_M_total.set_float( 0.0 );
  dr_N_total.set_float( 0.0 );

  dr_override_wing_forces.set_int( 1 );
  dr_fside_aero.set_float( 0.0 );
  dr_fnrml_aero.set_float( 0.0 );
  dr_faxil_aero.set_float( 0.0 );
  dr_L_aero.set_float( 0.0 );
  dr_M_aero.set_float( 0.0 );
  dr_N_aero.set_float( 0.0 );
  
  dr_override_engine_forces.set_int( 1 );
  dr_fside_prop.set_float( 0.0 );
  dr_fnrml_prop.set_float( 0.0 );
  dr_faxil_prop.set_float( 0.0 );
  dr_L_prop.set_float( 0.0 );
  dr_M_prop.set_float( 0.0 );
  dr_N_prop.set_float( 0.0 );

  // needed?
  dr_pos_P.set_float( 0.0 );
  dr_pos_Q.set_float( 0.0 );
  dr_pos_R.set_float( 0.0 );
  dr_pos_Prad.set_float( 0.0 );
  dr_pos_Qrad.set_float( 0.0 );
  dr_pos_Rrad.set_float( 0.0 );
  

  // end orientation
  dr_override_forces.set_int( 0 );
  dr_override_wing_forces.set_int( 0 );
  dr_override_engine_forces.set_int( 0 );
  
  dr_override_flightcontrol.set_int( 0 );  
}

// This could be reversed, warp the AI to me...
static void warp_to_next_ai() {

  int ai = dr_tcas_num_acf.get_int();
#ifdef DBG
  lg.xplm( "warp_to_next_ai():get_tcas_positions() = " + std::to_string(ai) + "\n" );
#endif
  if ( ai < 2 ) {
    return;
  }

  // take closest? Should prolly keep a sorted list? Sort?
  // Or just step through the list!
  int closest_idx   = 1;
  float max_dist    = 1000000;
  float user_x      = dr_pos_x.get_float();
  float user_z      = dr_pos_z.get_float();
  bool  res         = get_tcas_positions();
#if 0
  for( auto i = 1; i < ai; ++i) {
    float lx   = static_cast<float>(dr_tcas_pos_x.get_memory(i)); 
    float lz   = static_cast<float>(dr_tcas_pos_z.get_memory(i));
    float dist = sqrt( ((user_x-lx)*(user_x-lx)) + ((user_z-lz)*(user_z-lz)) );
    if ( dist > 500 ) { // if too close, take another
      if ( dist < max_dist ) {
	max_dist = dist;
	closest_idx = i;
      }
    }
  }
#endif

  last_ai_ac_index += 1;
  if ( last_ai_ac_index >= ai ) {
    last_ai_ac_index = 1;
  }
  closest_idx = last_ai_ac_index;
  
  // #ifdef DBG
  lg.xplm( "warp_to_next_ai():closest_idx = " + std::to_string(closest_idx) + "\n" );
  // #endif
    
  int i = closest_idx;  float lx1  = static_cast<float>(dr_tcas_pos_x.get_memory(i)); // or were these doubles
  float lz1  = static_cast<float>(dr_tcas_pos_z.get_memory(i));
  float ly1  = static_cast<float>(dr_tcas_pos_y.get_memory(i));
  float ele1 = static_cast<float>(dr_tcas_pos_ele.get_memory(i));
  
  float vx1  = static_cast<float>(dr_tcas_vel_x.get_memory(i)); // or were these doubles
  float vz1  = static_cast<float>(dr_tcas_vel_z.get_memory(i));
  float vy1  = static_cast<float>(dr_tcas_vel_y.get_memory(i));

#ifdef DBG
  lg.xplm( "warp_to_next_ai():ai x/y/z vel = " + std::to_string(vx1) + ", "
	   + std::to_string(vy1) + ", "
	   + std::to_string(vz1) + ", "
	   + "\n" );
#endif

  float phi1 = static_cast<float>(dr_tcas_pos_phi.get_memory(i)); // or were these doubles
  float psi1 = static_cast<float>(dr_tcas_pos_psi.get_memory(i)); // heading
  float the1 = static_cast<float>(dr_tcas_pos_the.get_memory(i));

  double angle_offset = fmod((psi1 + 180.0), 360);
  double s =  sin(angle_offset * (double)(M_PI/180));
  double c = -cos(angle_offset * (double)(M_PI/180));
  
  double delta_x = warp_distance * s;
  double delta_z = warp_distance * c;
  lx1 += delta_x;
  lz1 += delta_z;

  dr_override_flightcontrol.set_int( 1 );
  
  dr_pos_x.set_float( lx1 );
  dr_pos_z.set_float( lz1 );
  //
  dr_pos_y.set_float( ly1 );

  // velocity (not doing this causes problems... why?)
  // because we need to put the vx/vz right, reclculate it for the new headingm aybe
  angle_offset = psi1; // just swap s and c instead?
  s =  sin(angle_offset * (double)(M_PI/180));
  c = -cos(angle_offset * (double)(M_PI/180));
  float user_vel = sqrt( (dr_vel_x.get_float() * dr_vel_x.get_float()) + (dr_vel_z.get_float() * dr_vel_z.get_float()) );
  
  //dr_vel_x.set_float( vx1 );
  dr_vel_x.set_float( user_vel * s );
  //dr_vel_z.set_float( vz1 );
  dr_vel_z.set_float( user_vel * c );
  dr_vel_y.set_float( vy1 ); 
  
  // convert AI eulers to Q
  Eulers ypr = {0, 0, 0};
  Quaternion q;
  ypr.psi = psi1;
  ypr.the = the1;
  ypr.phi = phi1;
  EulersToQuaternion(ypr, q); // convert back

  dr_plane_q.set_float( static_cast<float>(q.w), 0 );
  dr_plane_q.set_float( static_cast<float>(q.x), 1 );
  dr_plane_q.set_float( static_cast<float>(q.y), 2 );
  dr_plane_q.set_float( static_cast<float>(q.z), 3 );

  dr_plane_psi.set_float( psi1 );
  dr_plane_phi.set_float( phi1 );
  dr_plane_the.set_float( the1 );

  // Do this to align you aircraft, should reset forces!

  dr_override_forces.set_int( 1 );
  dr_L_total.set_float( 0.0 );
  dr_M_total.set_float( 0.0 );
  dr_N_total.set_float( 0.0 );

  dr_override_wing_forces.set_int( 1 );
  dr_fside_aero.set_float( 0.0 );
  dr_fnrml_aero.set_float( 0.0 );
  dr_faxil_aero.set_float( 0.0 );
  dr_L_aero.set_float( 0.0 );
  dr_M_aero.set_float( 0.0 );
  dr_N_aero.set_float( 0.0 );
  
  dr_override_engine_forces.set_int( 1 );
  dr_fside_prop.set_float( 0.0 );
  dr_fnrml_prop.set_float( 0.0 );
  dr_faxil_prop.set_float( 0.0 );
  dr_L_prop.set_float( 0.0 );
  dr_M_prop.set_float( 0.0 );
  dr_N_prop.set_float( 0.0 );

  // needed?
  dr_pos_P.set_float( 0.0 );
  dr_pos_Q.set_float( 0.0 );
  dr_pos_R.set_float( 0.0 );
  dr_pos_Prad.set_float( 0.0 );
  dr_pos_Qrad.set_float( 0.0 );
  dr_pos_Rrad.set_float( 0.0 );
  

  // end orientation
  dr_override_forces.set_int( 0 );
  dr_override_wing_forces.set_int( 0 );
  dr_override_engine_forces.set_int( 0 );
  
  dr_override_flightcontrol.set_int( 0 );  
}

static void warp_to_prev_ai() {

  int ai = dr_tcas_num_acf.get_int();
#ifdef DBG
  lg.xplm( "warp_to_next_ai():get_tcas_positions() = " + std::to_string(ai) + "\n" );
#endif
  if ( ai < 2 ) {
    return;
  }

  // take closest? Should prolly keep a sorted list? Sort?
  // Or just step through the list!
  int closest_idx   = 1;
  float max_dist    = 1000000;
  float user_x      = dr_pos_x.get_float();
  float user_z      = dr_pos_z.get_float();
  bool  res         = get_tcas_positions();
#if 0
  for( auto i = 1; i < ai; ++i) {
    float lx   = static_cast<float>(dr_tcas_pos_x.get_memory(i)); 
    float lz   = static_cast<float>(dr_tcas_pos_z.get_memory(i));
    float dist = sqrt( ((user_x-lx)*(user_x-lx)) + ((user_z-lz)*(user_z-lz)) );
    if ( dist > 500 ) { // if too close, take another
      if ( dist < max_dist ) {
	max_dist = dist;
	closest_idx = i;
      }
    }
  }
#endif

  last_ai_ac_index -= 1;
  if ( last_ai_ac_index <= 1 ) {
    last_ai_ac_index = ai - 1;
  }
  closest_idx = last_ai_ac_index;
  
  // #ifdef DBG
  lg.xplm( "warp_to_next_ai():closest_idx = " + std::to_string(closest_idx) + "\n" );
  // #endif
    
  int i = closest_idx;  float lx1  = static_cast<float>(dr_tcas_pos_x.get_memory(i)); // or were these doubles
  float lz1  = static_cast<float>(dr_tcas_pos_z.get_memory(i));
  float ly1  = static_cast<float>(dr_tcas_pos_y.get_memory(i));
  float ele1 = static_cast<float>(dr_tcas_pos_ele.get_memory(i));
  
  float vx1  = static_cast<float>(dr_tcas_vel_x.get_memory(i)); // or were these doubles
  float vz1  = static_cast<float>(dr_tcas_vel_z.get_memory(i));
  float vy1  = static_cast<float>(dr_tcas_vel_y.get_memory(i));

#ifdef DBG
  lg.xplm( "warp_to_next_ai():ai x/y/z vel = " + std::to_string(vx1) + ", "
	   + std::to_string(vy1) + ", "
	   + std::to_string(vz1) + ", "
	   + "\n" );
#endif

  float phi1 = static_cast<float>(dr_tcas_pos_phi.get_memory(i)); // or were these doubles
  float psi1 = static_cast<float>(dr_tcas_pos_psi.get_memory(i)); // heading
  float the1 = static_cast<float>(dr_tcas_pos_the.get_memory(i));

  double angle_offset = fmod((psi1 + 180.0), 360);
  double s =  sin(angle_offset * (double)(M_PI/180));
  double c = -cos(angle_offset * (double)(M_PI/180));
  
  double delta_x = warp_distance * s;
  double delta_z = warp_distance * c;
  lx1 += delta_x;
  lz1 += delta_z;

  dr_override_flightcontrol.set_int( 1 );
  
  dr_pos_x.set_float( lx1 );
  dr_pos_z.set_float( lz1 );
  //
  dr_pos_y.set_float( ly1 );

  // velocity (not doing this causes problems... why?)
  // because we need to put the vx/vz right, reclculate it for the new headingm aybe
  angle_offset = psi1; // just swap s and c instead?
  s =  sin(angle_offset * (double)(M_PI/180));
  c = -cos(angle_offset * (double)(M_PI/180));
  float user_vel = sqrt( (dr_vel_x.get_float() * dr_vel_x.get_float()) + (dr_vel_z.get_float() * dr_vel_z.get_float()) );
  
  //dr_vel_x.set_float( vx1 );
  dr_vel_x.set_float( user_vel * s );
  //dr_vel_z.set_float( vz1 );
  dr_vel_z.set_float( user_vel * c );
  dr_vel_y.set_float( vy1 ); 
  
  // convert AI eulers to Q
  Eulers ypr = {0, 0, 0};
  Quaternion q;
  ypr.psi = psi1;
  ypr.the = the1;
  ypr.phi = phi1;
  EulersToQuaternion(ypr, q); // convert back

  dr_plane_q.set_float( static_cast<float>(q.w), 0 );
  dr_plane_q.set_float( static_cast<float>(q.x), 1 );
  dr_plane_q.set_float( static_cast<float>(q.y), 2 );
  dr_plane_q.set_float( static_cast<float>(q.z), 3 );

  dr_plane_psi.set_float( psi1 );
  dr_plane_phi.set_float( phi1 );
  dr_plane_the.set_float( the1 );

  // Do this to align you aircraft, should reset forces!

  dr_override_forces.set_int( 1 );
  dr_L_total.set_float( 0.0 );
  dr_M_total.set_float( 0.0 );
  dr_N_total.set_float( 0.0 );

  dr_override_wing_forces.set_int( 1 );
  dr_fside_aero.set_float( 0.0 );
  dr_fnrml_aero.set_float( 0.0 );
  dr_faxil_aero.set_float( 0.0 );
  dr_L_aero.set_float( 0.0 );
  dr_M_aero.set_float( 0.0 );
  dr_N_aero.set_float( 0.0 );
  
  dr_override_engine_forces.set_int( 1 );
  dr_fside_prop.set_float( 0.0 );
  dr_fnrml_prop.set_float( 0.0 );
  dr_faxil_prop.set_float( 0.0 );
  dr_L_prop.set_float( 0.0 );
  dr_M_prop.set_float( 0.0 );
  dr_N_prop.set_float( 0.0 );

  // needed?
  dr_pos_P.set_float( 0.0 );
  dr_pos_Q.set_float( 0.0 );
  dr_pos_R.set_float( 0.0 );
  dr_pos_Prad.set_float( 0.0 );
  dr_pos_Qrad.set_float( 0.0 );
  dr_pos_Rrad.set_float( 0.0 );
  

  // end orientation
  dr_override_forces.set_int( 0 );
  dr_override_wing_forces.set_int( 0 );
  dr_override_engine_forces.set_int( 0 );
  
  dr_override_flightcontrol.set_int( 0 );  
}

// Put a smoke thingy at airports
static void smoke_airports() {

  if ( pois.size() == 0 ) {
    return;
  }
  
  // user's plane position
  float px = dr_pos_x.get_float();
  float py = dr_pos_y.get_float();
  float pz = dr_pos_z.get_float();
  double uplat = dr_pos_latitude.get_double(); 
  double uplon = dr_pos_longitude.get_double();
  
  double poi_x;
  double poi_y;
  double poi_z;

  char dist_buf[32];
  char alt_buf[32];
  char spd_buf[32];

  auto shown_counter = 12;
  for ( auto& poi : pois) {
    
    double plat = poi.lat; //dr_pos_latitude.get_double();
    double plon = poi.lon; //dr_pos_longitude.get_double();
    float latitude;
    float longitude;
    float alt = poi.alt;
    int max_dist = poi.dst;
    poi_x = poi.x;
    poi_y = poi.y;
    poi_z = poi.z;
    
    float dx = poi_x - px;
    float dz = poi_z - pz;
    float dy = poi_y - py; 

    double latlon_dist = dist_latlon(uplat, uplon, plat, plon); // but not exact.... hmmm
    //lg.xplm( "POI: "+poi.name+", "+std::to_string(latlon_dist)+"\n" );
    
    if ( latlon_dist >= max_dist ) {  // they can have different distances, so a short one disables a longer one after it. maybe sort on diff between distance and viewdistance?
      //break;//continue; // skip if too far away // break; // we are sorted
      continue;
    }

    Smoker *s = new Smoker();
    if ( s->load_obj( pss_obj->path ) ) {; // copy from the global one
    
      smokers.push_back( s );
    
      lg.xplm( "pss_obj->instantiate();\n");
      s->instantiate();
      lg.xplm( "pss_obj->set_pos();\n");
      s->set_pos( poi_x, poi_y, poi_z );
      lg.xplm( "pss_obj->set_pos() ready;\n");
    }
    
    if ( --shown_counter <= 0 ) {
      break;
    }
    
  } // for

}

int toggle_ac_label_handler( XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon ) {
  if (inPhase == xplm_CommandBegin) { // xplm_CommandContinue (1), xplm_CommandEnd (2)
    show_ac_label = ( ! show_ac_label );
  
    // draw an object at plane's pos.
    /*
    float px = dr_pos_x.get_float();
    float py = dr_pos_y.get_float();
    float pz = dr_pos_z.get_float();

    Smoker* s = new Smoker();
    s->load_obj( pss_obj->path ); // copy from the global one
    s->mode = 1;
    smokers.push_back( s );
    lg.xplm( "pss_obj->instantiate();\n");
    s->instantiate();
    lg.xplm( "pss_obj->set_pos();\n");
    s->set_pos( -4, 1, 0 ); // left wing, 1m up, CoG
    lg.xplm( "pss_obj->set_pos() ready;\n");

    s = new Smoker();
    s->load_obj( pss_obj->path ); // copy from the global one
    s->mode = 1;
    smokers.push_back( s );
    lg.xplm( "pss_obj->instantiate();\n");
    s->instantiate();
    lg.xplm( "pss_obj->set_pos();\n");
    s->set_pos( 4, 1, 0 ); // right wing, 1m up, CoG
    lg.xplm( "pss_obj->set_pos() ready;\n");
    */
  }
  
  return 0;
}

int toggle_ap_label_handler( XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon ) {
  if (inPhase == xplm_CommandBegin) { // xplm_CommandContinue (1), xplm_CommandEnd (2)
    show_ap_label = ( ! show_ap_label );
  }
  return 0;
}

int toggle_ap_smoker_handler( XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon ) {
  if (inPhase == xplm_CommandBegin) { // xplm_CommandContinue (1), xplm_CommandEnd (2)
    smoke_airports();
  }
  return 0;
}

// Update the root toml directly, and write on exit?
int toggle_units_handler( XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon ) {
  if (inPhase == xplm_CommandBegin) { // xplm_CommandContinue (1), xplm_CommandEnd (2)
    if ( units == 0 ) {
      units = 1; // maybe more than 1 later
    } else {
      units = 0;
    }
  }
  return 0;
}

int toggle_warp_to_next_ai_handler( XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon ) {
  if (inPhase == xplm_CommandBegin) { // xplm_CommandContinue (1), xplm_CommandEnd (2)
    warp_to_next_ai();
  }
  return 0;
}

int toggle_warp_to_prev_ai_handler( XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon ) {
  if (inPhase == xplm_CommandBegin) { // xplm_CommandContinue (1), xplm_CommandEnd (2)
    warp_to_prev_ai();
  }
  return 0;
}

int toggle_warp_to_closest_ai_handler( XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon ) {
  if (inPhase == xplm_CommandBegin) { // xplm_CommandContinue (1), xplm_CommandEnd (2)
    warp_to_closest_ai();
  }
  return 0;
}

int toggle_warp_forwards_handler( XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon ) {
  if (inPhase == xplm_CommandBegin) { // xplm_CommandContinue (1), xplm_CommandEnd (2)
    warp_forwards();
  }
  return 0;
}

// ----

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc) {
  strcpy(outName, "X-Label");
  strcpy(outSig, "durian.xlabel");
  strcpy(outDesc, "Plugin to label AI aircraft and POIs.");

  XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);  
  XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);

  encode_test();
  decode_test();
  neighbour_test();
  
  char filebase[255];
  //XPLMGetSystemPath(filebase); // Locate the X-System directory
  XPLMGetPrefsPath(filebase); // Locate the X-System Output directory
  XPLMExtractFileAndPath(filebase);
  const char *sep = XPLMGetDirectorySeparator();
  std::string prefsfile = std::string(filebase) + sep + "xlabel_pois.txt";
  lg.xplm( prefsfile + "\n" );
  (void)read_pois( prefsfile, pois );
  
  dr_tcas_num_acf.init();
  dr_tcas_pos_x.init();
  dr_tcas_pos_y.init();
  dr_tcas_pos_z.init();
  int ai = dr_tcas_num_acf.get_int();
  lg.xplm( "ai = "+std::to_string(ai)+"\n" );
  bool res = get_tcas_positions();
  for ( auto i = 0; i < ai; i++ ) {
    double lx  = static_cast<double>(dr_tcas_pos_x.get_memory(i)); // floats
    double lz  = static_cast<double>(dr_tcas_pos_z.get_memory(i));
    double ly  = static_cast<double>(dr_tcas_pos_y.get_memory(i));
    lg.xplm( " POS AI x/y/z "+std::to_string(lx)+", "+std::to_string(ly)+", "+std::to_string(lz)+"\n" );
  }

  // Load a particle object. Just one at the moment.
  std::string pss_obj_filename = std::string(filebase) + sep + "xlabel.obj";
  lg.xplm( pss_obj_filename + "\n" );
  pss_obj = new Smoker();
  pss_obj->load_obj( pss_obj_filename );
  if ( ! pss_obj ) {
    lg.xplm( "ERROR loading " + pss_obj_filename + "\n" );
  } else {
    lg.xplm( "Loaded " + pss_obj_filename + "\n" );
  }
  
  dr_pos_x.init();
  dr_pos_y.init();
  dr_pos_z.init();
  dr_matrix_wrl.init();
  dr_matrix_proj.init();
  dr_screen_width.init();
  dr_screen_height.init();

  toggle_ac_label_cmd = XPLMCreateCommand("durian/xlabel/toggle_ac_label", "Toggle a/c label");
  XPLMRegisterCommandHandler(toggle_ac_label_cmd, toggle_ac_label_handler, 0, (void *)0);

  toggle_ap_label_cmd = XPLMCreateCommand("durian/xlabel/toggle_ap_label", "Toggle airport/poi label");
  XPLMRegisterCommandHandler(toggle_ap_label_cmd, toggle_ap_label_handler, 0, (void *)0);

  toggle_ap_smoker_cmd = XPLMCreateCommand("durian/xlabel/toggle_ap_smoker", "Toggle airport/poi smoker");
  XPLMRegisterCommandHandler(toggle_ap_smoker_cmd, toggle_ap_smoker_handler, 0, (void *)0);

  toggle_units_cmd = XPLMCreateCommand("durian/xlabel/toggle_units", "Toggle meters/feet");
  XPLMRegisterCommandHandler(toggle_units_cmd, toggle_units_handler, 0, (void *)0);

  toggle_warp_to_next_ai_cmd = XPLMCreateCommand("durian/xlabel/warp_to_next_ai", "Warp user aicraft to next AI aircraft");
  XPLMRegisterCommandHandler(toggle_warp_to_next_ai_cmd, toggle_warp_to_next_ai_handler, 0, (void *)0);

  toggle_warp_to_prev_ai_cmd = XPLMCreateCommand("durian/xlabel/warp_to_prev_ai", "Warp user aircraft to previous AI aircraft");
  XPLMRegisterCommandHandler(toggle_warp_to_prev_ai_cmd, toggle_warp_to_prev_ai_handler, 0, (void *)0);

  toggle_warp_to_closest_ai_cmd = XPLMCreateCommand("durian/xlabel/warp_to_closest_ai", "Warp user aircraft to closest AI aircraft");
  XPLMRegisterCommandHandler(toggle_warp_to_closest_ai_cmd, toggle_warp_to_closest_ai_handler, 0, (void *)0);

  toggle_warp_forwards_cmd = XPLMCreateCommand("durian/xlabel/warp_forwards", "Warp user forwards");
  XPLMRegisterCommandHandler(toggle_warp_forwards_cmd, toggle_warp_forwards_handler, 0, (void *)0);

  XPLMRegisterDrawCallback(DrawCallback1, xplm_Phase_Window, 0, NULL);
  XPLMRegisterDrawCallback(DrawCallback2, xplm_Phase_Window, 0, NULL);// slow

  XPLMRegisterFlightLoopCallback(flight_loop, 10, NULL);
  XPLMRegisterFlightLoopCallback(smoker_loop, -1, NULL);

  // test toml
  std::string tomlfile = std::string(filebase) + sep + "xlabel.toml";
  lg.xplm( tomlfile + "\n" );

  std::ifstream ifs( tomlfile );
  if ( ! ifs ) {
    lg.xplm( "ERROR: tomlfile not found.\n" ); // pr will fall through?
    // Create new file
    std::ofstream file( tomlfile.c_str(), std::ios::out );
    if ( file ) {
      std::ostringstream ofs;
      toml::Table root;
    
      toml::Table general_Value = toml::Table();
      general_Value["units"] = 1;
      general_Value["warp_distance"] = 200; // m/f depending on units?
      general_Value["match_orientation"] = 1;
      general_Value["airport_smoker"] = "xlabel.pss";
      
      toml::Table debug_Value = toml::Table();
      debug_Value["verbose"] = 100;
      debug_Value["file"] = false;
      
      root["general"] = general_Value;
      root["debug"] = debug_Value;
      
      file << root;
      file.close();
      lg.xplm( ofs.str() );
    }
  } else { // file exists
    toml::ParseResult pr = toml::parse( ifs );
    
    if ( pr.valid() ) {
      lg.xplm( tomlfile + " is valid\n" );
      // pr.value is the parsed value.
      const toml::Value& v = pr.value;
      const toml::Value* x = v.find("general.useraircraft");
      if (x && x->is<std::string>()) {
	lg.xplm( x->as<std::string>() + "\n" );
      }
      if (x && x->is<int>()) {
	lg.xplm( std::to_string(x->as<int>()) + "\n" );
      }
      // lg.xplm( std::to_string(v.get<int>("general.useraircraft")) + "\n" ); // crashes if not found!
      
      std::ostringstream s;
      v.writeFormatted( &s, toml::FormatFlag::FORMAT_INDENT );
      lg.xplm( s.str() );
    } else {
      // not a valid toml file
      lg.xplm( "ERROR: tomlfile not valid.\n" );      
      // Create new file? do not want to overwrite a user file...
    }
  }
  
  return 1;
}

PLUGIN_API void	XPluginStop(void) {
  lg.xplm( "XPluginStop(void).\n" );
  XPLMUnregisterDrawCallback(DrawCallback1, xplm_Phase_Window, 0, NULL);
  XPLMUnregisterDrawCallback(DrawCallback2, xplm_Phase_Window, 0, NULL);
  XPLMUnregisterCommandHandler(toggle_ac_label_cmd, toggle_ac_label_handler, 0, (void *)0);
  XPLMUnregisterCommandHandler(toggle_ap_label_cmd, toggle_ap_label_handler, 0, (void *)0);
  XPLMUnregisterCommandHandler(toggle_ap_smoker_cmd, toggle_ap_smoker_handler, 0, (void *)0);
  XPLMUnregisterCommandHandler(toggle_units_cmd, toggle_units_handler, 0, (void *)0);
  XPLMUnregisterCommandHandler(toggle_warp_to_next_ai_cmd, toggle_warp_to_next_ai_handler, 0, (void *)0);
  XPLMUnregisterCommandHandler(toggle_warp_to_prev_ai_cmd, toggle_warp_to_prev_ai_handler, 0, (void *)0);
  XPLMUnregisterCommandHandler(toggle_warp_to_closest_ai_cmd, toggle_warp_to_closest_ai_handler, 0, (void *)0);
  XPLMUnregisterCommandHandler(toggle_warp_forwards_cmd, toggle_warp_forwards_handler, 0, (void *)0);
  XPLMUnregisterFlightLoopCallback(flight_loop, NULL);
  XPLMUnregisterFlightLoopCallback(smoker_loop, NULL);

  for ( auto si = smokers.begin(); si != smokers.end(); si++ ) {
    Smoker *s = *si;
    s->deinstantiate();
  }
  smokers.clear();
  delete pss_obj;

  // POIS
  pois.clear();
  show_ac_label = 0;
  show_ap_label = 0;
  lg.xplm( "XPluginStop(void) END.\n" );
}

PLUGIN_API int XPluginEnable(void) {
  lg.xplm( "XPluginEnable(void).\n" );
  /*
  int ai = dr_tcas_num_acf.get_int();
  lg.xplm( "ai = "+std::to_string(ai)+"\n" );
  bool res = get_tcas_positions();
  for ( auto i = 0; i < ai; i++ ) {
    double lx  = static_cast<double>(dr_tcas_pos_x.get_memory(i)); 
    double lz  = static_cast<double>(dr_tcas_pos_z.get_memory(i));
    double ly  = static_cast<double>(dr_tcas_pos_y.get_memory(i));
    lg.xplm( " POS AI x/y/z "+std::to_string(i)+": "+std::to_string(lx)+", "+std::to_string(ly)+", "+std::to_string(lz)+"\n" );
  }
  */
  lg.xplm( "XPluginEnable(void) END.\n" );
  return 1;
}

PLUGIN_API void XPluginDisable(void) {
  lg.xplm( "XPluginDisable(void).\n" );
  /*
  for ( auto si = smokers.begin(); si != smokers.end(); si++ ) {
    Smoker *s = *si;
    //s->deinstantiate();
  }
  smokers.clear();
  pois.clear();
  show_ac_label = 0;
  show_ap_label = 0;
  */
  lg.xplm( "XPluginDisable(void) END.\n" );
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) {
  if ( inMsg == XPLM_MSG_PLANE_LOADED ) {
    lg.xplm( "Plane loaded\n" );

    /*
    double poi_x;
    double poi_y;
    double poi_z;

    for ( auto& poi : pois) {
      double plat = poi.lat; //dr_pos_latitude.get_double();
      double plon = poi.lon; //dr_pos_longitude.get_double();
      float  alt  = poi.alt;
      int max_dist = poi.dst;
      poi_to_local(plat, plon, poi_x, poi_y, poi_z); // only needed on scenery switch?!
      poi.x = poi_x;
      poi.y = poi_y;
      poi_y += alt;
      poi.z = poi_z;
      poi.update = 0;
    }
    */
    
    double plat = dr_pos_latitude.get_double();
    double plon = dr_pos_longitude.get_double();
    poi p{plat, plon, 0, 0, "", 0, 0, 0 }; // hmmm
    std::sort( begin(pois),
	       end(pois),
	       [p](const poi& lhs, const poi& rhs) {
		 return dist_latlon(p.lat, p.lon, lhs.lat, lhs.lon) < dist_latlon(p.lat, p.lon, rhs.lat, rhs.lon);
	       }
	       );
  
    lg.xplm( "Plane loaded end\n" );
  }
  
  if ( inMsg == XPLM_MSG_SCENERY_LOADED ) {
    lg.xplm( "Scenery reload\n" );

    double plat;
    double plon;
    float alt;
    double poi_x;
    double poi_y;
    double poi_z;
    for ( auto& poi : pois) {
      poi.update = 1; // recalculate local position.
      plat = poi.lat; //dr_pos_latitude.get_double();
      plon = poi.lon; //dr_pos_longitude.get_double();
      alt  = poi.alt;
      poi_to_local(plat, plon, poi_x, poi_y, poi_z); // only needed on scenery switch?!
      poi.x = poi_x;
      poi.y = poi_y;
      poi_y += alt;
      poi.z = poi_z;
      poi.update = 0;
    }
    
    lg.xplm( "Scenery reloaded\n" );
  }
  
}

float flight_loop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon) {

  (void)inElapsedTimeSinceLastFlightLoop;
  (void)inCounter;
  (void)inRefcon;

  double elapsed = inElapsedSinceLastCall;

  if ( ! show_ap_label ) {
    return 1; // Note, 1 sec
  }

  // instead of time, calc distance, and every x km moved we resort.
  
  double plat = dr_pos_latitude.get_double();
  double plon = dr_pos_longitude.get_double();


  // Calc dist from last point saved
  double dist_moved = dist_latlon(plat, plon, plane_prev_lat, plane_prev_lon);
  if ( dist_moved < 2000.0 ) { // 2 km
    return 1; // Note, 1 sec
  }

  // Save new point
  plane_prev_lat = plat;
  plane_prev_lon = plon;

  // We moved, so recalculate
  lg.xplm( "Moved 2 km, resorting.\n" );

  std::string gh;
  geohash::encode( plat, plon, 8, gh );
#ifdef DBG
  lg.xplm( "GEOHASH: " + gh + "\n" );
#endif
  /*
  std::string prefix = gh.substr(0, 3);
  std::vector<poi> gh_pois = poimap[prefix];
  for ( auto& gh_poi : gh_pois) {
    lg.xplm( "poimap: "+gh_poi.name+", "+gh_poi.geohash + "\n" );
  }
  */

  float px = dr_pos_x.get_float();
  float py = dr_pos_y.get_float();
  float pz = dr_pos_z.get_float();

  poi p{plat, plon, 0, 0, "", 0, 0, 0, 1, ""};
  auto current_elem = 1;

  // selection sort, https://www.softwaretestinghelp.com/sorting-techniques-in-cpp/ ?

  lg.xplm("SORT START\n");
  std::partial_sort( pois.begin(), pois.begin() + max_shown, pois.end(),
		     [p](const poi& lhs, const poi& rhs) {
		       return dist_latlon(p.lat, p.lon, lhs.lat, lhs.lon) < dist_latlon(p.lat, p.lon, rhs.lat, rhs.lon);
		     }
		     );
  lg.xplm("SORT END\n");
  
  /*
  std::sort( begin(pois),
	     end(pois),
	     [p](const poi& lhs, const poi& rhs) {
	       return dist_latlon(p.lat, p.lon, lhs.lat, lhs.lon) < dist_latlon(p.lat, p.lon, rhs.lat, rhs.lon);
	     }
	     );
  */

#if 0
  int c = 10;
  for ( auto& poi : pois) {

    double latlon_dist = dist_latlon(plat, plon, poi.lat, poi.lon);
    float dx = px - poi.x;
    float dz = pz - poi.z;
    float dist = sqrt( dx*dx + dz*dz ); // more exact if locally close

#ifdef DBG
    lg.xplm( "Closest to: "+poi.name+", "+std::to_string(latlon_dist)+", "+std::to_string(dist)+
	     " "+poi.geohash + "\n" );
#endif
    if ( --c <= 0 ) {
      break;
    }
  }
#endif
  
  return 1; // 1 sec again...
}

// ----
