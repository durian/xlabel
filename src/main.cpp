// Downloaded from https://developer.x-plane.com/code-sample/x-plane-11-map/

#include "XPLMMap.h"
#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
#include "XPLMNavigation.h"
#include "XPLMScenery.h"
#include "XPLMPlugin.h"
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

// Mine
#include "dr.h"
#include "Log.h"
#include "global.h"

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
*/

std::vector<poi> pois;

// ----

static int DrawCallback1(XPLMDrawingPhase inPhase, int inIsBefore, void * inRefcon) {

  if ( ! show_label ) {
    return 1;
  }
  
  float mv[16], proj[16];
  // Read the model view and projection matrices from this frame
  dr_matrix_wrl.get_all_ref( &mv[0], 16 );
  dr_matrix_proj.get_all_ref( &proj[0], 16 );
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

  for ( auto i = 1; i < ai; i++ ) { // skip 0, it is user's plane, only when at a distance?
    float lx  = static_cast<float>(dr_tcas_pos_x.get_memory(i)); // or were these doubles
    float lz  = static_cast<float>(dr_tcas_pos_z.get_memory(i));
    float ly  = static_cast<float>(dr_tcas_pos_y.get_memory(i));

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

      if ( dist < 200 * 1000 ) { // only < 200 kms

	char v_spd = '-';
	if ( dr_vertical_speed.get_memory(i) > 0.5 ) {
	  v_spd = '^';
	} else if ( dr_vertical_speed.get_memory(i) < -0.5 ) {
	  v_spd = 'v';
	}
	
	float v_msc = dr_V_msc.get_memory(i); // 1.943844
	
	// Read a file:
	//  56.292109, 12.854471, 0, RGB, ESTA 
	
	// Not show when less than 1000m? and > 40km?
	//sim/cockpit2/tcas/indicators/relative_distance_mtrs     float[64]       y       meters  Distance to each other
	//sim/cockpit2/tcas/indicators/relative_altitude_mtrs     float[64]       y       meters  Relative altitude
	//sim/cockpit2/tcas/targets/position/vertical_speed       float[64]       y       feet/min
	//sim/cockpit2/tcas/targets/position/V_msc                float[64]       n       meter/s total true speed
	
	float colWHT[] = { 1.0, 1.0, 1.0 };
	if ( dist > 5000.0f ) {
	  sprintf( buffer, "%i/%.1f/%i/%i %c", i, dist/1000.0f, int(dy), int(v_msc*1.943844), v_spd );
	} else {
	  sprintf( buffer, "%i/%i/%i/%i %c", i, int(dist), int(dy), int(v_msc*1.943844), v_spd );
	}
	int len = strlen(buffer);
	
	float box_y = final_y + 12 + 10; // We put box above
	float box_x = final_x + 12; // We put box to right
	XPLMDrawTranslucentDarkBox(box_x - 5, box_y + 10, box_x + 6*len + 5, box_y - 8);
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
	float half_width  = 10;
	float half_height = 10;
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
  
  if ( ! show_label ) {
    return 1;
  }
  
  float mv[16], proj[16];
  // Read the model view and projection matrices from this frame
  dr_matrix_wrl.get_all_ref( &mv[0], 16 );
  dr_matrix_proj.get_all_ref( &proj[0], 16 );
  float acf_eye[4], acf_ndc[4];

  float px = dr_pos_x.get_float();
  float py = dr_pos_y.get_float();
  float pz = dr_pos_z.get_float();

  double poi_x;
  double poi_y;
  double poi_z;
  
  // Testing testing
  for ( auto& poi : pois) {
    
    double plat = poi.lat; //dr_pos_latitude.get_double();
    double plon = poi.lon; //dr_pos_longitude.get_double();
    float latitude;
    float longitude;
    float alt = poi.alt;
    int max_dist = poi.dst;
    if ( poi.update == 1 ) { // should be made 0 on scenery reload FIXME
      poi_to_local(plat, plon, poi_x, poi_y, poi_z); // only needed on scenery switch?!
      poi.x = poi_x;
      poi.y = poi_y;
      poi_y += alt;
      poi.z = poi_z;
      poi.update = 0;
      lg.xplm( "Init POI\n" );
    } else {
      poi_x = poi.x;
      poi_y = poi.y;
      poi_z = poi.z;
    }
    
    float dx = poi_x - px;
    float dz = poi_z - pz;
    float dy = poi_y - py; 
    float dist = sqrt( dx*dx + dz*dz ); // there is a tcas / relative_distance_mtrs dataref

    if ( dist >= max_dist ) {
      continue; // skip if too far away
    }
    
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
      
      float screen_w = (float)dr_screen_width.get_int();
      float screen_h = (float)dr_screen_height.get_int();
      
      float final_x = screen_w * (acf_ndc[0] * 0.5f + 0.5f);
      float final_y = screen_h * (acf_ndc[1] * 0.5f + 0.5f);
      
      float colWHT[] = { 1.0, 1.0, 1.0 };
      if ( dist < 5000.0f ) {
	sprintf( buffer, "%s %s %.1f m", id, poi.name.c_str(), dist );
      } else {
	dist /= 1000.0f;
	sprintf( buffer, "%s %s %.1f km", id, poi.name.c_str(), dist );
      }
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
	glColor3f(0, 1, 0); // green
	float half_width  = 10;
	float half_height = 10;
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
      }
      
    }
  }
  
  return 1;
}

int toggle_label_handler( XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void *inRefcon ) {
  if (inPhase == xplm_CommandBegin) { // xplm_CommandContinue (1), xplm_CommandEnd (2)
    show_label = ( ! show_label );
  }
  return 0;
}

// ----

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc) {
  strcpy(outName, "X-Label");
  strcpy(outSig, "durian.xlabel");
  strcpy(outDesc, "Plugin to label AI aircraft.");

  XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);  
  XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);
  
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
    float lx  = static_cast<double>(dr_tcas_pos_x.get_memory(i)); // floats
    float lz  = static_cast<double>(dr_tcas_pos_z.get_memory(i));
    float ly  = static_cast<double>(dr_tcas_pos_y.get_memory(i));
    lg.xplm( " POS AI x/y/z "+std::to_string(lx)+", "+std::to_string(ly)+", "+std::to_string(lz)+"\n" );
  }

  dr_pos_x.init();
  dr_pos_y.init();
  dr_pos_z.init();
  dr_matrix_wrl.init();
  dr_matrix_proj.init();
  dr_screen_width.init();
  dr_screen_height.init();

  toggle_label_cmd = XPLMCreateCommand("durian/xlabel/toggle_label", "Toggle label");
  XPLMRegisterCommandHandler(toggle_label_cmd, toggle_label_handler, 0, (void *)0);

  XPLMRegisterDrawCallback(DrawCallback1, xplm_Phase_Window, 0, NULL);
  XPLMRegisterDrawCallback(DrawCallback2, xplm_Phase_Window, 0, NULL);// slow
  
  return 1;
}

PLUGIN_API void	XPluginStop(void) {
  XPLMUnregisterDrawCallback(DrawCallback1, xplm_Phase_Window, 0, NULL);
  XPLMUnregisterDrawCallback(DrawCallback2, xplm_Phase_Window, 0, NULL);
  XPLMUnregisterCommandHandler(toggle_label_cmd, toggle_label_handler, 0, (void *)0);
}

PLUGIN_API int XPluginEnable(void) {
  int ai = dr_tcas_num_acf.get_int();
  lg.xplm( "ai = "+std::to_string(ai)+"\n" );
  bool res = get_tcas_positions();
  for ( auto i = 0; i < ai; i++ ) {
    double lx  = static_cast<double>(dr_tcas_pos_x.get_memory(i)); 
    double lz  = static_cast<double>(dr_tcas_pos_z.get_memory(i));
    double ly  = static_cast<double>(dr_tcas_pos_y.get_memory(i));
    lg.xplm( " POS AI x/y/z "+std::to_string(i)+": "+std::to_string(lx)+", "+std::to_string(ly)+", "+std::to_string(lz)+"\n" );
  }

  return 1;
}

PLUGIN_API void XPluginDisable(void) {
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) {
  if ( inMsg == XPLM_MSG_SCENERY_LOADED ) {
    lg.xplm( "Scenery reload\n" );
    for ( auto& poi : pois) {
      poi.update = 1;
    }
    lg.xplm( "Scenery reloaded\n" );
  }
}


// ----

