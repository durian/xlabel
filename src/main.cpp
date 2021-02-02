// Downloaded from https://developer.x-plane.com/code-sample/x-plane-11-map/

#include "XPLMMap.h"
#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
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

using namespace XLABEL;

XPLMMapLayerID		g_layer = NULL;

static int	coord_in_rect(float x, float y, const float bounds_ltrb[4])  {	return ((x >= bounds_ltrb[0]) && (x < bounds_ltrb[2]) && (y >= bounds_ltrb[3]) && (y < bounds_ltrb[1])); }

void createOurMapLayer(const char * mapIdentifier, void * refcon);

static void prep_cache(         XPLMMapLayerID layer, const float * inTotalMapBoundsLeftTopRightBottom, XPLMMapProjectionID projection, void * inRefcon);
static void draw_markings(      XPLMMapLayerID layer, const float * inMapBoundsLeftTopRightBottom, float zoomRatio, float mapUnitsPerUserInterfaceUnit, XPLMMapStyle mapStyle, XPLMMapProjectionID projection, void * inRefcon);
static void draw_marking_icons( XPLMMapLayerID layer, const float * inMapBoundsLeftTopRightBottom, float zoomRatio, float mapUnitsPerUserInterfaceUnit, XPLMMapStyle mapStyle, XPLMMapProjectionID projection, void * inRefcon);
static void draw_marking_labels(XPLMMapLayerID layer, const float * inMapBoundsLeftTopRightBottom, float zoomRatio, float mapUnitsPerUserInterfaceUnit, XPLMMapStyle mapStyle, XPLMMapProjectionID projection, void * inRefcon);
static void will_be_deleted(    XPLMMapLayerID layer, void * inRefcon);

XPLMCommandRef toggle_label_cmd;
bool show_label = false;
int  label_kind = 0;
  
DRefInt dr_tcas_num_acf{"sim/cockpit2/tcas/indicators/tcas_num_acf"};
DRefInt dr_override_TCAS{"sim/operation/override/override_TCAS"};
DRefInt dr_acf_modeS_id{"sim/aircraft/view/acf_modeS_id"};
DRefFloatArray dr_tcas_pos_x{"sim/cockpit2/tcas/targets/position/x"};
DRefFloatArray dr_tcas_pos_y{"sim/cockpit2/tcas/targets/position/y"};
DRefFloatArray dr_tcas_pos_z{"sim/cockpit2/tcas/targets/position/z"};
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
DRefFloatArray dr_rel_dist_mtrs{"sim/cockpit2/tcas/indicators/relative_distance_mtrs"};
DRefFloatArray dr_ref_alt_mtrs{"sim/cockpit2/tcas/indicators/relative_altitude_mtrs"};
DRefFloatArray dr_vertical_speed{"sim/cockpit2/tcas/targets/position/vertical_speed"};
DRefFloatArray dr_V_msc{"sim/cockpit2/tcas/targets/position/V_msc"};
DRefIntArray   dr_tcas_modeS{"sim/cockpit2/tcas/targets/modeS_id"};
bool get_tcas_positions();
bool get_tcas_positions() {
#ifdef DBG
    lg.xplm( "get_tcas_positions() START\n" );
#endif
    // Number is in dr_tcas_num_acf
    if ( dr_tcas_num_acf.is_initialised() ) {
      lg.xplm( "dr_tcas_num_acf says "+std::to_string(dr_tcas_num_acf.get_int())
	       +" "+std::to_string(dr_tcas_num_acf.init())
	       +"\n" );
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

DRefFloat dr_pos_x{"sim/flightmodel/position/local_x"};
DRefFloat dr_pos_y{"sim/flightmodel/position/local_y"};
DRefFloat dr_pos_z{"sim/flightmodel/position/local_z"};
DRefFloatArray dr_matrix_wrl{"sim/graphics/view/world_matrix"};
DRefFloatArray dr_matrix_proj{"sim/graphics/view/projection_matrix_3d"};
DRefInt dr_screen_width{"sim/graphics/view/window_width"};
DRefInt dr_screen_height{"sim/graphics/view/window_height"};
static void mult_matrix_vec(float dst[4], const float m[16], const float v[4]) {
  dst[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8] + v[3] * m[12];
  dst[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9] + v[3] * m[13];
  dst[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10] + v[3] * m[14];
  dst[3] = v[0] * m[3] + v[1] * m[7] + v[2] * m[11] + v[3] * m[15];
}

static int DrawCallback1(XPLMDrawingPhase inPhase, int inIsBefore, void * inRefcon) {
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
  
  float acf_wrl[4] = {	
    px,
    py,
    pz,
    1.0f };

  int ai = dr_tcas_num_acf.get_int();
  bool res = get_tcas_positions();
  char buffer[256];

  //DRefFloatArray dr_V_msc{"sim/cockpit2/tcas/targets/position/V_msc"};
  
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

      char v_spd = '-';
      if ( dr_vertical_speed.get_memory(i) > 0.5 ) {
	v_spd = '^';
      } else if ( dr_vertical_speed.get_memory(i) < -0.5 ) {
	v_spd = 'v';
      }

      float v_msc = dr_V_msc.get_memory(i); // 1.943844
      
      // Show airports as well?

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
  
  return 1;
}

PLUGIN_API void	XPluginStop(void) {
  // Clean up our map layer: if we created it, we should be good citizens and destroy it before the plugin is unloaded
  if(g_layer)
    {
      // Triggers the will-be-deleted callback of the layer, causing g_layer to get set back to NULL
      XPLMDestroyMapLayer(g_layer);
    }
}

PLUGIN_API int XPluginEnable(void) {
  // We want to create our layer in the standard map used in the UI (not other maps like the IOS).
  // If the map already exists in X-Plane (i.e., if the user has opened it), we can create our layer immediately.
  // Otherwise, though, we need to wait for the map to be created, and only *then* can we create our layers.
  if(XPLMMapExists(XPLM_MAP_USER_INTERFACE))
    {
      createOurMapLayer(XPLM_MAP_USER_INTERFACE, NULL);
    }
  // Listen for any new map objects that get created
  XPLMRegisterMapCreationHook(&createOurMapLayer, NULL);
  
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
PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }

void createOurMapLayer(const char * mapIdentifier, void * refcon) {
  if(!g_layer && // Confirm we haven't created our markings layer yet (e.g., as a result of a previous callback), or if we did, it's been destroyed
     !strcmp(mapIdentifier, XPLM_MAP_USER_INTERFACE)) // we only want to create a layer in the normal user interface map (not the IOS)
    {
      XPLMCreateMapLayer_t params;
      params.structSize = sizeof(XPLMCreateMapLayer_t);
      params.mapToCreateLayerIn = XPLM_MAP_USER_INTERFACE;
      params.willBeDeletedCallback = &will_be_deleted;
      params.prepCacheCallback = &prep_cache;
      params.showUiToggle = 1;
      params.refcon = NULL;
      params.layerType = xplm_MapLayer_Markings;
      params.drawCallback = &draw_markings;
      params.iconCallback = &draw_marking_icons;
      params.labelCallback = &draw_marking_labels;
      params.layerName = "Markings";
      // Note: this could fail (return NULL) if we hadn't already confirmed that params.mapToCreateLayerIn exists in X-Plane already
      g_layer = XPLMCreateMapLayer(&params);
    }
}

int s_num_cached_coords = 0;
#define MAX_COORDS (360 * 180)
float s_cached_x_coords[MAX_COORDS]; // The map x coordinates at which we will draw our icons; only the range [0, s_num_cached_coords) are valid
float s_cached_y_coords[MAX_COORDS]; // The map y coordinates at which we will draw our icons; only the range [0, s_num_cached_coords) are valid
float s_cached_lon_coords[MAX_COORDS]; // The real latitudes that correspond to our cached map (x, y) coordinates; only the range [0, s_num_cached_coords) are valid
float s_cached_lat_coords[MAX_COORDS]; // The real latitudes that correspond to our cached map (x, y) coordinates; only the range [0, s_num_cached_coords) are valid
float s_icon_width = 0; // The width, in map units, that we should draw our icons.

void prep_cache(XPLMMapLayerID layer, const float * inTotalMapBoundsLeftTopRightBottom, XPLMMapProjectionID projection, void * inRefcon)
{
  // We're simply going to cache the locations, in *map* coordinates, of all the places we want to draw.
  s_num_cached_coords = 0;
  for(int lon = -180; lon < 180; ++lon)
    {
      for(int lat =  -90; lat <  90; ++lat)
	{
	  float x, y;
	  const float offset = 0.25; // to avoid drawing on grid lines
	  XPLMMapProject(projection, lat + offset, lon + offset, &x, &y);
	  if(coord_in_rect(x, y, inTotalMapBoundsLeftTopRightBottom))
	    {
	      s_cached_x_coords[s_num_cached_coords] = x;
	      s_cached_y_coords[s_num_cached_coords] = y;
	      s_cached_lon_coords[s_num_cached_coords] = lon + offset;
	      s_cached_lat_coords[s_num_cached_coords] = lat + offset;
	      ++s_num_cached_coords;
	    }
	}
    }

  // Because the map uses true cartographical projections, the size of 1 meter in map units can change
  // depending on where you are asking about. We'll ask about the midpoint of the available bounds
  // and assume the answer won't change too terribly much over the size of the maps shown in the UI.
  const float midpoint_x = (inTotalMapBoundsLeftTopRightBottom[0] + inTotalMapBoundsLeftTopRightBottom[2]) / 2;
  const float midpoint_y = (inTotalMapBoundsLeftTopRightBottom[1] + inTotalMapBoundsLeftTopRightBottom[3]) / 2;
  // We'll draw our icons to be 5000 meters wide in the map
  s_icon_width = XPLMMapScaleMeter(projection, midpoint_x, midpoint_y) * 5000;
}

void draw_markings(XPLMMapLayerID layer, const float * inMapBoundsLeftTopRightBottom, float zoomRatio, float mapUnitsPerUserInterfaceUnit, XPLMMapStyle mapStyle, XPLMMapProjectionID projection, void * inRefcon)
{
  // The arbitrary OpenGL drawing done for our markings layer.
  // We will simply draw a green box around the icon; the icon itself will be enqueued when we get a callback to draw_marking_icons().

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

  const float half_width = s_icon_width / 2;
  const float half_height = half_width * 0.6667; // our images are in a 3:2 aspect ratio, so the height is 2/3 the width
  for(int coord = 0; coord < s_num_cached_coords; ++coord)
    {
      const float x = s_cached_x_coords[coord];
      const float y = s_cached_y_coords[coord];
      if(coord_in_rect(x, y, inMapBoundsLeftTopRightBottom))
	{
	  // Draw the box around the icon (we use half the width and height, since the icons will be *centered* at this (x, y)
	  glBegin(GL_LINE_LOOP);
	  {
	    glVertex2f(x - half_width, y + half_height);
	    glVertex2f(x + half_width, y + half_height);
	    glVertex2f(x + half_width, y - half_height);
	    glVertex2f(x - half_width, y - half_height);
	  }
	  glEnd();
	}
    }
}

void draw_marking_icons(XPLMMapLayerID layer, const float * inMapBoundsLeftTopRightBottom, float zoomRatio, float mapUnitsPerUserInterfaceUnit, XPLMMapStyle mapStyle, XPLMMapProjectionID projection, void * inRefcon)
{
  for(int coord = 0; coord < s_num_cached_coords; ++coord)
    {
      const float x = s_cached_x_coords[coord];
      const float y = s_cached_y_coords[coord];
      if(coord_in_rect(x, y, inMapBoundsLeftTopRightBottom))
	{
#define SAMPLE_IMG "Resources/plugins/map-sample-image.png"
	  if(coord % 2)
	    {
	      XPLMDrawMapIconFromSheet(
				       layer, SAMPLE_IMG,
				       0, 0, // draw the image cell at (s, t) == (0, 0) (i.e., the bottom left cell in the sample image)
				       2, 2, // our sample image is two image cell wide, and two image cells tall
				       x, y,
				       xplm_MapOrientation_Map, // Orient the icon relative to the map itself, rather than relative to the UI
				       0, // Zero degrees rotation
				       s_icon_width);
	    }
	  else
	    {
	      // Draw the image at cell (s, t) == (1, 1) (i.e., the top right cell in the sample image)
	      XPLMDrawMapIconFromSheet(layer, SAMPLE_IMG, 1, 1, 2, 2, x, y, xplm_MapOrientation_Map, 0, s_icon_width);
	    }
	}
    }
}

void draw_marking_labels(XPLMMapLayerID layer, const float * inMapBoundsLeftTopRightBottom, float zoomRatio, float mapUnitsPerUserInterfaceUnit, XPLMMapStyle mapStyle, XPLMMapProjectionID projection, void * inRefcon)
{
  if(zoomRatio >= 18) // don't label when zoomed too far out... everything will run together in a big, illegible mess
    {
      for(int coord = 0; coord < s_num_cached_coords; ++coord)
	{
	  const float x = s_cached_x_coords[coord];
	  const float y = s_cached_y_coords[coord];
	  if(coord_in_rect(x, y, inMapBoundsLeftTopRightBottom))
	    {
	      char scratch_buffer[150];
	      sprintf(scratch_buffer, "%0.2f / %0.2f Lat/Lon", s_cached_lat_coords[coord], s_cached_lon_coords[coord]);
				
	      // The text will be centered at the (x, y) we pass in. But, instead of drawing the label in the center
	      // of the icon, we'd really like the text to be shifted down *beneath* the icon we drew,
	      // so we'll subtract some amount from the y coordinate
	      const float icon_bottom = y - s_icon_width / 2;
	      const float text_center_y = icon_bottom - (mapUnitsPerUserInterfaceUnit * icon_bottom / 2); // top of the text will touch the bottom of the icon
	      XPLMDrawMapLabel(layer, scratch_buffer, x, text_center_y, xplm_MapOrientation_Map, 0);
	    }
	}
    }
}

void will_be_deleted(XPLMMapLayerID layer, void * inRefcon)
{
  if(layer == g_layer)
    g_layer = NULL;
}

// ----

