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
#include <functional>

#include <stdlib.h>

#include "defs.h"
#include "global.h"
#include "custom_drefs.h"
#include "Log.h"
#include <string.h>

namespace XLABEL {

  //https://forums.x-plane.org/index.php?/forums/topic/33621-publish-custom-dataref-array/
  void create_custom_drefs() {

    cdr_shade = XPLMRegisterDataAccessor("durian/xlabel/shade",
					 xplmType_Int,
					 0,
					 cdr_shade_read, nullptr,
					 nullptr, nullptr,
					 nullptr, nullptr,
					 nullptr, nullptr,
					 nullptr, nullptr,
					 nullptr, nullptr,
					 nullptr, nullptr);

    cdr_warp = XPLMRegisterDataAccessor("durian/xlabel/warp",
						  xplmType_Float,
						  1,
						  nullptr, nullptr,
						  cdr_warp_read, cdr_warp_write, 
						  nullptr, nullptr,
						  nullptr, nullptr,
						  nullptr, nullptr,
						  nullptr, nullptr,
						  nullptr, nullptr);
    
    const intptr_t MSG_ADD_DATAREF = 0x01000000;
    XPLMPluginID plugin_id = XPLMFindPluginBySignature("com.leecbaker.datareftool");
    if ( XPLM_NO_PLUGIN_ID != plugin_id ) {
      lg.xplm( "Sending message to 'com.leecbaker.datareftool'.\n" );
      XPLMSendMessageToPlugin( plugin_id, MSG_ADD_DATAREF,
			       const_cast<char*>("durian/xlabel/shade"));
      XPLMSendMessageToPlugin( plugin_id, MSG_ADD_DATAREF,
			       const_cast<char*>("durian/xlabel/warp"));
    }
  }

  int cdr_shade_read(void* inRefcon) {
    return 1;
  }

  float cdr_warp_read(void* inRefcon) {
    return 1.0;
  }
  
  
  void cdr_warp_write(void* inRefcon, float inValue) {
    // G.cam_chute = inValue;
  }

  /*
  float cdr_particle_strength_read(void* inRefcon) {
    // what now
    return 0.0;
  } 
  void cdr_particle_strength_write(void* inRefcon, float inValue) {
    static XPLMCommandRef particle_strength = XPLMFindCommand("durian/xdrop/particle/strength");
    // PJB what now?
  }
  */

  // clean up
  
  void unregister_custom_drefs() {
    /*
    XPLMUnregisterDataAccessor( G.cdr_num );
    XPLMUnregisterDataAccessor( G.cdr_active );
    */
  }

}

