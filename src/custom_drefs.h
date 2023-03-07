#pragma once

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

// ----------------------------------------------------------------------------
// Class
// ----------------------------------------------------------------------------

namespace XLABEL {

  const int BYTEARRAY_SHORT = 8;
  const int BYTEARRAY_LONG = 24;
  const int MODES_ID_OFFSET  = 10000;
  
  void create_custom_drefs();
  int cdr_shade_read(void* inRefcon);
  float cdr_warp_read(void* inRefcon);
  void cdr_warp_write(void* inRefcon, float inValue);

  void unregister_custom_drefs();
}

