#include "XPLMUtilities.h"

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

}
