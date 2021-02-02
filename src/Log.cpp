#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <ctime>

#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "XPLMUtilities.h"

#include "Log.h"

namespace XLABEL {

  // ----------------------------------------------------------------------------
  // Code
  // ----------------------------------------------------------------------------

  Log::Log() {
    time_format = std::string("%Y%m%d %H:%M:%S");
  }
  
  void Log::xplm(const char* s) {
    std::string msg = ts()+": XLABEL: "+std::string(s);
    XPLMDebugString( msg.c_str() );
  }
  void Log::xplm(const std::string& s) {
    std::string msg = ts()+": XLABEL: "+s;
    XPLMDebugString( msg.c_str() );
  }
  
  void Log::xplm_lf() {
    XPLMDebugString( "\n" );
  }

#ifdef WIN32
  std::string Log::ts() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y%m%d@%X", &tstruct);

    return buf;
    //return "";
  }
#else
  std::string Log::ts() {    
    gettimeofday(&tv, nullptr);
    t = localtime(&tv.tv_sec);
    strftime(timestring, 32, time_format.c_str(),  t);
    
    // We only display with milli-seconds accuracy.
    //
    int msec = (tv.tv_usec + 500) / 1000;  // ms
    //int msec = (tv.tv_usec + 5000) / 10000;
    std::ostringstream ostr;
    ostr << timestring << "." << std::setfill('0') << std::setw(3) << msec; //ms
    //ostr << timestring << "." << std::setfill('0') << std::setw(4) << msec;
    return ostr.str();
  }
#endif
  Log lg;
}
