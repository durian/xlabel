#ifndef _LOG_H
#define _LOG_H

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

// C includes
//
#ifdef APL
#include <sys/time.h>
#include <unistd.h>
#endif
#ifdef LIN
#include <sys/time.h>
#include <unistd.h>
#endif
//     IBM?
#ifdef WIN32
#include <windows.h>
#endif


// ----------------------------------------------------------------------------
// Class
// ----------------------------------------------------------------------------

namespace XLABEL {

  class Log {
  private:
    char   timestring[32];
    struct tm *t;
    struct timespec time_now = {0}; // https://github.com/janestreet/core/issues/102
    std::string time_format;
    timeval   tv;
    //std::ostringstream ostr;

  public:
    
    // Constructor.
    Log();
    
    // Destructor.
    ~Log() { };

    void xplm(const char*);
    void xplm(const std::string&);
    void xplm_lf();
    std::string ts();
  };
  extern Log lg;
}

#endif
