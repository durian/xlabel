#include <string>

#include "XPLMDataAccess.h"

#include "defs.h"
#include "dr.h"
#include "Log.h"

namespace XLABEL {

  /*
    This code really needs improving...
  */
  
  DRef::DRef( const std::string &a_path ) : path{ a_path } {
    initialised = false;
    path        = a_path;
    //init(); // lazy init
  }
  DRef::~DRef() {
    initialised = false;
#ifdef DBG
      lg.xplm( path+" deleted\n" );
#endif    
  }

  bool DRef::init() {
    if ( initialised ) {
      return true;
    }
    dref = XPLMFindDataRef( path.c_str() ); 
    if (dref == nullptr) {
#ifdef DGB
      lg.xplm( "DRef "+path+" Not Found!\n" );
#endif
    } else {
#ifdef DGB
      lg.xplm( "DRef "+path+" Found!\n" );
#endif
      if ( XPLMCanWriteDataRef(dref) ) {
	writeable = true;
      } else {
	writeable = false;
      }
      initialised = true;
    }
#ifdef DGB
    lg.xplm( "DRef["+path+"]::init("+std::to_string(initialised)+")\n" );
#endif
    return initialised;
  }

  // --------------------------------------------------------------------------
  // DRefInt
  // --------------------------------------------------------------------------
  
  DRefInt::DRefInt( const std::string &a_path ) : DRef{ a_path } {
    // parent function is ok.
  }
  
  int DRefInt::get_int() {
    if ( init() ) {
      return XPLMGetDatai( dref ); // 0 if fails... (how usefull)
    }
    return 0;
  }
  
  bool DRefInt::set_int(int val) {
    if ( init() ) {
      if ( writeable ){
	XPLMSetDatai( dref, val ); // no return value...
	return true; // only means we tried...
      }
    }
    return false; // means we can't
  }

  // --------------------------------------------------------------------------
  // DRefFloat
  // --------------------------------------------------------------------------

  DRefFloat::DRefFloat( const std::string &a_path ) : DRef{ a_path } {
    // use parent
  }
  
  float DRefFloat::get_float() {
    if ( init() ) {
      return XPLMGetDataf( dref ); // 0 if fails... (how usefull)
    }
    return 0.0;
  }
  
  bool DRefFloat::set_float(float val) {
    if ( init() ) {
      if ( writeable ){
	XPLMSetDataf( dref, val ); // no return value...
	return true; // only means we tried...
      }
    }
    return false; // means we can't
  }

  // --------------------------------------------------------------------------
  // DRefDouble
  // --------------------------------------------------------------------------

  DRefDouble::DRefDouble( const std::string &a_path ) : DRef{ a_path } {
    // use parent
  }
  
  double DRefDouble::get_double() {
    if ( init() ) {
      return XPLMGetDatad( dref ); // 0 if fails... (how usefull)
    }
    return 0.0;
  }
  
  bool DRefDouble::set_double(double val) {
    if ( init() ) {
      if ( writeable ) {
	XPLMSetDatad( dref, val ); // no return value...
	return true; // only means we tried...
      }
    }
    return false; // means we can't
  }

  // --------------------------------------------------------------------------
  // DRefIntArray
  // --------------------------------------------------------------------------

  DRefIntArray::DRefIntArray( const std::string &a_path ) : DRef{ a_path } {
    initialised = false;
    path        = a_path;
    //init(); // ours // lazy
  }
  
  DRefIntArray::~DRefIntArray() {
    if ( initialised ) {
      initialised = false;
      delete[] memory;
    }
#ifdef DBG
    lg.xplm( path+" deleted/deallocated\n" );
#endif
  }
  
  bool DRefIntArray::init() {
    //lg.xplm( "DRefIntArray["+path+"]::init("+std::to_string(initialised)+")\n" );
    if ( initialised ) {
      return true;
    }
    dref = XPLMFindDataRef( path.c_str() );
    if (dref == nullptr) {
#ifdef DBG
      lg.xplm( "DRef "+path+" Not Found!\n" );
#endif
      return false;
    } else {
#ifdef DBG
      lg.xplm( "DRef "+path+" Found!\n" );
#endif
      if ( XPLMCanWriteDataRef(dref) ) {
	writeable = true;
      } else {
	writeable = false;
      }
      initialised = true;
    }
    cur_size = std::min( XPLMGetDatavi( dref, nullptr, 0, 0 ), max_size ); // not more than allocated
#ifdef DGB
    lg.xplm( "DRefIntArray cur_size "+std::to_string(cur_size)+"\n" );
#endif
    memory = new int[max_size+1];
    initialised = true;
#ifdef DGB
    lg.xplm( "DRefIntArray["+path+"]::init("+std::to_string(initialised)+")\n" );
#endif
    return initialised;
  }
  
  int DRefIntArray::get_memory(int idx) {
    if ( init() ) {
      return memory[idx];
    }
    return 0;
  }
  
  int DRefIntArray::get_int(int idx) {
    if ( init() ) {
      // return value of get should maybe be different.... what does it return? bytes read
      int bytes_read = XPLMGetDatavi( dref, memory+idx, idx, 1 ); // idx goes to memory+idx
      // check value?
      return memory[idx];
    }
    return 0;
  }
  
  int DRefIntArray::get_all() {
    if ( init() ) {
      // update max?
      // return value of get should maybe be different.... what does it return? bytes read
      cur_size = std::min( XPLMGetDatavi( dref, nullptr, 0, 0 ), max_size ); 
      int bytes_read = XPLMGetDatavi( dref, memory, 0, cur_size ); 
      // check value?
#ifdef DBG
      lg.xplm( path+" read "+std::to_string(bytes_read)+" items\n" );
#endif
      return bytes_read;
    }
    return 0;
  }
  
  bool DRefIntArray::set_int(int val, int idx) {
    if ( init() ) {
      if ( writeable ){
	XPLMSetDatavi( dref, &val, idx, 1 ); // PJB FIXME
	return true; // only means we tried...
      }
    }
    return false; // means we can't
  }

  // --------------------------------------------------------------------------
  // DRefFloatArray
  // --------------------------------------------------------------------------

  DRefFloatArray::DRefFloatArray( const std::string &a_path ) : DRef{ a_path } {
    initialised = false;
    path = a_path;
    //init();
  }
  
  bool DRefFloatArray::init() {
    //lg.xplm( "DRefFloatArray["+path+"]::init("+std::to_string(initialised)+")\n" );
    if ( initialised ) {
      return true;
    }
    dref = XPLMFindDataRef( path.c_str() );
    if (dref == nullptr) {
#ifdef DGB
      lg.xplm( "DRef "+path+" Not Found!\n" );
#endif
      return false;
    } else {
#ifdef DGB
      lg.xplm( "DRef "+path+" Found!\n" );
#endif
      if ( XPLMCanWriteDataRef(dref) ) {
	writeable = true;
      } else {
	writeable = false;
      }
      initialised = true;
    }
    cur_size = std::min( XPLMGetDatavf( dref, nullptr, 0, 0 ), max_size ); // not more than allocated
#ifdef DGB
    lg.xplm( "DRefFloatArray cur_size "+std::to_string(cur_size)+"\n" );
#endif
    memory = new float[max_size+1];
    initialised = true;
#ifdef DGB
    lg.xplm( "DRefFloatArray["+path+"]::init("+std::to_string(initialised)+")\n" );
#endif
    return initialised;
  }
  
  float DRefFloatArray::get_memory(int idx) {
    if ( init() ) {
      return memory[idx];
    }
    return 0.0;
  }

  void* DRefFloatArray::get_memory_adr() {
    if ( init() ) {
      return &memory[0];
    }
    return nullptr;
  }
  
  float DRefFloatArray::get_float(int idx) {
    if ( init() ) {
      // return value of get should maybe be different.... what does it return? bytes read
      int bytes_read = XPLMGetDatavf( dref, memory+idx, idx, 1 ); // idx goes to memory+idx
      // check value?
      return memory[idx];
    }
    return 0;
  }
  
  int DRefFloatArray::get_all() {
    if ( init() ) {
      // update max?
      // return value of get should maybe be different.... what does it return? bytes read
      cur_size = std::min( XPLMGetDatavf( dref, nullptr, 0, 0 ), max_size ); 
      int bytes_read = XPLMGetDatavf( dref, memory, 0, cur_size );
#ifdef DBGOFF
      lg.xplm( path+" read "+std::to_string(bytes_read)+" items\n" );
#endif
      // check value?
      return bytes_read;
    }
    return 0;
  }

  int DRefFloatArray::get_all_ref(float* mem, int num) {
    if ( init() ) {
      // update max?
      // return value of get should maybe be different.... what does it return? bytes read
      cur_size = std::min( XPLMGetDatavf( dref, nullptr, 0, 0 ), max_size );
      cur_size = std::min( cur_size, num );
      int bytes_read = XPLMGetDatavf( dref, mem, 0, num );
#ifdef DBGOFF
      lg.xplm( path+" read "+std::to_string(bytes_read)+" items\n" );
#endif
      // check value?
      return bytes_read;
    }
    return 0;
  }
  
  bool DRefFloatArray::set_float(float val, int idx) {
    if ( init() ) {
      if ( writeable ){
	XPLMSetDatavf( dref, &val, idx, 1 ); // PJB FIXME
	return true; // only means we tried...
      }
    }
    return false; // means we can't
  }

  // --------------------------------------------------------------------------
  // DRef ...
  // --------------------------------------------------------------------------
}
