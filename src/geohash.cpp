#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <map>

#include "geohash.h"

namespace geohash {

// Static array of 0-9, a-z
const char base32_chars[] = {
  '0',  '1',  '2',  '3',  '4',  '5',  '6',  '7',
  '8',  '9',  'b',  'c',  'd',  'e',  'f',  'g',
  'h',  'j',  'k',  'm',  'n',  'p',  'q',  'r',
  's',  't',  'u',  'v',  'w',  'x',  'y',  'z'
};

// Build a map of characters -> index position from the above array
const std::map<char, int> build_base32_indexes();
const std::map<char, int> base32_indexes = build_base32_indexes();

// Reverse map of characters --> index position
const std::map<char, int> build_base32_indexes() {
  std::map<char, int> output;
  
  int i = 0;
  for ( char c : base32_chars ) {
    output.insert( std::pair<char, int>(c, i) );
    ++i;
  }
  return output;
}

// Convert the index position to the character in the array
char base32_chars_value_of(int index) {
    return base32_chars[index];
}

// Convert the character to the index position in the array
int base32_chars_index_of(char c) {
    return base32_indexes.find(c)->second;
}

void encode( const double latitude, const double longitude, unsigned long precision, std::string& output ) {
  // bounding_box for the lat/lon + errors
  bounding_box bbox;
  bbox.maxlat = 90;
  bbox.maxlon = 180;
  bbox.minlat = -90;
  bbox.minlon = -180;
  double mid        = 0;
  bool   islon      = true;
  int    num_bits   = 0;
  int    hash_index = 0;
  
  // Pre-allocate the hash string
  output = std::string( precision, ' ' );
  unsigned int output_length = 0;
  
  while ( output_length < precision ) {
    if ( islon ) {
      mid = (bbox.maxlon + bbox.minlon) / 2;
      if ( longitude > mid ) {
	hash_index = (hash_index << 1) + 1;
	bbox.minlon=mid;
      } else {
	hash_index = (hash_index << 1) + 0;
	bbox.maxlon=mid;
      }
    } else {
      mid = (bbox.maxlat + bbox.minlat) / 2;
      if ( latitude > mid ) {
	hash_index = (hash_index << 1) + 1;
	bbox.minlat = mid;
      } else {
	hash_index = (hash_index << 1) + 0;
	bbox.maxlat = mid;
      }
    }
    islon = !islon;
    
    ++num_bits;
    if ( num_bits == 5 ) {
      // Append the character to the pre-allocated string
      // This gives us roughly a 2x speed boost
      output[output_length] = base32_chars[hash_index];
      
      output_length++;
      num_bits   = 0;
      hash_index = 0;
    }
  }
}
  
// Encode a pair of latitude and longitude into a geohash
// All precisions from [1 to 9] (inclusive)
void encode_all_precisions( const double latitude, const double longitude, std::vector<std::string> & output ) {
  encode_range_precisions( latitude, longitude, 1, 9, output );
}

// Encode a pair of latitude and longitude into geohash
// All Precisions from [min to max] (inclusive)
void encode_range_precisions( const double latitude, const double longitude, const size_t min, const size_t max,
			     std::vector<std::string> & output ) {
  const size_t num_precisions = max - min + 1;
  output.resize( num_precisions );

  std::string buffer;
  encode( latitude, longitude, max, buffer );
  
  // Set the "end" value
  output[num_precisions - 1] = buffer;
  
  for ( int i = num_precisions - 2; i >= 0; --i ) {
    const std::string & last = output[i+1];
    output[i] = last.substr( 0, last.length()-1 );
  }
}

bounding_box decode_bbox( const std::string& _hash_string ) {
  // Copy of the string down-cased
  std::string hash_string(_hash_string);
  std::transform( _hash_string.begin(),
		  _hash_string.end(),
		  hash_string.begin(),
		  ::tolower );
  
  bounding_box output;
  output.maxlat =   90;
  output.maxlon =  180;
  output.minlat =  -90;
  output.minlon = -180;
  
  bool islon = true;
  
  for ( int i = 0, max = hash_string.length(); i < max; i++ ) {
    int char_index = base32_chars_index_of( hash_string[i] );
    
    for ( int bits = 4; bits >= 0; --bits ) {
      int bit = (char_index >> bits) & 1;
      if ( islon ) {
	double mid = (output.maxlon + output.minlon) / 2;
	if ( bit == 1 ) {
	  output.minlon = mid;
	} else {
	  output.maxlon = mid;
	}
      } else {
	double mid = (output.maxlat + output.minlat) / 2;
	if ( bit == 1 ) {
	  output.minlat = mid;
	} else {
	  output.maxlat = mid;
	}
      }
      islon = !islon;
    }
  }
  return output;
}

decoded_latlon decode( const std::string& hash_string ) {
  bounding_box   bbox = decode_bbox(hash_string);
  decoded_latlon output;
  output.latitude      = (bbox.minlat + bbox.maxlat) / 2;
  output.longitude     = (bbox.minlon + bbox.maxlon) / 2;
  output.latitude_err  = bbox.maxlat - output.latitude;
  output.longitude_err = bbox.maxlon - output.longitude;
  return output;
}

std::string neighbour( const std::string& hash_string, const int direction [] ) {
  // Adjust the decoded_latlon for the direction of the neighbours
  decoded_latlon lonlat = decode( hash_string );
  lonlat.latitude   += direction[0] * lonlat.latitude_err * 2;
  lonlat.longitude  += direction[1] * lonlat.longitude_err * 2; // was 2, with 4 more square (?) PJB
  
  std::string output;
  encode( lonlat.latitude, lonlat.longitude, hash_string.length(), output );
  return output;
}

} // end namespace geohash

