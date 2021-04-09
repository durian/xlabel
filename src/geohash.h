#ifndef _GEOHASH_H
#define _GEOHASH_H

#include <string>
#include <vector>

namespace geohash {
  
struct bounding_box {
    double minlat, minlon, maxlat, maxlon;
};

struct decoded_latlon {
    double latitude;
    double longitude;

    double latitude_err;
    double longitude_err;
};

// Encode a pair of latitude and longitude into geohash
void encode( const double latitude, const double longitude, const unsigned long precision, std::string& output );

// Encode a pair of latitude and longitude into geohash
// All Precisions from [1 to 9] (inclusive)
void encode_all_precisions( const double latitude, const double longitude, std::vector<std::string>& output );

// Encode a pair of latitude and longitude into a geohash
// All Precisions from [min to max] (inclusive)
void encode_range_precisions( const double latitude, const double longitude,
			      const size_t min, const size_t max,
			      std::vector<std::string>& output );

// Decode a hash string into pair of latitude and longitude
decoded_latlon decode( const std::string& hash_string );

// Decode hashstring into a bound box matches it
bounding_box decode_bbox( const std::string& hash_string );

// Find neighbour of a geohash string in certain direction.
// [ 1, 0] == north
// [-1,-1] == southwest, etc
std::string neighbour( const std::string& hash_string, const int direction[] );

} // namespace geohash

#endif 
