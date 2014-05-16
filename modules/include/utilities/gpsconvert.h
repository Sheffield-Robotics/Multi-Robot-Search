#ifndef GPSTOUTM_H
#define GPSTOUTM_H


// Definition of routines to convert between GPS (latitude/longitude) to UTM
// and vice versa.


/**
 * converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
 * East Longitudes are positive, West longitudes are negative. 
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in decimal degrees
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
void gps_LLtoUTM(double Lat, double Long, double *UTMNorthing, 
                 double *UTMEasting, char *UTMZone);

/**
 * converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in decimal degrees.
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
void gps_UTMtoLL(double UTMNorthing, double UTMEasting, const char *UTMZone,
                 double *Lat,  double *Long);

#endif
