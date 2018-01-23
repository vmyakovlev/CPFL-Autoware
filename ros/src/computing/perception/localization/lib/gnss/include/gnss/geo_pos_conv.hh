#ifndef __GEO_POS_CONV__
#define __GEO_POS_CONV__

#include <string>
#include <math.h>

class geo_pos_conv {
public:
	double m_x;  //m
	double m_y;  //m
	double m_z;  //m

	double m_lat;  //latitude
	double m_lon; //longitude
	double m_h;
  
	double m_PLato;        //plane lat
	double m_PLo;          //plane lon

	std::string m_zone;
public:
	double x() const;
	double y() const;
	double z() const;
  
	void set_plane(double lat,   double lon);
	void set_plane(int num);
	void set_xyz(double cx,   double cy,   double cz);

	//set llh in radians
	void set_llh(double lat, double lon, double h);

	//set llh in nmea degrees
	void set_llh_nmea_degrees(double latd,double lond, double h);

        void llh_to_xyz(double lat, double lon, double ele);

	void conv_llh2xyz(void);
	void conv_xyz2llh(void);

	//Find conversion zones according to the longitude, earth has 60 zones. each 6 degrees represent a zone.
	std::string FindZone(const double& longitude);

	//lat ,lon are in Degrees
	void llaToxyz_proj(const double& lat, const double& lon, const double& alt, double& x_out, double& y_out, double& z_out);
	//out lat, lon in Degrees
	void xyzTolla_proj(const double& x_in, const double& y_in, const double& z_in, double& lat, double& lon, double& alt);

	void correct_gps_coor(double& lat,double& lon);
	void correct_nmea_coor(double& lat,double& lon);
};

#endif
