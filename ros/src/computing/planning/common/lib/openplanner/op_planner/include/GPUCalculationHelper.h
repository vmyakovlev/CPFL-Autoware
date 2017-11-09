#ifndef GPUCALCULATIONHELPERS_H_
#define GPUCALCULATIONHELPERS_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <math.h>
#include <algorithm>

#include "RoadNetwork.h"

namespace PlannerHNS {
    namespace ForGPU {

        /*
         * Porting classes used in trajectory cost calculation
         */
#define distance2pointsSqr(from , to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)
#define RAD2DEG 180. / M_PI

        enum CAR_TYPE
        {
            Mv2Car, //!< Mv2Car
            PHVCar, //!< PHVCar
            HVCar,  //!< HVCar
            RoboCar,//!< RoboCar
            SimulationCar
        };

        class POINT2D
        {
        public:
            double x;
            double y;
            double z;

            __device__
            POINT2D()
                {
                    x=0;y=0;z=0;
                }

            __device__
            POINT2D(double px, double py, double pz = 0)
                {
                    x = px;
                    y = py;
                    z = pz;
                }
        };  // class POINT2D

        class GPSPoint
        {
        public:
            double lat, x;
            double lon, y;
            double alt, z;
            double dir, a;

            __host__ __device__
            GPSPoint() {
		lat = x = 0;
		lon = y = 0;
		alt = z = 0;
		dir = a = 0;
            }

            __host__ __device__
            GPSPoint(const double& x, const double& y, const double& z, const double& a) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->a = a;

		lat = 0;
		lon = 0;
		alt = 0;
		dir = 0;
            }

            GPSPoint operator = (const PlannerHNS::GPSPoint& p) {
                lat = p.lat;
                x = p.x;
                lon = p.lon;
                y = p.y;
                alt = p.alt;
                z = p.z;
                dir = p.dir;
                a = p.a;

                return *this;
            }

        };  // class GPSPoint


        class Rotation
        {
        public:
            double x;
            double y;
            double z;
            double w;

            __device__
            Rotation() {
                x = 0;
                y = 0;
                z = 0;
                w = 0;
            }
        };  // class Rotation


        class WayPoint
        {
        public:
            GPSPoint	pos;
            /* Rotation 	rot; */
            double  	v;
            /* double  	cost; */
            /* double  	timeCost; */
            /* double  	totalReward; */
            /* double  	collisionCost; */
            /* double 		laneChangeCost; */
            /* int 		laneId; */
            /* int 		id; */
            /* int 		LeftLaneId; */
            /* int 		RightLaneId; */
            /* int 		stopLineID; */

            __host__ __device__
            WayPoint() {
                /*         id = 0; */
                        v = 0;
                /*         cost = 0; */
                /*         laneId = -1; */
                /*         LeftLaneId = 0; */
                /*         RightLaneId = 0; */
                /*         timeCost = 0; */
                /*         totalReward = 0; */
                /*         collisionCost = 0; */
                /*         laneChangeCost = 0; */
                /*         stopLineID = -1; */
            }

            __host__ __device__
            WayPoint(const double& x, const double& y, const double& z, const double& a) {
                pos.x = x;
                pos.y = y;
                pos.z = z;
                pos.a = a;

                /* id = 0; */
                v = 0;
                /* cost = 0; */
                /* laneId = -1; */
                /* LeftLaneId = 0; */
                /* RightLaneId = 0; */
                /* timeCost = 0; */
                /* totalReward = 0; */
                /* collisionCost = 0; */
                /* laneChangeCost = 0; */
                /* stopLineID = -1; */
            }

            WayPoint operator = (const PlannerHNS::WayPoint& p) {
                pos.x = p.pos.x;
                pos.y = p.pos.y;
                pos.z = p.pos.z;
                pos.a = p.pos.a;
                v = p.v;
                return *this;
            }
        };  // class WayPoint


        class RelativeInfo
        {
        public:
            double perp_distance;
            double to_front_distance; //negative
            double from_back_distance;
            int iFront;
            int iBack;
            int iGlobalPath;
            WayPoint perp_point;
            double angle_diff; // degrees

            __device__
            RelativeInfo()
                {
                    perp_distance = 0;
                    to_front_distance = 0;
                    from_back_distance = 0;
                    iFront = 0;
                    iBack = 0;
                    iGlobalPath = 0;
                    angle_diff = 0;
                }
        };  // class RelativeInfo


        class Mat3
        {
            double m11, m12, m13;
            double m21, m22, m23;
            double m31, m32, m33;

            double m[3][3];

        public:
            __device__
            Mat3() {
                //initialize Identity by default
                m11 = m22 = m33 = 1;
                m12 = m13 = m21 = m23 = m31 = m32 = 0;
            }

            __device__
            Mat3(double angle, POINT2D trans) {
                //Initialize Rotation Matrix
                double c = cos(angle);
                double s = sin(angle);
                m11 = c;
                m12 = s;
                m21 = -s;
                m22 = c;
                m31 = trans.x;
                m32 = trans.y;
                m13 = m23= 0;
                m33 = 1;
            }

            __device__
            Mat3(double transX, double transY, bool mirrorX, bool mirrorY ) {
                m11 = m22 = m33 = 1;
                m12 = m13 = m21 = m23 = m31 = m32 = 0;
                m[0][0] = (mirrorX == true ) ? -1 : 1; m[0][1] =  0; m[0][2] =  transX;
                m[1][0] = 0; m[1][1] =  (mirrorY==true) ? -1 : 1; m[1][2] =  transY;
                m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
            }

            __device__
            Mat3(double transX, double transY) {
                    m11 = m22 = m33 = 1;
                    m12 = m13 = m21 = m23 = m31 = m32 = 0;
                    m[0][0] = 1; m[0][1] =  0; m[0][2] =  transX;
                    m[1][0] = 0; m[1][1] =  1; m[1][2] =  transY;
                    m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
                }

            __device__
            Mat3(double rotation_angle) {
                m11 = m22 = m33 = 1;
                m12 = m13 = m21 = m23 = m31 = m32 = 0;
                double c = cos(rotation_angle);
                double s = sin(rotation_angle);
                m[0][0] = c; m[0][1] = -s; m[0][2] =  0;
                m[1][0] = s; m[1][1] =  c; m[1][2] =  0;
                m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
            }

            __device__
            Mat3(GPSPoint rotationCenter) {
                m11 = m22 = m33 = 1;
                m12 = m13 = m21 = m23 = m31 = m32 = 0;
                double c = cos(rotationCenter.a);
                double s = sin(rotationCenter.a);
                double u = rotationCenter.x;
                double v = rotationCenter.y;
                m[0][0] = c; m[0][1] = -s; m[0][2] = -u*c + v*s + u;
                m[1][0] = s; m[1][1] =  c; m[1][2] = -u*s - v*c + v;
                m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
            }


            __device__
            GPSPoint operator * (GPSPoint v) {
                GPSPoint _v = v;
                v.x = m[0][0]*_v.x + m[0][1]*_v.y + m[0][2]*1;
                v.y = m[1][0]*_v.x + m[1][1]*_v.y + m[1][2]*1;
                return v;
            }

            /* POINT2D operator * (POINT2D v) */
            /*     { */
            /*         Mat3 m = *this; */
            /*         POINT2D r; */
            /*         r.x = m.m11 * v.x + m.m21 * v.y + m.m31 * 1; */
            /*         r.y = m.m12 * v.x + m.m22 * v.y + m.m32 * 1; */
            /*         return r; */
            /*     } */

            __device__
            Mat3 operator *(Mat3 m2) {
                Mat3 m1 = *this;
                Mat3 r;
                r.m11 = m1.m11 * m2.m11 + m1.m12 * m2.m21 + m1.m13 * m2.m31;
                r.m12 = m1.m11 * m2.m12 + m1.m12 * m2.m22 + m1.m13 * m2.m32;
                r.m13 = m1.m11 * m2.m13 + m1.m12 * m2.m23 + m1.m13 * m2.m33;

                r.m21 = m1.m21 * m2.m11 + m1.m22 * m2.m21 + m1.m23 * m2.m31;
                r.m22 = m1.m21 * m2.m12 + m1.m22 * m2.m22 + m1.m23 * m2.m32;
                r.m23 = m1.m21 * m2.m13 + m1.m22 * m2.m23 + m1.m23 * m2.m33;

                r.m31 = m1.m31 * m2.m11 + m1.m32 * m2.m21 + m1.m33 * m2.m31;
                r.m32 = m1.m31 * m2.m12 + m1.m32 * m2.m22 + m1.m33 * m2.m32;
                r.m33 = m1.m31 * m2.m13 + m1.m32 * m2.m23 + m1.m33 * m2.m33;

                return r;
            }
        };  // class Mat3


        class PolygonShape
        {
        public:
            __device__
            /* std::vector<GPSPoint> points; */
            static inline int PointInsidePolygon(const GPSPoint* polygon,
                                                 const int num_of_points,
                                                 const GPSPoint& p) {
		int counter = 0;
                int i;
                double xinters;
                GPSPoint p1,p2;
                int N = num_of_points;
                if(N <=0 ) return -1;

                p1 = polygon[0];
                for (i=1;i<=N;i++) {
                    p2 = polygon[i % N];

                    if (p.y > MIN(p1.y,p2.y)) {
                        if (p.y <= MAX(p1.y,p2.y)) {
                            if (p.x <= MAX(p1.x,p2.x)) {
                            if (p1.y != p2.y) {
                                xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                                if (p1.x == p2.x || p.x <= xinters)
                                    counter++;
                            }
		        }
		      }
		    }
		    p1 = p2;
                }

                if (counter % 2 == 0)
		    return 0;
                else
		    return 1;
            }
        };  // class PolygonShape


        __device__
        bool GetRelativeInfo(const WayPoint* trajectory,
                             const int& trajectory_length,
                             const WayPoint& p,
                             ForGPU::RelativeInfo& info,
                             const int& prevIndex = 0);

        __device__
        double GetExactDistanceOnTrajectory(const WayPoint* trajectory,
                                            const int& trajectory_length,
                                            const RelativeInfo& p1,
                                            const RelativeInfo& p2);

    } /* namespace ForGPU */
} /* namespace PlannerHNS */

#endif /* GPUCALCULATIONHELPERS_H_ */
