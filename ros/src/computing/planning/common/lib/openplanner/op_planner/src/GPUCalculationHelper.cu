#include "GPUCalculationHelper.h"

namespace PlannerHNS
{
    namespace ForGPU
    {
	__device__
	double FixNegativeAngle(const double& a)
	{
	    double angle = 0;
	    if (a < -2.0*M_PI || a > 2.0*M_PI)
		{
		    angle = fmod(a, 2.0*M_PI);
		}
	    else
		angle = a;

	    if(angle < 0)
		{
		    angle = 2.0*M_PI + angle;
		}

	    return angle;
	}


	__device__
	int GetClosestNextPointIndex(const WayPoint* trajectory,
				     const int& trajectory_length,
				     const WayPoint& p,
				     const int& prevIndex) {
	    if(trajectory_length == 0 || prevIndex < 0) return 0;

	    double d = 0, minD = 9999999999;
	    int min_index  = prevIndex;

	    for(unsigned int i = prevIndex; i < trajectory_length; i++)
		{
		    d  = distance2pointsSqr(trajectory[i].pos, p.pos);
		    if(d < minD)
			{
			    min_index = i;
			    minD = d;
			}
		}

	    if(min_index < (int)trajectory_length-2)
		{
		    GPSPoint curr, next;
		    curr = trajectory[min_index].pos;
		    next = trajectory[min_index+1].pos;
		    POINT2D v_1(p.pos.x - curr.x   ,p.pos.y - curr.y);
		    double norm1 = pointNorm(v_1);
		    POINT2D v_2(next.x - curr.x,next.y - curr.y);
		    double norm2 = pointNorm(v_2);
		    double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;
		    double a = FixNegativeAngle(acos(dot_pro/(norm1*norm2)));
		    if(a <= M_PI_2)
			min_index = min_index+1;
		}

	    return min_index;
	}


        __device__
        double AngleBetweenTwoAnglesPositive(const double& a1, const double& a2) {
            double diff = a1 - a2;
            if(diff < 0)
                diff = a2 - a1;

            if(diff > M_PI)
                diff = 2.0*M_PI - diff;

            return diff;
        }


        __device__
        bool GetRelativeInfo(const WayPoint* trajectory,
                             const int& trajectory_length,
                             const WayPoint& p,
                             RelativeInfo& info,
                             const int& prevIndex) {
            if (trajectory_length < 2) return false;

            WayPoint p0, p1;
            if (trajectory_length == 2) {
                p0 = trajectory[0];
                p1 = WayPoint((trajectory[0].pos.x + trajectory[1].pos.x) / 2.0,
                              (trajectory[0].pos.y + trajectory[1].pos.y) / 2.0,
                              (trajectory[0].pos.z + trajectory[1].pos.z) / 2.0,
                              trajectory[0].pos.a);

                info.iFront = 1;
                info.iBack = 0;

            } else {
                info.iFront = GetClosestNextPointIndex(trajectory, trajectory_length, p, prevIndex);

                if (info.iFront > 0) {
                    info.iBack = info.iFront - 1;
                } else {
                    info.iBack = 0;
                }

                if (info.iFront == 0) {
                    p0 = trajectory[info.iFront];
                    p1 = trajectory[info.iFront + 1];
                } else if (info.iFront > 0 && info.iFront < trajectory_length-1) {
                    p0 = trajectory[info.iFront - 1];
                    p1 = trajectory[info.iFront];
                } else {
                    p0 = trajectory[info.iFront - 1];
                    p1 = WayPoint((p0.pos.x + trajectory[info.iFront].pos.x) / 2.0,
                                  (p0.pos.y + trajectory[info.iFront].pos.y) / 2.0,
                                  (p0.pos.z + trajectory[info.iFront].pos.z) / 2.0,
                                  p0.pos.a);
                }
            }

            WayPoint prevWP = p0;
            Mat3 rotationMat(-p1.pos.a);

            Mat3 translationMat(-p.pos.x, -p.pos.y);
            Mat3 invRotationMat(p1.pos.a);
            Mat3 invTranslationMat(p.pos.x, p.pos.y);

            p0.pos = translationMat*p0.pos;
            p0.pos = rotationMat*p0.pos;

            p1.pos = translationMat*p1.pos;
            p1.pos = rotationMat*p1.pos;

            double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
            info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

            if(isnan(info.perp_distance) || isinf(info.perp_distance)) info.perp_distance = 0;

            info.to_front_distance = fabs(p1.pos.x); // distance on the x axes


            info.perp_point = p1;
            info.perp_point.pos.x = 0; // on the same y axis of the car
            info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

            info.perp_point.pos = invRotationMat  * info.perp_point.pos;
            info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

            // info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);
            info.from_back_distance = sqrt(pow(info.perp_point.pos.y - prevWP.pos.y, 2) + pow(info.perp_point.pos.x - prevWP.pos.x, 2));

            info.angle_diff = AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

	    return true;

        }


	__device__
	double GetExactDistanceOnTrajectory(const WayPoint* trajectory,
					    const int& trajectory_length,
                                            const RelativeInfo& p1,
                                            const RelativeInfo& p2) {
	    if (trajectory_length == 0) return 0;

	    if(p2.iFront == p1.iFront && p2.iBack == p1.iBack) {
		return p2.to_front_distance - p1.to_front_distance;
	    } else if(p2.iBack >= p1.iFront) {
		double d_on_path = p1.to_front_distance + p2.from_back_distance;
		for(unsigned int i = p1.iFront; i < p2.iBack; i++) {
		    // d_on_path += hypot(trajectory[i+1].pos.y - trajectory[i].pos.y, trajectory[i+1].pos.x - trajectory[i].pos.x);
		    d_on_path += sqrt(pow(trajectory[i+1].pos.y - trajectory[i].pos.y, 2) + pow(trajectory[i+1].pos.x - trajectory[i].pos.x, 2));
		}

		return d_on_path;
	    } else if(p2.iFront <= p1.iBack) {
		double d_on_path = p1.from_back_distance + p2.to_front_distance;
		for(unsigned int i = p2.iFront; i < p1.iBack; i++)
		    // d_on_path += hypot(trajectory[i+1].pos.y - trajectory[i].pos.y, trajectory[i+1].pos.x - trajectory[i].pos.x);
		    d_on_path += sqrt(pow(trajectory[i+1].pos.y - trajectory[i].pos.y, 2) + pow(trajectory[i+1].pos.x - trajectory[i].pos.x, 2));

		return -d_on_path;
	    } else {
		return 0;
	    }
	}
    }  // namespace ForGPU
}  // namespace PlannerHNS
