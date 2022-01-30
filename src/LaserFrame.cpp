#include "msf/LaserFrame.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace msf
{
LaserFrame::LaserFrame(){}

LaserFrame::~LaserFrame()
{

}

void LaserFrame::Create(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    
    nrays = scan->intensities.size();

	theta = new double[nrays];
	valid = new int[nrays];
	readings = new double[nrays];
	points = new point2d[nrays]; 
	points_w = new point2d[nrays];
	corr = new correspondence[nrays];
	submap = p_submap.makeShared();

    for(int i = 0;i < nrays;i++)
    {
        double dist = scan->ranges[i];
        if(dist > 0.1 && dist < 30)
        {
            valid[i] = 1;
            readings[i] = dist;
        }
        else
        {
            valid[i] = 0;
            readings[i] = -1;
        }
        theta[i] = scan->angle_min+scan->angle_increment*i;
    }
    time_stamp = scan->header.stamp;
	corr_info = 0.0;
	is_key_frame_ = false;
	is_loop_frame_ = false;
    
}

LaserFrame::Ptr LaserFrame::createFrame()
{
    static long factory_id = 0;
    return LaserFrame::Ptr( new LaserFrame() );
}

void LaserFrame::compute_cartesian() 
{
	int i;
	for(i=0;i<nrays;i++) 
	{
		if(!valid[i]) continue;
		double x = cos(theta[i])*readings[i];
		double y = sin(theta[i])*readings[i];

		if(x<0)
		{
			valid[i] = 0;
            readings[i] = -1;
			continue;
		}
		
		points[i].p[0] = x, 
		points[i].p[1] = y;
		points[i].rho = sqrt( x*x+y*y);
		points[i].phi = atan2(y, x);
	}
}

void LaserFrame::compute_world_coords(Eigen::Vector3d pose) 
{
	double pose_x = pose(0);
	double pose_y = pose(1);
	double pose_theta = pose(2);
	double cos_theta = cos(pose_theta); 
	double sin_theta = sin(pose_theta);

	//point2d * points = points;
	//point2d * points_w = points_w;
	int i; for(i=0;i<nrays;i++) {
		if(!valid[i]) continue;
		double x0 = points[i].p[0], 
		       y0 = points[i].p[1]; 
		
		if(isnan(x0) || isnan(y0)) {
			ROS_INFO("compute_world_coords(): I expected that cartesian coords were already computed: ray #%d: %f %f.\n", i, x0, y0);
		}
		
		points_w[i].p[0] = cos_theta * x0 - sin_theta*y0 + pose_x;
		points_w[i].p[1] = sin_theta * x0 + cos_theta*y0 + pose_y;
		/* polar coordinates */
	}
	
	for(i=0;i<nrays;i++) {
		double x = points_w[i].p[0];
		double y = points_w[i].p[1];
		points_w[i].rho = sqrt( x*x+y*y);
		points_w[i].phi = atan2(y, x);
	}
	
}
void LaserFrame::set_true_pose(Eigen::Vector3d pose)
{
    true_pose(0) = pose(0);
    true_pose(1)  = pose(1);
    true_pose(2)  = pose(2);
}

void LaserFrame::set_odometry(Eigen::Vector3d pose)
{
    odometry(0)  = pose(0);
    odometry(1)  = pose(1);
    odometry(2)  = pose(2);
}

void LaserFrame::set_estimate(Eigen::Vector3d pose)
{
    estimate(0)  = pose(0);
    estimate(1)  = pose(1);
    estimate(2)  = pose(2);
}

void LaserFrame::set_corr_null(int i)
{
	corr[i].valid = 0;
	corr[i].j1 = -1;
	corr[i].j2 = -1;
	corr[i].dist2_j1 = -1.0;
	corr[i].succ_cnt = -1;
}

void LaserFrame::inc_succ_cnt(int i)
{
	corr[i].succ_cnt +=  1;
}

void LaserFrame::set_correspondence(int i, int j, int k, double dist)
{
	corr[i].valid = 1;
	corr[i].j1 = j;
	corr[i].j2 = k;
	corr[i].dist2_j1 = dist;
}
/*
void LaserFrame::possible_interval(const double *p_i_w,double max_angular_correction_deg, double max_linear_correction, int*from, int*to, int*start_cell)
{
	double angle_res = (max_theta-min_theta)/nrays;
	double delta = fabs(deg2rad(max_angular_correction_deg))+
					fabs(atan(max_linear_correction/norm_d(p_i_w)));
	int range = (int) ceil(delta/angle_res);

	double start_theta = atan2(p_i_w[1],p_i_w[1]);

	if(start_theta<min_theta) start_theta += 2*M_PI;
	if(start_theta>max_theta) start_theta -= 2*M_PI;

	*start_cell  = (int)
		((start_theta - min_theta) / (max_theta-min_theta) * nrays);

	*from = minmax(0,nrays-1, *start_cell-range).first;
	*to =   minmax(0,nrays-1, *start_cell+range).second;
}
*/

}