#ifndef LASERFRAME_H
#define LASERFRAME_H

#include "msf/commen_include.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace msf 
{
    


typedef struct {
	double p[2];
	double rho, phi;
} point2d;

struct correspondence {
	// 1 if this correspondence is valid  
	int valid; 
	// Closest point in the other scan.
	int j1;
	// Second closest point in the other scan. 
	int j2;
	// Type of correspondence (point to point, or point to line
	enum { corr_pp = 0, corr_pl = 1} type;
	// Squared distance from p(i) to point j1 
	double dist2_j1; 
    //successful correspondence count
    int succ_cnt;
};


class LaserFrame
{
public:
    typedef std::shared_ptr<LaserFrame> Ptr;
    unsigned long                  id_;             //id in global sequence
    unsigned long                  keyframe_id_;    //id in keyframe sequence    
    int                            nrays;           //number of rays in a laser frame
    double                         min_theta;       //minimum angle in a laser frame
    double                         max_theta;       //maximum angle in a laser frame
    double*                         theta;          //array of angle in a laser frame 
    int*                            valid;          //we have some constraits to filter the point
    double*                         readings;       //range of a laser frame

    double                          corr_info;

    ros::Time                      time_stamp;      //time stamp of the laser frame

    Eigen::Vector3d                true_pose;       //true pose
    Eigen::Vector3d                odometry;        //odometry pose
    Eigen::Vector3d                estimate;        //estimated pose
    bool                           is_key_frame_;   //1=keyframe, 0=not keyframe
    bool                           is_loop_frame_;   //1=keyframe, 0=not keyframe

    /** Cartesian representation */
	point2d*                        points;         //points in laser coordinate
	
	point2d*                        points_w;       //points in coordinate transformed by true pose

    struct correspondence*           corr;          //the correspondence situation

    std::vector<LaserFrame::Ptr>     connected_frames_; //if frame is keyframe, then it has a list of connected trival frames

    PointCloud p_submap; //consists of points from the trival frames connected with this keyframe
    PointCloud::Ptr submap; //consists of points from the trival frames connected with this keyframe
    
public: // data members 
    LaserFrame();
    ~LaserFrame();
    
    static LaserFrame::Ptr createFrame(); //create a new LaserFrame, empty
    void Create(const sensor_msgs::LaserScan::ConstPtr& scan); //create a new LaaserFrame from a laser scan
    void insertConnectedFrame( LaserFrame::Ptr frame );  //add trival frame to connected_frames_ of keyframe
    
    void compute_cartesian();        //fullfill the array points of a frame
    
    
    void compute_world_coords(Eigen::Vector3d pose); //fullfill the array points_w of a frame
    
    void set_true_pose(Eigen::Vector3d pose); //set true pose
    void set_odometry(Eigen::Vector3d pose);  //set odometry pose
    void set_estimate(Eigen::Vector3d pose);  //set estimated pose

    void set_corr_null(int i);                //set null correspondence
    void set_correspondence(int i, int j, int k, double dist); //set corrected correspondence
    void inc_succ_cnt(int i);
    //void possible_interval(const double *p_i_w,double max_angular_correction_deg, double max_linear_correction, int*from, int*to, int*start_cell);

    inline double deg2rad(double deg) {
	    return deg * (M_PI / 180);
    }
    inline double norm_d(const double p[2]) {
	    return sqrt(p[0]*p[0]+p[1]*p[1]);
    }
};

}

#endif
