#ifndef LIDARODOMETRY_H
#define LIDARODOMETRY_H

#include "msf/commen_include.h"
#include "msf/map.h"
#include "LaserFrame.h"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace msf 
{
class LidarOdometry
{
public:
    typedef shared_ptr<LidarOdometry> Ptr;
    enum LOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };                      //a State Machine is LO
    
    LOState     state_;     // current status
    Map::Ptr    map_;       // map with all keyframes and map points
    
    LaserFrame::Ptr  ref_;       // reference frame 
    LaserFrame::Ptr  curr_;      // current frame 
    LaserFrame::Ptr  latest_key_frame_; //the latest keyframe
    PointCloud::Ptr PointMap; //the pointcloud map or submap used in NDT and PLICP
    PointCloud::Ptr second_pc;
    Eigen::Vector3d  latest_key_frame_pose_; //pose of the latest keyframe
    Eigen::Vector3d  initial_frame_pose_;    //initial frame pose
    
   
    Eigen::Vector3d pose_estimated_;    // the estimated pose of current frame 
    int num_inliers_;        // number of inlier in icp
    int num_lost_;           // number of lost times
    int key_frame_cnt_;      //count of keyframe
    int frame_cnt_;          //count of all frame
    unsigned long int mappoint_cnt; //count of mappoint

    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // minimum inliers
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames

    double max_correspondece_dist; //maximum correspondence tolerance
    double max_tolerance_error;    //maximum tolerance error of a whole frame
    double max_correspondece_dist_ba;  //maximum correspondence tolerance in constructing submap
    
public: // functions 
    LidarOdometry();
    ~LidarOdometry();
    
    bool addFrame( LaserFrame::Ptr frame,Eigen::Vector3d odom_pose, int flag,PointCloud::Ptr map);      // add a new frame, main processing 

    inline double distance_squared_d(const double a[2], const double b[2]) 
    {
        double x = a[0]-b[0];
        double y = a[1]-b[1];
        return x*x + y*y;
    }

    inline Eigen::Matrix3f euler2rot(const float x_pi,const float y_pi,const float z_pi){
    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(z_pi, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(y_pi, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(x_pi, Eigen::Vector3f::UnitX());
    return m;
    }

    inline double auxilaryFunction_PsiMT(double a, double f_a, double f_0, double g_0, double mu)
    {
        return (f_a - f_0 - mu * g_0 * a);
    }

    inline double auxilaryFunction_dPsiMT(double g_a, double g_0, double mu)
    {
        return (g_a - mu * g_0);
    }
    
    inline double circlefit(Eigen::MatrixXd input)
    {
        int n = input.rows();
        if(n<3)
        {
            return 0;
        }

        int i=0;
        double X1 = 0;
        double Y1 = 0;
        double X2 = 0;
        double Y2 = 0;
        double X3 = 0;
        double Y3 = 0;
        double X1Y1 = 0;
        double X1Y2 = 0;
        double X2Y1 = 0;
        for(int i=0;i<n;i++)
        {
            double x =  input(i,0);
            double y =  input(i,1);
            X1  += x;
            Y1  += y;
            X2  += x*x;
            Y2  += y*y;
            X3  += x*x*x;
            Y3  += y*y*y;
            X1Y1  += x*y;
            X1Y2  += x*y*y;
            X2Y1  += x*x*y;
        }

        double C,D,E,G,H,N;
        double a,b,c;

        C  =  n * X2 - X1*X1;
        D  =  n * X1Y1  - X1 * Y1;
        E   = n  * X3  +  n * X1Y2  -  (X2  +  Y2)  *  X1;
        G  =  n  *  Y2    -  Y1  *  Y1;
        H  =  n  *  X2Y1   +  n  *  Y3  -  (X2  +   Y2)  *  Y1;
        a = (H   *   D  -  E   *  G)/(C*G    -   D*D);
        b   =  (H*C   -  E*D) /(D*D  -  G  * C);
        c   =  -(a*X1  +  b*Y1  +  X2  +  Y2)/n;

        double A,B,R;
        
        A = a/(-2);
        B = b/(-2);

        R = sqrt(a*a +  b*b  -4*c)/2;

        return R;
    }

    void Frame2PointCloud(PointCloud::Ptr target, LaserFrame::Ptr source)
    {
        for(int i=0;i<source->nrays;i++)
        {
            if(!(source->valid[i])) continue;
            PointT point;
            point.x =  source->points_w[i].p[0];
            point.y =  source->points_w[i].p[1];
            point.z = 0.0;
            target->points.push_back(point);
        }
    }


    
public:  
    /*pose estimation by ndt algorithm*/
    bool poseEstimationNDT(PointCloud::Ptr cloud_target, LaserFrame::Ptr source, Eigen::Vector3d& pose_prior, double& ratio); 
    void poseEstimationNDTFrame(PointCloud::Ptr cloud_target, LaserFrame::Ptr source, Eigen::Vector3d pose_prior);

    /*add Keyframe to keyframe list and map point list of Map*/
    void addKeyFrame(); 
    /*check if a LaserFrame is Keyframe or not*/
    bool checkKeyFrame();
    /*do keyframe matching with Submap consists of some keyframes*/
    void MatchWithSubMap(LaserFrame::Ptr frame,Eigen::Vector3d pose_estimated, double ratio);
    /*pose estimaition with Submap using PLICP algorithm*/
    void poseEstPLICPWithMap(PointCloud::Ptr cloud, LaserFrame::Ptr frame, Eigen::Vector3d pose_estimated);
    void poseEstPLICPWithMapFrame(PointCloud::Ptr cloud, LaserFrame::Ptr frame, Eigen::Vector3d pose_estimated);

    Eigen::Vector3d cal_relative_pose(Eigen::Vector3d Refpose, Eigen::Vector3d Worldpose);
    Eigen::Vector3d relative2world(Eigen::Vector3d SourcePose, Eigen::Vector3d relativePose);
    double computeDerivatives(Eigen::Vector3d& gradient, Eigen::Matrix3d& hessian, PointCloud::Ptr cloud, LaserFrame::Ptr frame, Eigen::Vector3d& iter_pose,double distri_count);
    double computeDerivativesRadius(Eigen::Vector3d& gradient, Eigen::Matrix3d& hessian, PointCloud::Ptr cloud, LaserFrame::Ptr frame, Eigen::Vector3d& iter_pose, double distri_count);
    double updateIntervalMT (double &a_l, double &f_l, double &g_l,
																							double &a_u, double &f_u, double &g_u,
																							double a_t, double f_t, double g_t);
    
    double  trialValueSelectionMT (double a_l, double f_l, double g_l,
																								double a_u, double f_u, double g_u,
																								double a_t, double f_t, double g_t);

    double computeStepLengthMT(Eigen::Vector3d x,Eigen::Vector3d step_dir,double step_init,double step_max,
                                                                                                        double step_min, double error,Eigen::Vector3d gradient,Eigen::Matrix3d hessian,
                                                                                                        PointCloud::Ptr cloud,LaserFrame::Ptr frame);
    void pointCloudFilter(PointCloud::Ptr cloud);

};
}

#endif // LIDARODOMETRY_H