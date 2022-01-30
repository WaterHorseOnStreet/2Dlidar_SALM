#include "msf/commen_include.h"
#include "sensor_msgs/LaserScan.h"
#include "msf/LaserFrame.h"
#include "msf/lidar_odometry.h"
#include "msf/config.h"
#include "nav_msgs/OccupancyGrid.h"
#include <thread>
#include <ctime>
#include <cstdlib>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "msf/config.h"
#include "msf/lidar_odometry.h"
#include "msf/backend/gaussian_newton.h"

#include "msf/Odom_Calib.h"


using namespace msf;
using namespace cv;


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

clock_t start_frame;
clock_t end_frame;

//frame defination
std::string odom_frame_;
std::string base_frame_;

//odometry data processing
bool init_imu = true;
int imu_init_num = 0;
double imu_yaw_bias =0.0;
double sum_yaw = 0.0;

bool init_wheel = true;
double last_imu_t = 0;
double last_wheel_t =0;

int loop_flag = 0;

msf::LidarOdometry::Ptr LO(new msf::LidarOdometry);
PointCloud::Ptr pointCloud(new PointCloud);
PointCloud::Ptr tmp(new PointCloud);

PointCloud::Ptr pC(new PointCloud);
PointCloud::Ptr pC_tmp(new PointCloud);

PointCloud::Ptr mymap(new PointCloud);

std::mutex m_odom;

Eigen::Vector3d rel2map({0,0,0});
Eigen::Vector3d odom_pose_prev({0,0,0});
Eigen::Vector3d update_pose;

double dis_lidar_odom_x = 0.3;
double dis_lidar_odom_y = 0.0;
double dis_lidar_odom = 0.3;

//state of robot in the frame of baselink
double robot_x = (-1)*dis_lidar_odom_x;
double robot_y =  0.0;
double robot_yaw = 0.0;

//state of robot without scan 
double raw_robot_x = (-1)*dis_lidar_odom_x;
double raw_robot_y = 0.0;
double raw_robot_yaw = 0.0;

//state of robot in the frame of scan
double scan_x = 0.0;
double scan_y = 0.0;
double scan_yaw = 0.0;
bool match_success1 =false;
bool match_success2 =false;

//several publisher
ros::Publisher pub_odom;
ros::Publisher pub_raw_odom;
ros::Publisher pcl_pub;
ros::Publisher pub_keyframe;

ros::Time current_time;

cv::Mat line_img = cv::Mat::zeros(400,400,CV_32FC1);
cv::Mat line_img1 = cv::Mat::zeros(400,400,CV_32FC1);

OdomCalib Odom_calib;
bool calib_f  = true; 
std::ofstream calib_result;
bool first_loop = true;
Eigen::Vector3d latest_loop_odom({0,0,0});

int laser_freq = 0;

//QrCode elements
std::map<string, int> qr_codeMap;
std::vector<Eigen::Vector3d> loop_chain;
int loop_count = 0;
int qr_freq  = 2;
int qr_msg_count = 0;

//wheel odometer calibration params
double last_sample_time = 0.0;
double init_sample_time = 0.0;
int calib_count = 0;

Eigen::Vector3d speed_time({0,0,0});
Eigen::Vector3d last_speed_time({0,0,0});
bool first_scan = true;

Eigen::Vector3d obser4calib({0,0,0});
Eigen::Vector3d last_obser4calib({0,0,0});

std::vector<Eigen::Matrix<double,2,4>> calib_mat;
 Eigen::Matrix<double,2,4> calib_matrix;

std::ofstream wheel_calib;

std::ofstream fusion_result;

double coff_left = 2.0*63.5639/125.0;
double coff_right = 2.0*63.2035/125.0;
double robot_x_psuo = (-1)*dis_lidar_odom_x;
double robot_y_psuo = 0.0;
double robot_yaw_psuo = 0.0;


double distance_eigen(Eigen::Vector3d a, Eigen::Vector3d b)
{
    return sqrt((a(0)-b(0))*(a(0)-b(0)) + (a(1)-b(1))*(a(1)-b(1)) + (a(2)-b(2))*(a(2)-b(2)) );
}

double circlefit(Eigen::MatrixXd input)
{
	int n = input.rows();
	if(n<3)
	{
		return false;
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

void feature_extractor(PointCloud::Ptr tmp, cv::Mat cartisian_ftr)
{
/*
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(tmp);

  const int K = 15;
  std::vector<int> pointsIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDist(K);
  std::vector<std::vector<double>> corner;
  int line_count = 0;
  int corner_count = 0;
*/

	std::vector<std::vector<int>> indices(6400);
  for(int i =0;i<tmp->points.size();i++)
	{
        double x0 = tmp->points[i].x;
        double y0 = tmp->points[i].y;
/*
        double theta_rad = atan2(y0,x0);
        int theta_angle = floor(180*(theta_rad + M_PI)/M_PI);

        double radius = sqrt(x0*x0 + y0*y0);
        int radius_index = floor(radius*4);


				if(theta_angle<0 & theta_angle>180) continue;
				if(radius_index<80) polar_ftr.ptr<double>(radius_index)[theta_angle] += 1.0 ;
*/
				int x_index = floor((x0 + 20)*2);
				int y_index = floor((y0 + 20)*2);
				if(x_index < 80 & !(x_index <0) & y_index<80 & !(y_index<0)) indices[x_index*80 + y_index].push_back(i);


        // 	Eigen::Matrix<double,K,2> input;
        //     input.setZero();

        // 	if(kdtree.nearestKSearch(tmp->points[i], K, pointsIdxKNNSearch, pointKNNSquaredDist)>0)
        // 	{
        // 		for(int i=0;i<K;i++)
        // 		{
        // 			input(i,0) = tmp->points[pointsIdxKNNSearch[i]].x;
        // 			input(i,1) = tmp->points[pointsIdxKNNSearch[i]].y;
        // 		}

        // 		//std::cout<<input.size()<<std::endl;
        // 		Eigen::MatrixXd meanVec = input.colwise().mean();
        // 		Eigen::RowVectorXd meanVectorRow(Eigen::RowVectorXd::Map(meanVec.data(),input.cols()));
        // 		// std::cout<<meanVec<<std::endl;
        // 		// std::cout<<meanVectorRow<<std::endl;

        // 		Eigen::MatrixXd zeroMeanMat = input;
        // 		zeroMeanMat.rowwise() -= meanVectorRow;
        // 		// std::cout<<zeroMeanMat<<std::endl;
        // 		Eigen::MatrixXd covMat ;

        // 		if(input.rows() == 1)
        // 		{
        // 			covMat = (zeroMeanMat.adjoint()*zeroMeanMat)/double(input.rows());
        // 		}
        // 		else
        // 		{
        // 			covMat = (zeroMeanMat.adjoint()*zeroMeanMat)/double(input.rows() - 1);
        // 		}

        // 		Eigen::EigenSolver<Eigen::Matrix2d> eigen_solver(covMat);
        // 		double first_ev = eigen_solver.eigenvalues()[0].real();
        // 		double second_ev = eigen_solver.eigenvalues()[1].real();
        // 		 //std::cout<<first_ev<<"   "<<second_ev<<std::endl;
        // 		Eigen::Vector2d line_dirct;
        // 		double first = max(first_ev, second_ev);
        // 		double second = min(first_ev, second_ev);

        // 		if(floor(first/second)>9)
        // 		{
        // 				line_dirct = (first == first_ev)?  eigen_solver.eigenvectors().col(0).real() : eigen_solver.eigenvectors().col(1).real();
        // 				line_dirct.normalize();
        // 				double theta_rad = atan2(line_dirct(1),line_dirct(0));
        // 				int theta_angle = floor(180*(theta_rad + M_PI)/M_PI);
        // 				if(theta_angle<0 & theta_angle>180) continue;
        // 				if(line_count<600) line_ftr.ptr<double>(theta_angle)[line_count] = 255 ;
                        
        // 				line_count++;
        // 		}
        // 		else
        // 		{
        // 				Eigen::Vector2d center;
        // 				double rs;

        // 				rs = circlefit(input.block<6,2>(0,0));
        //                 if(rs <1e-6) continue;
        // 				double curvature = 1/rs;

        // 				//std::cout<<curvature<<std::endl;
                        
        // 				if(curvature >4)
        // 				{
        // 					int curvature_index = floor((curvature-4)*4);

        // 					// std::cout<<curvature_index<<std::endl;
        // 					// std::cout<<corner_count<<std::endl;
        // 					if(curvature_index > 160) continue;
        // 					if(corner_count<100) corner_ftr.ptr<double>(corner_count)[curvature_index] = 255;

        // 					corner_count++;

        // 				}
        // 				else
        // 				{
        //                     continue;						
        // 				}					
        // 		}
        // 	}
        // 	else
        // 	{
        // 	  continue;
        // 	}
	
	}

	for(int i=0;i<indices.size();i++)
	{
		int col = (int)(i%80);
		int row = (int)(i/80);

		if(indices[i].size() < 5) continue;

		const int K = indices[i].size();
		Eigen::MatrixXd input;
		input.resize(K,2);

		for(int j=0; j< indices[i].size();j++)
		{
			int p_index = indices[i][j];
			input(j,0) = tmp->points[p_index].x;
			input(j,1) = tmp->points[p_index].y;
		}
		
		Eigen::MatrixXd meanVec = input.colwise().mean();
		Eigen::RowVectorXd meanVectorRow(Eigen::RowVectorXd::Map(meanVec.data(),input.cols()));
		Eigen::MatrixXd zeroMeanMat = input;
		zeroMeanMat.rowwise() -= meanVectorRow;

		Eigen::MatrixXd covMat ;
		if(input.rows() == 1)
		{
			covMat = (zeroMeanMat.adjoint()*zeroMeanMat)/double(input.rows());
		}
		else
		{
			covMat = (zeroMeanMat.adjoint()*zeroMeanMat)/double(input.rows() - 1);
		}

		Eigen::EigenSolver<Eigen::Matrix2d> eigen_solver(covMat);
		double first_ev = eigen_solver.eigenvalues()[0].real();
		double second_ev = eigen_solver.eigenvalues()[1].real();

		double first = max(first_ev, second_ev);
		double second = min(first_ev, second_ev);
		if(floor(first/second)>10)
		{
			cartisian_ftr.ptr<double>(row)[col] = 128;
		}
		else 
		{
					double rs;
					rs = circlefit(input);
					double curvature = 1/rs;

					if(curvature >3) cartisian_ftr.ptr<double>(row)[col] = 255;
		}
		
	}
}

double NormalizeAngle(double angle)
{
    double a = fmod(angle+M_PI, 2.0*M_PI);
    if(a<0.0)
    {
        a+= (2.0*M_PI);
    }

    return a -M_PI;
}
class Scan2
{
public:
    Scan2();

    std::string odom_frame_;
    std::string base_frame_;

    ros::NodeHandle node_;
    tf::TransformListener tf_;

    tf::TransformBroadcaster locationBroadcaster;
    tf::StampedTransform locationTrans;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_scan2;
    ros::Subscriber sub_wheel;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_pg;
    ros::Subscriber sub_loop_dect;
    ros::Subscriber sub_qrcode;

    /*QrCode params*/
    double qr_odom = 0.15;
    double timestep_now = 0;

    double x_now = 0.15;
    double y_now = 0;
    double theta_now = 0;

    bool init_qrcode = false;
    int qr_id = 0;
    double last_timestamp = 0;
    double lastlast_timestamp = 0;
    Eigen::Matrix3d last_TCE = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d last_TLC = Eigen::Matrix3d::Identity();

    //defining subscriber and publisher in demo
    void StartSlam();
    //posegraph thread
    void StartPosegraph();
    void QrCodeProcess();
    //processing scan data, pose estiamtion and update, map construction
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    //processing wheel data, updating translation info, linear interpolaton
    void wheelCallback(const diff_msgs::WheelSpeedStatus::ConstPtr &wheel_msg);
    //processing imu data, updating rotation info.
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    //updating translation
    void wheel_predict(const diff_msgs::WheelSpeedStatus::ConstPtr &wheel_msg);
    void LoopDetectionflag(const std_msgs::Empty& msg);
    //updating rotation 
    void imu_predict(sensor_msgs::Imu imu_msg);
    //add points in keyframe to pointcloud
    int addMapPointToCldoud(msf::LaserFrame::Ptr KeyFrame);
    //QrCode for loop closure detection
    void QRcodeCallback(const qrc_msgs::QuickResponseCode::ConstPtr& QRc);
    //processing keyframe in another thread, trying to find loop closure and optimize it
    void posegraph(const sensor_msgs::LaserScan::ConstPtr& scan);
    
};

Scan2::Scan2(){}

void Scan2::StartSlam()
{
   
    ros::NodeHandle private_nh_("~");

    if(!private_nh_.getParam("odom_frame", odom_frame_))
        odom_frame_ = "odom";
    if(!private_nh_.getParam("base_frame", base_frame_))
        base_frame_ = "base_link";

    // if(pcl::io::loadPCDFile<PointT>("/home/zz/ros_test_ws/src/msf/src/mymap.pcd",*mymap) == -1)
    // {
    //     PCL_ERROR("Could not read file mymap.pcd\n");
    //     return ;
    // }

    sub_loop_dect = node_.subscribe("loop_dect", 5, &Scan2::LoopDetectionflag, this);
    sub_scan = node_.subscribe<sensor_msgs::LaserScan>("/r2000_node/scan",1000,boost::bind(&Scan2::scanCallback, this, _1));
    pub_odom = node_.advertise<nav_msgs::Odometry>("odom", 1000);
    pub_raw_odom = node_.advertise<nav_msgs::Odometry>("raw_odom", 1000);
    pub_keyframe = node_.advertise<sensor_msgs::LaserScan>("keyframe", 1000);
    
    sub_imu = node_.subscribe<sensor_msgs::Imu>("/imu", 1000, boost::bind(&Scan2::imuCallback, this, _1)); 
    sub_wheel = node_.subscribe<diff_msgs::WheelSpeedStatus>("/wheel_speed_status", 1000, boost::bind(&Scan2::wheelCallback, this, _1));

    locationTrans.frame_id_ = "odom";
    locationTrans.child_frame_id_ = "base_link";

    pcl_pub = node_.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
     std::cout<<"startslam"<<std::endl;
}

void Scan2::StartPosegraph()
{
    //sub_pg = node_.subscribe<sensor_msgs::LaserScan>("/keyframe",1000,boost::bind(&Scan2::posegraph, this, _1));
}

void Scan2::QrCodeProcess()
{
    //sub_qrcode = node_.subscribe<qrc_msgs::QuickResponseCode>("/qrc_info",1000,boost::bind(&Scan2::QRcodeCallback, this, _1));
}

void Scan2::QRcodeCallback(const qrc_msgs::QuickResponseCode::ConstPtr& QRc)
{
    if(!(qr_msg_count%qr_freq == 0)) 
    {
         qr_msg_count++;
         return;
    }
    qr_msg_count++;

   if( !QRc->time_stamp)
    {    last_timestamp = 0;
        lastlast_timestamp = 0;
        last_TCE.setIdentity();
        last_TLC.setIdentity();
        return;
    }

    //qrcp_point
    geometry_msgs::Point p = QRc->qrcp_point;
    if(abs(p.x) > 0.1 || abs(p.y)>0.1)
    return;

    float qrc_x,qrc_y,qrc_theta;
    qrc_x = p.x; 
    qrc_y = p.y; 
    qrc_theta = QRc->angle*3.1415926/180;
    Eigen::Matrix3d T_CE;
    T_CE<<cos(qrc_theta),-sin(qrc_theta),qrc_x,
    sin(qrc_theta),cos(qrc_theta),qrc_y,
    0,0,1;
    Eigen::Matrix3d T_EC = T_CE.inverse();
    Eigen::Matrix3d TLC = last_TCE * T_EC;

    //检查二维码结果速度是否一致, 只要二维码本身的速度结果，3帧以上一致再更新结果，TODO速度和航迹推演一致
    if(lastlast_timestamp)
    {

        double last_dtime = (last_timestamp - lastlast_timestamp)/1000.0;
        double dtime =  (QRc->time_stamp - last_timestamp)/1000.0;

        double d_vx = abs(last_TLC(0,2)/last_dtime - TLC(0,2)/dtime);
        double d_vy = abs(last_TLC(1,2)/last_dtime - TLC(1,2)/dtime);
        //std::cout<<d_vx<<"---------d_vx d_vy-------------"<<d_vy<<QRc->QRC_id<<std::endl;

        lastlast_timestamp = last_timestamp;
        last_timestamp = QRc->time_stamp;

        last_TLC = TLC;
        last_TCE = T_CE;
        if(d_vx > 0.1 || d_vy > 0.1)
        return;

        qrc_x = T_EC(0,2); 
        qrc_y = -T_EC(1,2);
        qrc_theta = -atan2(T_EC(1,0), T_EC(0,0));

        switch (qr_codeMap[QRc->QRC_id])//二维码角度都是0
        {
            case 0:
            qrc_x += 0; qrc_y += 0; qr_id = 0;//二维码坐标是0,0 
            break;

            case 1:
            qrc_x += 0; qrc_y += 3.0; qr_id = 1;//二维码坐标是0,3
            break;

            case 2:
            qrc_x += 1.0; qrc_y += 3.0; qr_id = 2;//二维码坐标是1,3
            break;

            case 3:
            qrc_x += 2.0; qrc_y += 3.0; qr_id = 3;//二维码坐标是2,3
            break;

            case 4:
            qrc_x += 2.0; qrc_y += 2.0; qr_id = 4;//二维码坐标是2,2
            break;

            case 5:
            qrc_x += 2.0; qrc_y += 1.0; qr_id = 5;//二维码坐标是2,1
            break;

            case 6:
            qrc_x += 2.0; qrc_y += 0.0; qr_id = 6;//二维码坐标是2,0
            break;

            case 7:
            qrc_x += 1.0; qrc_y += 0.0; qr_id = 7;//二维码坐标是1,0
            break;

            case 8:
            qrc_x += 0; qrc_y += 0.0; qr_id = 8;//二维码坐标是0,0
            break;

            case 9:
            qrc_x += 0; qrc_y += 1.0; qr_id = 9;//二维码坐标是0,1
            break;

            case 10:
            qrc_x += 0; qrc_y += 2.0; qr_id = 10;//二维码坐标是0,2
            break;

            default:
            std::cout<<"--------- QR CODE NOT SPECIFIED -------------"<<std::endl;
            return;
        }
      //std::cout<<"---------the current pose is -------------"<<std::endl;
     // std::cout<<p.x<<"                 "<< p.y<<"         "<<QRc->angle<<std::endl;
        {
            theta_now = qrc_theta;
            x_now = qrc_x + qr_odom * cos(theta_now);
            y_now = qrc_y + qr_odom * sin(theta_now);;
            init_qrcode = true;
        }
         /*loop detection phase*/
        loop_chain.push_back({x_now,y_now,theta_now});
        int loop_chain_size = loop_chain.size();
        int keyframe_size = LO->map_->keyframes_.size();

        if(loop_chain_size) 
        {
            Eigen::Vector3d tmp;
            tmp = loop_chain[loop_chain_size-1];
            calib_result<<tmp.transpose()<<"  "<<loop_chain_size<<"  "<<loop_count<<std::endl;
            if(!ros::ok()) calib_result.close();
        }

        if(loop_chain_size<10) return;
        Eigen::Vector3d third_latest_qr_odom = loop_chain[loop_chain_size-3];
        Eigen::Vector3d second_latest_qr_odom = loop_chain[loop_chain_size-2];
        Eigen::Vector3d latest_qr_odom = loop_chain[loop_chain_size-1];
        double threshold_loop = 0.025;
        for(int i = 0;i<1;i++)
        {
            calib_result<<distance_eigen(third_latest_qr_odom, loop_chain[i])<<std::endl;
            if(distance_eigen(third_latest_qr_odom, loop_chain[i])<threshold_loop & distance_eigen(second_latest_qr_odom, loop_chain[i+1])<threshold_loop & distance_eigen(latest_qr_odom, loop_chain[i+2])<threshold_loop & keyframe_size>10)
            {
                if(first_loop)
                {
                    first_loop = false;
                    latest_loop_odom << robot_x, robot_y, robot_yaw;
                    loop_flag = 1;
                    loop_count ++;
                    return;
                }
                else
                {
                    Eigen::Vector3d now_loop_odom;
                    now_loop_odom << robot_x, robot_y, robot_yaw;
                    if(distance_eigen(latest_loop_odom, now_loop_odom)>1.0)
                    {
                        loop_flag = 1;
                        loop_count++;
                        latest_loop_odom = now_loop_odom;
                        return;
                    }
                    else
                    {
                        return;
                    }
                    
                }
            }
        }
        return;
    }
    lastlast_timestamp = last_timestamp;
    last_timestamp = QRc->time_stamp;

    last_TLC = TLC;
    last_TCE = T_CE;

}

void Scan2::LoopDetectionflag(const std_msgs::Empty& msg)
{
    loop_flag = 1;
}
 

void Scan2::imu_predict(sensor_msgs::Imu imu_msg)
{
    
    double t = imu_msg.header.stamp.toSec();
    double dt = t - last_imu_t;
    last_imu_t = t;

    double rz = imu_msg.angular_velocity.z;

    
    if(match_success1)
    {
        robot_yaw = update_pose(2);
        match_success1 = false;
    }
    
    robot_yaw += rz*dt;
    raw_robot_yaw += rz*dt;

    robot_yaw = NormalizeAngle(robot_yaw);
    raw_robot_yaw = NormalizeAngle(raw_robot_yaw);


}

void Scan2::wheel_predict(const diff_msgs::WheelSpeedStatus::ConstPtr &wheel_msg)
{
    double t = wheel_msg->header.stamp.toSec();
    double dt = t - last_wheel_t;
    last_wheel_t = t;

    double velocity_l = wheel_msg->leftWheelSpeed * 0.001;
    double velocity_r = wheel_msg->rightWheelSpeed * 0.001;

    double velocity_l_psuo = coff_left*wheel_msg->leftWheelSpeed * 0.001;
    double velocity_r_psuo = coff_right*wheel_msg->rightWheelSpeed * 0.001;

    double robot_vel = (velocity_l + velocity_r)/2;

    double robot_vel_psuo = (velocity_l_psuo + velocity_r_psuo)/2; 
    double robot_omega_psuo = (velocity_r_psuo - velocity_l_psuo)/0.31012;
    
    if(match_success2)
    {
        robot_x = update_pose(0);
        robot_y = update_pose(1);
        match_success2 = false;
    }
    
    robot_x = robot_x + dt*robot_vel*cos(robot_yaw);
    robot_y = robot_y + dt*robot_vel*sin(robot_yaw);

    raw_robot_x = raw_robot_x + dt*robot_vel*cos(raw_robot_yaw);
    raw_robot_y = raw_robot_y + dt*robot_vel*sin(raw_robot_yaw);

    robot_x_psuo += dt*robot_vel_psuo*cos(raw_robot_yaw);
    robot_y_psuo += dt*robot_vel_psuo*sin(raw_robot_yaw);
    robot_yaw_psuo += dt*robot_omega_psuo;

    speed_time(0) = 2.0*wheel_msg->leftWheelSpeed /125.0;
    speed_time(1) = 2.0*wheel_msg->rightWheelSpeed /125.0;

}

void Scan2::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    current_time = imu_msg->header.stamp;
    double yaw = imu_msg->angular_velocity.z;
    sensor_msgs::Imu imuMsg;
    imuMsg.header.stamp = current_time;
    if(init_imu)
    {
        if(imu_init_num<100)
        {
            sum_yaw += yaw;
            imu_init_num++;
            last_imu_t = imuMsg.header.stamp.toSec();
            return;
        }
        else
        {
            imu_yaw_bias = sum_yaw/100;
            std::cout<<"-------------imu_yaw_bias is-------------------"<<std::endl;
            std::cout<<imu_yaw_bias<<std::endl;
            last_imu_t = imuMsg.header.stamp.toSec();
            init_imu = false;
        }
    }
    else if (imuMsg.header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    else
    {
        imuMsg = *imu_msg;
        imuMsg.angular_velocity.z = yaw - imu_yaw_bias;
    }
    
    imu_predict(imuMsg);
    
}

void Scan2::wheelCallback(const diff_msgs::WheelSpeedStatus::ConstPtr &wheel_msg)
{
    if(init_wheel)
    {
        init_wheel = false;
        last_wheel_t = wheel_msg->header.stamp.toSec();
        return;
    }
    if (wheel_msg->header.stamp.toSec() <= last_wheel_t)
    {
        ROS_WARN("wheel message in disorder!");
        return;
    }
    wheel_predict(wheel_msg);
    current_time = wheel_msg->header.stamp; 
}


int Scan2::addMapPointToCldoud(msf::LaserFrame::Ptr KeyFrame)
{
    int n = LO->map_->keyframes_.size();

    if(n == 10000)
    {
        pC->points.clear();
        pC_tmp->points.clear();
		auto its = LO->map_->keyframes_.find(n-2);
/*
   	    its->second->compute_world_coords(its->second->true_pose);
        for(int i=0;i<its->second->nrays;i++)
        {
            if(!(its->second->valid[i])) continue;
            PointT point;
            point.x =  its->second->points_w[i].p[0];
            point.y =  its->second->points_w[i].p[1];
            point.z = 0.0;
            pC_tmp->points.push_back(point);
        }
*/

        for(auto ity = its->second->connected_frames_.begin();ity!=its->second->connected_frames_.end();ity++)
        {
			auto it = *ity;
            it->compute_world_coords(it->true_pose);
            for(int i=0;i<it->nrays;i++)
            {
                if(!(it->valid[i])) continue;
                //if(!(it->second->corr[i].valid)) continue;
                PointT point;
                point.x =  it->points_w[i].p[0];
                point.y =  it->points_w[i].p[1];
                point.z = 0.0;
                pC_tmp->points.push_back(point);
            }
        


            pcl::RadiusOutlierRemoval<PointT> statistical_filter;
            statistical_filter.setRadiusSearch(0.5);
            statistical_filter.setMinNeighborsInRadius(10);
            statistical_filter.setNegative(false);

            statistical_filter.setInputCloud(pC_tmp);
            statistical_filter.filter(*pC);

            pcl::VoxelGrid<PointT> voxel_filter;
            voxel_filter.setLeafSize(0.1,0.1,0.1);
            voxel_filter.setInputCloud(pC);
            voxel_filter.filter(*pC_tmp);
            //pcl::io::savePCDFileBinary<PointT>("mapkeyframenew48.pcd", *pC_tmp);
        }
    }

    if(n == 10000)
    {
        auto its = LO->map_->keyframes_.begin();
        cv::Mat line_ftr = cv::Mat::zeros(600,180,CV_8UC1);
        cv::Mat corner_ftr = cv::Mat::zeros(100,160,CV_8UC1);
		cv::Mat polar_ftr = cv::Mat::zeros(80,180,CV_8UC1);
        cv::Mat cartisian_ftr = cv::Mat::zeros(80,80,CV_8UC1);

        PointCloud::Ptr first_pc(new PointCloud);
         PointCloud::Ptr second_pc(new PointCloud);
        std::ofstream ofs_com;
        ofs_com.open("/home/zz/ros_test_ws/src/msf/src/result.txt", fstream::out);
        if(!ofs_com.is_open())
        {
            std::cout<<"ofs_com is not open"<<endl;
        }

        its->second->compute_world_coords(its->second->true_pose);
        for(int i=0;i<its->second->nrays;i++)
        {
            if(!(its->second->valid[i])) continue;
            PointT point;
            point.x =  its->second->points_w[i].p[0];
            point.y =  its->second->points_w[i].p[1];
            point.z = 0.0;
            first_pc->points.push_back(point);
        }

        for(auto ity = its->second->connected_frames_.begin();ity!=its->second->connected_frames_.end();ity++)
        {
			auto it = *ity;
            it->compute_world_coords(it->true_pose);
            for(int i=0;i<it->nrays;i++)
            {
                if(!(it->valid[i])) continue;
                //if(!(it->second->corr[i].valid)) continue;
                PointT point;
                point.x =  it->points_w[i].p[0];
                point.y =  it->points_w[i].p[1];
                point.z = 0.0;
                first_pc->points.push_back(point);
            }
        }

        pcl::RadiusOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setRadiusSearch(0.5);
        statistical_filter.setMinNeighborsInRadius(10);
        statistical_filter.setNegative(false);

        statistical_filter.setInputCloud(first_pc);
        statistical_filter.filter(*second_pc);

        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setLeafSize(0.1,0.1,0.1);
        voxel_filter.setInputCloud(second_pc);
        voxel_filter.filter(*first_pc);

        feature_extractor(first_pc, cartisian_ftr);
        cv::normalize(cartisian_ftr, cartisian_ftr, 1.0,0.0,NORM_MINMAX);

        int count = 0;

        PointCloud::Ptr first_pc1(new PointCloud);
        PointCloud::Ptr second_pc1(new PointCloud);
        for(auto it = LO->map_->keyframes_.begin();it!=LO->map_->keyframes_.find(n-1);it++)
        {
            cv::Mat line_ftr1 = cv::Mat::zeros(600,180,CV_8UC1);
            cv::Mat corner_ftr1 = cv::Mat::zeros(100,160,CV_8UC1);
			cv::Mat polar_ftr1 = cv::Mat::zeros(80,180,CV_8UC1);
            cv::Mat cartisian_ftr1 = cv::Mat::zeros(80,80,CV_8UC1);


           first_pc1->points.clear();
           second_pc1->points.clear();

            it->second->compute_world_coords(it->second->true_pose);
            for(int i=0;i<it->second->nrays;i++)
            {
                if(!(it->second->valid[i])) continue;
                //if(!(it->second->corr[i].valid)) continue;
                PointT point;
                point.x =  it->second->points_w[i].p[0];
                point.y =  it->second->points_w[i].p[1];
                point.z = 0.0;
                first_pc->points.push_back(point);
            }

            // it->second->compute_world_coords(it->second->true_pose);
            // for(int i=0;i<it->second->nrays;i++)
            // {
            //     if(!(it->second->valid[i])) continue;
            //     PointT point;
            //     point.x =  it->second->points_w[i].p[0];
            //     point.y =  it->second->points_w[i].p[1];
            //     point.z = 0.0;
            //     first_pc1->points.push_back(point);
            // }
            if(!it->second->connected_frames_.empty())
            {
                for(auto ity = it->second->connected_frames_.begin();ity!=it->second->connected_frames_.end();ity++)
                {
                    auto itu = *ity;
                    itu->compute_world_coords(itu->true_pose);
                    for(int i=0;i<itu->nrays;i++)
                    {
                        if(!(itu->valid[i])) continue;
                        //if(!(it->second->corr[i].valid)) continue;
                        PointT point;
                        point.x =  itu->points_w[i].p[0];
                        point.y =  itu->points_w[i].p[1];
                        point.z = 0.0;
                        first_pc1->points.push_back(point);
                    }
                }
            }

            pcl::RadiusOutlierRemoval<PointT> statistical_filter;
            statistical_filter.setRadiusSearch(0.5);
            statistical_filter.setMinNeighborsInRadius(10);
            statistical_filter.setNegative(false);

            statistical_filter.setInputCloud(first_pc1);
            statistical_filter.filter(*second_pc1);

            pcl::VoxelGrid<PointT> voxel_filter;
            voxel_filter.setLeafSize(0.1,0.1,0.1);
            voxel_filter.setInputCloud(second_pc1);
            voxel_filter.filter(*first_pc1);

            feature_extractor(first_pc1, cartisian_ftr1);
            cv::normalize(cartisian_ftr1, cartisian_ftr1, 1.0,0.0,NORM_MINMAX);


            //double line_result = cv::norm(line_ftr,line_ftr1,CV_L2);
            //double corner_result = cv::norm(corner_ftr,corner_ftr1,CV_L2);
			double cartisian_result = cv::norm(cartisian_ftr, cartisian_ftr1,CV_L2);

            std::cout<<count<<", "<<cartisian_result<<std::endl;

            //ofs_com<<count<<", "<<line_result<<", "<<corner_result<<std::endl;
			ofs_com<<count<<", "<<cartisian_result<<std::endl;
            count++;
        }
        ofs_com.close();
    }

    tmp->points.clear();
    pointCloud->points.clear();
    for(auto it = LO->map_->keyframes_.begin();it!=LO->map_->keyframes_.end();it++)
    {
        it->second->compute_world_coords(it->second->true_pose);
        for(int i=0;i<it->second->nrays;i++)
        {
            if(!(it->second->valid[i])) continue;
            //if(!(it->second->corr[i].valid)) continue;
            PointT point;
            point.x =  it->second->points_w[i].p[0];
            point.y =  it->second->points_w[i].p[1];
            point.z = 0.0;
            tmp->points.push_back(point);
        }
    }
       
    //add filter or not
    tmp->is_dense = false;
    
    tmp->swap(*pointCloud);
    

    pointCloud->width = 1;
    pointCloud->height = pointCloud->points.size();
    if(n==30) pcl::io::savePCDFileBinary<PointT>("mapkeyframe23.pcd", *pointCloud);

    return 1;

}

void Scan2::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // if(!(laser_freq%50 == 0)) 
    // {
    //      laser_freq++;
    //      return;
    // }
    // laser_freq++;
    
    msf::LaserFrame::Ptr LFscan = msf::LaserFrame::createFrame();
    LFscan->Create(scan);
    LFscan->compute_cartesian();
    odom_pose_prev << robot_x,robot_y,robot_yaw ;
    Eigen::Vector3d raw_odom_pose({raw_robot_x,raw_robot_y,raw_robot_yaw});
    Eigen::Vector3d raw_odom_pose_psuo({robot_x_psuo, robot_y_psuo, robot_yaw_psuo});
    // Eigen::Matrix3d raw_correct; 
    // raw_correct  <<      1.04502 ,   0.0331593 ,   0.0464866,
    //                                                                         0.00993044    ,  1.01411     , 0.23582,
    //                                                                         -0.000338163  , 0.00488138 ,    0.998861;
    // raw_odom_pose = raw_correct*raw_odom_pose;

    //set loop closure detection flag
    // if(abs(raw_robot_x+dis_lidar_odom_x) < 0.2 & abs(raw_robot_y + dis_lidar_odom_y) <0.3 & abs(raw_robot_yaw - 12.56) < 0.2)
    // {
    //     loop_flag++;
    // }
    std::cout<<"---------loop flag is -------------"<<std::endl;
    std::cout<<loop_flag<<std::endl;

    std::cout<<"---------the raw odom pose is -------------"<<std::endl;
    std::cout<<raw_odom_pose.transpose()<<std::endl;
    std::cout<<raw_odom_pose_psuo.transpose()<<std::endl;

    std::cout<<"---------the odom pose is -------------"<<std::endl;
    std::cout<<odom_pose_prev.transpose()<<std::endl;
    
    //transform the odom pose to laserscan to provide prior pose
    //comment it while calibration
    scan_x = robot_x + dis_lidar_odom*cos(odom_pose_prev(2)) ;
    scan_y = robot_y +  dis_lidar_odom*sin(odom_pose_prev(2)) ;
    scan_yaw = odom_pose_prev(2);
    Eigen::Vector3d prior({scan_x,scan_y,scan_yaw}) ;

    if(loop_flag  == 1)
    {
        LFscan->is_key_frame_ = true;
        LFscan->is_loop_frame_ = true;
        loop_flag = 0;
    }
    /*main process function, estimate the pose of current scan and set the match flag true to update the 
    robot state in callback function of imu and wheelspeedstatus message*/
    start_frame = clock();
    match_success1 = LO->addFrame(LFscan,prior,loop_flag, mymap);
    match_success2 = match_success1;
    end_frame = clock();

    double time_cost = (double)(end_frame - start_frame)/CLOCKS_PER_SEC;
    std::cout<<"---------the frame processing costs -------------"<<std::endl;
    std::cout<<time_cost*1000<<"ms"<<std::endl;


    std::cout<<"---------the corr_ratio is -------------"<<std::endl;
    std::cout<<LFscan->corr_info<<std::endl;
   

    //optimized pose is in the laser frame
    scan_x = LFscan->true_pose(0);
    scan_y = LFscan->true_pose(1);
    scan_yaw = LFscan->true_pose(2);

    Eigen::Vector3d scan_cal({scan_x,scan_y,scan_yaw});

    // if(!Odom_calib.is_full())  Odom_calib.Add_Data(raw_odom_pose, scan_cal);
    
    // if(Odom_calib.is_full() & calib_f)
    // {
    //     std::cout<<"start calibration!!!!!!!!!"<<std::endl;
    //     Eigen::Matrix3d correct_matrix = Odom_calib.Solve();
    //     std::ofstream calib_result;
    //     calib_result.open("result.txt", std::fstream::out);
    //     if(!calib_result.is_open())
    //     {
    //         std::cout<<"can not open file !!!!!!"<<std::endl;
    //     }
    //     Eigen::Matrix3d pre_calib;
    //     calib_result<<correct_matrix<<std::endl;
    //     calib_result.close();
    //     calib_f = false;
    // }

    //get the odom pose now
    Eigen::Vector3d odom_pose_now({robot_x,robot_y,robot_yaw});
    
    Eigen::Vector3d relative_pose;
    Eigen::Vector3d tmp;

    //compute the translation and rotation in the optimization time, which is tiny and neglectable in most case
    relative_pose = LO->cal_relative_pose(odom_pose_prev, odom_pose_now);
    //std::cout<<"----------relative pose is ---------------------"<<std::endl;
    //std::cout<<relative_pose.transpose()<<std::endl;

    //retransform the optimized pose from laser frame to odom frame 
    tmp(0) = scan_x - dis_lidar_odom * cos(scan_yaw);
    tmp(1) = scan_y - dis_lidar_odom * sin(scan_yaw);
    tmp(2) = scan_yaw;
    
    std::cout<<"---------the current pose is -------------"<<std::endl;
    std::cout<<tmp.transpose()<<std::endl;


    // fusion_result<<std::fixed<<scan->header.stamp.toNSec()<<"  "<<tmp.transpose()<<std::endl;
    // if(!ros::ok()) fusion_result.close();
    //add data to calibration vector
    if(first_scan)
    {
        last_sample_time = scan->header.stamp.toSec();
        last_speed_time = speed_time;
        init_sample_time = last_sample_time;
        last_obser4calib = obser4calib = tmp;
        first_scan = false;
    }
    else
    {
        double tmp_t = scan->header.stamp.toSec();
        double delta_time = tmp_t - last_sample_time;
        double last2init = last_sample_time - init_sample_time;
        double now2init = tmp_t - init_sample_time;
        last_sample_time = tmp_t;

        speed_time(2) = tmp(2);

        obser4calib = tmp;
        Eigen::Vector3d delta_obser = LO->cal_relative_pose(last_obser4calib, obser4calib);

       
        calib_matrix.setZero();
        calib_matrix.block<1,3>(0,0) =  speed_time.transpose();
        calib_matrix.block<1,3>(1,0) =  delta_obser.transpose();
        calib_matrix(0,3) = last2init;
        calib_matrix(1,3) = now2init;

        // std::cout<<calib_matrix<<std::endl;
        if(fabs(calib_matrix(0,0))>0.0001 & fabs(calib_matrix(0,1))>0.0001 & fabs(calib_matrix(1,0))>0.0001 & fabs(calib_matrix(1,1))>0.0001) 
        {
            if(calib_count<100)
            {
                wheel_calib<<calib_matrix<<std::endl;
                wheel_calib<<"\n"<<std::endl;
            }
            if(calib_count == 99)
            {
                wheel_calib.close();
            }

            //calib_mat.push_back(calib_matrix);
            calib_count++;
        }
        
        last_speed_time = speed_time;
        last_obser4calib = obser4calib;
    }
    // int calib_mat_size = calib_mat.size();
    // std::cout<<"---------mat size------------"<<std::endl;
    // std::cout<<calib_mat_size<<std::endl;
    // if(calib_mat_size == 100)
    // {
    //     for(int i=0;i<calib_mat_size;i++)
    //     {
    //         wheel_calib<<calib_mat[i]<<std::endl;
    //         wheel_calib<<"\n"<<std::endl;
    //     }
    //     wheel_calib.close();
    // }

 
    //add the tiny relative pose
    update_pose = LO->relative2world(tmp, relative_pose);
    
    //if the scan is keyframe, then send it to backend and add it to map to visualize
    if(LFscan->is_key_frame_ == true || LFscan->is_loop_frame_ == true)
    {
        //publish the keyframe to backend
        sensor_msgs::LaserScan KeyScan;
        KeyScan = *scan;
        KeyScan.header.frame_id = "keyframe";
        std::cout<<"---------keyframe published -------------"<<std::endl;
        pub_keyframe.publish(KeyScan);
        
        //add keyframe points to map and publish it 
        if(addMapPointToCldoud(LFscan))
        {
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*pointCloud, output);
            output.header.frame_id = "odom";

            pcl_pub.publish(output); 
        }
        
    }
    
    
    //send the odom pose to topic raw_odom to visualize in rviz
    nav_msgs::Odometry raw_odom;
    raw_odom.header.stamp = scan->header.stamp;
    raw_odom.header.frame_id = "odom";

    //set the position
    geometry_msgs::Quaternion raw_odom_quat = tf::createQuaternionMsgFromYaw(raw_odom_pose(2));
    raw_odom.pose.pose.position.x = raw_odom_pose(0);
    raw_odom.pose.pose.position.y = raw_odom_pose(1);
    raw_odom.pose.pose.position.z = 0.0;
    raw_odom.pose.pose.orientation = raw_odom_quat;

    //set the velocity
    raw_odom.child_frame_id = "base_link";
    raw_odom.twist.twist.linear.x = 0.0;
    raw_odom.twist.twist.linear.y = 0.0;
    raw_odom.twist.twist.angular.z = 0.0;

    //publish the message
    pub_raw_odom.publish(raw_odom);

    /*-----------------------------------------------------------------------------*/

    //send the scan pose to topic odom to visualize in rviz
    nav_msgs::Odometry odom;
    odom.header.stamp = scan->header.stamp;
    odom.header.frame_id = "odom";

    //set the position
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(LFscan->true_pose(2));
    odom.pose.pose.position.x = LFscan->true_pose(0);
    odom.pose.pose.position.y = LFscan->true_pose(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pub_odom.publish(odom);
    
}


void Scan2::posegraph(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    std::cout<<"posegraph"<<std::endl;

    {
        int n = LO->map_->keyframes_.size();
        if(n == 1) return;
        //get the second latest keyframe
        auto sec_latest_kf = LO->map_->keyframes_.find(n-2);

        std::cout<<"----------posegraph with trivals--------"<<std::endl;
        int size_connected_frames = sec_latest_kf->second->connected_frames_.size();
        std::cout<<sec_latest_kf->second->connected_frames_.size()<<std::endl;

        for(int i=0;i<sec_latest_kf->second->nrays;i++)
        {
            if(!(sec_latest_kf->second->valid[i])) continue;
            PointT point;
            point.x =  sec_latest_kf->second->points[i].p[0];
            point.y =  sec_latest_kf->second->points[i].p[1];
            point.z = 0.0;
            sec_latest_kf->second->submap->points.push_back(point);
        }


        int interval = (int)(size_connected_frames/5) +1;
        int count =0;
        for(auto it : sec_latest_kf->second->connected_frames_)
        {
            if(count%interval == 0)
            {
                it->compute_world_coords(LO->cal_relative_pose(sec_latest_kf->second->true_pose,it->true_pose));
                for(int i=0;i<it->nrays;i++)
                {
                    if(!(it->valid[i])) continue;
                    PointT point;
                    point.x =  it->points_w[i].p[0];
                    point.y =  it->points_w[i].p[1];
                    point.z = 0.0;
                    sec_latest_kf->second->submap->points.push_back(point);
                }
            }
            count++;
        }

        std::cout<<"----------points in submap--------"<<std::endl;
        std::cout<<sec_latest_kf->second->submap->points.size()<<std::endl;


        unsigned long old_cnt = 0;

        //get the latest keyframe
        auto lp = LO->map_->keyframes_.find(n-1);
        if(lp->second->is_loop_frame_ == true)
        {
            std::vector<Eigen::Vector3d> Vertices;
            std::vector<Edge> Edges;

            auto its = LO->map_->keyframes_.begin();
            old_cnt = its->first;
            std::cout<<"----------posegraph member--------"<<std::endl;
            std::cout<<old_cnt<<"and"<<lp->first<<std::endl;
            
            //add vertices to the optimization problem
            for(auto it = LO->map_->keyframes_.begin();it!=LO->map_->keyframes_.find(n-2);it++)
            {
                Vertices.push_back(it->second->true_pose);    
            }
            int vertices_size = Vertices.size();
            std::cout<<"-----------vertices size is------------"<<std::endl;
            std::cout<<vertices_size<<std::endl;
            //add edges to the optimization problem
            for(int i=0;i<Vertices.size();i++)
            {
                //the last vertex connect with the first one by loop edge
                 if(i == Vertices.size()-1)
                {
                    int j = 0;
                    Edge tmpEdge;

                    tmpEdge.xi = i;
                    tmpEdge.xj = j;

                    auto new_one = LO->map_->keyframes_.find(j);
                    auto old_one = LO->map_->keyframes_.find(i);
                    Eigen::Vector3d rel_pose;

                    rel_pose = LO->cal_relative_pose(old_one->second->true_pose,new_one->second->true_pose);
                    std::cout<<"-----------initial  is------------"<<std::endl;
                    std::cout<<rel_pose.transpose()<<std::endl;
                    Eigen::Vector3d delta_rel;
                    delta_rel.setZero();
                    double ratio = 1.0;
                    if(LO->poseEstimationNDT(old_one->second->submap, new_one->second, delta_rel,ratio))
                    {
                        rel_pose = delta_rel;
                    }
                    std::cout<<"-----------after opt  is------------"<<std::endl;
                    std::cout<<rel_pose.transpose()<<std::endl;

                    tmpEdge.measurement = rel_pose;

                    tmpEdge.infoMatrix =  ratio*Eigen::Matrix3d::Identity();
                    Edges.push_back(tmpEdge);

                }
                //other vertices only have sequatial edges
                else if(i < vertices_size - 5)
                {
                    auto old_one = LO->map_->keyframes_.find(old_cnt + i);
                    for(int j=i+1;j<i+6;j++)
                    {
                        Edge tmpEdge;

                        tmpEdge.xi = i;
                        tmpEdge.xj = j;           

                        auto new_one = LO->map_->keyframes_.find(old_cnt + j);
                        Eigen::Vector3d rel_pose;

                        // rel_pose = LO->cal_relative_pose(old_one->second->true_pose,new_one->second->true_pose);
                        // std::cout<<"-----------initial  is------------"<<std::endl;
                        // std::cout<<rel_pose.transpose()<<std::endl;
                        // Eigen::Vector3d delta_rel;
                        // delta_rel.setZero();
                        // double ratio = 1.0;
                        // if(!LO->poseEstimationNDT(new_one->second->submap, old_one->second->submap, delta_rel,ratio))
                        // {
                        //     delta_rel.setZero();
                        //     if(j!=i+1) continue;
                        // }

                        rel_pose = LO->cal_relative_pose(old_one->second->true_pose,new_one->second->true_pose);
                        double ratio = 1.0;
                        Eigen::Vector3d delta_rel;
                        delta_rel.setZero();
                        if(LO->poseEstimationNDT(old_one->second->submap, new_one->second, delta_rel,ratio))
                        {
                            rel_pose = delta_rel;
                        }
                        else
                        {
                            if(j!=i+1) continue;
                        }
                        
                        // rel_pose = LO->relative2world(rel_pose, delta_rel);

                        // std::cout<<"-----------after opt  is------------"<<std::endl;
                        // std::cout<<rel_pose.transpose()<<std::endl;
                        tmpEdge.measurement = rel_pose;

                        tmpEdge.infoMatrix =  ratio*Eigen::Matrix3d::Identity();
                        Edges.push_back(tmpEdge);
                    }  
                }    
                else
                {
                    auto old_one = LO->map_->keyframes_.find(old_cnt + i);
                    for(int j=i+1;j<vertices_size;j++)
                    {
                        Edge tmpEdge;

                        tmpEdge.xi = i;
                        tmpEdge.xj = j;           

                        auto new_one = LO->map_->keyframes_.find(old_cnt + j);
                        Eigen::Vector3d rel_pose;

                        // rel_pose = LO->cal_relative_pose(old_one->second->true_pose,new_one->second->true_pose);
                        //  std::cout<<"-----------initial  is------------"<<std::endl;
                        // std::cout<<rel_pose.transpose()<<std::endl;       
                        // Eigen::Vector3d delta_rel;       
                        // delta_rel.setZero();       
                        // double ratio = 1.0;  
                        // if(!LO->poseEstimationNDT(new_one->second->submap, old_one->second->submap, delta_rel,ratio))
                        // {
                        //     delta_rel.setZero();
                        //     if(j!=i+1) continue;
                        // }                        
                        // rel_pose = LO->relative2world(rel_pose, delta_rel);

                        rel_pose = LO->cal_relative_pose(old_one->second->true_pose,new_one->second->true_pose);
                        double ratio = 1.0;
                        Eigen::Vector3d delta_rel;
                        delta_rel.setZero();
                        if(LO->poseEstimationNDT(old_one->second->submap, new_one->second, delta_rel,ratio))
                        {
                            rel_pose = delta_rel;
                        }
                        else
                        {
                            if(j!=i+1) continue;
                        }

                        tmpEdge.measurement = rel_pose;

                        tmpEdge.infoMatrix =  ratio*Eigen::Matrix3d::Identity();
                        Edges.push_back(tmpEdge);

                    }
                }
                               
            }
            std::cout<<"problem has been constructed"<<std::endl;
            //use gaussian_newtion algorithm to solve the problem, details defined in gaussian_newton.cpp
            double initError = ComputeError(Vertices, Edges);
            std::cout << "initError:" << initError << std::endl;

            int maxIteration = 30;
            double epsilon = 1e-4;

            for (int i = 0; i < maxIteration; i++) {
                std::cout << "Iterations:" << i << std::endl;
                Eigen::VectorXd dx = LinearizeAndSolve(Vertices, Edges);

                //update
                for (size_t j = 0; j < Vertices.size(); j++) {
                    Vertices[j] += dx.block(3 * j, 0, 3, 1);
                    NormalAngle(Vertices[j][2]);
                }

                double maxError = -1;
                for (int k = 0; k < 3 * Vertices.size(); k++) {
                    if (maxError < std::fabs(dx(k))) {
                        maxError = std::fabs(dx(k));
                    }
                }
                std::cout << "Iterations-" << i;
                std::cout << "-error: " << ComputeError(Vertices, Edges) << std::endl;

                if (maxError < epsilon)
                    break;
            }

            double finalError = ComputeError(Vertices, Edges);

            std::cout << "FinalError:" << finalError << std::endl;
            //solve problem and update pose using optimization result
            std::cout<<"posegraph has been solved"<<std::endl;
            tmp->points.clear();
            pointCloud->points.clear();
            int i = 0;
            for(auto it = LO->map_->keyframes_.lower_bound(old_cnt);it!=LO->map_->keyframes_.find(n-2);it++)
            {
                // std::cout<<"-----------initial  is------------"<<std::endl;
                // std::cout<<it->second->true_pose.transpose()<<std::endl;
                it->second->set_true_pose(Vertices[i]);
                // std::cout<<"-----------updatedpose is------------"<<std::endl;
                // std::cout<<it->second->true_pose.transpose()<<std::endl;
                i++;
            }
            //visulize the pointcloud after optimization
            for(auto it = LO->map_->keyframes_.begin();it!=LO->map_->keyframes_.find(n-2);it++)
            {
                it->second->compute_world_coords(it->second->true_pose);
                for(int i=0;i<it->second->nrays;i++)
                {
                    if(!(it->second->valid[i])) continue;
                    PointT point;
                    point.x =  it->second->points_w[i].p[0];
                    point.y =  it->second->points_w[i].p[1];
                    point.z = 0.0;
                    tmp->points.push_back(point);
                }
            }
            tmp->is_dense = false;

            tmp->swap(*pointCloud);

            pointCloud->width = 1;
            pointCloud->height = pointCloud->points.size();

            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*pointCloud, output);
            output.header.frame_id = "odom";

            pcl_pub.publish(output); 
        }
        
    }
    
    
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "record");
    ros::NodeHandle n;

    Odom_calib.Set_data_len(2000);
    Odom_calib.set_data_zero();

    qr_codeMap["qrc0000"] = 0;
    qr_codeMap["slam01"] = 1;
    qr_codeMap["slam02"] = 2;
    qr_codeMap["slam03"] = 3;
    qr_codeMap["slam04"] = 4;
    qr_codeMap["slam05"] = 5;
    qr_codeMap["slam06"] = 6;
    qr_codeMap["slam07"] = 7;
    qr_codeMap["slam08"] = 8;
    qr_codeMap["slam09"] = 9;
    qr_codeMap["slam10"] = 10;

    // std::ifstream calib_info;
    // calib_info.open("result.txt");
    // if(!calib_info.is_open())
    // {
    //     std::cout<<"can not open file !!!!!!"<<std::endl;
    //     return 0;
    // }
    // std::string calib_line;
    // int count = 0;
    // while(std::getline(calib_info, calib_line) && !calib_line.empty())
    // {
    //     int col_count = 0;
    //    Eigen::Vector3d datas;
    //     double data;
    //     std::string token;
    //     std::istringstream ssMat(calib_line);
    //     while(std::getline(ssMat, token, ' '))
    //     {
    //         // if((token == ' ') continue;
    //         data = atof(token.c_str()); 
    //         if(data == 0.0) continue;
    //         datas <<  data;
    //     }
    //     odom2scan.block<3,1>(count,0) = datas.transpose();
    //     count++;
    // }
    // calib_info.close();

    // std::cout<< odom2scan<< std::endl;

    
    calib_result.open("result.txt", std::fstream::out);
    if(!calib_result.is_open())
    {
        std::cout<<"can not open file !!!!!!"<<std::endl;
    }

    wheel_calib.open("result1.txt", std::fstream::out);
    if(!wheel_calib.is_open())
    {
        std::cout<<"can not open file !!!!!!"<<std::endl;
    }
    
    // fusion_result.open("result4.txt", std::fstream::out);
    // if(!fusion_result.is_open())
    // {
    //     std::cout<<"can not open file !!!!!!"<<std::endl;
    // }
     

    std::shared_ptr<Scan2> scan = std::shared_ptr<Scan2>(std::make_shared<Scan2>());



    std::thread Front(&Scan2::StartSlam, scan);

    std::thread Backend(&Scan2::StartPosegraph,scan);

    std::thread QrProcess(&Scan2::QrCodeProcess,scan);

   
    ros::MultiThreadedSpinner s(3);

   
    ros::spin(s);

    return 0;
}

