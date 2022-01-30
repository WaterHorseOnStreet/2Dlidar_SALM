#include <algorithm>
#include <boost/timer.hpp>
#include <ctime>
#include <cstdlib>


#include "msf/config.h"
#include "msf/lidar_odometry.h"


#include <Eigen/SVD>
#include <Eigen/Dense>
#include <msf/ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/filters/radius_outlier_removal.h>



typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
clock_t start_frame1;
clock_t end_frame1;
double time_cost1;

namespace msf
{

LidarOdometry::LidarOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), latest_key_frame_(nullptr), curr_ ( nullptr ), second_pc(new PointCloud), PointMap(new PointCloud),latest_key_frame_pose_({0,0,0}), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), pose_estimated_({0,0,0})
{
    initial_frame_pose_ = Eigen::Vector3d({0,0,0});
    key_frame_cnt_ = 0;
    mappoint_cnt =0;
    frame_cnt_ = 0;
    max_num_lost_       = 200000;
    min_inliers_        = 190;
    key_frame_min_rot   = 0.3;
    key_frame_min_trans = 0.4;
    max_correspondece_dist = 0.02;
    max_tolerance_error = 20.0;

    max_correspondece_dist_ba = 0.02;

}

LidarOdometry::~LidarOdometry()
{

}

bool LidarOdometry::addFrame(LaserFrame::Ptr frame,Eigen::Vector3d odom_pose, int flag, PointCloud::Ptr map)
{
    switch(state_)
    {
        case INITIALIZING:
        {
            curr_ = ref_ = frame;
            //initial_frame_pose_ = odom_pose;

            ref_->set_true_pose(initial_frame_pose_);
            curr_->set_true_pose(initial_frame_pose_);
            curr_->is_key_frame_ = true;

            ROS_INFO("----------INITIALIZING finished!!");
            addKeyFrame();

            latest_key_frame_ = curr_;
            latest_key_frame_pose_ = curr_->true_pose;
            state_ = OK;
            break;
        }
        case OK:
        {
            curr_ = frame;
            pose_estimated_ = odom_pose;
            //PointMap = map;
                
            curr_->set_odometry(pose_estimated_);
            curr_->set_estimate(pose_estimated_);
            {
                // start_frame1 = clock();
                // second_pc->points.clear();
                // curr_->compute_world_coords(curr_->true_pose);
                // Frame2PointCloud(second_pc, curr_);
                   
                // end_frame1 = clock();

                // time_cost1 = (double)(end_frame1 - start_frame1)/CLOCKS_PER_SEC;
                // std::cout<<"---------the ndt costs -------------"<<std::endl;
                // std::cout<<time_cost1*1000<<"ms"<<std::endl;

                poseEstimationNDTFrame(PointMap, curr_,curr_->odometry);
                curr_->set_true_pose(curr_->estimate);
                //MatchWithSubMap(curr_, curr_->true_pose , 0.5);  
                // poseEstimationNDTFrame(PointMap, curr_,curr_->true_pose);
                // curr_->set_true_pose(curr_->estimate);
        
                //MatchWithSubMap(curr_, curr_->true_pose , 0.7);  
                // clock_t start_frame2 = clock();
                // clock_t end_frame2 = clock();

                // double time_cost2 = (double)(end_frame2 - start_frame2)/CLOCKS_PER_SEC;
                // std::cout<<"---------the PLICP costs -------------"<<std::endl;
                // std::cout<<time_cost2*1000<<"ms"<<std::endl;

                
                num_lost_ = 0;

                if(checkKeyFrame() == true)
                {
                        
                    ROS_INFO("-----------add Keyframe-------------"); 
                    //MatchWithSubMap(curr_, curr_->true_pose , 0.5);     
                    
                    // start_frame1 = clock();
                    addKeyFrame();
                    // end_frame1 = clock();

                    // time_cost1 = (double)(end_frame1 - start_frame1)/CLOCKS_PER_SEC;
                    // std::cout<<"---------the Keyframe adding costs -------------"<<std::endl;
                    // std::cout<<time_cost1*1000<<"ms"<<std::endl;

                    std::cout<<"Keyframe id is: "<<curr_->keyframe_id_<<std::endl;
                    latest_key_frame_ = curr_;
                    latest_key_frame_pose_ = curr_->true_pose;
                }
				latest_key_frame_->connected_frames_.push_back(curr_);
                    
                ref_ = curr_;
                frame_cnt_++;
            }
            break;   
        }
        case LOST:
        {
            cout<<"LO has lost."<<endl;
            break;
        }
                    
    }
    return true;
}

Eigen::Vector3d LidarOdometry::cal_relative_pose(Eigen::Vector3d Refpose, Eigen::Vector3d Worldpose)
{
    Eigen::Vector3d d_pos;  //return value
    Eigen::Vector3d now_pos = Worldpose;
    Eigen::Vector3d last_pos = Refpose;

    //TODO:

    Eigen::Matrix3d Tnow,Tprev;

    double theta = last_pos(2);
    double x = last_pos(0);
    double y = last_pos(1);

    //前一次的位姿
    Tprev << cos(theta),-sin(theta),x,
             sin(theta), cos(theta),y,
                      0,          0,       1;

    //当前的位姿
    x = now_pos(0);
    y = now_pos(1);
    theta = now_pos(2);
    Tnow << cos(theta),-sin(theta),x,
             sin(theta), cos(theta),y,
                      0,          0,       1;

    //相对位姿
    Eigen::Matrix3d T = Tprev.inverse() * Tnow;
   

    d_pos(0) = T(0,2);
    d_pos(1) = T(1,2);
    d_pos(2) = atan2(T(1,0),T(0,0));

    //end of TODO:

    return d_pos;
}

Eigen::Vector3d LidarOdometry::relative2world(Eigen::Vector3d SourcePose, Eigen::Vector3d relativePose)
{
    Eigen::Matrix3d d_T,Tprev;
    Eigen::Vector3d last_pos = SourcePose;
    Eigen::Vector3d d_pos = relativePose;
    Eigen::Vector3d now_pos;

    double theta = last_pos(2);
    double x = last_pos(0);
    double y = last_pos(1);

    //前一次的位姿
    Tprev << cos(theta),-sin(theta),x,
             sin(theta), cos(theta),y,
                      0,          0,       1;

    x = d_pos(0);
    y = d_pos(1);
    theta = d_pos(2);
    d_T << cos(theta),-sin(theta),x,
             sin(theta), cos(theta),y,
                      0,          0,       1;

    Eigen::Matrix3d Tnow = Tprev * d_T;
    //Eigen::Matrix3d Tnow = d_T.inverse() * Tprev;

    now_pos(0) = Tnow(0,2);
    now_pos(1) = Tnow(1,2);
    now_pos(2) = atan2(Tnow(1,0),Tnow(0,0));

    return now_pos;

}

void LidarOdometry::addKeyFrame()
{
    curr_->keyframe_id_ = key_frame_cnt_;
    map_->insertKeyFrame(curr_);

    curr_->compute_world_coords(curr_->true_pose);
    for ( size_t i=0; i<curr_->nrays; i++ )
    {
        if(!curr_->valid[i]) continue;
        Vector2d p_world = Vector2d(curr_->points_w[i].p[0],curr_->points_w[i].p[1]);
        Vector2d n = p_world;
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            mappoint_cnt,curr_->keyframe_id_, p_world, n, curr_.get()
        );
        mappoint_cnt++;
        map_->insertMapPoint( map_point );
            
    }
    key_frame_cnt_++; 

    //keep a rigid size of keyframes, like a slidewindow and erase the points belonging to the oldest keyframe
    if(map_->keyframes_.size()>4000)
    {
        std::map<unsigned long, MapPoint::Ptr>::iterator it;
        for(it = map_->map_points_.begin();it!=map_->map_points_.end();)
        {
            if(it->second->factory_id_ == key_frame_cnt_-1)
            {
                map_->map_points_.erase(it++);
            }
            else
            {
                it++;
            }
        }
    }
    std::cout<<"the size of Map is "<<map_->map_points_.size()<<std::endl;

    // start_frame1 = clock();
    PointMap->points.clear();
    second_pc->points.clear();
    double x0 = curr_->true_pose(0);
    double y0 = curr_->true_pose(1);
    for(auto it = map_->map_points_.begin();it!=map_->map_points_.end();)
    {
        PointT point;
        double x1 = it->second->pos_(0);
        double y1 = it->second->pos_(1);
        if(abs(x1 - x0)> 7 || abs( y1 - y0)>7)
        {
            it ++;
            continue;
        }
        point.x = it->second->pos_(0);
        point.y = it->second->pos_(1);
        point.z = 0.0;
        PointMap->points.push_back(point);     
        it++;  
    }

    for(auto it = map_->keyframes_.begin();it!=map_->keyframes_.end();it++)
    {
        it->second->compute_world_coords(it->second->true_pose);
        for(int i=5;i<it->second->nrays-5;i++)
        {
            if(!(it->second->valid[i])) continue;
            Eigen::Matrix<double,10,2> curvature_matrix;
            for(int j= -5;j<5;j++)
            {
                curvature_matrix(j+5,0) = it->second->points[i+j].p[0];
                curvature_matrix(j+5,1) = it->second->points[i+j].p[1];
            }
            double curvature;
            curvature = 1.0/(circlefit(curvature_matrix)+10e-4);
            if(curvature > 30)
            {
                PointT point;
                point.x =  it->second->points_w[i].p[0];
                point.y =  it->second->points_w[i].p[1];
                point.z = 0.0;
                second_pc->points.push_back(point);
            }

        }
    }

    std::cout<<"points in corner pointcloud are"<<std::endl;
    std::cout<<second_pc->points.size()<<std::endl;
    // pcl::VoxelGrid<PointT> voxel_filter;
    // voxel_filter.setLeafSize(0.01,0.01,0.01);
    // voxel_filter.setInputCloud(PointMap);
    // voxel_filter.filter(*second_pc);

    //second_pc->swap(*PointMap);
    // end_frame1 = clock();

    // time_cost1 = (double)(end_frame1 - start_frame1)/CLOCKS_PER_SEC;
    // std::cout<<"---------the map update costs -------------"<<std::endl;
    // std::cout<<time_cost1*1000<<"ms"<<std::endl;

  
}

bool LidarOdometry::poseEstimationNDT(PointCloud::Ptr cloud_target, LaserFrame::Ptr source, Eigen::Vector3d& pose_prior, double& ratio)
{  

    PointCloud::Ptr query_cloud(new PointCloud);
    for(int i=0;i<source->nrays;i++)
    {
        if(!(source->valid[i])) continue;
        PointT point;
        point.x =  source->points[i].p[0];
        point.y =  source->points[i].p[1];
        point.z = 0.0;
        query_cloud->points.push_back(point);
    }

    
    //transform the Vector3d to Matrix
    Eigen::Matrix4f pose_estimate = Eigen::Matrix4f ::Identity();

    pose_estimate.block(0,0,3,3) = euler2rot(0,0,pose_prior(2));
    pose_estimate(0,3) = pose_prior(0);
    pose_estimate(1,3) = pose_prior(1);
    pose_estimate(2,3) = 0;

    Eigen::Matrix4f pose_optimized;
    double trans_probability;
    cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_optimizer;


    ndt_optimizer.setResolution(1.0);
    ndt_optimizer.setMaximumIterations(30);
    ndt_optimizer.setStepSize(0.1);
    ndt_optimizer.setTransformationEpsilon(0.01);
    
    /*
    pcl::VoxelGrid<PointT> downSampler;
    pcl::VoxelGrid<PointT> lastdownSampler;

    PointCloud::Ptr filtered_query(new PointCloud);
    lastdownSampler.setLeafSize(0.10,0.10,0.0);

    //downsample
    lastdownSampler.setInputCloud(query_cloud);
    lastdownSampler.filter(*filtered_query);

    PointCloud::Ptr filtered_table(new PointCloud);
    downSampler.setLeafSize(0.10,0.10,0.0);
    //downsample
    downSampler.setInputCloud(table_cloud);
    downSampler.filter(*filtered_table);
     */

    ndt_optimizer.setInputTarget(cloud_target);
    ndt_optimizer.setInputSource(query_cloud);
    ndt_optimizer.align(pose_estimate);

    pose_optimized = ndt_optimizer.getFinalTransformation();

    trans_probability = ndt_optimizer.getTransformationProbability();

    if(trans_probability<2.0)
    {
        std::cout<<"-----------------------------trans_probability------- =  "<<trans_probability<<std::endl;
        pose_optimized = pose_estimate;
        return false;
    }

    ratio = 10*trans_probability;
    std::cout<<"-----------------------------trans_probability------- =  "<<trans_probability<<std::endl;

    Eigen::Quaternionf quat(pose_optimized.block<3,3>(0, 0));
    quat.normalize();

    double roll,pitch,yaw;
    tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())).getRPY(roll, pitch, yaw);

    pose_prior(0) = pose_optimized(0,3);
    pose_prior(1) = pose_optimized(1,3);
    pose_prior(2) = yaw;

    return true;
}

void LidarOdometry::poseEstimationNDTFrame(PointCloud::Ptr cloud_target, LaserFrame::Ptr source, Eigen::Vector3d pose_prior)
{
    PointCloud::Ptr query_cloud(new PointCloud);
    for(int i=0;i<source->nrays;i++)
    {
        if(!(source->valid[i])) continue;
        PointT point;
        point.x =  source->points[i].p[0];
        point.y =  source->points[i].p[1];
        point.z = 0.0;
        query_cloud->points.push_back(point);
    }

    //transform the Vector3d to Matrix
    Eigen::Matrix4f pose_estimate = Eigen::Matrix4f ::Identity();

    pose_estimate.block(0,0,3,3) = euler2rot(0,0,pose_prior(2));
    pose_estimate(0,3) = pose_prior(0);
    pose_estimate(1,3) = pose_prior(1);
    pose_estimate(2,3) = 0;

    Eigen::Matrix4f pose_optimized;
    double trans_probability;
    cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_optimizer;


    ndt_optimizer.setResolution(0.5);
    ndt_optimizer.setMaximumIterations(30);
    ndt_optimizer.setStepSize(0.1);
    ndt_optimizer.setTransformationEpsilon(0.01);
    
    /*
    pcl::VoxelGrid<PointT> downSampler;
    pcl::VoxelGrid<PointT> lastdownSampler;

    PointCloud::Ptr filtered_query(new PointCloud);
    lastdownSampler.setLeafSize(0.10,0.10,0.0);

    //downsample
    lastdownSampler.setInputCloud(query_cloud);
    lastdownSampler.filter(*filtered_query);

    PointCloud::Ptr filtered_table(new PointCloud);
    downSampler.setLeafSize(0.10,0.10,0.0);
    //downsample
    downSampler.setInputCloud(table_cloud);
    downSampler.filter(*filtered_table);
     */

    ndt_optimizer.setInputTarget(PointMap);
    ndt_optimizer.setInputSource(query_cloud);
    ndt_optimizer.align(pose_estimate);

    pose_optimized = ndt_optimizer.getFinalTransformation();

    trans_probability = ndt_optimizer.getTransformationProbability();

    if(trans_probability<0.5 || sqrt(pow((pose_optimized(0,3) - ref_->true_pose(0)),2)+ pow((pose_optimized(1,3)-ref_->true_pose(1)),2))>0.15)
    {
        pose_optimized = pose_estimate;
    }

    std::cout<<"-----------------------------trans_probability------- =  "<<trans_probability<<std::endl;

    Eigen::Quaternionf quat(pose_optimized.block<3,3>(0, 0));
    quat.normalize();

    double roll,pitch,yaw;
    tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())).getRPY(roll, pitch, yaw);

    Eigen::Vector3d pose_1;
    pose_1(0) = pose_optimized(0,3);
    pose_1(1) = pose_optimized(1,3);
    pose_1(2) = yaw;

    source->set_estimate(pose_1);
}

bool LidarOdometry::checkKeyFrame()
{
    
    Eigen::Vector3d d_pos;  

    d_pos = cal_relative_pose(latest_key_frame_pose_, pose_estimated_);

    double dist = sqrt(d_pos(0) * d_pos(0) + d_pos(1) * d_pos(1));
    if(dist >key_frame_min_trans )// || abs(d_pos(2)) > key_frame_min_rot)
    //if(abs(d_pos(0))>key_frame_min_trans || abs(d_pos(1))>key_frame_min_trans   || dist >key_frame_min_trans )
    {
        curr_->is_key_frame_ = true;
    }
    
    if(curr_->is_key_frame_)
    {
        return true;
    }
    return false;
}

void LidarOdometry::MatchWithSubMap(LaserFrame::Ptr frame,Eigen::Vector3d pose_estimate, double ratio)
{
    poseEstPLICPWithMap(PointMap, frame, pose_estimate);
    //poseEstPLICPWithMapFrame(PointMap, frame, pose_estimate);

    // PointCloud& spec = *PointMap;
    // poseEstPLICPIMLS(spec,frame,pose_estimate);
    
}


double LidarOdometry::computeDerivatives(Eigen::Vector3d& gradient, Eigen::Matrix3d& hessian, PointCloud::Ptr cloud, LaserFrame::Ptr frame, Eigen::Vector3d& iter_pose, double distri_count)
{
    gradient.setZero();
    hessian.setZero();

    double error;
    double coff = 0.0;

    const int M = 30;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointsIdxKNNSearch(M);
    std::vector<float> pointKNNSquaredDist(M);
    
    //int distri_count = 0;
    for(int i=1;i<frame->nrays-1;++i)
    {
        if(!(frame->valid[i])) 
        {
            continue;
        }  

        Eigen::Vector2d p_i_w({ frame->points_w[i].p[0],frame->points_w[i].p[1]});

        PointT searchPoint;
        searchPoint.x = frame->points_w[i].p[0];
        searchPoint.y = frame->points_w[i].p[1];
        searchPoint.z = 0.0;

        if(kdtree.nearestKSearch(searchPoint, M, pointsIdxKNNSearch, pointKNNSquaredDist)>0)
        {
            if(pointKNNSquaredDist[M-1]<0.2 && pointKNNSquaredDist[1]<0.02)
            {
                Eigen::Matrix<double,M,2> input;
                for(int i=0;i<M;i++)
                {
                    input(i,0) = cloud->points[pointsIdxKNNSearch[i]].x;
                    input(i,1) = cloud->points[pointsIdxKNNSearch[i]].y;
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
        
                Eigen::Vector2d line_dirct;
                double first = max(first_ev, second_ev);
                double second = min(first_ev, second_ev);

                if(floor(first/second)>9)
                {
                    line_dirct = (first == first_ev)?  eigen_solver.eigenvectors().col(0).real() : eigen_solver.eigenvectors().col(1).real();
                    line_dirct.normalize();

                    Eigen::Vector2d norm_vec;

                    double y = -line_dirct(0)/line_dirct(1);
                    Eigen::Vector2d p_j1_i = p_i_w - meanVec;
                    norm_vec(0) = 1;
                    norm_vec(1) = y;
                    if((norm_vec.transpose()*p_j1_i)<0)
                    {
                        norm_vec = -norm_vec;
                    }
                    norm_vec.normalize();
                    
                    error += norm_vec.transpose()*p_j1_i;

                    double cz, sz;
                    if(fabs(iter_pose(2) <10e-5))
                    {
                        cz = 1.0;
                        sz = 0.0;
                    }
                    else
                    {
                        cz = cos(iter_pose(2));
                        sz = sin(iter_pose(2));
                    }

                    double x0,y0;
                    x0 = frame->points[i].p[0];
                    y0 = frame->points[i].p[1];
        
                    Eigen::Vector3d tmp;
                    tmp(0) = norm_vec(0);
                    tmp(1) = norm_vec(1);
                    tmp(2) = norm_vec(0) * (- sz*x0 - cz*y0) + norm_vec(1) * (cz*x0 - sz*y0);

                    double weight = (double)covMat(0,1)/(sqrt(covMat(0,0)*covMat(1,1)));
                    hessian += weight*tmp*(tmp.transpose());
                    gradient -= weight*tmp*(norm_vec.transpose()*p_j1_i); 
                    distri_count++;
                }
                else
                {
                    continue;
                }
                
            }
            else 
            {
                continue;
            }
        }
        else
        {
            continue;
        }
    }
    //std::cout<<"there are"<<distri_count<<"points in problem construction"<<std::endl;
    return error;
}

double LidarOdometry::computeDerivativesRadius(Eigen::Vector3d& gradient, Eigen::Matrix3d& hessian, PointCloud::Ptr cloud, LaserFrame::Ptr frame, Eigen::Vector3d& iter_pose, double distri_count)
{
   gradient.setZero();
    hessian.setZero();

    double error = 0.0;
    double coff = 0.0;

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    float radius = 0.1;
    std::vector<int> pointsIdxKNNSearch;
    std::vector<float> pointKNNSquaredDist;

    pcl::KdTreeFLANN<PointT> kdtree_corner;
    kdtree_corner.setInputCloud(second_pc);

    std::vector<int> pointsIdxKNNSearch_corner;
    std::vector<float> pointKNNSquaredDist_corner;
    
    
    //int distri_count = 0;
    for(int i=5;i<frame->nrays-5;++i)
    {
        if(!(frame->valid[i])) 
        {
            continue;
        }  

        Eigen::Matrix<double,10,2> curvature_matrix;
        for(int j= -5;j<5;j++)
        {
            curvature_matrix(j+5,0) = frame->points[i+j].p[0];
            curvature_matrix(j+5,1) = frame->points[i+j].p[1];
        }
        double curvature;
        curvature = 1.0/(circlefit(curvature_matrix)+10e-4);
        if(curvature > 30)
        {
            //then this corner point will try to find correspondence in corner pointcloud
            Eigen::Vector2d p_i_w({ frame->points_w[i].p[0],frame->points_w[i].p[1]});

            PointT searchPoint;
            searchPoint.x = frame->points_w[i].p[0];
            searchPoint.y = frame->points_w[i].p[1];
            searchPoint.z = 0.0;

            if(kdtree_corner.radiusSearch(searchPoint, 0.1, pointsIdxKNNSearch_corner, pointKNNSquaredDist_corner)>0)
            {
                if(pointsIdxKNNSearch_corner.size() >5 & pointKNNSquaredDist_corner[0]<0.02 )
                {
                    double sum_x = 0.0;
                    double sum_y = 0.0;
                    for(int j = 0;j<pointsIdxKNNSearch_corner.size();j++)
                    {
                        sum_x += second_pc->points[pointsIdxKNNSearch_corner[j]].x;
                        sum_y += second_pc->points[pointsIdxKNNSearch_corner[j]].y;
                    }

                    double ave_x = sum_x/pointsIdxKNNSearch_corner.size();
                    double ave_y = sum_y/pointsIdxKNNSearch_corner.size();

                    double dist = sqrt((frame->points_w[i].p[0] - ave_x)*(frame->points_w[i].p[0] - ave_x)
                                                            +(frame->points_w[i].p[1] - ave_y)*(frame->points_w[i].p[1] - ave_y));
                    
                    if(dist>0.5) continue;
                    error += dist;

                    double cz, sz;
                    if(fabs(iter_pose(2) <10e-5))
                    {
                        cz = 1.0;
                        sz = 0.0;
                    }
                    else
                    {
                        cz = cos(iter_pose(2));
                        sz = sin(iter_pose(2));
                    }

                    double x0,y0;
                    x0 = frame->points[i].p[0];
                    y0 = frame->points[i].p[1];
        
                    Eigen::Vector3d tmp;
                    tmp(0) = (frame->points_w[i].p[0] - ave_x)/dist;
                    tmp(1) = (frame->points_w[i].p[1] - ave_y)/dist;
                    tmp(2) = (frame->points_w[i].p[0] - ave_x) * (- sz*x0 - cz*y0) + (frame->points_w[i].p[1] - ave_y)* (cz*x0 - sz*y0);

                    double weight = 1 - 1.8*dist;
                    hessian += 0.1*weight*tmp*(tmp.transpose());
                    gradient -= 0.1*weight*tmp*dist; 
                    distri_count++;   
                }
                else 
                {
                    continue;
                }
            } 
            else
            {
                continue;
            }                
        }

        Eigen::Vector2d p_i_w({ frame->points_w[i].p[0],frame->points_w[i].p[1]});

        PointT searchPoint;
        searchPoint.x = frame->points_w[i].p[0];
        searchPoint.y = frame->points_w[i].p[1];
        searchPoint.z = 0.0;

        if(kdtree.radiusSearch(searchPoint, radius, pointsIdxKNNSearch, pointKNNSquaredDist)>0)
        {
            if(pointsIdxKNNSearch.size() >10 & pointKNNSquaredDist[0]<0.02 )
            {
                int M = pointsIdxKNNSearch.size();
                Eigen::MatrixXd input;
                 input.conservativeResize(M,2);
                for(int i=0;i<M;i++)
                {
                    input(i,0) = cloud->points[pointsIdxKNNSearch[i]].x;
                    input(i,1) = cloud->points[pointsIdxKNNSearch[i]].y;
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
        
                Eigen::Vector2d line_dirct;
                double first = max(first_ev, second_ev);
                double second = min(first_ev, second_ev);

                if(floor(first/second)>12)
                {
                    line_dirct = (first == first_ev)?  eigen_solver.eigenvectors().col(0).real() : eigen_solver.eigenvectors().col(1).real();
                    line_dirct.normalize();

                    Eigen::Vector2d norm_vec;

                    double y = -line_dirct(0)/line_dirct(1);
                    Eigen::Vector2d p_j1_i = p_i_w - meanVec;
                    norm_vec(0) = 1;
                    norm_vec(1) = y;
                    if((norm_vec.transpose()*p_j1_i)<0)
                    {
                        norm_vec = -norm_vec;
                    }
                    norm_vec.normalize();
                    
                    error += norm_vec.transpose()*p_j1_i;

                    double cz, sz;
                    if(fabs(iter_pose(2) <10e-5))
                    {
                        cz = 1.0;
                        sz = 0.0;
                    }
                    else
                    {
                        cz = cos(iter_pose(2));
                        sz = sin(iter_pose(2));
                    }

                    double x0,y0;
                    x0 = frame->points[i].p[0];
                    y0 = frame->points[i].p[1];
        
                    Eigen::Vector3d tmp;
                    tmp(0) = norm_vec(0);
                    tmp(1) = norm_vec(1);
                    tmp(2) = norm_vec(0) * (- sz*x0 - cz*y0) + norm_vec(1) * (cz*x0 - sz*y0);

                    double weight = (double)covMat(0,1)/(sqrt(covMat(0,0)*covMat(1,1)));
                    hessian += weight*tmp*(tmp.transpose());
                    gradient -= weight*tmp*(norm_vec.transpose()*p_j1_i); 
                    distri_count++;
                }
                else
                {
                    continue;
                }
                
            }
            else 
            {
                continue;
            }
        }
        else
        {
            continue;
        }
    }
    std::cout<<"there are"<<distri_count<<"points in problem construction"<<std::endl;
    return error;
}
//develop
double LidarOdometry::computeStepLengthMT(Eigen::Vector3d x,Eigen::Vector3d step_dir,double step_init,double step_max,
                                                                                                        double step_min, double error,Eigen::Vector3d gradient,Eigen::Matrix3d hessian,
                                                                                                        PointCloud::Ptr cloud,LaserFrame::Ptr frame)
{
    double threshold;
    double phi_0 = (-1)*error;
	double d_phi_0 = (-1)*gradient.dot(step_dir);

	Eigen::Vector3d x_t;

	if (d_phi_0 >= 0) {
		if (d_phi_0 == 0) {
			return 0;
		} else {
			d_phi_0 *= -1;
			step_dir *= -1;
		}
	}

	int max_step_iterations = 10;
	int step_iterations = 0;

	double mu = 1.e-4;
	double nu = 0.9;
	double a_l = 0, a_u = 0;

	double f_l = auxilaryFunction_PsiMT(a_l, phi_0, phi_0, d_phi_0, mu);
	double g_l = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

	double f_u = auxilaryFunction_PsiMT(a_u, phi_0, phi_0, d_phi_0, mu);
	double g_u = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

	bool interval_converged = (step_max - step_min) > 0, open_interval = true;

	double a_t = step_init;
	a_t = std::min(a_t, step_max);
	a_t = std::max(a_t, step_min);

	x_t = x + step_dir * a_t;

    frame->set_true_pose(x_t);
    frame->compute_world_coords(x_t);

	error = computeDerivatives(gradient, hessian, cloud,frame, x_t,threshold);

	double phi_t = (-1)*error;
	double d_phi_t = (-1)*gradient.dot(step_dir);
	double psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
	double d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

	while (!interval_converged && step_iterations < max_step_iterations && !(psi_t <= 0 && d_phi_t <= -nu * d_phi_0)) {
		if (open_interval) {
			a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
		} else {
			a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
		}

		a_t = (a_t < step_max) ? a_t : step_max;
		a_t = (a_t > step_min) ? a_t : step_min;

		x_t = x + step_dir * a_t;

        frame->set_true_pose(x_t);
        frame->compute_world_coords(x_t);

        error = computeDerivatives(gradient, hessian, cloud,frame, x_t,threshold);

		phi_t += (-1)*error;
		d_phi_t += (-1)*(gradient.dot(step_dir));
		psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
		d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

		if (open_interval && (psi_t <= 0 && d_psi_t >= 0)) {
			open_interval = false;

			f_l += phi_0 - mu * d_phi_0 * a_l;
			g_l += mu * d_phi_0;

			f_u += phi_0 - mu * d_phi_0 * a_u;
			g_u += mu * d_phi_0;
		}

		if (open_interval) {
			interval_converged = updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
		} else {
			interval_converged = updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
		}
		step_iterations++;
	}

	if (step_iterations) {
         frame->set_true_pose(x_t);
        frame->compute_world_coords(x_t);
		computeDerivatives(gradient, hessian, cloud,frame, x_t,threshold);
	}

	//real_iterations_ += step_iterations;

	return a_t;
}
//develop
double LidarOdometry::updateIntervalMT (double &a_l, double &f_l, double &g_l,
																							double &a_u, double &f_u, double &g_u,
																							double a_t, double f_t, double g_t)
{
  // Case U1 in Update Algorithm and Case a in Modified Update Algorithm [More, Thuente 1994]
	if (f_t > f_l) {
		a_u = a_t;
		f_u = f_t;
		g_u = g_t;
		return (false);
	}
	// Case U2 in Update Algorithm and Case b in Modified Update Algorithm [More, Thuente 1994]
	else if (g_t * (a_l - a_t) > 0) {
		a_l = a_t;
		f_l = f_t;
		g_l = g_t;
		return (false);
	}
	// Case U3 in Update Algorithm and Case c in Modified Update Algorithm [More, Thuente 1994]
	else if (g_t * (a_l - a_t) < 0) {
		a_u = a_l;
		f_u = f_l;
		g_u = g_l;

		a_l = a_t;
		f_l = f_t;
		g_l = g_t;
		return (false);
	}
	// Interval Converged
	else {
		return (true);
	}
}
//develop
double  LidarOdometry::trialValueSelectionMT (double a_l, double f_l, double g_l,
																								double a_u, double f_u, double g_u,
																								double a_t, double f_t, double g_t)
{
	// Case 1 in Trial Value Selection [More, Thuente 1994]
	if (f_t > f_l) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		double w = std::sqrt (z * z - g_t * g_l);
		// Equation 2.4.56 [Sun, Yuan 2006]
		double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

		// Calculate the minimizer of the quadratic that interpolates f_l, f_t and g_l
		// Equation 2.4.2 [Sun, Yuan 2006]
		double a_q = a_l - 0.5 * (a_l - a_t) * g_l / (g_l - (f_l - f_t) / (a_l - a_t));

		if (std::fabs (a_c - a_l) < std::fabs (a_q - a_l)) {
		  return (a_c);
		} else {
		  return (0.5 * (a_q + a_c));
		}
	}
	// Case 2 in Trial Value Selection [More, Thuente 1994]
	else if (g_t * g_l < 0) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		double w = std::sqrt (z * z - g_t * g_l);
		// Equation 2.4.56 [Sun, Yuan 2006]
		double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

		// Calculate the minimizer of the quadratic that interpolates f_l, g_l and g_t
		// Equation 2.4.5 [Sun, Yuan 2006]
		double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

		if (std::fabs (a_c - a_t) >= std::fabs (a_s - a_t)) {
		  return (a_c);
		} else {
		  return (a_s);
		}
	}
	// Case 3 in Trial Value Selection [More, Thuente 1994]
	else if (std::fabs (g_t) <= std::fabs (g_l)) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		double w = std::sqrt (z * z - g_t * g_l);
		double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

		// Calculate the minimizer of the quadratic that interpolates g_l and g_t
		// Equation 2.4.5 [Sun, Yuan 2006]
		double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

		double a_t_next;

		if (std::fabs (a_c - a_t) < std::fabs (a_s - a_t)) {
		  a_t_next = a_c;
		} else {
		  a_t_next = a_s;
		}

		if (a_t > a_l) {
		  return (std::min (a_t + 0.66 * (a_u - a_t), a_t_next));
		} else {
		  return (std::max (a_t + 0.66 * (a_u - a_t), a_t_next));
		}
	}
	// Case 4 in Trial Value Selection [More, Thuente 1994]
	else {
		// Calculate the minimizer of the cubic that interpolates f_u, f_t, g_u and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		double z = 3 * (f_t - f_u) / (a_t - a_u) - g_t - g_u;
		double w = std::sqrt (z * z - g_t * g_u);
		// Equation 2.4.56 [Sun, Yuan 2006]
		return (a_u + (a_t - a_u) * (w - g_u - z) / (g_t - g_u + 2 * w));
	}
}
void LidarOdometry::poseEstPLICPWithMap(PointCloud::Ptr cloud, LaserFrame::Ptr frame, Eigen::Vector3d pose_estimated)
{
    bool converged = false;
    int n_iterations = 0;
    double step_size = 0.1;
    double epsilon = 0.05;
    double last_error;


    Eigen::Matrix3d hessian;
    Eigen::Vector3d p,delta_p,gradient;

    p = pose_estimated;

    double error = 0.0;

   //error = computeDerivatives(gradient, hessian, cloud, frame, p);
   double ratio = 0.01;
   double threshold;

   //filter
   for(int i=1;i<frame->nrays-1;i++)
   {
        if(!(frame->valid[i])) 
        {
            continue;
        }  
        //filte the outlier points 
        double diffX = frame->points[i+1].p[0] - frame->points[i].p[0];
        double diffY = frame->points[i+1].p[1] - frame->points[i].p[1];

        double diff = diffX*diffX + diffY*diffY;
        // std::cout<<"the dist between two points is"<<std::endl;
        // std::cout<<diff<<std::endl;

        if(diff<50e-6) 
        {
            frame->valid[i] = 0;
            continue;
        }

        double depth1 = sqrt(frame->points[i].p[0]*frame->points[i].p[0]
                                                    + frame->points[i].p[1]*frame->points[i].p[1]);

        double depth2 = sqrt(frame->points[i+1].p[0]*frame->points[i+1].p[0]
                                            + frame->points[i+1].p[1]*frame->points[i+1].p[1]);

        // std::cout<<"the dist between two points is"<<std::endl;
        // std::cout<<abs(depth1-depth2)<<std::endl;

        if(abs(depth1-depth2)>0.1)
        {
            frame->valid[i] = 0;
            frame->valid[i+1] = 0;
            frame->valid[i+2] = 0;
            continue;

        }
        // if(depth1>depth2)
        // {
        //     diffX = frame->points[i+1].p[0] - frame->points[i].p[0]*depth2 / depth1;
        //     diffY = frame->points[i+1].p[1] - frame->points[i].p[1]*depth2 / depth1;

        //     std::cout<<"the dist between two points is"<<std::endl;
        //     std::cout<<(sqrt(diffX*diffX+diffY*diffY)/depth2)<<std::endl;

        //     if(sqrt(diffX*diffX+diffY*diffY)/depth2<0.1) 
        //     {
        //         frame->valid[i] = 0;
        //         frame->valid[i+1] = 0;
        //         continue;
        //     }
        // }
        // else
        // {
        //     diffX = frame->points[i+1].p[0]*depth1 / depth2 - frame->points[i].p[0];
        //     diffY = frame->points[i+1].p[1]*depth1 / depth2 - frame->points[i].p[1];

        //     if(sqrt(diffX*diffX+diffY*diffY)/depth1 < 0.1) 
        //     {
        //         frame->valid[i] = 0;
        //         frame->valid[i+1] = 0;
        //         continue;
        //     }
        // }

        double diffX2 = frame->points[i].p[0] - frame->points[i-1].p[0];
        double diffY2 = frame->points[i].p[1] - frame->points[i-1].p[1];

        double diff2 = diffX2*diffX2 + diffY2*diffY2;

        double dist = frame->points[i].p[0]*frame->points[i].p[0]
                                    + frame->points[i].p[1]*frame->points[i].p[1];

        // std::cout<<"the dist between two points is"<<std::endl;
        // std::cout<<diff/dist<<std::endl;
        if(diff>0.0002*dist && diff2 >0.0002*dist)
        {
            frame->valid[i] = 0;
            continue;
        }
   }

    while(!converged)
    {
        frame->compute_world_coords(p);
        //error = computeDerivatives(gradient, hessian, cloud, frame, p, threshold);
        error =   computeDerivativesRadius(gradient, hessian, cloud, frame, p, threshold);
       //if(threshold <10) break;

        Eigen::JacobiSVD<Eigen::Matrix3d> sv(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);

        delta_p = sv.solve(gradient);

        double delta_p_norm = delta_p.norm();

        if(delta_p_norm == 0) 
        {
            frame->set_true_pose(p);
            return;
        }

        delta_p.normalize();

        //double ratio = computeStepLengthMT(p,delta_p,delta_p_norm, step_size,epsilon,error,gradient,hessian,cloud,frame);
        //std::cout<<"----------delta_p_norm--------------"<<std::endl;
        //std::cout<<delta_p_norm<<std::endl;

        if(!(error < last_error) && n_iterations >5) ratio *= 0.8; 

        delta_p *= ratio*delta_p_norm;

        p = p + delta_p;

        last_error = error;

        if(n_iterations > 10 || (n_iterations && (std::fabs(delta_p_norm)<0.001)))
        {
            converged = true;
        }
        n_iterations++;
    }
    if(sqrt(pow((p(0) - ref_->true_pose(0)),2)+ pow((p(1)-ref_->true_pose(1)),2))>0.10)
    {
        p = pose_estimated;
    }
    frame->set_true_pose(p);
}

void LidarOdometry::poseEstPLICPWithMapFrame(PointCloud::Ptr cloud, LaserFrame::Ptr frame, Eigen::Vector3d pose_estimated)
{
    /*the process is basicly the same with PLICP between two frames, the differrence is that we use 
    pointcloud instead of reference frame                                                        
    */
    int iteration_cnt = 15;
    Eigen::Vector3d iter_pose;
    iter_pose = pose_estimated;
    int fail_iteration = 0;
    double ni = 2.0;
    double current_lambda;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    for(int j=0;j<iteration_cnt;++j)
    {
        frame->compute_world_coords(iter_pose);
        //first step: find correspondence using tools in pcl library
        int corr_cnt = 0;
        int valid_sum = 0;

        for(int i=0; i<frame->nrays; i++)
        {
            if(!(frame->valid[i])) 
            {
                frame->set_corr_null(i);
                continue;
            }  
            valid_sum ++;

            int K = 2;
            std::vector<int> pointsIdxKNNSearch(K);
            std::vector<float> pointKNNSquaredDist(K);

            PointT searchPoint;
            searchPoint.x = frame->points_w[i].p[0];
            searchPoint.y = frame->points_w[i].p[1];
            searchPoint.z = 0.0;
            if(kdtree.nearestKSearch(searchPoint, K, pointsIdxKNNSearch, pointKNNSquaredDist)>0)
            {
                if(pointKNNSquaredDist[1] > max_correspondece_dist) 
                {
                    frame->set_corr_null(i);
                    continue;
                }
                frame->set_correspondence(i,pointsIdxKNNSearch[0],pointsIdxKNNSearch[1], pointKNNSquaredDist[0]);
                corr_cnt++;
            }
            else
            {
                frame->set_corr_null(i);
                continue;
            }          
        }
        //use correspondence ratio as information matrix factor in posegraph
        frame->corr_info = (double)corr_cnt/valid_sum;
        //std::cout<<"in"<<valid_sum<<"points"<<corr_cnt<<"correspondence"<<std::endl;
        if(!(corr_cnt>0.30*valid_sum))
        {
            std::cout<<"the number of good correspondence can not be accepted"<<std::endl;
            frame->set_true_pose(pose_estimated);
            return;
        }

        //second step: compute the error and solve the problem
        Eigen::Vector3d Jacobian({0,0,0});
        Eigen::Matrix<double,3,3> Hessian;
        Hessian.setZero();
        Eigen::Vector3d b_prior({0,0,0});
        double error = 0;

        for(int i=0;i<frame->nrays;++i)
        {
            if(!(frame->corr[i].valid)) continue;
            Eigen::Vector2d norm_vec;
            Eigen::Vector2d p_i_w({ frame->points_w[i].p[0],frame->points_w[i].p[1]});
            Eigen::Vector2d p_j1({cloud->points[frame->corr[i].j1].x,cloud->points[frame->corr[i].j1].y});
            Eigen::Vector2d p_j2({cloud->points[frame->corr[i].j2].x,cloud->points[frame->corr[i].j2].y});
            Eigen::Vector2d p_1_2 = p_j2 - p_j1;
            Eigen::Vector2d p_j1_i = p_i_w - p_j1;
            double y = -p_1_2(0)/p_1_2(1);
            norm_vec(0) = 1;
            norm_vec(1) = y;
            if((norm_vec.transpose()*p_j1_i)<0)
            {
                norm_vec = -norm_vec;
            }
            norm_vec.normalize();
           
            error += norm_vec.transpose()*p_j1_i;
            
            Eigen::Vector3d tmp;
            tmp(0) = norm_vec(0);
            tmp(1) = norm_vec(1);
            tmp(2) = norm_vec(0) * (-sin(iter_pose(2))*frame->points[i].p[0]-cos(iter_pose(2))*frame->points[i].p[1])
                      + norm_vec(1) * (cos(iter_pose(2))*frame->points[i].p[0]-sin(iter_pose(2))*frame->points[i].p[1]);

            Hessian += tmp*(tmp.transpose());
            b_prior -= tmp*(norm_vec.transpose()*p_j1_i); 
        }

        ulong size = Hessian.cols();
        if(j==0)
        {
            double maxDiagonal = 0;
            
            for (ulong i = 0; i < size; ++i) 
            {
                maxDiagonal = std::max(fabs(Hessian(i, i)), maxDiagonal);
            }
            maxDiagonal = std::min(5e10, maxDiagonal);
            double tau = 1e-4;  // 1e-5
            current_lambda = tau * maxDiagonal;
            //current_lambda = 0;
        }

        for (ulong i = 0; i < size; ++i) 
        {
            Hessian(i, i) += current_lambda;
        }
        Hessian(2,1) = Hessian(1,2);
        Hessian(3,1) = Hessian(1,3);
        Hessian(3,2) = Hessian(2,3);

        Eigen::Vector3d delta_pose = Hessian.colPivHouseholderQr().solve(b_prior);

        //third step: compute the error and decide whether to update
        frame->compute_world_coords(iter_pose+delta_pose);
        double total=0.0;
        for(int i=0;i<frame->nrays;i++)
        {
            if(!frame->valid[i]) continue;
            if(!frame->corr[i].valid) continue;

            Eigen::Vector2d norm_vec;
            Eigen::Vector2d p_i_w({ frame->points_w[i].p[0],frame->points_w[i].p[1]});
            Eigen::Vector2d p_j1({cloud->points[frame->corr[i].j1].x,cloud->points[frame->corr[i].j1].y});
            Eigen::Vector2d p_j2({cloud->points[frame->corr[i].j2].x,cloud->points[frame->corr[i].j2].y});
            Eigen::Vector2d p_1_2 = p_j2 - p_j1;
            Eigen::Vector2d p_j1_i = p_i_w - p_j1;
            double y = -p_1_2(0)/p_1_2(1);
            norm_vec(0) = 1;
            norm_vec(1) = y;
            if((norm_vec.transpose()*p_j1_i)<0)
            {
                norm_vec = -norm_vec;
            }
            norm_vec.normalize();

            total += (norm_vec.transpose()*p_j1_i);
        }

        double scale = 0;
        scale = 0.5* delta_pose.transpose() * (current_lambda * delta_pose + b_prior);
        double rho = (error - total) / scale;
        if(!(rho>0))
        {
            current_lambda *= ni;
            ni *= 2;
            fail_iteration++;
            continue;
        }
        //justify the current_lambda to make algorithm work better
        double alpha = 1.0 - pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        current_lambda *= scaleFactor;
        ni = 4.0;
        iter_pose += delta_pose;
        fail_iteration =0;

    }
    frame->set_true_pose(iter_pose);
}

//develop
void LidarOdometry::pointCloudFilter(PointCloud::Ptr cloud)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    //const int K = 10;
    float radius = 0.5;

    std::vector<int> pointsIdxKNNSearch;
    std::vector<float> pointKNNSquaredDist;

    Eigen::MatrixXd input;
    input.setZero();

    for(int i=0; i<cloud->points.size();i++)
    {
        if(kdtree.radiusSearch(cloud->points[i], radius, pointsIdxKNNSearch, pointKNNSquaredDist)>0)
        {
            int K = pointsIdxKNNSearch.size();
            input.conservativeResize(K,2);

            for(int i=0;i<K;i++)
            {
                input(i,0) = cloud->points[pointsIdxKNNSearch[i]].x;
                input(i,1) = cloud->points[pointsIdxKNNSearch[i]].y;
            }

            //std::cout<<input.size()<<std::endl;
            Eigen::MatrixXd meanVec = input.colwise().mean();
            Eigen::RowVectorXd meanVectorRow(Eigen::RowVectorXd::Map(meanVec.data(),input.cols()));
            // std::cout<<meanVec<<std::endl;
            // std::cout<<meanVectorRow<<std::endl;

            Eigen::MatrixXd zeroMeanMat = input;
            zeroMeanMat.rowwise() -= meanVectorRow;
            // std::cout<<zeroMeanMat<<std::endl;
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
            //std::cout<<first_ev<<"   "<<second_ev<<std::endl;
            Eigen::Vector2d line_dirct;
            double first = max(first_ev, second_ev);
            double second = min(first_ev, second_ev);

            //how to filter the outlier points of a line feature, so that line can be 

            if(floor(first/second)>9)
            {
                    //
            }
            else
            {
                Eigen::Vector2d center;
                double rs;

                //rs = circlefit(input.block<6,2>(0,0));
                if(rs <1e-6) continue;
                double curvature = 1/rs;	
            }
        }
        else
        {
            continue;
        }

    }



}


}
