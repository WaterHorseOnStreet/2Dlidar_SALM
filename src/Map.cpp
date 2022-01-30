#include "msf/map.h"

namespace msf
{

void Map::insertKeyFrame ( LaserFrame::Ptr frame )
{
    cout<<"Key frame size = "<<keyframes_.size()<<endl;
    if ( keyframes_.find(frame->keyframe_id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->keyframe_id_, frame) );
    }
    else
    {
        keyframes_[ frame->keyframe_id_ ] = frame;
    }
}

void Map::insertMapPoint ( MapPoint::Ptr map_point )
{
    if ( map_points_.find(map_point->id_) == map_points_.end() )
    {
        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else 
    {
        map_points_[map_point->id_] = map_point;
    }
}

void Map::show_pointcloud()
{
    for(auto& mappoint:map_points_)
    {
        Eigen::Vector2d p;
        p = mappoint.second->pos_;
        PointT point;
        point.x = p(0);
        point.y = p(1);
        point.z = 0.0;
        pointCloud->points.push_back(point);
    }
    pointCloud->is_dense = false;
    pcl::io::savePCDFile("map.pcd", *pointCloud);
}


}