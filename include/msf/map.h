#ifndef MAP_H
#define MAP_H

#include "msf/commen_include.h"
#include "msf/LaserFrame.h"
#include "msf/mappoint.h"

namespace msf
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    std::map<unsigned long, MapPoint::Ptr >  map_points_;        // all landmarks
    std::map<unsigned long, LaserFrame::Ptr >     keyframes_;         // all key-frames
    PointCloud::Ptr                                    pointCloud;

    Map() {}
    
    void insertKeyFrame( LaserFrame::Ptr frame );
    void insertMapPoint( MapPoint::Ptr map_point );
    void show_pointcloud();
};
}

#endif // MAP_H