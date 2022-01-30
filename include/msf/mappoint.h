#ifndef MAPPOINT_H
#define MAPPOINT_H


#include "msf/commen_include.h"
namespace msf
{
    
class LaserFrame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long      id_;        // ID
    unsigned long factory_id_;    // keyframe id
    bool        good_;      // wheter a good point 
    Vector2d    pos_;       // Position in world
    Vector2d    norm_;      // Normal of viewing direction 
    //Mat         descriptor_; // Descriptor for matching 
    
    list<LaserFrame*>    observed_frames_;   // key-frames that can observe this point 
    
    int         matched_times_;     // being an inliner in pose estimation
    int         visible_times_;     // being visible in current frame 
    
    MapPoint();
    MapPoint( 
        unsigned long id,
        unsigned long factory_id,
        const Vector2d& position, 
        const Vector2d& norm, 
        LaserFrame* frame=nullptr
        //const Mat& descriptor=Mat() 
    );
    
    
    //static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint( 
        unsigned long id,
        unsigned long factory_id,
        const Vector2d& pos_world, 
        const Vector2d& norm_,
        LaserFrame* frame );
};
}

#endif // MAPPOINT_H