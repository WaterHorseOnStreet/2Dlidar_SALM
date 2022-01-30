#include "msf/mappoint.h"

namespace msf
{

MapPoint::MapPoint()
: id_(-1), pos_(Vector2d(0,0)), norm_(Vector2d(0,0)), good_(true), visible_times_(0), matched_times_(0)
{

}

MapPoint::MapPoint ( long unsigned int id, long unsigned int factory_id, const Vector2d& position, const Vector2d& norm, LaserFrame* frame )
: id_(id), factory_id_(factory_id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1)
{
    observed_frames_.push_back(frame);
}
/*
MapPoint::Ptr MapPoint::createMapPoint()
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id, Vector2d(0,0), Vector2d(0,0) )
    );
}
*/
MapPoint::Ptr MapPoint::createMapPoint ( 
    long unsigned int id,
    unsigned long factory_id,
    const Vector2d& pos_world, 
    const Vector2d& norm,  
    LaserFrame* frame )
{
    return MapPoint::Ptr( 
        new MapPoint( id, factory_id, pos_world, norm, frame)
    );
}

//unsigned long MapPoint::factory_id_ = 0;

}