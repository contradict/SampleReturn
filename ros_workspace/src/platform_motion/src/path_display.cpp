#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>

#include "visualization/path_visual.h"

#include "visualization/path_display.h"

namespace path_viz
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
PathDisplay::PathDisplay()
{
  history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT( updateHistoryLength() ));
  history_length_property_->setMin( 1 );
  history_length_property_->setMax( 100000 );

  slow_color_property_ = new rviz::ColorProperty( "Slow Color", QColor( 255, 0, 0 ),
                                          "Color to draw low speeds.",
                                           this, SLOT( updateSlowColor() ));

  fast_color_property_ = new rviz::ColorProperty( "Fast Color", QColor( 0, 255, 0 ),
                                          "Color to draw high speeds.",
                                           this, SLOT( updateFastColor() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is transparent, 1 is opaque.",
                                             this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0.0 );
  alpha_property_->setMax( 1.0 );

  pose_length_property_ = new rviz::FloatProperty( "Pose Length", 0.3,
                                             "Length of pose gnomon arms.",
                                             this, SLOT( updatePoseLength() ));
  pose_length_property_->setMin( 0. );
  pose_length_property_->setMax( 5. );

  pose_radius_property_ = new rviz::FloatProperty( "Pose Radius", 0.03,
                                             "Radius of pose gnomon arms.",
                                             this, SLOT( updatePoseRadius() ));
  pose_radius_property_->setMin( 0. );
  pose_radius_property_->setMax( 1. );

  shaft_length_property_ = new rviz::FloatProperty( "Shaft Length", 0.1,
                                             "Length of arrow shafts.",
                                             this, SLOT( updateShaftLength() ));
  shaft_length_property_->setMin( 0. );
  shaft_length_property_->setMax( 5. );


  shaft_radius_property_ = new rviz::FloatProperty( "Shaft Radius", 0.04,
                                             "Radius of arrow shafts.",
                                             this, SLOT( updateShaftRadius() ));
  shaft_radius_property_->setMin( 0. );
  shaft_radius_property_->setMax( 1. );

  head_length_property_ = new rviz::FloatProperty( "Head Length", 0.08,
                                             "Length of arrow head.",
                                             this, SLOT( updateHeadLength() ));
  head_length_property_->setMin( 0. );
  head_length_property_->setMax( 5. );


  head_radius_property_ = new rviz::FloatProperty( "Head Radius", 0.06,
                                             "Radius of arrow head.",
                                             this, SLOT( updateHeadRadius() ));
  head_radius_property_->setMin( 0. );
  head_radius_property_->setMax( 2. );

  velocity_scale_property_ = new rviz::FloatProperty( "Velocity Scale", 2.0,
                                             "Velocity to length scale.",
                                             this, SLOT( updateVelocityScale() ));
  velocity_scale_property_->setMin( 0.01 );
  velocity_scale_property_->setMax( 10.0 );

  angular_rate_scale_property_ = new rviz::FloatProperty( "Angular Rate Scale", 2.7,
                                             "Angular rate to length scale.",
                                             this, SLOT( updateAngularRateScale() ));
  velocity_scale_property_->setMin( 0.01 );
  velocity_scale_property_->setMax( 10.0 );

}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void PathDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

PathDisplay::~PathDisplay()
{
}

// Clear the visuals by deleting their objects.
void PathDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Set the number of past visuals to show.
void PathDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// Set the current color and alpha values for each visual.
void PathDisplay::updateSlowColor()
{
  Ogre::ColourValue c = slow_color_property_->getOgreColor();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setSlowColor( c );
  }
}

void PathDisplay::updateFastColor()
{
  Ogre::ColourValue c = fast_color_property_->getOgreColor();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setFastColor( c );
  }
}

void PathDisplay::updateAlpha()
{
    float alpha = alpha_property_->getFloat();
    for( size_t i = 0; i < visuals_.size(); i++ )
    {
        visuals_[ i ]->setAlpha( alpha );
    }
}

void PathDisplay::updatePoseLength()
{
    float l = pose_radius_property_->getFloat();
    for( size_t i = 0; i < visuals_.size(); i++ )
    {
        visuals_[ i ]->setPoseLength( l );
    }
}

void PathDisplay::updatePoseRadius()
{
    float r = pose_radius_property_->getFloat();
    for( size_t i = 0; i < visuals_.size(); i++ )
    {
        visuals_[ i ]->setPoseRadius( r );
    }
}

void PathDisplay::updateShaftRadius()
{
    float r = shaft_radius_property_->getFloat();
    for( size_t i = 0; i < visuals_.size(); i++ )
    {
        visuals_[ i ]->setShaftRadius( r );
    }
}

void PathDisplay::updateShaftLength()
{
    float l = shaft_length_property_->getFloat();
    for( size_t i = 0; i < visuals_.size(); i++ )
    {
        visuals_[ i ]->setShaftLength( l );
    }
}

void PathDisplay::updateHeadRadius()
{
    float r = head_radius_property_->getFloat();
    for( size_t i = 0; i < visuals_.size(); i++ )
    {
        visuals_[ i ]->setHeadRadius( r );
    }
}

void PathDisplay::updateHeadLength()
{
    float l = head_length_property_->getFloat();
    for( size_t i = 0; i < visuals_.size(); i++ )
    {
        visuals_[ i ]->setHeadLength( l );
    }
}

void PathDisplay::updateVelocityScale()
{
    float s = velocity_scale_property_->getFloat();
    for( size_t i = 0; i < visuals_.size(); i++ )
    {
        visuals_[ i ]->setVelocityScale( s );
    }
}

void PathDisplay::updateAngularRateScale()
{
    float r = angular_rate_scale_property_->getFloat();
    for( size_t i = 0; i < visuals_.size(); i++ )
    {
        visuals_[ i ]->setAngularRateScale( r );
    }
}

// This is our callback to handle an incoming message.
void PathDisplay::processMessage( const platform_motion_msgs::Path::ConstPtr& msg )
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Path message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<PathVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new PathVisual( context_->getSceneManager(), scene_node_ ));
  }

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue c = slow_color_property_->getOgreColor();
  visual->setSlowColor( c );
  c = fast_color_property_->getOgreColor();
  visual->setFastColor( c );
  float a = alpha_property_->getFloat();
  visual->setAlpha( a );
  float l = pose_length_property_->getFloat();
  visual->setPoseLength( l );
  float r = pose_radius_property_->getFloat();
  visual->setPoseRadius( r );
  r = shaft_radius_property_->getFloat();
  visual->setShaftRadius( r );
  l = shaft_length_property_->getFloat();
  visual->setShaftLength( l );
  r = head_radius_property_->getFloat();
  visual->setHeadRadius( r );
  l = head_length_property_->getFloat();
  visual->setHeadLength( l );
  float s = velocity_scale_property_->getFloat();
  visual->setVelocityScale( s );
  r = angular_rate_scale_property_->getFloat();
  visual->setAngularRateScale( r );

  // Now set or update the contents of the chosen visual.
  visual->setMessage( msg );
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

} // end namespace scavislam_rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_viz::PathDisplay,rviz::Display )
