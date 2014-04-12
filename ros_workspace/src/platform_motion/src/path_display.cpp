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
  path_color_ = new rviz::ColorProperty( "Point Color", QColor( 204, 51, 204 ),
                                          "Color to draw the points.",
                                           this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT( updateHistoryLength() ));
  history_length_property_->setMin( 1 );
  history_length_property_->setMax( 100000 );
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

// Set the current color and alpha values for each visual.
void PathDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue pathcolor = path_color_->getOgreColor();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setPathColor( pathcolor, alpha );
  }
}

// Set the number of past visuals to show.
void PathDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
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
  Ogre::ColourValue pathcolor = path_color_->getOgreColor();
  visual->setPathColor( pathcolor, alpha);
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
