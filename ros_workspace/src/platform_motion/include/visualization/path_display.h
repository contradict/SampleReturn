#ifndef PATH_DISPLAY_H
#define PATH_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <platform_motion_msgs/Path.h>
#include <rviz/message_filter_display.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BoolProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace path_viz
{

class PathVisual;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// SLAMraphDisplay will show a 3D arrow showing the direction and magnitude
// of the IMU acceleration vector.  The base of the arrow will be at
// the frame listed in the header of the SLAMGraph message, and the
// direction of the arrow will be relative to the orientation of that
// frame.  It will also optionally show a history of recent
// acceleration vectors, which will be stored in a circular buffer.
//
// The SLAMGraphDisplay class itself just implements the circular buffer,
// editable parameters, and Display subclass machinery.  The visuals
// themselves are represented by a separate class, SLAMGraphVisual.  The
// idiom for the visuals is that when the objects exist, they appear
// in the scene, and when they are deleted, they disappear.
class PathDisplay: public rviz::MessageFilterDisplay<platform_motion_msgs::Path>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  PathDisplay();
  virtual ~PathDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateHistoryLength();
  void updateSlowColor();
  void updateFastColor();
  void updateAlpha();
  void updatePoseLength();
  void updatePoseRadius();
  void updateShaftRadius();
  void updateShaftLength();
  void updateHeadRadius();
  void updateHeadLength();
  void updateVelocityScale();
  void updateAngularRateScale();


  // Function to handle an incoming ROS message.
private:
  void processMessage( const platform_motion_msgs::Path::ConstPtr& msg );

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<PathVisual> > visuals_;

  // User-editable property variables.
  rviz::IntProperty* history_length_property_;
  rviz::ColorProperty* fast_color_property_;
  rviz::ColorProperty* slow_color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* pose_length_property_;
  rviz::FloatProperty* pose_radius_property_;
  rviz::FloatProperty* shaft_radius_property_;
  rviz::FloatProperty* shaft_length_property_;
  rviz::FloatProperty* head_radius_property_;
  rviz::FloatProperty* head_length_property_;
  rviz::FloatProperty* velocity_scale_property_;
  rviz::FloatProperty* angular_rate_scale_property_;

};
// END_TUTORIAL

} // end namespace path_viz

#endif // PATH_DISPLAY_H
