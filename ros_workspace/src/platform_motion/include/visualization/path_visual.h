#ifndef PATH_VISUAL_H
#define PATH_VISUAL_H

#include <OGRE/OgreColourValue.h>

#include <platform_motion_msgs/Path.h>

namespace Ogre
{
class Vector3;
class Quaternion;
class SceneManager;
class SceneNode;
}

namespace rviz
{
class Axes;
class Arrow;
}

namespace path_viz
{

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of SLAMGraphVisual represents the visualization of a single
// scavislam_messages::SLAMGraph message.
class PathVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  PathVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~PathVisual();

  // Configure the visual to show the data in the message.
  void setMessage( const platform_motion_msgs::Path::ConstPtr& msg );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way SLAMGraphVisual is only
  // responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the SLAMGraph message.
  void setSlowColor( Ogre::ColourValue &slowcolor );
  void setFastColor( Ogre::ColourValue &fastcolor );
  void setAlpha( double a );
  void setPoseLength( double l );
  void setPoseRadius( double r );
  void setShaftRadius( double r );
  void setShaftLength( double l );
  void setHeadRadius( double r );
  void setHeadLength( double l );
  void setVelocityScale( double s );
  void setAngularRateScale( double r );


private:
  void redrawArrows( void );

  // The object implementing the actual arrow shape
  std::vector<boost::shared_ptr<rviz::Axes> > poses_;
  std::vector<double> scaled_velocities_;
  std::vector<boost::shared_ptr<rviz::Arrow> > velocity_arrows_;
  std::vector<double> scaled_rates_;
  std::vector<boost::shared_ptr<rviz::Arrow> > angular_rate_arrows_;

  Ogre::ColourValue slow_color_;
  Ogre::ColourValue fast_color_;

  float alpha_;

  double pose_length_;
  double pose_radius_;
  double shaft_radius_;
  double shaft_length_;
  double head_radius_;
  double head_length_;
  double velocity_scale_;
  double angular_rate_scale_;


  // A SceneNode whose pose is set to match the coordinate frame of
  // the SLAMGraph message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};
// END_TUTORIAL

} // end namespace path_viz

#endif // PATH_VISUAL_H
