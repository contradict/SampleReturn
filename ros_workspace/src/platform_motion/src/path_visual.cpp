
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/arrow.h>

#include <visualization/path_visual.h>

#include <eigen_conversions/eigen_msg.h>

namespace path_viz
{

// BEGIN_TUTORIAL
PathVisual::PathVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
    scene_manager_ = scene_manager;

    // Ogre::SceneNode s form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    //
    // Here we create a node to store the pose of the graph's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();

}

PathVisual::~PathVisual()
{
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode( frame_node_ );
}

void PathVisual::setMessage( const platform_motion_msgs::Path::ConstPtr& msg )
{

    if( poses_.size() < msg->knots.size() )
    {
        poses_.resize(msg->knots.size());
        velocities_.resize(msg->knots.size());
        angular_rates_.resize(msg->knots.size());
    }
    int i=0;
    double pose_length = 0.5;
    double shaft_radius = 0.05;
    double head_radius = 0.1;
    double head_length = 0.2;
    double velocity_scale = 1.0;
    for(const auto knot : msg->knots){
        boost::shared_ptr<rviz::Axes> &pose = poses_.at(i);
        if( pose.get() == 0 )
            pose.reset(new rviz::Axes(scene_manager_, frame_node_));

        Ogre::Vector3 pos( knot.pose.position.x,
                           knot.pose.position.y,
                           knot.pose.position.z);
        pose->setPosition(pos);
        Ogre::Quaternion orient( knot.pose.orientation.w,
                                 knot.pose.orientation.x,
                                 knot.pose.orientation.y,
                                 knot.pose.orientation.z);
        pose->setOrientation(orient);
        pose->setToDefaultColors();
        pose->set( pose_length, shaft_radius );

        boost::shared_ptr<rviz::Arrow> &velocity = velocities_.at(i);
        if( velocity.get() == 0 )
            velocity.reset(new rviz::Arrow(scene_manager_, frame_node_));
        velocity->setPosition( pos );
        Ogre::Vector3 vel( knot.twist.linear.x,
                           knot.twist.linear.y,
                           knot.twist.linear.z);
        double s = vel.length();
        vel.normalise();
        velocity->setDirection( vel );
        velocity->set(s/velocity_scale, shaft_radius, head_length, head_radius);
        velocity->setColor( path_color_ );

        boost::shared_ptr<rviz::Arrow> &rate = angular_rates_.at(i);
        if( rate.get() == 0 )
            rate.reset(new rviz::Arrow(scene_manager_, frame_node_));
        Ogre::Vector3 omega( knot.twist.angular.x,
                           knot.twist.angular.y,
                           knot.twist.angular.z);
        double l = 0.75*omega.length()/velocity_scale;
        if( l==0 )
        {
            omega.x=0;
            omega.y=0;
            omega.z=1;
        }
        omega.normalise();
        pos.z+=pose_length;
        if( knot.twist.angular.z< 0)
        {
            pos.z+=l+head_length;
        }
        rate->setPosition( pos );
        rate->setDirection( omega );
        rate->set(l, shaft_radius, head_length, head_radius);
        rate->setColor( path_color_ );

        i++;
    }
    for( size_t j=i; j<poses_.size(); j++ ) {
        Ogre::ColourValue off( 0.0f, 0.0f, 0.0f, 0.0f );
        poses_[j]->setXColor( off );
        poses_[j]->setYColor( off );
        poses_[j]->setZColor( off );
        velocities_[j]->setColor( off );
        angular_rates_[j]->setColor( off );
    }

}

// Position and orientation are passed through to the SceneNode.
void PathVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void PathVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

void PathVisual::setPathColor( Ogre::ColourValue &pathcolor,
                                     float a )
{
  path_color_ = pathcolor;
  alpha_ = a;
}

} // end namespace path_viz

