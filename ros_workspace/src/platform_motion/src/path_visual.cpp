
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
PathVisual::PathVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) :
  pose_length_(0.3),
  pose_radius_(0.03),
  shaft_radius_(0.04),
  shaft_length_(0.1),
  head_radius_(0.06),
  head_length_(.08),
  velocity_scale_(2.0),
  angular_rate_scale_(velocity_scale_/0.75)
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
        velocity_arrows_.resize(msg->knots.size());
        scaled_velocities_.resize(msg->knots.size());
        angular_rate_arrows_.resize(msg->knots.size());
        scaled_rates_.resize(msg->knots.size());
    }
    int i=0;
    for(const auto knot : msg->knots)
    {
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
        pose->set( pose_length_, pose_radius_ );

        boost::shared_ptr<rviz::Arrow> &velocity = velocity_arrows_.at(i);
        if( velocity.get() == 0 )
            velocity.reset(new rviz::Arrow(scene_manager_, frame_node_));
        velocity->setPosition( pos );
        Ogre::Vector3 vel( knot.twist.linear.x,
                           knot.twist.linear.y,
                           knot.twist.linear.z);
        double s = vel.length();
        vel.normalise();
        velocity->setDirection( vel );
        velocity->set(shaft_length_, shaft_radius_, head_length_, head_radius_);
        s = std::max(0.0, std::min( s/velocity_scale_, 1.0 ) );
        scaled_velocities_[i] = s;
        velocity->setColor( slow_color_*(1.0-s) + fast_color_*s );

        boost::shared_ptr<rviz::Arrow> &rate = angular_rate_arrows_.at(i);
        if( rate.get() == 0 )
            rate.reset(new rviz::Arrow(scene_manager_, frame_node_));
        Ogre::Vector3 omega( knot.twist.angular.x,
                           knot.twist.angular.y,
                           knot.twist.angular.z);
        double l = omega.length();
        if( l==0 )
        {
            omega.x=0;
            omega.y=0;
            omega.z=1;
        }
        omega.normalise();
        pos.z+=pose_length_;
        if( knot.twist.angular.z< 0)
        {
            pos.z+=head_length_+shaft_length_;
        }
        rate->setPosition( pos );
        rate->setDirection( omega );
        rate->set(shaft_length_, shaft_radius_, head_length_, head_radius_);
        l = std::max( 0.0, std::min( l/angular_rate_scale_, 1.0 ) );
        scaled_rates_[i] = l;
        rate->setColor( slow_color_*(1.0-l) + fast_color_*l );

        i++;
    }
    for( size_t j=i; j<poses_.size(); j++ ) {
        Ogre::ColourValue off( 0.0f, 0.0f, 0.0f, 0.0f );
        poses_[j]->setXColor( off );
        poses_[j]->setYColor( off );
        poses_[j]->setZColor( off );
        velocity_arrows_[j]->setColor( off );
        angular_rate_arrows_[j]->setColor( off );
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

void PathVisual::setSlowColor( Ogre::ColourValue &slowcolor )
{
  slow_color_ = slowcolor;
  redrawArrows();
}

void PathVisual::setFastColor( Ogre::ColourValue &fastcolor )
{
  fast_color_ = fastcolor;
  redrawArrows();
}

void PathVisual::setAlpha( double a )
{
    alpha_ = a;
    for( auto p : poses_ )
    {
        Ogre::ColourValue c;
        c=p->getDefaultXColor();
        c.a=a;
        p->setXColor( c );
        c=p->getDefaultYColor();
        c.a=a;
        p->setYColor( c );
        c=p->getDefaultZColor();
        c.a=a;
        p->setZColor( c );
    }
    int i=0;
    for( auto v : velocity_arrows_ )
    {
        Ogre::ColourValue c=slow_color_*(1.0-scaled_velocities_[i]) + fast_color_*scaled_velocities_[i];
        c.a = alpha_;
        v->setColor(c);
        i++;
    }
    i=0;
    for( auto v : angular_rate_arrows_ )
    {
        Ogre::ColourValue c=slow_color_*(1.0-scaled_rates_[i]) + fast_color_*scaled_rates_[i];
        c.a = alpha_;
        v->setColor(c);
        i++;
    }
}

void PathVisual::setPoseLength( double l )
{
    pose_length_ = l;
    for( auto p : poses_ )
    {
        p->set( pose_length_, pose_radius_ );
    }
}

void PathVisual::setPoseRadius( double r )
{
    pose_radius_ = r;
    for( auto p : poses_ )
    {
        p->set( pose_length_, pose_radius_ );
    }
}

void PathVisual::redrawArrows( void )
{
    int i=0;
    for( auto v : velocity_arrows_ )
    {
        v->set( shaft_length_, shaft_radius_, head_length_, head_radius_);
        Ogre::ColourValue c=slow_color_*(1.0-scaled_velocities_[i]) +
                            fast_color_*scaled_velocities_[i];
        v->setColor(c);
        i++;
    }
    i=0;
    for( auto v : angular_rate_arrows_ )
    {
        v->set( shaft_length_, shaft_radius_, head_length_, head_radius_);
        Ogre::ColourValue c=slow_color_*(1.0-scaled_rates_[i]) +
                            fast_color_*scaled_rates_[i];
        v->setColor(c);
        i++;
    }
}

void PathVisual::setShaftRadius( double r )
{
    shaft_radius_ = r;
    redrawArrows();
}

void PathVisual::setShaftLength( double l )
{
    shaft_length_ = l;
    redrawArrows();
}

void PathVisual::setHeadRadius( double r )
{
    head_radius_ = r;
    redrawArrows();
}

void PathVisual::setHeadLength( double l )
{
    head_length_ = l;
    redrawArrows();
}

void PathVisual::setVelocityScale( double s )
{
    for( double &x : scaled_velocities_ )
    {
        x /= s/velocity_scale_;
    }
    velocity_scale_ = s;
    redrawArrows();
}

void PathVisual::setAngularRateScale( double r )
{
    for( double &x : scaled_rates_ )
    {
        x /= r/angular_rate_scale_;
    }
    angular_rate_scale_ = r;
    redrawArrows();
}

} // end namespace path_viz

