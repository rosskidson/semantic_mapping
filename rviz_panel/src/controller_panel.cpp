#include <stdio.h>

#include "rviz_panel/controller_panel.h"
#include <QWidget>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>

namespace rviz_panel
{

ControllerPanel::ControllerPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  //layout->addWidget( drive_widget_ );
  setLayout( layout );

  // Next we make signal/slot connections.
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  //connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  Q_EMIT configChanged();
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void ControllerPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

// Set the topic name we are publishing to.
void ControllerPanel::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.

    Q_EMIT configChanged();
  }
}

//// Publish the control velocities if ROS is not shutting down and the
//// publisher is ready with a valid topic name.
//void TeleopPanel::sendVel()
//{
//  if( ros::ok() && velocity_publisher_ )
//  {
//    geometry_msgs::Twist msg;
//    msg.linear.x = linear_velocity_;
//    msg.linear.y = 0;
//    msg.linear.z = 0;
//    msg.angular.x = 0;
//    msg.angular.y = 0;
//    msg.angular.z = angular_velocity_;
//    velocity_publisher_.publish( msg );
//  }
//}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ControllerPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  //config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void ControllerPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
//  QString topic;
//  if( config.mapGetString( "Topic", &topic ))
//  {
//    output_topic_editor_->setText( topic );
//    updateTopic();
//  }
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_panel::ControllerPanel, rviz::Panel )
//PLUGINLIB_DECLARE_CLASS( rviz_panel, ControllerPanel, rviz_panel::ControllerPanel, rviz::Panel )
// END_TUTORIAL
