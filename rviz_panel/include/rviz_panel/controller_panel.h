#ifndef CONTROLLER_PANEL_H
#define CONTROLLER_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>

class QLineEdit;

namespace rviz_panel
{

  class ControllerPanel: public rviz::Panel
  {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
  public:
    // QWidget subclass constructors usually take a parent widget
    // parameter (which usually defaults to 0).  At the same time,
    // pluginlib::ClassLoader creates instances by calling the default
    // constructor (with no arguments).  Taking the parameter and giving
    // a default of 0 lets the default constructor work and also lets
    // someone using the class for something else to pass in a parent
    // widget as they normally would with Qt.
    ControllerPanel( QWidget* parent = 0 );

    // A couple of public Qt slots.
  public Q_SLOTS:
    // In this example setTopic() does not get connected to any signal
    // (it is called directly), but it is easy to define it as a public
    // slot instead of a private function in case it would be useful to
    // some other user.
    void setTopic( const QString& topic );

    // Here we declare some internal slots.
  protected Q_SLOTS:
    // updateTopic() reads the topic name from the QLineEdit and calls
    // setTopic() with the result.
    void updateTopic();

  protected:
    // One-line text editor for entering the outgoing ROS topic name.
    QLineEdit* output_topic_editor_;

    // The current name of the output topic.
    QString output_topic_;

    // The ROS node handle.
    ros::NodeHandle nh_;
  };

} // end namespace rviz_plugin_tutorials

#endif // CONTROLLER_PANEL_H
