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
    ControllerPanel( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    // A couple of public Qt slots.
  public Q_SLOTS:

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
