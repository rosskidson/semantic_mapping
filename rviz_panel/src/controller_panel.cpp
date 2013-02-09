#include <stdio.h>

#include "rviz_panel/controller_panel.h"
#include <QWidget>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QGroupBox>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>

namespace rviz_panel
{

ControllerPanel::ControllerPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // import box
  QPushButton* import_button_ptr = new QPushButton;
  import_button_ptr->setText("Import Scan");

  // align principle axis
  QPushButton* align_button_ptr = new QPushButton;
  align_button_ptr->setText("Align Model to Axis");

  // ROI extraction
  QVBoxLayout* extract_ROI_layout_ptr = new QVBoxLayout;
  QPushButton* extract_ROI_button_ptr = new QPushButton;
  extract_ROI_button_ptr->setText("Extract ROI");
  extract_ROI_layout_ptr->addWidget(new QLabel("TODO: interactive ROI selection tool"));
  extract_ROI_layout_ptr->addWidget(extract_ROI_button_ptr);

  // extract normals
  QPushButton* extract_normals_button_ptr = new QPushButton;
  extract_normals_button_ptr->setText("Extract Normals");

  // Segment Planes
  QPushButton* segment_planes_button_ptr = new QPushButton;
  segment_planes_button_ptr->setText("Segment Planes");

  // Segment Fixtures
  QPushButton* segment_fixtures_button_ptr = new QPushButton;
  segment_fixtures_button_ptr->setText("Segment Fixtures");

  // Entire layout
  QVBoxLayout* layout_ptr = new QVBoxLayout;
  layout_ptr->addWidget(import_button_ptr);
  layout_ptr->addWidget(align_button_ptr);
  layout_ptr->addLayout(extract_ROI_layout_ptr);
  layout_ptr->addWidget(extract_normals_button_ptr);
  layout_ptr->addWidget(segment_planes_button_ptr);
  layout_ptr->addWidget(segment_fixtures_button_ptr);
  setLayout( layout_ptr );

  // Next we make signal/slot connections.
  //connect( output_topic_editor_ptr_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  //connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  Q_EMIT configChanged();
}

void ControllerPanel::updateTopic()
{
  setTopic( output_topic_editor_ptr_->text() );
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
//    output_topic_editor_ptr_->setText( topic );
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
