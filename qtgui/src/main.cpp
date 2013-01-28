
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <QApplication>
#include <QMessageBox>
#include <QPushButton>
#include <QWidget>
#include <QObject>
#include <QString>

//class MainWindow : public QWidget
//{
//    Q_OBJECT
//public slots:
//      void buttonPressed();
//};

//void MainWindow::buttonPressed()
//{
//    QMessageBox::information(0, QString("Information"), QString("You've pressed the button \"Press Me!\""), QMessageBox::Ok);
//}

void displayMessage(const std_msgs::String::ConstPtr& msg)
{
  QMessageBox msgBox;
  msgBox.setText(msg->data.c_str());
  msgBox.exec();
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ui_popup", 1000, displayMessage);

  QApplication a(argc, argv);
  ros::spin();
  return 0;
}
