
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "qtgui/inputDialog.h"

#include <QApplication>
#include <QMessageBox>
#include <QPushButton>
#include <QWidget>
#include <QObject>
#include <QString>
#include <QInputDialog>
#include <QSize>

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

bool getUserInput(qtgui::inputDialog::Request& req, qtgui::inputDialog::Response& res)
{
  bool ok;
  QInputDialog inputDialog;
  inputDialog.setOptions(QInputDialog::NoButtons);

  QString text =  inputDialog.getText(NULL ,"Rename Object",
                                       "Name:", QLineEdit::Normal,
                                      req.input.data.c_str(), &ok);

  if (ok && !text.isEmpty())
  {
    std::cout<<text.toStdString()<<std::endl;
    res.output.data = text.toStdString();
  }
  else
  {
    res.output.data = req.input.data;
  }
  return true;
}

void displayMessage(const std_msgs::String::ConstPtr& msg)
{
  QMessageBox msgBox;
  msgBox.setWindowTitle("Object Name           ");
  msgBox.setText(msg->data.c_str());
  msgBox.exec();
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "semantic_mapping_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ui_popup", 1000, displayMessage);
  ros::ServiceServer service = n.advertiseService("ui_dialog_service", getUserInput);

  QApplication a(argc, argv);

  ros::spin();
  return 0;
}
