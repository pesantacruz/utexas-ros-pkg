#include <iostream>
#include <QtGui>
#include <QMessageBox>

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <color_classifier/main_window.h>

namespace color_classifier {

  using namespace Qt;

  MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    initialize();
    classWindow.loadDataDirectory(getBaseDirectory() + "/data/");
    classWindow.openDefaultColorTable();
  }

  MainWindow::~MainWindow() {}

  void MainWindow::on_actionOpen_Bag_triggered() {

    images.clear();

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Bag"),
        QString((getBaseDirectory() + "/data/").c_str()),
        tr("ROS Bag Files (*.bag)"));

    if (fileName.isNull()) {
      ui.statusBar->showMessage("User cancelled operation");
      return;
    }
    rosbag::Bag bag;
    try {
      bag.open(fileName.toStdString(), rosbag::bagmode::Read);
    } catch (rosbag::BagIOException e) {
      ui.statusBar->showMessage("Error opening bag file!!");
      return;
    }

    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/image_color"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      sensor_msgs::ImageConstPtr image = m.instantiate<sensor_msgs::Image>();
      images.push_back(image);
    }

    ui.frameSlider->setEnabled(true);
    ui.frameSlider->setMaximum(images.size() - 1);
    ui.frameSlider->setMinimum(0);

    ui.currentFrameSpin->setEnabled(true);
    ui.currentFrameSpin->setMaximum(images.size() - 1);
    ui.currentFrameSpin->setMinimum(0);

    ui.numFrameEdit->setText(QString((boost::lexical_cast<std::string>(images.size() - 1)).c_str()));
    
    classWindow.changeImage(images[0]); 

    ui.statusBar->showMessage(("Bag file with " + boost::lexical_cast<std::string>(images.size()) + " frames").c_str());
    
  }

  /**
   * \brief   Helper function to get the base directory of the package, primarily to locate the default color table
   */
  std::string MainWindow::getBaseDirectory() {
    return ros::package::getPath("color_classifier");
  }

  void MainWindow::on_currentFrameSpin_valueChanged(int value) {
    ui.frameSlider->setValue(value);
    classWindow.changeImage(images[value]);
  }

  void MainWindow::on_frameSlider_sliderMoved(int value) {
    ui.currentFrameSpin->setValue(value);  //implicitly calls the slot for currentFrameSpin -> value Changed
  }

  void MainWindow::initialize() {
    ui.frameSlider->setEnabled(false);
    ui.currentFrameSpin->setEnabled(false);
    ui.numFrameEdit->setText(QString("0"));    
  }

  void MainWindow::closeEvent(QCloseEvent *event) {
    classWindow.close();
    //WriteSettings();
    event->accept();
  }

  void MainWindow::show() {
    QMainWindow::show();
    classWindow.show();
  }

}  // namespace color_classifier
