/**
 * \file  classification_window.hpp
 * \brief Header for the classification window 
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Piyush Khandelwal
 *
 * License: Modified BSD License
 *
 * $ Id: 08/10/2011 03:09:11 PM piyushk $
 */

#ifndef CLASSIFICATION_WINDOW_GZ6WWOJ1
#define CLASSIFICATION_WINDOW_GZ6WWOJ1

#include <QtGui/QMainWindow>
#include <sensor_msgs/Image.h>

#include "common.h"
#include "ui_classification_window.h"

namespace color_classifier {

  typedef Rgb RgbImage[IMAGE_HEIGHT][IMAGE_WIDTH];
  typedef uint8_t SegImage[IMAGE_HEIGHT][IMAGE_WIDTH];

  enum Image {
    RGB,
    SEG
  };

  enum ClickMode {
    ADD,
    DELETE
  };

  class ClassificationWindow : public QMainWindow {
  Q_OBJECT

  public:
    ClassificationWindow(QWidget *parent = 0);
    ~ClassificationWindow();

    void ReadSettings(); // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing

    void closeEvent(QCloseEvent *event); // Overloaded function
    void changeImage(sensor_msgs::ImageConstPtr image);

    void drawRgbImage(ImageWidget *widget);
    void drawSegImage(ImageWidget *widget);

    void segmentImage(bool useTempColorTable);
    void redrawImages(bool useTempColorTable = false);

    void setColor(int color);
    void updateStatus();

  public slots:
    void on_bigImage_clicked(int x, int y, int button);
    void on_bigImage_mouseXY(int x, int y);
    void on_rawImage_clicked(int x, int y, int button);
    void on_segImage_clicked(int x, int y, int button);

    void on_addRadio_clicked();
    void on_deleteRadio_clicked();

    void on_orangeButton_clicked();
    void on_pinkButton_clicked();
    void on_blueButton_clicked();
    void on_greenButton_clicked();
    void on_whiteButton_clicked();
    void on_yellowButton_clicked();

    void on_colorCombo_currentIndexChanged(int index);

    void on_actionNew_triggered();
    void on_actionOpen_triggered();
    void on_actionSave_triggered();
    void on_actionSave_As_triggered();

  private:
    Ui::ClassificationWindow ui;
    std::string colorTableFilename;

    RgbImage rgbImage;
    SegImage segImage;
    ColorTable colorTable;
    ColorTable tempColorTable;  

    QRgb segColors[NUM_COLORS];
    std::string segColorNames[NUM_COLORS];

    Image imageSelected;
    ClickMode clickMode;
    Color currentColor;

  };

}  

#endif /* end of include guard: CLASSIFICATION_WINDOW_GZ6WWOJ1 */
