/**
 * \file  main.cpp
 * \brief Runs the color classifier
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Piyush Khandelwal
 *
 * License: Modified BSD License
 *
 * $ Id: 08/11/2011 11:19:31 AM piyushk $
 */

#include <QtGui>
#include <QApplication>

#include <color_classifier/main_window.h>

int main(int argc, char **argv) {

  QApplication app(argc, argv);
  color_classifier::MainWindow main;
  main.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

	return result;
}
