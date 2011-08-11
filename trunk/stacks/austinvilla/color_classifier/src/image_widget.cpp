#include <QtGui>
#include <iostream>

#include <color_classifier/image_widget.h>
#include <color_classifier/common.h>

namespace color_classifier {

  ImageWidget::ImageWidget(QWidget *parent) : QWidget(parent) {
    img = new QImage(IMAGE_WIDTH,IMAGE_HEIGHT,QImage::Format_RGB32);
    img->fill(Qt::black);
  }

  void ImageWidget::reduceImageSize(int factor){
    img = new QImage(IMAGE_WIDTH/factor,IMAGE_HEIGHT/factor,QImage::Format_RGB32);
    img->fill(Qt::black);
  }
   
  void ImageWidget::paintEvent(QPaintEvent *event) {
    QPainter painter;
    painter.begin(this);

    painter.drawImage(event->rect(), *img);
    painter.end();
  }

  void ImageWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() != Qt::LeftButton && event->button() != Qt::RightButton) return;
    float x=(float)event->x()/(float)width()*IMAGE_WIDTH;
    float y=(float)event->y()/(float)height()*IMAGE_HEIGHT;
    emit clicked((int)x,(int)y, (int) event->button());
  }

  void ImageWidget::mouseMoveEvent(QMouseEvent *event) {
    float x=(float)event->x()/(float)width()*IMAGE_WIDTH;
    float y=(float)event->y()/(float)height()*IMAGE_HEIGHT;
    emit mouseXY((int)x,(int)y);
  }

}
