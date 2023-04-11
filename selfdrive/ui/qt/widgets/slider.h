#pragma once
#include <QLabel>
#include <QSlider>
#include <QWidget>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include "common/params.h"

class CustomSlider : public QSlider {
  Q_OBJECT

public:
  explicit CustomSlider(const QString &param, const QString &title, const QString &unit, double paramMin, double paramMax, double defaultVal, Params &params, Qt::Orientation orientation, QWidget *parent = nullptr)
      : QSlider(orientation, parent), param(param), title(title), unit(unit), paramMin(paramMin), paramMax(paramMax), params(params) {
    initialize();
  }

  QWidget *getSliderItem() {
    return sliderItem;
  }

signals:
  void sliderReleasedWithValue(int value);

protected:
  void mouseReleaseEvent(QMouseEvent *event) override {
    QSlider::mouseReleaseEvent(event);
    emit sliderReleasedWithValue(value());
  }

private:
  void initialize();

  QString param;
  QString title;
  QString unit;
  double paramMin;
  double paramMax;
  double paramRange = paramMax - paramMin;
  double defaultVal;
  double scaleFactor;
  Params &params;

  QWidget *sliderItem;
  QLabel *label;
  int sliderMin = 0;
  int sliderMax = 10000;
  int sliderRange = sliderMax - sliderMin;

  QString SliderStyle = R"(
    QSlider::groove:horizontal 
      {border: none;height: 60px;background-color: #393939;border-radius: 30px;}
    QSlider::handle:horizontal 
      {background-color: #fafafa;border: none;width: 80px;height: 80px;margin-top: -10px;margin-bottom: -10px;border-radius: 40px;}
  )";
  QString lockedSliderStyle = R"(
    QSlider::groove:horizontal 
      {border: none;height: 60px;background-color: #393939;border-radius: 30px;}
    QSlider::handle:horizontal 
      {background-color: #787878;border: none;width: 80px;height: 80px;margin-top: -10px;margin-bottom: -10px;border-radius: 40px;}
  )";
  // label
  QString LabelStyle = R"(
    QLabel {
      color: #fafafa;
    }
  )";
  QString lockedLabelStyle = R"(
    QLabel {
      color: #787878;
    }
  )";
};