#include "selfdrive/ui/qt/widgets/slider.h"
#include "selfdrive/ui/qt/widgets/controls.h"

void CustomSlider::initialize() {
  
  sliderItem = new QWidget(parentWidget());
  QVBoxLayout *mainLayout = new QVBoxLayout(sliderItem);

  QHBoxLayout *titleLayout = new QHBoxLayout();
  mainLayout->addLayout(titleLayout);

  label = new QLabel(title);
  label->setStyleSheet(LabelStyle);
  label->setTextFormat(Qt::RichText);
  titleLayout->addWidget(label, 0, Qt::AlignLeft);

  ButtonControl *resetButton = new ButtonControl("        ", tr("RESET"));
  connect(resetButton, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset ") + param + "?", tr("Reset"), this)) {
      Params().put(param.toStdString(),  std::to_string(defaultVal));
      this->setValue(sliderMin + (defaultVal - paramMin) / (paramRange) * (sliderRange));
    }
  });
  titleLayout->addWidget(resetButton, 0, Qt::AlignRight);

  setFixedHeight(100);
  setMinimum(sliderMin);
  setMaximum(sliderMax);
  
  try {
    double value = std::stod(params.get(param.toStdString(), "0.0")); // Get the value from the param. It could be float so its casted to string and then to double and QSlider only accepts int
    setValue(sliderMin + (value - paramMin) / (paramRange) * (sliderRange)); // Set the value of the slider. The value is scaled to the slider range 
    label->setText(title + " " + QString::number(value, 'f', 2) + " " + unit);

    if (Params().getBool((param + "Lock").toStdString())) {
      setEnabled(false);
      setStyleSheet(lockedSliderStyle);
      label->setStyleSheet(lockedLabelStyle);
    } else {
      setEnabled(true);
      setStyleSheet(SliderStyle);
      label->setStyleSheet(LabelStyle);
    }
  } catch (const std::invalid_argument &e) {
    // Handle the error, e.g. set a default value
    setValue(0);
    label->setText(title + "Error: Param not found. Contact support.");
    setEnabled(false);
    setStyleSheet(lockedSliderStyle);
  }

  mainLayout->addWidget(this);

  connect(this, &CustomSlider::valueChanged, [=](int value) {
    // Update the label as the slider is moved. Don't save the value to params here
    double floatValue = paramMin + (paramRange) * (value - sliderMin) / (sliderRange);
    label->setText(title + " " + QString::number(floatValue, 'f', 2) + " " + unit);
  });

  connect(this, &CustomSlider::sliderReleasedWithValue, [=](int value) {
    // save the value to params only when the slider is released
    double floatValue = paramMin + (paramRange) * (value - sliderMin) / (sliderRange);
    params.put(param.toStdString(), std::to_string(floatValue));
  });

}