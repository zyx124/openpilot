#include "selfdrive/ui/qt/widgets/slider.h"
#include "selfdrive/ui/qt/widgets/controls.h"

void CustomSlider::initialize()
{

  sliderItem = new QWidget(parentWidget());
  QVBoxLayout *mainLayout = new QVBoxLayout(sliderItem);

  QHBoxLayout *titleLayout = new QHBoxLayout();
  mainLayout->addLayout(titleLayout);

  label = new QLabel(title);
  label->setStyleSheet(LabelStyle);
  label->setTextFormat(Qt::RichText);
  titleLayout->addWidget(label, 0, Qt::AlignLeft);

  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"behavior"});
  ButtonControl *resetButton = new ButtonControl("        ", tr("RESET"));
  connect(resetButton, &ButtonControl::clicked, [&]()
          {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset ") + param + "?", tr("Reset"), this)) {
      MessageBuilder msg;
      auto val = msg.initEvent().initBehavior();
      val.setComfortBrake(defaultVal);
      pm->send("behavior", msg);
      this->setValue(sliderMin + (defaultVal - paramMin) / (paramMax - paramMin) * (sliderMax - sliderMin));
    } });
  titleLayout->addWidget(resetButton, 0, Qt::AlignRight);

  setFixedHeight(100);
  setMinimum(sliderMin);
  setMaximum(sliderMax);

  MessageBuilder msg;
  auto val = msg.initEvent().initBehavior();
  setValue(sliderMin + (defaultVal - paramMin) / (paramMax - paramMin) * (sliderMax - sliderMin));
  label->setText(title + " " + QString::number(defaultVal, 'f', 2) + " " + unit);

  try
  {
    QString valueStr = QString::fromStdString(Params().get(param.toStdString()));
    double value = QString(valueStr).toDouble();
    val.setComfortBrake(value);
    pm->send("behavior", msg);
    
    setValue(sliderMin + (value - paramMin) / (paramMax - paramMin) * (sliderMax - sliderMin)); // Set the value of the slider. The value is scaled to the slider range
    label->setText(title + " " + QString::number(value, 'f', 2) + " " + unit);
    if (Params().getBool((param + "Lock").toStdString()))
    {
      setEnabled(false);
      setStyleSheet(lockedSliderStyle);
      label->setStyleSheet(lockedLabelStyle);
    }
    else
    {
      setEnabled(true);
      setStyleSheet(SliderStyle);
      label->setStyleSheet(LabelStyle);
    }
  }
  catch (const std::invalid_argument &e)
  {
    // Handle the error, e.g. set a default value
    setValue(0);
    label->setText(title + "Error: Param not found. Contact support.");
    setEnabled(false);
    setStyleSheet(lockedSliderStyle);
  }

  mainLayout->addWidget(this);

  connect(this, &CustomSlider::valueChanged, [=](int value)
  {
    // Update the label as the slider is moved. Don't save the value to params here
    double dValue = paramMin + (paramMax - paramMin) * (value - sliderMin) / (sliderMax - sliderMin);
    label->setText(title + " " + QString::number(dValue, 'f', 2) + " " + unit); 
    
  });

connect(this, &CustomSlider::sliderReleasedWithValue, [this](int value) {
    this->sliderReleasedWithValueHandler(value);
});

}

void CustomSlider::sliderReleasedWithValueHandler(int value) {
    double dValue = paramMin + (paramMax - paramMin) * (value - sliderMin) / (sliderMax - sliderMin);
    MessageBuilder msg;
    auto val = msg.initEvent().initBehavior();
    val.setComfortBrake(dValue);
    pm->send("behavior", msg);
}
