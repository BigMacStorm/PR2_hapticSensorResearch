#ifndef SENSORVISUALIZER_H
#define SENSORVISUALIZER_H

#include <QWidget>

namespace Ui {
class SensorVisualizer;
}

class SensorVisualizer : public QWidget
{
    Q_OBJECT

public:
    explicit SensorVisualizer(QWidget *parent = 0);
    ~SensorVisualizer();

private:
    Ui::SensorVisualizer *ui;
};

#endif // SENSORVISUALIZER_H
