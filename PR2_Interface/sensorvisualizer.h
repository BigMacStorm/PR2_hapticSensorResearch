#ifndef SENSORVISUALIZER_H
#define SENSORVISUALIZER_H

#include <QDialog>

namespace Ui {
class SensorVisualizer;
}

class SensorVisualizer : public QDialog
{
    Q_OBJECT

public:
    explicit SensorVisualizer(QWidget *parent = 0);
    ~SensorVisualizer();

private:
    Ui::SensorVisualizer *ui;
};

#endif // SENSORVISUALIZER_H
