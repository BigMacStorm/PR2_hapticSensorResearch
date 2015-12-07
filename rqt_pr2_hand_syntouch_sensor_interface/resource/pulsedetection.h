#ifndef PULSEDETECTION_H
#define PULSEDETECTION_H

#include <QWidget>

namespace Ui {
class pulseDetection;
}

class pulseDetection : public QWidget
{
    Q_OBJECT

public:
    explicit pulseDetection(QWidget *parent = 0);
    ~pulseDetection();

private:
    Ui::pulseDetection *ui;
};

#endif // PULSEDETECTION_H
