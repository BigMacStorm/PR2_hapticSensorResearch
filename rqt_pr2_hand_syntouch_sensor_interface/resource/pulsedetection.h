#ifndef PULSEDETECTION_H
#define PULSEDETECTION_H

#include <QDialog>

namespace Ui {
class pulseDetection;
}

class pulseDetection : public QDialog
{
    Q_OBJECT

public:
    explicit pulseDetection(QWidget *parent = 0);
    ~pulseDetection();

private:
    Ui::pulseDetection *ui;
};

#endif // PULSEDETECTION_H
