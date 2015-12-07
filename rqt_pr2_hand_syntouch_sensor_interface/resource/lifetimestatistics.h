#ifndef LIFETIMESTATISTICS_H
#define LIFETIMESTATISTICS_H

#include <QDialog>

namespace Ui {
class lifeTimeStatistics;
}

class lifeTimeStatistics : public QDialog
{
    Q_OBJECT

public:
    explicit lifeTimeStatistics(QWidget *parent = 0);
    ~lifeTimeStatistics();

private:
    Ui::lifeTimeStatistics *ui;
};

#endif // LIFETIMESTATISTICS_H
