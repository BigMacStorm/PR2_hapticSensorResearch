#ifndef LIFETIMESTATISTICS_H
#define LIFETIMESTATISTICS_H

#include <QWidget>

namespace Ui {
class lifeTimeStatistics;
}

class lifeTimeStatistics : public QWidget
{
    Q_OBJECT

public:
    explicit lifeTimeStatistics(QWidget *parent = 0);
    ~lifeTimeStatistics();

private:
    Ui::lifeTimeStatistics *ui;
};

#endif // LIFETIMESTATISTICS_H
