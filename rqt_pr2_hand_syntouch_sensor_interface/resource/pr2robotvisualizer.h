#ifndef PR2ROBOTVISUALIZER_H
#define PR2ROBOTVISUALIZER_H

#include <QWidget>

namespace Ui {
class PR2RobotVisualizer;
}

class PR2RobotVisualizer : public QWidget
{
    Q_OBJECT

public:
    explicit PR2RobotVisualizer(QWidget *parent = 0);
    ~PR2RobotVisualizer();

private:
    Ui::PR2RobotVisualizer *ui;
};

#endif // PR2ROBOTVISUALIZER_H
