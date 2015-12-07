#ifndef SWAPHANDS_H
#define SWAPHANDS_H

#include <QWidget>

namespace Ui {
class swapHands;
}

class swapHands : public QWidget
{
    Q_OBJECT

public:
    explicit swapHands(QWidget *parent = 0);
    ~swapHands();

private:
    Ui::swapHands *ui;
};

#endif // SWAPHANDS_H
