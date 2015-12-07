#ifndef SWAPHANDS_H
#define SWAPHANDS_H

#include <QDialog>

namespace Ui {
class swapHands;
}

class swapHands : public QDialog
{
    Q_OBJECT

public:
    explicit swapHands(QWidget *parent = 0);
    ~swapHands();

private:
    Ui::swapHands *ui;
};

#endif // SWAPHANDS_H
