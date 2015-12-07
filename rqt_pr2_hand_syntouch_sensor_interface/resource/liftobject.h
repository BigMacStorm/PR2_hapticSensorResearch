#ifndef LIFTOBJECT_H
#define LIFTOBJECT_H

#include <QWidget>

namespace Ui {
class liftObject;
}

class liftObject : public QWidget
{
    Q_OBJECT

public:
    explicit liftObject(QWidget *parent = 0);
    ~liftObject();

private:
    Ui::liftObject *ui;
};

#endif // LIFTOBJECT_H
