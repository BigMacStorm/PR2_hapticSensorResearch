#ifndef LIFTOBJECT_H
#define LIFTOBJECT_H

#include <QDialog>

namespace Ui {
class liftObject;
}

class liftObject : public QDialog
{
    Q_OBJECT

public:
    explicit liftObject(QWidget *parent = 0);
    ~liftObject();

private:
    Ui::liftObject *ui;
};

#endif // LIFTOBJECT_H
