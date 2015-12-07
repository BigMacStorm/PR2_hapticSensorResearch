#ifndef PLACEOBJECT_H
#define PLACEOBJECT_H

#include <QDialog>

namespace Ui {
class placeObject;
}

class placeObject : public QDialog
{
    Q_OBJECT

public:
    explicit placeObject(QWidget *parent = 0);
    ~placeObject();

private:
    Ui::placeObject *ui;
};

#endif // PLACEOBJECT_H
