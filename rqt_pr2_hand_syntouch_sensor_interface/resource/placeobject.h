#ifndef PLACEOBJECT_H
#define PLACEOBJECT_H

#include <QWidget>

namespace Ui {
class placeObject;
}

class placeObject : public QWidget
{
    Q_OBJECT

public:
    explicit placeObject(QWidget *parent = 0);
    ~placeObject();

private:
    Ui::placeObject *ui;
};

#endif // PLACEOBJECT_H
