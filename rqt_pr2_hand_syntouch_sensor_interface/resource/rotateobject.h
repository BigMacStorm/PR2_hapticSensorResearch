#ifndef ROTATEOBJECT_H
#define ROTATEOBJECT_H

#include <QWidget>

namespace Ui {
class rotateObject;
}

class rotateObject : public QWidget
{
    Q_OBJECT

public:
    explicit rotateObject(QWidget *parent = 0);
    ~rotateObject();

private:
    Ui::rotateObject *ui;
};

#endif // ROTATEOBJECT_H
