#ifndef PALCEOBJECT_H
#define PALCEOBJECT_H

#include <QDialog>

namespace Ui {
class palceObject;
}

class palceObject : public QDialog
{
    Q_OBJECT

public:
    explicit palceObject(QWidget *parent = 0);
    ~palceObject();

private:
    Ui::palceObject *ui;
};

#endif // PALCEOBJECT_H
