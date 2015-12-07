#ifndef PROGRAMLIST_H
#define PROGRAMLIST_H

#include <QDialog>

namespace Ui {
class programList;
}

class programList : public QDialog
{
    Q_OBJECT

public:
    explicit programList(QWidget *parent = 0);
    ~programList();

private:
    Ui::programList *ui;
};

#endif // PROGRAMLIST_H
