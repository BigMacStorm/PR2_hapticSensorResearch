#ifndef PROGRAMLIST_H
#define PROGRAMLIST_H

#include <QWidget>

namespace Ui {
class programList;
}

class programList : public QWidget
{
    Q_OBJECT

public:
    explicit programList(QWidget *parent = 0);
    ~programList();

private:
    Ui::programList *ui;
};

#endif // PROGRAMLIST_H
