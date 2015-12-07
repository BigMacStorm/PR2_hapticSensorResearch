#ifndef DATAGRAPHS_H
#define DATAGRAPHS_H

#include <QDialog>

namespace Ui {
class dataGraphs;
}

class dataGraphs : public QDialog
{
    Q_OBJECT

public:
    explicit dataGraphs(QWidget *parent = 0);
    ~dataGraphs();

private:
    Ui::dataGraphs *ui;
};

#endif // DATAGRAPHS_H
