#include "connectioninfo.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ConnectionInfo w;
    w.show();

    return a.exec();
}
