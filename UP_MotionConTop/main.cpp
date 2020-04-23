#include "UPMotionCon.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    UPMotionCon w;
    w.show();
    return a.exec();
}
