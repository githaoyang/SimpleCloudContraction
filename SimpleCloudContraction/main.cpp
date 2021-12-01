#include "SimpleCloudContraction.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SimpleCloudContraction w;
    w.show();
    return a.exec();
}
