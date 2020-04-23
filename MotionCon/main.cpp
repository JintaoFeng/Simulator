#include "MotionCon.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MotionCon w;
 /*   QThread servoCycleThread;
    ServoCycle_t servo;
    servo.moveToThread(&servoCycleThread);
    QObject::connect(&servoCycleThread,SIGNAL(started()),&servo,SLOT(ServoCycle()));
    servoCycleThread.start(QThread::TimeCriticalPriority);
*/
 //   ServoCycle_t servoCycle;
 //   connect(servoCycle,&stop(),)
//    QObject::connect(w,w.close(),servoCycle,servoCycle.stop());
  //  servoCycle.start();
    w.show();
//    servoCycleThread.quit();
  //  servoCycle.exit();
    return a.exec();

}
