#ifndef UPMOTIONCON_H
#define UPMOTIONCON_H

#include <QMainWindow>
//#include <QAbstractSocket>
#include "comWithPPC.h"
#include <QDebug>

QT_BEGIN_NAMESPACE

namespace Ui { class UPMotionCon; }

QT_END_NAMESPACE

class UPMotionCon : public QMainWindow
{
    Q_OBJECT

public:
    UPMotionCon(QWidget *parent = nullptr);
    ~UPMotionCon();
public slots:
 //       void displayError(QString a);
        void displayError(QAbstractSocket::SocketError);
        void updateConnectStatus();
        void readData();
   //     void readData(QByteArray);
private slots:
 //      void on_Connect_clicked();
        void on_kpEdit_editingFinished();

        void on_KiEdit_editingFinished();

        void on_KdEdit_editingFinished();

        void on_connectBtn_clicked();

        void on_posEdit_editingFinished();

        void on_velEdit_editingFinished();

        void on_accEdit_editingFinished();

        void on_jerkEdit_editingFinished();

        void on_snapEdit_editingFinished();

        void on_enableBtn_clicked();

        void on_startMotionBtn_clicked();

        void on_jogStopBtn_clicked();

        void on_forwardBtn_clicked();

        void on_moveAbs_clicked();

        void on_setFPosEdit_editingFinished();

        void on_recordBtn_clicked();

        void on_saveBtn_clicked();

        void on_connectTypeBox_currentIndexChanged(const QString &arg1);

        void on_clearBtn_clicked();

signals:
        void Connect(QString addr,quint16 port);
        void disConnect();
private:
    Ui::UPMotionCon *ui;
    client_t *client;
    RXData_t rxData;
    QString addr;
    quint16 port;
    TXData_two txData;
    QByteArray data;
    comData_t comData;
};
#endif // UPMOTIONCON_H
