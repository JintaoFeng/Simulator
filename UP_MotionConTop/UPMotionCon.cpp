#include "UPMotionCon.h"
#include "ui_UPMotionCon.h"

UPMotionCon::UPMotionCon(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::UPMotionCon)
{
    ui->setupUi(this);
    ui->displayMessage->setReadOnly(true);
    ui->IPText->setText("192.168.1.5");
    ui->portText->setText("1100");
    ui->posViewEdit->setReadOnly(true);
    ui->velViewEdit->setReadOnly(true);
    ui->accViewEdit->setReadOnly(true);
    client=new client_t;
//    connect(ui->connectBtn,SIGNAL(clicked()),this,SLOT(on_Connect_clicked()));
    connect(client,SIGNAL(connected()),this,SLOT(updateConnectStatus()));
    connect(client,SIGNAL(error(QAbstractSocket::SocketError)),this,SLOT(displayError(QAbstractSocket::SocketError)));
    connect(client,SIGNAL(disconnected()),this,SLOT(updateConnectStatus()));
    connect(client,SIGNAL(readyRead()),this,SLOT(readData()));
}

UPMotionCon::~UPMotionCon()
{
    delete ui;
    delete client;
}


void UPMotionCon::displayError(QAbstractSocket::SocketError)
{
    ui->displayMessage->append(client->errorString());
}
void UPMotionCon::readData()
{
    QByteArray data;
    if(client->bytesAvailable()<(qint64)sizeof(RXData_t))
        return;
    data = client->read(sizeof(RXData_t));
    memcpy(&rxData,data.data(),sizeof(rxData));
    comData.intByteConvert(&(rxData.axisStatus[ui->axisComboBox->currentIndex()]));
    ui->axisState->setText(QString::number(rxData.axisStatus[ui->axisComboBox->currentIndex()]));
    comData.doubleByteConvert(&(rxData.feedPosition[ui->axisComboBox->currentIndex()]));
    ui->posViewEdit->setText(QString::number(rxData.feedPosition[ui->axisComboBox->currentIndex()],'f',0));
    comData.doubleByteConvert(&(rxData.refPosition[ui->axisComboBox->currentIndex()]));
    ui->refPosEdit->setText(QString::number(rxData.refPosition[ui->axisComboBox->currentIndex()],'f',0));
    comData.intByteConvert(&(rxData.motorState[ui->axisComboBox->currentIndex()]));
    ui->motStateEdit->setText(QString::number(rxData.motorState[ui->axisComboBox->currentIndex()]));
    comData.doubleByteConvert(&(rxData.refVel[ui->axisComboBox->currentIndex()]));
    ui->refVelEdit->setText(QString::number(rxData.refVel[ui->axisComboBox->currentIndex()],'f',0));
    comData.doubleByteConvert(&(rxData.conDist[ui->axisComboBox->currentIndex()]));
    ui->conDistLineEdit->setText(QString::number(rxData.conDist[ui->axisComboBox->currentIndex()],'f',0));
    comData.doubleByteConvert(&(rxData.posError[ui->axisComboBox->currentIndex()]));
    ui->posErrorLineEdit->setText(QString::number(rxData.posError[ui->axisComboBox->currentIndex()],'f',0));
}
void UPMotionCon::updateConnectStatus()
{
    if(client->state() ==  QAbstractSocket::ConnectedState)
    {
        ui->connectBtn->setText("Disconnect");
        ui->displayMessage->append("connect successfully!");
        ui->IPText->setEnabled(false);
        ui->portText->setEnabled(false);
        ui->connectTypeBox->setEnabled(false);
        comData.setConnectType((ConnectType)ui->connectTypeBox->currentIndex());
        client->write((char*)(&comData),sizeof(comData));

    }
    else if(client->state() ==  QAbstractSocket::UnconnectedState)
    {
        ui->connectBtn->setText("Connect");
        ui->IPText->setEnabled(true);
        ui->portText->setEnabled(true);
        ui->connectTypeBox->setEnabled(true);
    }
}
/*
void UPMotionCon::on_Connect_clicked()
{
    if(ui->connectBtn->text()=="Connect")
    {
        addr = ui->IPText->text();
        port = ui->portText->text().toUShort();
        if(addr.isEmpty())
        {
            QMessageBox::warning(this,tr("Warning"),tr("IP不能为空！"));
            return;
        }
        client->connectToHost(addr,port);
    }
    else if(ui->connectBtn->text()=="Disconnect")
    {
        client->disconnectFromHost();
    }
}
*/

void UPMotionCon::on_kpEdit_editingFinished()
{
//    txData.setPID_Kp(ui->axisComboBox->currentIndex(),ui->kpEdit->text().toFloat());
//    client->write((char*)(&txData),sizeof(txData));
 //   qDebug()<<ui->kpEdit->text().toFloat();
    comData.setKp(ui->axisComboBox->currentIndex(),ui->kpEdit->text().toDouble());
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_KiEdit_editingFinished()
{
 //   txData.setPID_Ki(ui->axisComboBox->currentIndex(),ui->KiEdit->text().toFloat());
 //   client->write((char*)(&txData),sizeof(txData));
    comData.setKi(ui->axisComboBox->currentIndex(),ui->KiEdit->text().toDouble());
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_KdEdit_editingFinished()
{

//    txData.setPID_Kd(ui->axisComboBox->currentIndex(),ui->KdEdit->text().toFloat());
//    client->write((char*)(&txData),sizeof(txData));
    comData.setKd(ui->axisComboBox->currentIndex(),ui->KdEdit->text().toDouble());
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_connectBtn_clicked()
{
    if(ui->connectBtn->text()=="Connect")
    {
        addr = ui->IPText->text();
        port = ui->portText->text().toUShort();
        if(addr.isEmpty())
        {
            QMessageBox::warning(this,tr("Warning"),tr("IP不能为空！"));
            return;
        }
        client->connectToHost(addr,port);
    }
    else if(ui->connectBtn->text()=="Disconnect")
    {
        client->disconnectFromHost();
    }
}

void UPMotionCon::on_posEdit_editingFinished()
{
 //   txData.setTrajPos(ui->axisComboBox->currentIndex(),ui->posEdit->text().toDouble());
    comData.setPos(ui->axisComboBox->currentIndex(),ui->posEdit->text().toDouble());
 //   client->write((char*)(&txData),sizeof(txData));
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_velEdit_editingFinished()
{
 //   txData.setTrajVel(ui->axisComboBox->currentIndex(),ui->velEdit->text().toDouble());
 //   client->write((char*)(&txData),sizeof(txData));
    comData.setVel(ui->axisComboBox->currentIndex(),ui->velEdit->text().toDouble());
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_accEdit_editingFinished()
{
  //  txData.setTrajAcc(ui->axisComboBox->currentIndex(),ui->accEdit->text().toDouble());
 //   client->write((char*)(&txData),sizeof(txData));
    comData.setAcc(ui->axisComboBox->currentIndex(),ui->accEdit->text().toDouble());
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_jerkEdit_editingFinished()
{
 //   txData.setTrajJerk(ui->axisComboBox->currentIndex(),ui->jerkEdit->text().toDouble());
  //  client->write((char*)(&txData),sizeof(txData));
    comData.setJerk(ui->axisComboBox->currentIndex(),ui->jerkEdit->text().toDouble());
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_snapEdit_editingFinished()
{
  //  txData.setTrajSnap(ui->axisComboBox->currentIndex(),ui->snapEdit->text().toDouble());
  //  client->write((char*)(&txData),sizeof(txData));
}

void UPMotionCon::on_enableBtn_clicked()
{
    if(rxData.axisStatus[ui->axisComboBox->currentIndex()] == 6502)
    {
        comData.close(ui->axisComboBox->currentIndex());
        client->write((char*)(&comData),sizeof(comData));
        ui->enableBtn->setText("DISABLE");
    }
    else// if(rxData.axisStatus[ui->axisComboBox->currentIndex()]!=6501)
    {
        comData.open(ui->axisComboBox->currentIndex());
        client->write((char*)(&comData),sizeof(comData));
        ui->enableBtn->setText("ENABLE");
    }
}

void UPMotionCon::on_startMotionBtn_clicked()
{
    comData.jog(ui->axisComboBox->currentIndex(),1);
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_jogStopBtn_clicked()
{
    comData.jogStop(ui->axisComboBox->currentIndex());
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_forwardBtn_clicked()
{
    comData.moveRel(ui->axisComboBox->currentIndex(),ui->posEdit->text().toDouble());
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_moveAbs_clicked()
{
    comData.moveAbs(ui->axisComboBox->currentIndex(),ui->posEdit->text().toDouble());
    client->write((char*)(&comData),sizeof(comData));
}



void UPMotionCon::on_setFPosEdit_editingFinished()
{
    comData.setFPos(ui->axisComboBox->currentIndex(),ui->setFPosEdit->text().toDouble());
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_recordBtn_clicked()
{
    comData.setCMD(27);
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_saveBtn_clicked()
{
    comData.setCMD(28);
    client->write((char*)(&comData),sizeof(comData));
}

void UPMotionCon::on_connectTypeBox_currentIndexChanged(const QString &arg1)
{
    if(arg1 == "TCP/IP")
    {
  //      qDebug()<<"TCP/IP\n";
        ui->IPText->setDisabled(false);
        ui->portText->setDisabled(false);
    }

    else if(arg1 == "simulater")
    {
        ui->IPText->setText("127.0.0.1");
        ui->IPText->setDisabled(true);
        ui->portText->setDisabled(true);
        qDebug()<<"simulater!!\n";
    }
}

void UPMotionCon::on_clearBtn_clicked()
{
    comData.setCMD(51);
    client->write((char*)(&comData),sizeof(comData));
}
