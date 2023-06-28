#ifndef WIDGET_H
#define WIDGET_H

#define REGADDR_MOTOR_RUN_STATE 1
#define REGADDR_MOTOR_CUR_POS   2

#include <QDebug>
#include <QMessageBox>

#include <QWidget>
#include <QSlider>
#include <QStatusBar>

#include <QTimer>

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QModbusRtuSerialMaster>

QT_BEGIN_NAMESPACE

namespace Ui
{
class Widget;
}

QT_END_NAMESPACE

class Widget : public QWidget {
    Q_OBJECT

public:
    Widget(QWidget* parent = nullptr);
    ~Widget();

private slots:
    void on_btnMotorRun_toggled(bool checked);
    void on_btnConnect_clicked();

private:
    Ui::Widget* ui;
    QStatusBar* m_statusBar;

    QModbusRtuSerialMaster* m_mbmst = nullptr;
    QTimer*                 m_timer = nullptr;

    void handleReply(QModbusReply* reply, std::function<void(const QModbusDataUnit& mbdat)> onSuccess);
    void readRegister( int startAddress, std::function<void(const QModbusDataUnit& mbdat)> onSuccess);

};
#endif  // WIDGET_H
