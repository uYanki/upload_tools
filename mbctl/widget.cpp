#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget* parent)
    : QWidget(parent), ui(new Ui::Widget)
{
    ui->setupUi(this);

    m_statusBar = new QStatusBar();
    layout()->addWidget(m_statusBar);

    m_mbmst = new QModbusRtuSerialMaster(this);
    connect(m_mbmst, &QModbusClient::errorOccurred, [this](QModbusDevice::Error) {
        m_statusBar->showMessage(m_mbmst->errorString());
    });

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, [=]() {
        readRegister(REGADDR_MOTOR_CUR_POS,[=](const QModbusDataUnit& mbdat){
            ui->lblMotorCurPos->display(mbdat.value(0));
        });
    });

#if 1
    {
        connect(ui->hsldValue1, &QSlider::valueChanged, [=](int value) {
            QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, ui->hsldAddr1->value(), 1);
            mbdat.setValue(0, value);
            QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
            handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
        });
        connect(ui->hsldValue2, &QSlider::valueChanged, [=](int value) {
            QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, ui->hsldAddr2->value(), 1);
            mbdat.setValue(0, value);
            QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
            handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
        });
        connect(ui->hsldValue3, &QSlider::valueChanged, [=](int value) {
            QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, ui->hsldAddr3->value(), 1);
            mbdat.setValue(0, value);
            QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
            handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
        });
        connect(ui->hsldValue4, &QSlider::valueChanged, [=](int value) {
            QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, ui->hsldAddr4->value(), 1);
            mbdat.setValue(0, value);
            QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
            handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
        });
        connect(ui->hsldValue5, &QSlider::valueChanged, [=](int value) {
            QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, ui->hsldAddr5->value(), 1);
            mbdat.setValue(0, value);
            QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
            handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
        });
        connect(ui->hsldValue6, &QSlider::valueChanged, [=](int value) {
            QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, ui->hsldAddr6->value(), 1);
            mbdat.setValue(0, value);
            QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
            handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
        });
        connect(ui->hsldValue7, &QSlider::valueChanged, [=](int value) {
            QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, ui->hsldAddr7->value(), 1);
            mbdat.setValue(0, value);
            QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
            handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
        });
        connect(ui->hsldValue8, &QSlider::valueChanged, [=](int value) {
            QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, ui->hsldAddr8->value(), 1);
            mbdat.setValue(0, value);
            QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
            handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
        });
        connect(ui->hsldValue9, &QSlider::valueChanged, [=](int value) {
            QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, ui->hsldAddr9->value(), 1);
            mbdat.setValue(0, value);
            QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
            handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
        });
    }
#endif
}

void Widget::readRegister(int startAddress,  std::function<void(const QModbusDataUnit& mbdat)> onSuccess)
{
    QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, startAddress /* register */, 1 /* count */);
    QModbusReply*   reply = m_mbmst->sendReadRequest(mbdat, ui->spnSalveID->value());
    handleReply(reply, [=](const QModbusDataUnit& mbdat) { onSuccess(mbdat); });
}

void Widget::handleReply(QModbusReply* reply, std::function<void(const QModbusDataUnit& mbdat)> onSuccess)
{
    if (reply) {  // not nullptr

        connect(reply, &QModbusReply::finished, [=]() {
            const QModbusDataUnit mbdat = reply->result();  // response

            if (reply->error() == QModbusDevice::NoError) {
                onSuccess(mbdat);

            } else {
                // occur error
                m_statusBar->showMessage((reply->error() == QModbusDevice::ProtocolError ? tr("Read response error: %1 (Mobus exception: 0x%2)") : tr("Read response error: %1 (code: 0x%2)")).arg(reply->errorString()).arg(reply->error(), -1, 16), 5000);
            }
            reply->deleteLater();
        });
    } else {
        m_statusBar->showMessage(tr("Write error: ") + m_mbmst->errorString(), 5000);
    }
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_btnConnect_clicked()
{
    m_statusBar->clearMessage();

    if (m_mbmst->state() != QModbusDevice::ConnectedState) {
        // auto scan available serialport
        Q_FOREACH (auto port, QSerialPortInfo::availablePorts()) {
            if (!port.isBusy()) {
                m_mbmst->setConnectionParameter(QModbusDevice::SerialPortNameParameter, port.portName());
                m_mbmst->setConnectionParameter(QModbusDevice::SerialBaudRateParameter, QSerialPort::Baud19200);
                m_mbmst->setConnectionParameter(QModbusDevice::SerialDataBitsParameter, QSerialPort::DataBits::Data8);
                m_mbmst->setConnectionParameter(QModbusDevice::SerialStopBitsParameter, QSerialPort::StopBits::OneStop);
                m_mbmst->setConnectionParameter(QModbusDevice::SerialParityParameter, QSerialPort::Parity::EvenParity);
                m_mbmst->setTimeout(1000);

                // connect
                if (m_mbmst->connectDevice()) {
                    // success
                    ui->btnConnect->setText("disconnect");
                    m_timer->start(100);

#if 0
                    QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, 1 /* register */, 10 /* count */);
                    QModbusReply*   reply = m_mbmst->sendReadRequest(mbdat, ui->spnSalveID->value());
                    handleReply(reply, [=](const QModbusDataUnit& mbdat) {
                        // handle data
                        for (int i = 0, total = int(mbdat.valueCount()); i < total; ++i) {
                            qDebug() << (mbdat.startAddress() + i) << " = " << mbdat.value(i);
                        }
                    });
#endif

                } else {
                    // fail
                    m_statusBar->showMessage(tr("Connect failed: ") + m_mbmst->errorString(), 5000);
                }

                return;
            }
        }

    } else {
        // disconnect
        m_mbmst->disconnectDevice();
        ui->btnConnect->setText("connect");
        m_timer->stop();
    }
}

void Widget::on_btnMotorRun_toggled(bool checked)
{
    {
        QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, REGADDR_MOTOR_RUN_STATE /* register */, 1 /* count */);
        mbdat.setValue(0, checked);
        QModbusReply* reply = m_mbmst->sendWriteRequest(mbdat, ui->spnSalveID->value());
        handleReply(reply, [=](const QModbusDataUnit& mbdat) {});
    }
    {
        QModbusDataUnit mbdat(QModbusDataUnit::RegisterType::HoldingRegisters, REGADDR_MOTOR_RUN_STATE /* register */, 1 /* count */);
        QModbusReply*   reply = m_mbmst->sendReadRequest(mbdat, ui->spnSalveID->value());
        handleReply(reply, [=](const QModbusDataUnit& mbdat) { ui->btnMotorRun->setChecked(mbdat.value(0) & 0x0001); });
    }
}
