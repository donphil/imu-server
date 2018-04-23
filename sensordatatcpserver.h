#ifndef SENSORDATATCPSERVER_H
#define SENSORDATATCPSERVER_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>

#include <QtWebSockets/QtWebSockets>

#include "berryimu.h"
#include "databuffer.h"


QT_FORWARD_DECLARE_CLASS(QWebSocketServer)
QT_FORWARD_DECLARE_CLASS(QWebSocket)


class IMUThread : public QThread
{
    Q_OBJECT
public:
    IMUThread();

    BerryIMU::BerryIMU m_imu;
    DataBuffer m_buffer, m_transmission_data;
    QMutex m_mutex;

    inline void setDelay(int delay_microseconds) { m_mus_delay = delay_microseconds; }

    bool setParametersAcc(int ires, int iodr);
    bool setParametersMag(int ires, int iodr);
    bool setParametersGyr(int ires, int iodr, int ibw);

    // These reads are slow, so they are only performed when requested (1x per request).
    void readTemperature() { m_read_temperature = true; }
    void readPressure()    { m_read_pressure = true; }
    void readTP()          { m_read_temperature = true; m_read_pressure = true; }

//public slots:
    void run();

    void stop();
    void exitloop();
    void clearBuffer();

    inline void setTransmissionInterval(int x) { m_ms_between_transmissions = x; }
signals:
    void data_ready(DataBuffer * data);
    //void finished();
private:

    int m_ms_between_transmissions=1000;     // milliseconds
    int m_mus_delay               =1000*1000;// microseconds: 1000 = 1ms. Hardcoded in QML

    bool m_read_temperature = false;
    bool m_read_pressure    = false;
};

class SensorDataTcpServer : public QObject
{
    Q_OBJECT
 public:
    enum Status { offline, listening, connected, emitting };
    //Q_ENUM(Status)

    explicit SensorDataTcpServer(quint16 port, bool debug = false, QObject *parent = Q_NULLPTR);
    explicit SensorDataTcpServer(QObject *parent=0);
    ~SensorDataTcpServer();

    void setStatus(const QString &a) {
        if(m_debug)
            qDebug() << "Status changed to: " << a;
        emit statusChanged(a);
    }
    void setMsg(const QString &a) {
        emit msgChanged(a);
    }
    void setPort(const quint16 &p, const bool override = false) {
            if (p != m_port) {
                m_port = p;
            }
            if(override) { // Write the m_port to QML
                qDebug() << "emitting portChanged(" << p <<")";
                emit portChanged(QString::number(m_port));
            }
        }

    void startPolling();    // (Re-Activate) sensor and begin loop
    void pausePolling();    // Ends sensor loop but leaves sensor alive.
    void endPolling();      // Ends loop and deactivates sensor

    Q_INVOKABLE void resetServer();
    Q_INVOKABLE void resetIMU();


signals:    // To send data to QML
    void closed();
    void msgChanged(QString message);
    void statusChanged(QString status);
    void nrClientsChanged(QString nrClients);
    void portChanged(QString port);
    void delayChanged(QString delay);
    void msgSent();

public slots:   // To receive from QML
    void sendBuffer(DataBuffer * buffer);
    void pollSensor(); // FOR 1x polling. not implemented

    void sendData();
    void onPortChanged(const QString &msg);
    void onNewConnection();
    Q_INVOKABLE void processTextMessage(QString message);
    void processBinaryMessage(QByteArray message);
    void socketDisconnected();
private:
    QWebSocketServer * m_pWebSocketServer;
    QList<QWebSocket*> m_clients;
    bool m_debug;
    quint16 m_port;
    QTimer * m_timer;
    Status m_status;

    //QThread m_thread;//http://stackoverflow.com/questions/6382937/qt-how-to-put-my-function-into-a-thread
    IMUThread m_imuthread;
};

#endif // SENSORDATATCPSERVER_H
