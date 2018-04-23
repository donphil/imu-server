#include "sensordatatcpserver.h"
#include "QtWebSockets/qwebsocketserver.h"
#include "QtWebSockets/qwebsocket.h"
#include <QtCore/QDebug>

// For timer
#include <chrono>
#include <QElapsedTimer>
// higher accuracy: count cpu cycles: http://blog.regehr.org/archives/794

// FOR DEBUG
#include <iostream>
#include <thread>

QT_USE_NAMESPACE


void IMUThread::run()   // Poll sensor loop
{
    Vector4D acc,mag,gyr;
    QElapsedTimer timer;

    qint64 cycle_starttime, elapsed_mus, ms_elapsed_since_last_transmission;

    if(!m_imu.isEnabled())
        m_imu.enableIMU();
    uint8_t chip_id, version;
    m_imu.getSensorID(chip_id, version);
    qDebug() << "Sensor activated. ChipID: " << chip_id << " Version: " << version;

    int read_counter=0;
    timer.start();
    ms_elapsed_since_last_transmission = timer.elapsed();

    while(true) {
        cycle_starttime = timer.nsecsElapsed();
        m_imu.readACC(acc.xyz,false); // 1.4ms per read()
        acc.time = timer.nsecsElapsed()/1000;
        m_buffer.acc.append(acc);

        m_imu.readMAG(mag.xyz,true); // Mag and Acc share an address on the device, therefore not necesarry to selectDevice.
        mag.time = timer.nsecsElapsed()/1000;
        m_buffer.mag.append(mag);

        m_imu.readGYR(gyr.xyz,false);
        gyr.time = timer.nsecsElapsed()/1000;
        m_buffer.gyr.append(gyr);

         ++read_counter;
        // Before: read_counter>=min_transmission_length &&
        // Now send every 50ms     -> THIS MIGHT BE CRITICAL!!
        if( (timer.elapsed()-ms_elapsed_since_last_transmission>m_ms_between_transmissions)
             && m_mutex.tryLock(0)
            ) {
            ms_elapsed_since_last_transmission = timer.elapsed();
            //qDebug() << "Try sending:"<< read_counter << " acc\t" << acc.norm() << "\t" << acc.x <<"\t" << acc.y <<"\t"<< acc.z;
            // Push at least min_transmission_length data points into send buffer and emit
            while(--read_counter>=0)
                m_transmission_data.moveFirstFrom(m_buffer);
            m_mutex.unlock();
            emit data_ready(&m_transmission_data);
            read_counter = 0;
            if(isInterruptionRequested()) {
                qDebug() << "Thread interrupted.";
                return;
            }
        }
        // Read T&P only at request. Notice that the T reads are set to MAG_ODR
        if (m_read_temperature || m_read_pressure ) {
            int timestamp = timer.nsecsElapsed()/1000;
            if(m_read_pressure) {   // Both values needed, read from BMP180 sensor
                float T=-1,P=-1;
                m_imu.readTandP(T,P,false);
                m_buffer.P.append(Vector2D(timestamp, P));
                if(m_read_temperature)
                    m_buffer.T.append(Vector2D(timestamp, T));
                qDebug() << "T=" << T << " P="<< P ;
            }
            else {                  // Read from LSM9DS0
                float T = -1;
                m_imu.readTlsm(T);
                m_buffer.T.append(Vector2D(timestamp, T));
                qDebug() << "T=" << T;
            }
            m_read_temperature = false;
            m_read_pressure    = false;
        }

        elapsed_mus = (timer.nsecsElapsed()-cycle_starttime)/1000.;
        if(elapsed_mus<m_mus_delay) {
            //qDebug() << "sleeping for " << (m_delay_mus-elapsed_mus)/1000. << "ms";
            std::this_thread::sleep_for(std::chrono::microseconds(m_mus_delay-elapsed_mus));
        }
    }

}

IMUThread::IMUThread() {
    m_imu.setDatarate(BerryIMU::A_ODR_1600Hz);
    m_imu.setDatarate(BerryIMU::G_ODR_760_BW_50);
    m_imu.setDatarate(BerryIMU::M_ODR_100Hz);
}

void IMUThread::exitloop() {
    requestInterruption();
    wait();
    qDebug() << "Thread paused. IMU Status:" << m_imu.isEnabled();
}

void IMUThread::stop()
{
    requestInterruption();
    wait();
    m_imu.disableIMU();
    qDebug() << "Thread paused and IMU disabled.";
}

void IMUThread::clearBuffer() {
        if(m_mutex.tryLock(0)) {
            m_buffer.clear();
            m_transmission_data.clear();
            m_mutex.unlock();
        }
}
// --------------------------------------------------
//                  DATA SERVER
// --------------------------------------------------

SensorDataTcpServer::SensorDataTcpServer(quint16 port, bool debug, QObject *parent) :
    QObject(parent),
    m_pWebSocketServer(new QWebSocketServer(QStringLiteral("Sensor-Data Server"),
                                            QWebSocketServer::NonSecureMode, this)),
    m_debug(debug),
    m_status(offline)
{
    setMsg("Server is being constructed...");
    // Connect signals
    connect(m_pWebSocketServer, &QWebSocketServer::newConnection, this, &SensorDataTcpServer::onNewConnection);
    // Not doing anything, just relaying the signal
    connect(m_pWebSocketServer, &QWebSocketServer::closed, this, &SensorDataTcpServer::closed);

    connect( &m_imuthread, &IMUThread::data_ready, this, &SensorDataTcpServer::sendBuffer, Qt::QueuedConnection );

    setPort(port);
    resetServer();

    m_status = offline;
    startPolling();
}

void SensorDataTcpServer::resetServer() {
    setStatus("Shutting down server...");
    m_pWebSocketServer->close();
    qDeleteAll(m_clients.begin(), m_clients.end());

    setStatus("Restarting...");
    setPort(m_port,true);

    if (m_pWebSocketServer->listen(QHostAddress::Any, m_port)) {
        setStatus("Listening on port: " + QString::number(m_port) +"...");
        m_status = listening;
    }
    else {
        QWebSocketProtocol::CloseCode error_code = m_pWebSocketServer->error();
        setStatus("Error (server offline): " + QString::number(error_code));
        m_status = offline;
    }
}
void SensorDataTcpServer::resetIMU() {
    setStatus("Stopping thread...");
    endPolling();
    setStatus("Clearing buffers...");
    m_imuthread.clearBuffer();
    setStatus("Restarting thread...");
    startPolling();
}

void SensorDataTcpServer::sendBuffer(DataBuffer * buffer)
{   // We have just received a signal with buffer data.
    // now send it to receiver
    // First check if there are active clients
    if (m_clients.isEmpty())
        qDebug() << "sendBuffer: not connected to client. buffer length:" << buffer->size();
    else {
        // Ensure that the thread is not changing the data
        QMutexLocker ml(&m_imuthread.m_mutex);
        if(buffer->size()<=0)
            qDebug() << "sendBuffer: no data available";
        else {
            // convert data to bytearray using a stream. DataBuffer is serializable
            QByteArray byteArray;
            QDataStream stream(&byteArray, QIODevice::WriteOnly);
            stream.setVersion(QDataStream::Qt_5_3);
            stream.setByteOrder(QDataStream::BigEndian);
            // Send to all clients
            stream << *buffer;
            foreach(QWebSocket *pClient, m_clients) {
                    qint64 bytes_sent = pClient->sendBinaryMessage(byteArray);
                    setMsg(QString("Binary msg sent:") + QString::number(bytes_sent) + QString("Bytes, buffer.size()=") + QString::number(buffer->size()));
            }
            emit msgSent();
            buffer->clear();
            }
    }
}

SensorDataTcpServer::~SensorDataTcpServer()
{
    m_pWebSocketServer->close();
    qDeleteAll(m_clients.begin(), m_clients.end());
    if(m_imuthread.isRunning()) {
        m_imuthread.m_imu.disableIMU();
        m_imuthread.terminate();
    }
}


void SensorDataTcpServer::pollSensor()
{
}

void SensorDataTcpServer::startPolling() {
    m_imuthread.start();
}
void SensorDataTcpServer::pausePolling() {
    m_imuthread.exitloop();
}
void SensorDataTcpServer::endPolling() {
    m_imuthread.stop();
}

void SensorDataTcpServer::onPortChanged(const QString &msg)
{
    m_port = msg.toInt();
    m_pWebSocketServer->close();
    qDeleteAll(m_clients.begin(), m_clients.end());
    if (m_pWebSocketServer->listen(QHostAddress::Any, m_port)) {
        setStatus(QString("Listening on new port:") + QString(m_port));
        m_status = listening;
    }
    else
        setStatus("Listening on new port failed: " + m_pWebSocketServer->errorString());
    emit nrClientsChanged(QString::number(m_clients.length()));
}
void SensorDataTcpServer::onNewConnection()
{
    QWebSocket *pSocket = m_pWebSocketServer->nextPendingConnection();

    connect(pSocket, &QWebSocket::textMessageReceived, this, &SensorDataTcpServer::processTextMessage);
    connect(pSocket, &QWebSocket::binaryMessageReceived, this, &SensorDataTcpServer::processBinaryMessage);
    connect(pSocket, &QWebSocket::disconnected, this, &SensorDataTcpServer::socketDisconnected);
    setStatus("New connection established: " + pSocket->peerAddress().toString());
    m_status = connected;
    m_clients << pSocket;
    emit nrClientsChanged(QString::number(m_clients.length()));
}

bool IMUThread::setParametersGyr(int ires, int iodr, int ibw) {
    // input (floats converted to int):
    // res (dps) : [ "245", "500", "2000", "2001" ] 2001 is actually 2000. Indistinguishable from datasheet
    // iodr + bw: [ "95Hz 12.5", "95Hz 25", "190Hz 12.5", "190Hz 25","190Hz 50","190Hz 70", "380Hz 20","380Hz 25","380Hz 50","380Hz 100",
    //              "760Hz 30","760Hz 35","760Hz 50", "760Hz 100"]
    // All options (not configurable in this function):
    //configureGyr(gyro_odr odr, gyr_power_mode pd=G_POWER_NORMAL, bool enableX=true, bool enableY=true, bool enableZ=true);
    //bool configureGyr(HighPassMode hpm, gyr_high_pass hpcf);
    //bool configureGyr(gyr_scale scale, gyr_bdu bdu=G_CONTINUOUS_UPDATE, bool bigEndian = false,
    //                    gyr_selftest_mode selftest=G_TEST_OFF, bool spi_interface_mode = 0) ;
    using namespace BerryIMU;
    gyr_scale scale;
    switch(ires) {
        case 245  : scale = G_SCALE_245dps; break;
        case 500  : scale = G_SCALE_500dps; break;
        case 2000 : scale = G_SCALE_2000dps; break;
        case 20001: scale = G_SCALE_2000dpsB; break;
    default:
        qDebug() << "G_SCALE not found:" << ires;
        return false;
    }
    gyro_odr odr;
    switch(iodr) {
    case 95:
        switch(ibw) {
            case 12: odr = G_ODR_95_BW_125; break;
            case 25: odr = G_ODR_95_BW_25; break;
        }
        break;
    case 190:
        switch(ibw) {
            case 12: odr = G_ODR_190_BW_125; break;
            case 25: odr = G_ODR_190_BW_25; break;
            case 50: odr = G_ODR_190_BW_50; break;
            case 70: odr = G_ODR_190_BW_70; break;
        }
        break;
    case 380:
        switch(ibw) {
            case 20: odr = G_ODR_380_BW_20; break;
            case 25: odr = G_ODR_380_BW_25; break;
            case 50: odr = G_ODR_380_BW_50; break;
            case 100:odr = G_ODR_380_BW_100; break;
        }
        break;
    case 760:
        switch(ibw) {
            case 30: odr = G_ODR_760_BW_30; break;
            case 35: odr = G_ODR_760_BW_35; break;
            case 50: odr = G_ODR_760_BW_50; break;
            case 100:odr = G_ODR_760_BW_100; break;
        }
        break;
    default:
        qDebug() << "G_ODR not found:" << iodr;
        return false;
    }
    if(isRunning())
        return false;
    bool ret = m_imu.configureGyr(odr);
    if(ret)
        ret &= m_imu.configureGyr(scale);
    return ret;
}

bool IMUThread::setParametersAcc(int ires, int iodr) {
    // input:
    // res = [ 2,4,6,8,16 ]
    // odr = [ 3,6,12,25,50,100,200,400,800,1600]
    // All options (not configurable in this function):
    // configureAcc(acc_odr datarate, acc_bdu bdu = A_CONTINUOUS_UPDATE, bool enableX = true,bool enableY = true,bool enableZ = true);
    using namespace BerryIMU;
    acc_scale scale;
    switch(ires) {
        case 2: scale = A_SCALE_2g; break;
        case 4: scale = A_SCALE_4g; break;
        case 6: scale = A_SCALE_6g; break;
        case 8: scale = A_SCALE_8g; break;
        case 16:scale = A_SCALE_16g; break;
    default:
        qDebug() << "A_SCALE not found:" << ires;
        return false;
    }
    acc_odr odr;
    switch (iodr) {
        case 3 :  odr = A_ODR_3p125Hz; break;
        case 6 :  odr = A_ODR_6p25Hz; break;
        case 12:  odr = A_ODR_12p5Hz; break;
        case 25:  odr = A_ODR_25Hz; break;
        case 50:  odr = A_ODR_50Hz; break;
        case 100: odr = A_ODR_100Hz; break;
        case 200: odr = A_ODR_200Hz; break;
        case 400: odr = A_ODR_400Hz; break;
        case 800: odr = A_ODR_800Hz; break;
        case 1600:odr = A_ODR_1600Hz; break;
    default:
        qDebug() << "A_ODR not found:" << iodr;
        return false;
    }
    if(isRunning())
        return false;
    bool ret = m_imu.configureAcc(odr);
    if(ret)
        ret &= m_imu.configureAcc(scale); // maybe only some bandwidths are compatible:, acc_aa_bandwidth anti_alias_bandwidth=A_BANDWIDTH_773Hz,
    return ret;
}


bool IMUThread::setParametersMag(int ires, int iodr) {
    // input:
    // res = [ 2,4,8,12 ]
    // odr = [ 3,6,12,25,50,100] (from ["3.125Hz", "6.25Hz", "12.5Hz", "25Hz", "50Hz", "100Hz"])
    //
    // All options (not configurable in this function):

    //                configureMag(HighPassMode highpass, mag_filter_acceleration filter,
    //                                                mag_sensor_mode sensormode, mag_power_mode powermode) {
    //                configureMag(mag_scale scale) {
    //                configureMag(mag_odr datarate, mag_resolution mag_resolution, bool temperature,
    //                                                bool latch_interrupt_on_int1_src,bool latch_interrupt_on_int2_src)
    using namespace BerryIMU;
    mag_scale scale;
    switch(ires) {
        case 2: scale = M_SCALE_2Gs; break;
        case 4: scale = M_SCALE_4Gs; break;
        case 8: scale = M_SCALE_8Gs; break;
        case 12:scale = M_SCALE_12Gs; break;
    default:
        qDebug() << "M_SCALE not found:" << ires;
        return false;
    }
    mag_odr odr;
    switch (iodr) {
        case 3 :  odr = M_ODR_3p125Hz; break;
        case 6 :  odr = M_ODR_6p25Hz; break;
        case 12:  odr = M_ODR_12p5Hz; break;
        case 25:  odr = M_ODR_25Hz; break;
        case 50:  odr = M_ODR_50Hz; break;
        case 100: odr = M_ODR_100Hz; break;
    default:
        qDebug() << "M_ODR not found:" << iodr;
        return false;
    }
    if(isRunning())
        return false;
    bool ret = m_imu.configureMag(odr);
    if(ret)
        ret &= m_imu.configureMag(scale);
    return ret;
}


void SensorDataTcpServer::processTextMessage(QString message)
{
    if (m_debug)
        qDebug() << "TextMessage received:" << message;
    setMsg(message);
    setStatus("Message recvd.");
    // Parse message:
    // The message if of format "Command(:optionalarg)(:...)...
    QStringList commands = message.split(":");
    if(!commands.isEmpty()) {
        QString command = commands[0];
        if(command == "SamplingInterval" && commands.length()==2) {
            int interval = commands[1].toInt();
            m_imuthread.setDelay(1000*interval);
            setStatus("New polling interval:" + QString::number(interval) + "ms");
            emit delayChanged(QString::number(interval));   // Update qml
        }
        else if(command == "ClearBuffer") {
            if(commands.length()==2)    // Update minimal time between transmissions if set
                m_imuthread.setTransmissionInterval(commands[1].toInt());
            resetIMU();
        }
        else if(command == "readTP")
            m_imuthread.readTP();
        else if(command == "SetSensor") {
            // COMMANDS TO CHANGE SENSOR SETTINGS
            // They consist of a string, separated by ":" of the form
            // "SetSensor:Gyr:" + res + ":" + odr
            if(commands[1]=="Gyr"){
                QString res           = commands[2];  // [ "245dps", "500dps", "2000dps", "2000dpsB" ]
                QString odr_composite = commands[3];  // [ "95Hz 12.5", "95Hz 25", "190Hz 12.5", "190Hz 25","190Hz 50","190Hz 70", "380Hz 20","380Hz 25","380Hz 50","380Hz 100","760Hz 30","760Hz 35","760Hz 50", "760Hz 100"]]
                res.chop(3);    // remove the units
                QStringList odr_settings = odr_composite.split(" ");
                QString odr = odr_settings[0];
                odr.chop(2);
                QString bw  = odr_settings[1];
                bw.chop(3);
                if(bw.length()==5) // Then it was 2000dpsB, but only 3 values have been chopped. Change to 20001
                    bw[4]='1';
                endPolling();

                setStatus(QString("Received: ") + res + QString(" odr:") + odr + " bw:" + bw);
                bool success = m_imuthread.setParametersGyr(res.toInt(), odr.toInt(), bw.toInt());
                setStatus(QString("Gyr has been reset?") + QString::number(success));
                startPolling();
            }
            else if(commands[1]=="Acc") {
                QString res = commands[2];  // [ "2g", "4g", "6g", "8g", "16g" ]
                QString odr = commands[3];  // [ "3.125Hz", "6.25Hz", "12.5Hz", "25Hz", "50Hz", "100Hz", "200Hz", "400Hz", "800Hz", "1600Hz" ]
                res.chop(1);    // remove the units
                odr.chop(2);
                endPolling();
                setStatus(QString("Received: ") + res + QString(" odr:") + odr);
                bool success = m_imuthread.setParametersAcc(res.toInt(), odr.toInt());
                setStatus(QString("Acc has been reset?") + QString::number(success));
                startPolling();
            }
            else if(commands[1]=="Mag") {
                QString res = commands[2];  // [ "2G", "4G", "8G", "12G" ]
                QString odr = commands[3];  // [ "3.125Hz", "6.25Hz", "12.5Hz", "25Hz", "50Hz", "100Hz"]
                res.chop(1);    // remove the units
                odr.chop(2);
                endPolling();
                setStatus(QString("Received: ") + res + QString(" odr:") + odr);
                bool success = m_imuthread.setParametersMag(res.toInt(), odr.toInt());
                setStatus(QString("Mag has been reset?") + QString::number(success));
                startPolling();
            }
        }
    }
}

void SensorDataTcpServer::socketDisconnected()
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (m_debug)
        setStatus(QString("socketDisconnected: ").append( QString(pClient->objectName())));
    if (pClient) {
        m_clients.removeAll(pClient);
        pClient->deleteLater();
    }
    emit nrClientsChanged(QString::number(m_clients.length()));
}


// THESE THINGS DONT DO ANYTHING REALLY. JUST KEEP UNTIL CODE FRAGMENTS ARE NOT NEEDED

// Sending back
void SensorDataTcpServer::sendData() {
    //QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    // Check if there are active clients
    return;
    if (!m_clients.isEmpty()) {
        // Prepare message
        QString x = QString::number(rand());
        QString y = QString::number(rand());
        QString z = QString::number(rand());
        // Format of message
        // date(qint64);Temperature;Pressure;Magx,Magy,Magz;Gyrx,Gyry,Gyrz;Accx,Accy,Accz#
        qint64 utc = QDateTime::currentMSecsSinceEpoch();


        QString xyz = x + "," + y + "," + z;
        QStringList outgoing;
        outgoing << QString::number(utc) << "10" << "0" << xyz << xyz << xyz;
        //setStatus("Send: " + outgoing.join(';'));
        emit msgSent();
        // Send to all clients
        foreach(QWebSocket *pClient, m_clients)
                pClient->sendTextMessage(outgoing.join(';'));
    }
}
void SensorDataTcpServer::processBinaryMessage(QByteArray message)
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (m_debug)
        setStatus(QString("Binary msg recvd."));

    if (pClient) {
        pClient->sendBinaryMessage(message);
    }
}

//

//    namespace chrono = std::chrono;
//    typedef std::chrono::high_resolution_clock high_resolution_clock_t;
//    typedef chrono::time_point<high_resolution_clock_t,chrono::nanoseconds> ns_time_point_t;

//    qDebug() << "finest measurable time interval:" <<
//                (double)high_resolution_clock_t::period::num / high_resolution_clock_t::period::den;
//    std::cout << "resolution (nano) = " << (double) std::chrono::high_resolution_clock::period::num
//            / std::chrono::high_resolution_clock::period::den * 1000 * 1000 * 1000 << std::endl;

//    m_imu.moveToThread(&m_thread);
