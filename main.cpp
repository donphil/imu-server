#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QtQuick>
#include "sensordatatcpserver.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    qRegisterMetaType<DataBuffer*>();

    SensorDataTcpServer server(9001,true);
    engine.rootContext()->setContextProperty("server", &server);

    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    QObject *rootObject = engine.rootObjects().first();

    QObject::connect(rootObject, SIGNAL(portSignal(QString)), &server, SLOT(onPortChanged(QString)));

    //QObject::connect(&server, SIGNAL(portChanged(QString)),
    //                 &rootObject, SLOT(portResetFromCpp(QString)));
 // Step 1: get access to the root object

    //QObject *qmlObject = rootObject->findChild<QObject*>("wnd");

        /*QList<QObject *> objects = rootObject->findChildren<QObject *>();
        foreach (QObject* b, objects)
            qDebug() << QString::fromStdString(b->objectName().toStdString());
            //*/
    //    if(qmlObject) {
    //        qmlObject->setProperty("text", QVariant("newtext"));
    //    }


    return app.exec();
}
