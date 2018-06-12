#ifndef DATABUFFER_H
#define DATABUFFER_H

#include <QDataStream>
#include <QQueue>
#include <QDebug>
#include "vectors.h"

// This class provides storage and serializing capabilities for the sensor readings
class DataBuffer {  // the last point stores the time
public:
    DataBuffer& moveFirstFrom(DataBuffer&rhs) { // moves the first element of the buffer
        if(rhs.gyr.length())
            gyr.append(rhs.gyr.takeFirst());
        if(rhs.acc.length())
            acc.append(rhs.acc.takeFirst());
        if(rhs.mag.length())
            mag.append(rhs.mag.takeFirst());
        if(rhs.T.length())
            T.append(rhs.T.takeFirst());
        if(rhs.P.length())
            P.append(rhs.P.takeFirst());
        return *this;
    }
    DataBuffer& appendFrom(const DataBuffer rhs) { // appends the whole buffer
        QList<Vector4D>::const_iterator it;
        QList<Vector2D>::const_iterator it2;
        for(it=rhs.gyr.begin(); it!=rhs.gyr.end(); it++)
            gyr.append(*it);
        for(it=rhs.acc.begin(); it!=rhs.acc.end(); it++)
            acc.append(*it);
        for(it=rhs.mag.begin(); it!=rhs.mag.end(); it++)
            mag.append(*it);
        for(it=rhs.gyr.begin(); it!=rhs.gyr.end(); it++)
            gyr.append(*it);
        for(it2=rhs.T.begin(); it2!=rhs.T.end(); it2++)
            T.append(*it2);
        for(it2=rhs.P.begin(); it2!=rhs.P.end(); it2++)
            P.append(*it2);
        return *this;
    }

    DataBuffer& operator<<(DataBuffer&rhs) { // Moves the whole buffer
        while(rhs.gyr.length())
            gyr.append(rhs.gyr.takeFirst());
        while(rhs.acc.length())
            acc.append(rhs.acc.takeFirst());
        while(rhs.mag.length())
            mag.append(rhs.mag.takeFirst());
        while(rhs.T.length())
            T.append(rhs.T.takeFirst());
        while(rhs.P.length())
            P.append(rhs.P.takeFirst());
        return *this;
    }
    void clear() {
        gyr.clear(); acc.clear(); mag.clear(); T.clear(); P.clear();
    }

    int size() {
        return gyr.size()+acc.size()+mag.size()+T.size()+P.size();
    }

    // Serialize
    friend QDataStream& operator<<(QDataStream &stream, const DataBuffer & buffer) {
        stream << buffer.mag;
        stream << buffer.gyr;
        stream << buffer.acc;
        stream << buffer.T;
        stream << buffer.P;
        return stream;
    }
    // De-Serialize
    friend QDataStream& operator>>(QDataStream &stream, DataBuffer & buffer) {
        stream >> buffer.mag;
        stream >> buffer.gyr;
        stream >> buffer.acc;
        stream >> buffer.T;
        stream >> buffer.P;
        return stream;
    }
    QQueue<Vector4D> gyr,acc,mag;
    QQueue<Vector2D> T,P;
};
Q_DECLARE_METATYPE(DataBuffer*)

#endif // DATABUFFER_H
