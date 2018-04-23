#ifndef VECTORS_H
#define VECTORS_H

#include <QDataStream>

class Vector2D {
public:
    quint64 time=0;
    double x=0;
    Vector2D() {}
    Vector2D(quint64 time, double x) : time(time),x(x) {}
    Vector2D(const Vector2D &vec4) : time(vec4.time) { x= vec4.x;}
    // Print
    operator QString() const { return QString("Vector2D(%1,%2)").arg(QString::number(time),QString::number(x)); }
    // Serialize
    friend QDataStream& operator<<(QDataStream &stream, const Vector2D &vec2d) {
        stream << vec2d.time << vec2d.x;
        return stream;
    }
    // Deserialize
    friend QDataStream &operator>>(QDataStream &stream, Vector2D &vec2d)
    {
        stream >> vec2d.time >> vec2d.x;
        return stream;
    }
};

class Vector4D {
public:
    Vector4D(double * vec3, quint64 time) : time(time), x(xyz[0]),y(xyz[1]),z(xyz[2])  {
        xyz[0]=vec3[0]; xyz[1]=vec3[1]; xyz[2]=vec3[2];
    }
    Vector4D(double u,double v,double w, quint64 time) : time(time), x(xyz[0]),y(xyz[1]),z(xyz[2])  {
        xyz[0]=u; xyz[1]=v; xyz[2]=w;
    }
    Vector4D(const Vector4D &copy) :
         time(copy.time),x(xyz[0]),y(xyz[1]),z(xyz[2])
        {
            xyz[0] = copy.x;
            xyz[1] = copy.y;
            xyz[2] = copy.z;
        }

    Vector4D() :x(xyz[0]),y(xyz[1]),z(xyz[2]) {}
    Vector4D(Vector2D v2) :x(xyz[0]),y(xyz[1]),z(xyz[2]) { time = v2.time; xyz[0]=v2.x; }
    Vector4D(quint64 time) : time(time),x(xyz[0]),y(xyz[1]),z(xyz[2]) {}

    // Convenience
    double norm() {
        return sqrt(x*x+y*y+z*z);
    }
    operator bool() const {
        return time!=0;
    }

    Vector4D operator+(const Vector4D& rhs) {
        // Notice that time is unchanged
        return Vector4D(x+rhs.x, y+rhs.y, z+rhs.z, time);
    }

    Vector4D operator*(double scalar) {
        // Notice that time is unchanged
        return Vector4D(x*scalar, y*scalar, z*scalar, time);
    }

    Vector4D& operator+=(const Vector4D& rhs) {
        xyz[0]+=rhs.xyz[0];
        xyz[1]+=rhs.xyz[1];
        xyz[2]+=rhs.xyz[2];
        return *this;
    }
    Vector4D& operator=(const Vector4D& rhs) {
        xyz[0]=rhs.xyz[0];
        xyz[1]=rhs.xyz[1];
        xyz[2]=rhs.xyz[2];
        time  =rhs.time;
        return *this;
    }
    // Print
    operator QString() const { return QString("Vector4D(%1,%2,%3,%4)").arg(QString::number(time),QString::number(x),QString::number(y),QString::number(z)); }
    // Serialize
    friend QDataStream& operator<<(QDataStream &stream, const Vector4D &vec4d) {
        stream << vec4d.time << vec4d.x << vec4d.y << vec4d.z;
        return stream;
    }
    friend QTextStream& operator<<(QTextStream &stream, const Vector4D &vec4d) {
        stream << vec4d.time << "," << vec4d.x << "," << vec4d.y << "," << vec4d.z;
        return stream;
    }
    // Deserialize
    friend QDataStream &operator>>(QDataStream &stream, Vector4D &vec4d)
    {
        stream >> vec4d.time >> vec4d.x >> vec4d.y >> vec4d.z;
        return stream;
    }

    inline double & operator[] (int i) { return xyz[i]; }

    quint64 time=0;
    double xyz[3]={0};
    double &x,&y,&z;
};


#endif // VECTORS_H
