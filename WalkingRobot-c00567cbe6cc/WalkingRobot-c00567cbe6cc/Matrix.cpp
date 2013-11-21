#include "Matrix.h"
#include "mbed.h"
#include <cmath>



vector3 vector3::operator+(const vector3& v) const
{
    vector3 r;
    r.x = x + v.x;
    r.y = y + v.y;
    r.z = z + v.z;
    return r;
}



vector3 vector3::operator-(const vector3& v) const
{
    vector3 r;
    r.x = x - v.x;
    r.y = y - v.y;
    r.z = z - v.z;
    return r;
}



vector3 vector3::operator*(const float f) const
{
    vector3 r;
    r.x = x * f;
    r.y = y * f;
    r.z = z * f;
    return r;
}



vector3 vector3::operator/(const float f) const
{
    vector3 r;
    r.x = x / f;
    r.y = y / f;
    r.z = z / f;
    return r;
}



float vector3::norm() const
{
    return sqrt(x*x + y*y + z*z);
}



vector3 vector3::unit() const
{
    return (*this)/norm();
}



void vector3::print(char* buf, unsigned int len)
{
    snprintf(buf, len, "%.4f\t%.4f\t%.4f", x, y, z);
}



matrix4::matrix4()
{
    // Initialize as identity matrix
    identity();
}



matrix4& matrix4::identity()
{
    a11 = 1.0f; a12 = 0.0f; a13 = 0.0f; a14 = 0.0f;
    a21 = 0.0f; a22 = 1.0f; a23 = 0.0f; a24 = 0.0f;
    a31 = 0.0f; a32 = 0.0f; a33 = 1.0f; a34 = 0.0f;
    return *this;
}



matrix4& matrix4::translate(const vector3 v)
{
    a14 += v.x;
    a24 += v.y;
    a34 += v.z;
    return *this;
}



matrix4& matrix4::rotateX(float radians)
{
    float b21 = a21;
    float b22 = a22;
    float b23 = a23;
    float b24 = a24;
    float sinx = sin(radians);
    float cosx = cos(radians);
    
    a21 = a21*cosx - a31*sinx;
    a22 = a22*cosx - a32*sinx;
    a23 = a23*cosx - a33*sinx;
    a24 = a24*cosx - a34*sinx;
    
    a31 = a31*cosx + b21*sinx;
    a32 = a32*cosx + b22*sinx;
    a33 = a33*cosx + b23*sinx;
    a34 = a34*cosx + b24*sinx;
    
    return *this;
}



matrix4& matrix4::rotateY(float radians)
{
    float b31 = a31;
    float b32 = a32;
    float b33 = a33;
    float b34 = a34;
    float sinx = sin(radians);
    float cosx = cos(radians);
    
    a31 = a31*cosx - a11*sinx;
    a32 = a32*cosx - a12*sinx;
    a33 = a33*cosx - a13*sinx;
    a34 = a34*cosx - a14*sinx;
    
    a11 = a11*cosx + b31*sinx;
    a12 = a12*cosx + b32*sinx;
    a13 = a13*cosx + b33*sinx;
    a14 = a14*cosx + b34*sinx;
    
    return *this;
}



matrix4& matrix4::rotateZ(float radians)
{
    float b11 = a11;
    float b12 = a12;
    float b13 = a13;
    float b14 = a14;
    float sinx = sin(radians);
    float cosx = cos(radians);
    
    a11 = a11*cosx - a21*sinx;
    a12 = a12*cosx - a22*sinx;
    a13 = a13*cosx - a23*sinx;
    a14 = a14*cosx - a24*sinx;
    
    a21 = a21*cosx + b11*sinx;
    a22 = a22*cosx + b12*sinx;
    a23 = a23*cosx + b13*sinx;
    a24 = a24*cosx + b14*sinx;
    
    return *this;
}



matrix4 matrix4::operator*(const matrix4& other) const
{
    matrix4 result;
    
    result.a11 = a11*other.a11 + a12*other.a21 + a13*other.a31;
    result.a12 = a11*other.a12 + a12*other.a22 + a13*other.a32;
    result.a13 = a11*other.a13 + a12*other.a23 + a13*other.a33;
    result.a14 = a11*other.a14 + a12*other.a24 + a13*other.a34 + a14;
    
    result.a21 = a21*other.a11 + a22*other.a21 + a23*other.a31;
    result.a22 = a21*other.a12 + a22*other.a22 + a23*other.a32;
    result.a23 = a21*other.a13 + a22*other.a23 + a23*other.a33;
    result.a24 = a21*other.a14 + a22*other.a24 + a23*other.a34 + a24;
    
    result.a31 = a31*other.a11 + a32*other.a21 + a33*other.a31;
    result.a32 = a31*other.a12 + a32*other.a22 + a33*other.a32;
    result.a33 = a31*other.a13 + a32*other.a23 + a33*other.a33;
    result.a34 = a31*other.a14 + a32*other.a24 + a33*other.a34 + a34;
    
    return result;
}



vector3 matrix4::operator*(const vector3& other) const
{
    vector3 result;
    
    result.x = a11*other.x + a12*other.y + a13*other.z + a14;
    result.y = a21*other.x + a22*other.y + a23*other.z + a24;
    result.z = a31*other.x + a32*other.y + a33*other.z + a34;
    
    return result;
}



matrix4 matrix4::inverse() const
{
    matrix4 result;
    float idet = 1.0f/(a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31);
    
    result.a11 = (a22*a33 - a23*a32)*idet;
    result.a12 = (a13*a32 - a12*a33)*idet;
    result.a13 = (a12*a23 - a13*a22)*idet;
    result.a14 = (a12*a24*a33 - a12*a23*a34 + a13*a22*a34 - a13*a24*a32 - a14*a22*a33 + a14*a23*a32)*idet;
    
    result.a21 = (a23*a31 - a21*a33)*idet;
    result.a22 = (a11*a33 - a13*a31)*idet;
    result.a23 = (a13*a21 - a11*a23)*idet;
    result.a24 = (a11*a23*a34 - a11*a24*a33 - a13*a21*a34 + a13*a24*a31 + a14*a21*a33 - a14*a23*a31)*idet;
    
    result.a31 = (a21*a32 - a22*a31)*idet;
    result.a32 = (a12*a31 - a11*a32)*idet;
    result.a33 = (a11*a22 - a12*a21)*idet;
    result.a34 = (a11*a24*a32 - a11*a22*a34 + a12*a21*a34 - a12*a24*a31 - a14*a21*a32 + a14*a22*a31)*idet;

    return result;
}



void matrix4::print(char* buf, unsigned int len)
{
    snprintf(buf, len, "%.4f\t%.4f\t%.4f\t%.4f\n"
                          "%.4f\t%.4f\t%.4f\t%.4f\n"
                          "%.4f\t%.4f\t%.4f\t%.4f\n"
                          "0     \t0     \t0     \t1\n",
            a11, a12, a13, a14,
            a21, a22, a23, a24,
            a31, a32, a33, a34);
}
