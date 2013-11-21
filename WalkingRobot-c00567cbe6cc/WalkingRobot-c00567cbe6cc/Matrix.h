#ifndef MATRIX_H
#define MATRIX_H



struct vector3
{
    float x, y, z;
    vector3() {}
    vector3(float x1, float y1, float z1) { x = x1; y = y1; z = z1; }
    vector3 operator+(const vector3& v) const;
    vector3 operator-(const vector3& v) const;
    vector3 operator*(const float f) const;
    vector3 operator/(const float f) const;
    float norm() const;
    vector3 unit() const;
    void print(char* buf, unsigned int len);
};



struct matrix4
{
    float a11, a12, a13, a14;
    float a21, a22, a23, a24;
    float a31, a32, a33, a34;
    // Bottom row is always 0, 0, 0, 1
    
    matrix4();
    matrix4& identity();
    matrix4& translate(vector3 v);
    matrix4& rotateX(float radians);
    matrix4& rotateY(float radians);
    matrix4& rotateZ(float radians);
    matrix4 operator*(const matrix4& other) const;
    vector3 operator*(const vector3& other) const;
    matrix4 inverse() const;
    void print(char* buf, unsigned int len);
};

#endif // MATRIX_H