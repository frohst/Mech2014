#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include "mbed.h"
#include "Servo.h"
#include "Matrix.h"



class RobotLeg
{
public:
    RobotLeg(PinName thetaPin, PinName phiPin, PinName psiPin, bool start = true);
    void setDimensions(float a, float b, float c, float d);
    void setAngleOffsets(float oth, float oph, float ops);
    void setStepCircle(float xc, float yc, float zc, float rc);
    vector3 getPosition();
    float getStepDistance();
    bool move(vector3 dest);
    void step(vector3 dest);
    vector3 reset(float f);
    bool update(const matrix4& deltaTransform);
    void apply();
    bool getStepping();

    Servo theta, phi, psi;
    vector3 nDeltaPosition;

protected:
    float thetaAngle, phiAngle, psiAngle;
    float circleRadius;
    float a, b, c, d;
    float oth, oph, ops;
    float stepDelta, stepTime, stepHeight;
    vector3 circleCenter;
    vector3 position;
    vector3 stepA;
    vector3 stepB;
    vector3 newPosition;
    
    enum state_t
    {
        neutral,
        stepping
    };
    
    state_t state;

    Timer stepTimer;
    
};

#endif // ROBOTLEG_H