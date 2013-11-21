#include "RobotLeg.h"
#include "utility.h"



RobotLeg::RobotLeg(PinName thetaPin, PinName phiPin, PinName psiPin, bool start) : theta(thetaPin, start), phi(phiPin, start), psi(psiPin, start)
{
    setDimensions(0.1f, 0.1f, 0.0f, 0.0f);
    setAngleOffsets(0.0f, 0.0f, 0.0f);
    
    state = neutral;
    stepTime = 0.4f;
    stepDelta = 3.141593f / stepTime;
    stepHeight = 0.05f;
}



void RobotLeg::setDimensions(float a, float b, float c, float d)
{
    this->a = a;
    this->b = b;
    this->c = c;
    this->d = d;
}



void RobotLeg::setAngleOffsets(float oth, float oph, float ops)
{
    this->oth = oth;
    this->oph = oph;
    this->ops = ops;
}



void RobotLeg::setStepCircle(float xc, float yc, float zc, float rc)
{
    circleCenter = vector3(xc, yc, zc);
    circleRadius = rc;
}



vector3 RobotLeg::getPosition()
{
    return (stepping != state) ? position : stepB;
    
    /*const float deg2rad = 0.01745329f;
    vector3 p;
    float L, thetaR, phiR, psiR;
    
    // Convert degrees to radians
    thetaR = theta.read() * deg2rad;
    phiR = phi.read() * deg2rad;
    psiR = psi.read() * deg2rad;
    
    // Calculate forward kinematics
    L = a*cos(phiR + oph) + b*sin(phiR + oph + psiR + ops);
    p.x = -c*sin(thetaR + oth) + (L + d)*cos(thetaR + oth);
    p.y = c*cos(thetaR + oth) + (L + d)*sin(thetaR + oth);
    p.z = a*sin(phiR + oph) - b*cos(phiR + oph + psiR + ops);
    
    return p;*/
}



float RobotLeg::getStepDistance()
{
    // Returns distance to step circle edge in the current direction of movement.
    float vx, vy, m, cosval;
    
    vx = position.x - circleCenter.x;
    vy = position.y - circleCenter.y;
    m = sqrt(vx*vx + vy*vy);
    cosval = (nDeltaPosition.x*vx + nDeltaPosition.y*vy) / 
        (m * sqrt(nDeltaPosition.x*nDeltaPosition.x + nDeltaPosition.y*nDeltaPosition.y));
    
    return m*cosval + sqrt(pos(circleRadius*circleRadius - m*m*(1.0f - cosval*cosval)));
}



bool RobotLeg::move(vector3 dest)
{
    const float pi2 = 1.5707963;
    const float rad2deg =  57.2958;
    float L;
    
    position = dest;
    
    // Calculate new angles
    L = sqrt(dest.x*dest.x + dest.y*dest.y - c*c) - d;
    thetaAngle = atan2( ((L + d)*dest.y - c*dest.x), ((L + d)*dest.x + c*dest.y) ) - oth;
    phiAngle = atan2(dest.z, L) + acos((a*a + L*L + dest.z*dest.z - b*b)/(2.0f*a*sqrt(L*L + dest.z*dest.z))) - oph;
    psiAngle = acos((a*a + b*b - L*L - dest.z*dest.z)/(2.0f*a*b)) - ops - pi2;
    
    // Convert radians to degrees
    thetaAngle *= rad2deg;
    phiAngle *= rad2deg;
    psiAngle *= rad2deg;
    
    // Return true if angle is reachable
    if (thetaAngle <= theta.upperLimit && thetaAngle >= theta.lowerLimit &&
        phiAngle <= phi.upperLimit && phiAngle >= phi.lowerLimit &&
        psiAngle <= psi.upperLimit && psiAngle >= psi.lowerLimit)
    {
        // Set new angles
        theta = thetaAngle;
        phi = phiAngle;
        psi = psiAngle;
    
        return true;
    }
    else
    {
        return false;
    }
}



void RobotLeg::step(vector3 dest)
{
    stepA = position;
    stepB = dest;

    stepTimer.reset();
    stepTimer.start();
    state = stepping;
}



vector3 RobotLeg::reset(float f)
{
    vector3 newPosition;
    newPosition = circleCenter + nDeltaPosition.unit() * circleRadius * f;
    step(newPosition);
    return nDeltaPosition;
}



bool RobotLeg::update(const matrix4& deltaTransform)
{
    float t, d;
    vector3 newNDeltaPosition, v;
    const float eps = 0.00001f;

    switch (state)
    {
    case neutral:
        // Calculate new position and position delta
        newPosition = deltaTransform*position;
        newNDeltaPosition = position - newPosition;
        newNDeltaPosition.z = 0.0f;
        if (fabs(newNDeltaPosition.x) > eps || fabs(newNDeltaPosition.y) > eps)
            nDeltaPosition = newNDeltaPosition;
        
        // Check if new position is outside the step circle
        v = newPosition - circleCenter;
        d = sqrt(v.x*v.x + v.y*v.y);
        
        // Attempt to move to the new position
        return d < circleRadius;
        
    case stepping:
        // Compute new position along step trajectory
        t = stepTimer.read();
        
        if (t < stepTime)
        {
            newPosition.x = stepA.x + (stepB.x - stepA.x)*0.5f*(1 - cos(stepDelta*t));
            newPosition.y = stepA.y + (stepB.y - stepA.y)*0.5f*(1 - cos(stepDelta*t));
            newPosition.z = stepA.z + (stepB.z - stepA.z)*stepDelta*t + stepHeight*sin(stepDelta*t);
        }
        else
        {
            newPosition = stepB;
            state = neutral;
            stepTimer.stop();
        }
        
        move(newPosition);
        
        return true;
        
    default:
        return false;
    }
}



void RobotLeg::apply()
{
    if (stepping != state) move(newPosition);
}



bool RobotLeg::getStepping()
{
    return stepping == state;
}
