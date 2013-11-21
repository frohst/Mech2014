#include "mbed.h"
#include "RobotLeg.h"
#include "Matrix.h"
#include "CircularBuffer.h"
#include "Radio.h"
#include "Terminal.h"
#include "utility.h"
#include <cstring>
#include <cmath>

#define MAXSPEED 0.1f
#define MAXTURN 1.0f
#define RESET_STEP_TIME 0.4f
#define DIM_A 0.125f
#define DIM_B 0.11f
#define DIM_C 0.0025f
#define DIM_D 0.0275f
#define CIRCLE_X 0.095f
#define CIRCLE_Y 0.095f
#define CIRCLE_Z -0.12f
#define CIRCLE_R 0.09f
#define PERIOD 0.005f



CircularBuffer<float,16> dataLog;
Radio radio(p5, p6, p7, p16, p17, p18);
RobotLeg legA(p26, p29, p30, false);
RobotLeg legB(p13, p14, p15, false);
RobotLeg legC(p19, p11, p8, false);
RobotLeg legD(p25, p24, p23, false);
RobotLeg* leg[4] = { &legA, &legB, &legC, &legD };
matrix4 QMat[4];
matrix4 PMat[4];

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);



CmdHandler* legpos(Terminal* terminal, const char*)
{
    char output[256];
    char abuf[64];
    char bbuf[64];
    char cbuf[64];
    char dbuf[64];
    legA.getPosition().print(abuf, 64);
    legB.getPosition().print(bbuf, 64);
    legC.getPosition().print(cbuf, 64);
    legD.getPosition().print(dbuf, 64);
    snprintf(output, 256, "A = [%s]\nB = [%s]\nC = [%s]\nD = [%s]", abuf, bbuf, cbuf, dbuf);
    terminal->write(output);
    return NULL;
}



CmdHandler* log(Terminal* terminal, const char* input)
{
    int start = 0;
    int end = 15;
    char output[256];
    
    if (sscanf(input, "log %d %d", &start, &end) == 1)
    {
        // Print only one item
        snprintf(output, 256, "%4d: %f\n", start, dataLog[start]);
        terminal->write(output);
    }
    else
    {
        // Print a range of items
        for (int i = start; i <= end; i++)
        {
            snprintf(output, 256, "%4d: %f\n", i, dataLog[i]);
            terminal->write(output);
        }
    }  
    
    return NULL;
} // log()



bool processMovement(matrix4& TMat);
void setupLegs();
void resetLegs();
float calcStability(vector3 p1, vector3 p2);



int main()
{
    Timer deltaTimer;
    Terminal terminal;
    
    terminal.addCommand("log", &log);
    terminal.addCommand("leg", &legpos);
    
    radio.reset();
    setupLegs();
    
    // Initialize matrices to change base from robot coordinates to leg coordinates
    QMat[0].translate(vector3(0.0508f, 0.0508f, 0.0f));
    QMat[1].translate(vector3(-0.0508f, -0.0508f, 0.0f));
    QMat[1].a11 = -1.0f; QMat[1].a22 = -1.0f;
    QMat[2].translate(vector3(-0.0508f, 0.0508f, 0.0f));
    QMat[2].a11 = -1.0f;
    QMat[3].translate(vector3(0.0508f, -0.0508f, 0.0f));
    QMat[3].a22 = -1.0f;
    
    PMat[0] = QMat[0].inverse();
    PMat[1] = QMat[1].inverse();
    PMat[2] = QMat[2].inverse();
    PMat[3] = QMat[3].inverse();
    
    matrix4 TMat;
    
    // Start timer
    deltaTimer.start();
    
    while (true)
    {
        while (deltaTimer.read() < PERIOD);
        
        // Read controller input
        float xaxis = 0.0078125f * deadzone((int8_t)((radio.rx_controller>>0)&0xff), 8); // Convert to +/-1.0f range
        float yaxis = -0.0078125f * deadzone((int8_t)((radio.rx_controller>>8)&0xff), 8);
        float turnaxis = -0.0078125f * deadzone((int8_t)((radio.rx_controller>>16)&0xff), 8);
        
        // Reset legs to sane positions when 'A' button is pressed
        if ((radio.rx_controller>>25)&0x1) resetLegs();
        
        deltaTimer.reset();
        dataLog.push(deltaTimer.read());
        
        // Compute delta movement vector and delta angle
        vector3 v(-xaxis, -yaxis, 0.0f);
        v = v * MAXSPEED * PERIOD;
        float angle = -turnaxis * MAXTURN * PERIOD;
        
        // Compute movement transformation in robot coordinates
        TMat.identity().rotateZ(angle).translate(v).inverse();
        
        processMovement(TMat);
        
    } // while (true)
} // main()



bool processMovement(matrix4& TMat)
{
    // Get points used to calculate stability
    vector3 point1[4];
    vector3 point2[4];
    point1[0] = QMat[2]*leg[2]->getPosition();
    point1[1] = QMat[3]*leg[3]->getPosition();
    point1[2] = QMat[1]*leg[1]->getPosition();
    point1[3] = QMat[0]*leg[0]->getPosition();
    point2[0] = QMat[3]*leg[3]->getPosition();
    point2[1] = QMat[2]*leg[2]->getPosition();
    point2[2] = QMat[0]*leg[0]->getPosition();
    point2[3] = QMat[1]*leg[1]->getPosition();
    
    // Check if each leg can perform this motion, find the next leg to step, and calculate stability of each leg
    bool legFree[4];
    float stepDist[4];
    float stability[4];
    for (int i = 0; i < 4; ++i)
    {
        legFree[i] = leg[i]->update(PMat[i]*TMat*QMat[i]);
        stepDist[i] = leg[i]->getStepDistance();
        stability[i] = calcStability(point1[i], point2[i]);
    }
    
    // Check if each leg needs to step, and then check if it's stable before stepping
    bool stepping = leg[0]->getStepping() || leg[1]->getStepping() || leg[2]->getStepping() || leg[3]->getStepping();
    const float borderMax = 0.015f; // radius of support base in meters
    const float borderMin = 0.007f;
    
    for (int i = 0; i < 4; ++i)
    {
        if (!legFree[i])
        {
            if (stepping)
            {
                TMat.identity();
                return false;
            }
            else 
            {
                if (stability[i] > borderMin)
                {
                    // If stable, step
                    leg[i]->reset(0.8);
                    stepping = true;
                }
                else
                {
//                    // If unstable, move towards a stable position
//                    vector3 n;
//                    n.x = point2[i].y - point1[i].y;
//                    n.y = point1[i].x - point2[i].x;
//                    n = n.unit() * MAXSPEED * PERIOD;
//                    TMat.identity().translate(n).inverse();
//                    return false;
                }
            }
        }
    }
    
    // Check if the next leg to step is stable
    int next = least(stepDist[0], stepDist[1], stepDist[2], stepDist[3]);
    if (stability[next] > borderMax)
    {
        // Continue to carry out step as normal
    }
    else if (stability[next] > borderMin)
    {
        if (stepping)
        {
            TMat.identity();
            return false;
        }
        else 
        {
            leg[next]->reset(0.8);
            stepping = true;
        }
    }
    else
    {
//        // If unstable, move towards a stable position
//        vector3 n;
//        n.x = point2[next].y - point1[next].y;
//        n.y = point1[next].x - point2[next].x;
//        n = n.unit() * MAXSPEED * PERIOD;
//        TMat.identity().translate(n).inverse();
//        return false;
    }
    
    for (int i = 0; i < 4; ++i)
    {
        leg[i]->apply();
    }
    
    // Debug info
    led1 = stability[0] > borderMin;
    led2 = stability[1] > borderMin;
    led3 = stability[2] > borderMin;
    led4 = stability[3] > borderMin;
    
    return true;
}



void resetLegs()
{
    matrix4 T;
    legA.reset(-0.6f);
    while (legA.getStepping())
    {
        legA.update(T);
        legA.apply();
    }
    legB.reset(-0.1f);
    while (legB.getStepping())
    {
        legB.update(T);
        legB.apply();
    }
    legC.reset(0.4f);
    while (legC.getStepping())
    {
        legC.update(T);
        legC.apply();
    }
    legD.reset(0.9f);
    while (legD.getStepping())
    {
        legD.update(T);
        legD.apply();
    }
}



void setupLegs()
{
    // Set leg parameters
    legA.setDimensions(DIM_A, DIM_B, DIM_C, DIM_D);
    legB.setDimensions(DIM_A, DIM_B, DIM_C, DIM_D);
    legC.setDimensions(DIM_A, DIM_B, DIM_C, DIM_D);
    legD.setDimensions(DIM_A, DIM_B, DIM_C, DIM_D);
    legA.setAngleOffsets(0.7853982f, 0.0f, 0.0f);
    legB.setAngleOffsets(0.7853982f, 0.0f, 0.0f);
    legC.setAngleOffsets(0.7853982f, 0.0f, 0.0f);
    legD.setAngleOffsets(0.7853982f, 0.0f, 0.0f);
    legA.setStepCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_Z, CIRCLE_R);
    legB.setStepCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_Z, CIRCLE_R);
    legC.setStepCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_Z, CIRCLE_R);
    legD.setStepCircle(CIRCLE_X, CIRCLE_Y, CIRCLE_Z, CIRCLE_R);
    legA.theta.calibrate(1130, 2080, 45.0f, -45.0f);
    legA.phi.calibrate(1150, 2080, 70.0f, -45.0f);
    legA.psi.calibrate(1985, 1055, 70.0f, -60.0f);
    legB.theta.calibrate(990, 1940, 45.0f, -45.0f);
    legB.phi.calibrate(1105, 2055, 70.0f, -45.0f);
    legB.psi.calibrate(2090, 1150, 70.0f, -60.0f);
    legC.theta.calibrate(1930, 860, 45.0f, -45.0f);
    legC.phi.calibrate(1945, 1000, 70.0f, -45.0f);
    legC.psi.calibrate(1085, 2005, 70.0f, -60.0f);
    legD.theta.calibrate(2020, 1080, 45.0f, -45.0f);
    legD.phi.calibrate(2085, 1145, 70.0f, -45.0f);
    legD.psi.calibrate(1070, 2010, 70.0f, -60.0f);
    
    // Initialize leg position deltas
    legA.nDeltaPosition = vector3(0.0f, 0.01f, 0.0f);
    legB.nDeltaPosition = vector3(0.0f, -0.01f, 0.0f);
    legC.nDeltaPosition = vector3(0.0f, 0.01f, 0.0f);
    legD.nDeltaPosition = vector3(0.0f, -0.01f, 0.0f);
    
    // Go to initial position
    legA.move(vector3(0.15f, 0.15f, 0.05f));
    legB.move(vector3(0.15f, 0.15f, 0.05f));
    legC.move(vector3(0.15f, 0.15f, 0.05f));
    legD.move(vector3(0.15f, 0.15f, 0.05f));
    legA.theta.enable(); wait(0.1f);
    legB.theta.enable(); wait(0.1f);
    legC.theta.enable(); wait(0.1f);
    legD.theta.enable(); wait(0.1f);
    legA.phi.enable(); wait(0.1f);
    legB.phi.enable(); wait(0.1f);
    legC.phi.enable(); wait(0.1f);
    legD.phi.enable(); wait(0.1f);
    legA.psi.enable(); wait(0.1f);
    legB.psi.enable(); wait(0.1f);
    legC.psi.enable(); wait(0.1f);
    legD.psi.enable(); wait(0.1f);
    wait(0.4f);
    legA.reset(-0.6f);
    legB.reset(-0.1f);
    legC.reset(0.4f);
    legD.reset(0.9f);
    matrix4 T;
    while (legA.getStepping())
    {
        legA.update(T);
        legA.apply();
        legB.update(T);
        legB.apply();
        legC.update(T);
        legC.apply();
        legD.update(T);
        legD.apply();
    }
}



float calcStability(vector3 p1, vector3 p2)
{
    float lx, ly, vx, vy;
    lx = p2.x - p1.x;
    ly = p2.y - p1.y;
    vx = -p1.x;
    vy = -p1.y;
    
    return (ly*vx - lx*vy)/sqrt(lx*lx + ly*ly);
}

