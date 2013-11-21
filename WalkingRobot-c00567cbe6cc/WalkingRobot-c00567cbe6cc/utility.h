inline float min(float a, float b)
{
    return (a < b ? a : b);
}



inline float pos(float f)
{
    return f > 0.0f ? f : 0.0f;
}



inline int deadzone(int input, int zone)
{
    if (input > zone) return input;
    else if (input < -zone) return input;
    else return 0;
}



inline int least(float f1, float f2, float f3, float f4)
{
    int value = 0;
    float temp = f1;
    
    if (f2 < temp)
    {
        value = 1;
        temp = f2;
    }
    
    if (f3 < temp)
    {
        value = 2;
        temp = f3;
    }
    
    if (f4 < temp)
    {
        value = 3;
    }
    
    return value;
}