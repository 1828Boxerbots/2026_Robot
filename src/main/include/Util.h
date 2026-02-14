#pragma once

bool CompareDoubles(double input, double targetValue, double tolerance)
{
    if ((input < (targetValue + tolerance)) && (input > (targetValue - tolerance)))
    {
        return true;
    }
    else
    {
        return false;
    }
}