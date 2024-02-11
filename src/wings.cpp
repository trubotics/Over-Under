#include "wings.h"

Wings::Wings(char extensionPort, char retractionPort)
{
    state = false;
    extensionCylinder = new pros::ADIDigitalOut(extensionPort);
    retractionCylinder = new pros::ADIDigitalOut(retractionPort);
}

bool Wings::getState()
{
    return state;
}

bool Wings::toggle()
{
    setState(!state);
    return state;
}

void Wings::setState(bool state)
{
    if (state == this->state)
    {
        return;
    }

    extensionCylinder->set_value(false);
    retractionCylinder->set_value(false);
    if (state) // To Extend
    {
        pros::Task extend([this, state]()
                          {
            pros::delay(250);
            if (this->state != state) // If state changes while waiting, cancel
            {
                return;
            }
            extensionCylinder->set_value(true); },
                          "Extend Task");
    }
    else // To Retract
    {
        pros::Task retractTask([this, state]()
                               {
            pros::delay(250);
            if (this->state != state) // If state changes while waiting, cancel
            {
                return;
            }
            retractionCylinder->set_value(true); },
                               "Retract Task");
    }

    this->state = state;
    return;
}