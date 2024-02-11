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

    if (state) // To Extend
    {
        retractionCylinder->set_value(false);
        pros::Task extendTask([this, state]() {
            pros::delay(250);
            if (state != this->state)
            {
                return;
            }
            extensionCylinder->set_value(true);
        });
    }
    else // To Retract
    {
        extensionCylinder->set_value(false);
        pros::Task retractTask([this, state]() {
            pros::delay(250);
            if (state != this->state)
            {
                return;
            }
            retractionCylinder->set_value(true);
        });
    }

    this->state = state;
    return;
}