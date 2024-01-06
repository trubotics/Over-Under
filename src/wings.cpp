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
        extensionCylinder->set_value(true);
        retractionCylinder->set_value(false);
    }
    else // To Retract
    {
        extensionCylinder->set_value(false);
        retractionCylinder->set_value(true);

        // Retract retractCylinder after 1 second (it's not needed to hold the wings retracted)
        // @todo: Should we actually retract the retraction cylinder?
        pros::Task retractTask([this, state]()
                               {
            pros::delay(1000);
            if (this->state != state) // If state changes while waiting, cancel
            {
                return;
            }
            retractionCylinder->set_value(false); },
                               "Retract Task");
    }

    this->state = state;
    return;
}