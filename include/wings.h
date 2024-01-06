#ifndef WINGS_H
#define WINGS_H

#include "main.h"

class Wings
{
public:
    /**
     * @brief Constructs a Wings object with the specified three wire ports.
     *
     */
    Wings(char extensionPort, char retractionPort);

    /**
     * @brief Gets the state of the wings. True is extended, false is retracted.
     *
     * @return The state of the wings.
     */
    bool getState();

    /**
     * @brief Toggles the state of the wings.
     *
     * @return The new state of the wings. True is extended, false is retracted.
     */
    bool toggle();

    /**
     * @brief Sets the state of the wings.
     *
     * @param state The new state of the wings. True is extended, false is retracted.
     */
    void setState(bool state);

private:
    bool state; // The current state of the wings. True is extended, false is retracted.
    pros::ADIDigitalOut* extensionCylinder; // The extension cylinder.
    pros::ADIDigitalOut* retractionCylinder; // The retraction cylinder.
};

#endif // WINGS_H
