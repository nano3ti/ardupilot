#include "Copter.h"

#if MODE_SQUARE_ENABLED == ENABLED

/*
 * Init and run calls for square flight mode
 */

// square_init - initialise square controller flight mode
bool ModeSquare::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "ModeSquare::init");
    return true;
}

void ModeSquare::run()
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "ModeSquare::run");
}

uint32_t ModeSquare::wp_distance() const { return 1; }
int32_t ModeSquare::wp_bearing() const { return 1; }

#endif