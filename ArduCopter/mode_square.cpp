#include "Copter.h"

#if MODE_SQUARE_ENABLED == ENABLED

// TODO use parameters
constexpr float AP_SQUARE_ALT = 500; // cm
constexpr float AP_SQUARE_SIZE = 400; // cm
constexpr float AP_SQUARE_WP_RADIUS = 20; // cm
constexpr float AP_SQUARE_LANDING_VEL = 20; // cm/s

/*
 * Init and run calls for square flight mode
 */

// square_init - initialise square controller flight mode
bool ModeSquare::init(bool ignore_checks)
{
    if (!is_disarmed_or_landed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Not landed");
        return false;
    }

    if (!copter.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Position is not valid");
        return false;
    }

    _state = _State::START;

    return true;
}

bool ModeSquare::_wp_reached() const {
    return (pos_control->get_pos_error_cm().length() < AP_SQUARE_WP_RADIUS);
}

void ModeSquare::_set_wp(uint8_t wp_idx) {
    if (wp_idx >= _n_wps) return;
    _wp_idx = wp_idx;
    const auto wp = _wps[wp_idx];
    pos_control->set_pos_target_xy_cm(wp.x, wp.y);
    gcs().send_text(MAV_SEVERITY_DEBUG, "WP %d", wp_idx);
}

void ModeSquare::_start() {
    // initialise the horizontal position controller
    pos_control->init_xy_controller_stopping_point();
    // initialise the vertical position controller
    pos_control->init_z_controller_stopping_point();

    // Get current NEU pos
    Vector3p current_pos = pos_control->get_pos_target_cm();

    _ground_alt = current_pos.z;

    // Set waypoints
    _wps[0] = Vector2p(current_pos.x + AP_SQUARE_ALT, current_pos.y);
    _wps[1] = Vector2p(current_pos.x + AP_SQUARE_ALT, current_pos.y + AP_SQUARE_ALT);
    _wps[2] = Vector2p(current_pos.x, current_pos.y + AP_SQUARE_ALT);
    _wps[3] = current_pos.xy();

    // Set takeoff target pos
    pos_control->set_pos_target_xy_cm(current_pos.x, current_pos.y);
    pos_control->set_pos_target_z_cm(current_pos.z + AP_SQUARE_ALT);
}

void ModeSquare::run()
{
    if (!motors->armed()) {
        _state = _State::START;
        make_safe_ground_handling();
        return;
    }

    switch (_state)
    {
    case _State::START:
        _start();
        _state = _State::TAKEOFF;
        gcs().send_text(MAV_SEVERITY_DEBUG, "Takeoff");
        break;

    case _State::TAKEOFF:
        if (_wp_reached()) {
            _state = _State::WP;
            _set_wp(0);
        }
        break;

    case _State::WP:
        if (_wp_reached()) {
            auto wp_idx = _wp_idx + 1;
            if (wp_idx < _n_wps) {
                _set_wp(wp_idx);
            } else {
                _state = _State::LANDING;
                pos_control->set_pos_target_z_cm(_ground_alt - 10);
                gcs().send_text(MAV_SEVERITY_DEBUG, "Landing");
            }
        }
        break;

    case _State::LANDING:
        if (copter.ap.land_complete) {
            _state = _State::LANDED;
            copter.arming.disarm(AP_Arming::Method::LANDED);
            gcs().send_text(MAV_SEVERITY_DEBUG, "Landed");
            return;
        } else if (fabs(inertial_nav.get_velocity_z_up_cms()) < 50) {
            // TODO better landing detection
            pos_control->relax_velocity_controller_xy();
            pos_control->relax_z_controller(0);
        } else if (_wp_reached()) {
            // Not touching ground, decrease z target
            pos_control->set_pos_target_z_cm(
                pos_control->get_pos_target_z_cm() - 100
            );
        }
        break;

    default:
        break;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Update pos controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // Set target attitude
    attitude_control->input_thrust_vector_heading(
        pos_control->get_thrust_vector(), pos_control->get_yaw_cd()
    );
}

uint32_t ModeSquare::wp_distance() const {
    return pos_control->get_pos_error_xy_cm();
}
int32_t ModeSquare::wp_bearing() const {
    return pos_control->get_bearing_to_target_cd();
}

#endif