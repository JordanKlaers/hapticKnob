#include "haptic.h"
#include "util.h"

static const float idle_velocity_ewma_alpha = 0.001;
static const float idle_velocity_rad_per_sec = 0.05;
static const int32_t idle_correction_delay = 500; //ms

PIDController default_pid{
    .P = 4,
    .I = 0,
    .D = 0.005,
    .output_ramp = 10000,
    .limit = 1.4
};

hapticState default_config;
hapticParms default_params;

HapticInterface::HapticInterface(BLDCMotor* _motor){
    motor = _motor;
    haptic_pid = &default_pid;
    haptic_config = &default_config;
    haptic_params = &default_params;
}

HapticInterface::HapticInterface(BLDCMotor* _motor, hapticState* _config){
    motor = _motor;
    haptic_pid = &default_pid;
    haptic_config = _config;
    haptic_params = &default_params;
}

HapticInterface::HapticInterface(BLDCMotor* _motor, PIDController* _pid){
    motor = _motor;
    haptic_pid = _pid;
    haptic_config = &default_config;
    haptic_params = &default_params;
}

HapticInterface::HapticInterface(BLDCMotor* _motor, PIDController* _pid, hapticState* _config){
    motor = _motor;
    haptic_pid = _pid;
    haptic_config = _config;
    haptic_params = &default_params;
}

HapticInterface::HapticInterface(BLDCMotor* _motor, PIDController* _pid, hapticState* _config, hapticParms* _params){
    motor = _motor;
    haptic_pid = _pid;
    haptic_config = _config;
    haptic_params = _params;
}

void HapticInterface::init(void){
    motor->velocity_limit = 10000;
    motor->controller = MotionControlType::torque;
    motor->foc_modulation = FOCModulationType::SpaceVectorPWM;
}

void HapticInterface::find_detent(void)
{
    //attract angle  seems to be whatever value is closest to the increment 0.08726646259 between 0 and 2 * pi (a value in radians)
    haptic_config->attract_angle = round(motor->shaft_angle / haptic_config->distance_pos) * haptic_config->distance_pos;
}

float HapticInterface::haptic_target(void)
{
    // TODO: When out of bounds and return to position introduce easing so we avoid overshoot.
    float error = haptic_config->last_attract_angle - motor->shaft_angle;
    
    // Call FOC loop
    motor->loopFOC();

    // Target Command
    if(fabsf(motor->shaft_velocity) > 60) {
        motor->move(0);
    // If velocity (turn speed is greather than x then dont apply any torque
    } else {
        motor->move(default_pid(error));
    }
    return haptic_pid->operator()(error);
}

void HapticInterface::haptic_click(void){
    float click_strength = haptic_config->click_strength;
    motor->move(click_strength);
    for(uint8_t i = 0; i< 3; i++){
        motor->loopFOC();
        delay(1);
    }
    motor->move(-click_strength);
    for(uint8_t i=0; i<3; i++){
        motor->loopFOC();
        delay(1);
    }
    motor->move(0);
}

void HapticInterface::change_haptic_mode(void)
{
    switch (haptic_config->total_pos)
    {
    case 2:
        haptic_config->total_pos = 4;
        break;

    case 4:
        haptic_config->total_pos = 8;
        break;

    case 8:
        haptic_config->total_pos = 12;
        break;

    case 12:
        haptic_config->total_pos = 2;
        break;
    }
    // haptic_click();
}

void HapticInterface::haptic_loop(void){
    
    // TODO: Include Check if Config is Correct before triggering loop

    while (1){
    correct_pid(); // Adjust PID (Derivative Gain)
    find_detent(); // Calculate attraction angle depending on configured distance position.
    state_update(); // Determine and update current position    
    haptic_target(); // PID Command
    }
}

void HapticInterface::correct_pid(void)
{
    bool bound;
    float d_lower_strength = haptic_config->detent_strength_unit * 0.008;
    float d_upper_strength = haptic_config->detent_strength_unit * 0.004;
    float d_lower_pos_width = radians(3);
    float d_upper_pos_width = radians(8);
    float raw = d_lower_strength + (d_upper_strength - d_lower_strength)/(d_upper_pos_width - d_lower_pos_width)*(haptic_config->distance_pos - d_lower_pos_width);

    haptic_pid->D = haptic_config->detent_count > 0 ? 0 : CLAMP(
        raw,
        min(d_lower_strength, d_upper_strength),
        max(d_lower_strength, d_upper_strength)
    );

    // Check if within range and apply voltage/current limit.
    if (haptic_config->attract_angle <= haptic_config->last_attract_angle - haptic_config->distance_pos || haptic_config->attract_angle >= haptic_config->last_attract_angle + haptic_config->distance_pos){
        bound = true;
    } else {
        bound = false;
    }
    haptic_pid->limit = bound ? haptic_config->endstop_strength_unit : haptic_config->detent_strength_unit;
}

void HapticInterface::state_update(void)
{
        // Determine and Update Current Position
    // Check if attractor angle is within calculated position between min and max position and update current position if in range.
    /*
        if the attract angle is 180 degress, its bigger than 1(current_pos) * 0.08726646259 (it would be) and the current pos is less than 255
    */
    if(haptic_config->attract_angle > haptic_config->current_pos * haptic_config->distance_pos && haptic_config->current_pos < haptic_config->end_pos){
        //add one to current pos (not sure why)
        haptic_config->current_pos++;
        //not sure what the point of the last attract angle is either
        haptic_config->last_attract_angle = haptic_config->attract_angle;
        Serial.print("[POS]: ");
        Serial.println(haptic_config->current_pos); 
    } else if (haptic_config->attract_angle < haptic_config->current_pos * haptic_config->distance_pos && haptic_config->current_pos > haptic_config->start_pos){
        haptic_config->current_pos--;
        haptic_config->last_attract_angle = haptic_config->attract_angle;
        Serial.print("[POS]: ");
        Serial.println(haptic_config->current_pos);
    }

}