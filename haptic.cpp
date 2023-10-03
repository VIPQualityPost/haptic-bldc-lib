#include "haptic.h"
#include "util.h"

static const float idle_velocity_ewma_alpha = 0.001;
static const float idle_velocity_rad_per_sec = 0.05;
static const int32_t idle_correction_delay = 500; //ms

PIDController default_pid{
    .P = 1,
    .I = 0,
    .D = 0.0005,
    .output_ramp = 1000,
    .limit = 20
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
    haptic_config->attract_angle = round(motor->shaft_angle / haptic_config->pos_width_rad) * haptic_config->pos_width_rad;
}

float HapticInterface::haptic_target(void)
{
    float error = haptic_config->attract_angle - motor->shaft_angle;
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
    switch (haptic_config->detent_num)
    {
    case 2:
        haptic_config->detent_num = 4;
        break;

    case 4:
        haptic_config->detent_num = 8;
        break;

    case 8:
        haptic_config->detent_num = 12;
        break;

    case 12:
        haptic_config->detent_num = 2;
        break;
    }
    // haptic_click();
}

void HapticInterface::haptic_loop(void){
    if(haptic_config->detent_num != haptic_config->detent_num_prev){
        correct_pid();
    }

    correct_idle();

    if(fabs(motor->shaft_velocity) > 60){
        motor->move(0);
    }
    else{
        motor->loopFOC();
        motor->move(haptic_target());

        find_detent();
    }
}

void HapticInterface::correct_pid(void)
{
    float d_lower_strength = haptic_config->detent_strength_unit * 0.02;
    float d_upper_strength = haptic_config->detent_strength_unit * 0.08;
    float d_lower_pos_width = radians(3);
    float d_upper_pos_width = radians(8);
    float raw = d_lower_strength + (d_upper_strength - d_lower_strength)/(d_upper_pos_width - d_lower_pos_width)*(haptic_config->pos_width_rad - d_lower_pos_width);

    haptic_pid->D = haptic_config->detent_count > 0 ? 0 : CLAMP(
        raw,
        min(d_lower_strength, d_upper_strength),
        max(d_lower_strength, d_upper_strength)
    );

    haptic_config->detent_num_prev = haptic_config->detent_num;
}

void HapticInterface::correct_idle(void)
{
    haptic_params->idle_check_velocity_ewma = motor->shaft_velocity * haptic_params->idle_velocity_ewma_alpha + haptic_params->idle_check_velocity_ewma * (1 - haptic_params->idle_velocity_ewma_alpha);
    if (fabsf(haptic_params->idle_check_velocity_ewma) > haptic_params->idle_velocity_rad_per_sec){
        haptic_params->last_idle_start = 0;
    }
    else{
        if(haptic_params->last_idle_start == 0){
            haptic_params->last_idle_start = millis();
        }
    }
    if(haptic_params->last_idle_start > 0 && millis() - haptic_params->last_idle_start > haptic_params->idle_correction_delay_millis && fabsf(motor->shaft_angle - haptic_config->current_detent_center) < haptic_params->idle_correction_max_angle_rad){
        haptic_config->current_detent_center = motor->shaft_angle * haptic_params->idle_correction_rate_alpha + haptic_config->current_detent_center * (1 - haptic_params->idle_correction_rate_alpha);
    }

    float angle_to_detent_center = motor->shaft_angle - haptic_config->current_detent_center;
    float bias_radians = haptic_config->pos_width_rad * haptic_config->snap_point_bias;
    float snap_point_radians = haptic_config->pos_width_rad * haptic_config->snap_point;
    float snap_point_rad_dec = snap_point_radians + (haptic_config->pos <= 0 ? bias_radians : -bias_radians);
    float snap_point_rad_inc = -snap_point_radians + (haptic_config->pos <= 0 ? -bias_radians : bias_radians);

    if(angle_to_detent_center > snap_point_rad_dec && (haptic_config->detent_num <= 0 || haptic_config->pos > haptic_config->min_pos)){
        haptic_config->current_detent_center += haptic_config->pos_width_rad;
        angle_to_detent_center -= haptic_config->pos_width_rad;
        haptic_config->pos--;
    }else if (angle_to_detent_center < snap_point_radians_increase && (haptic_config->detent_num <= 0 || haptic_config->pos < haptic_config->min_pos)) {
            haptic_config->current_detent_center -= haptic_config->pos_width_rad;
            angle_to_detent_center += haptic_config->pos_width_rad
            haptic_config->pos++;
}
