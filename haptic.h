#ifndef HAPTIC_H
#define HAPIC_H

#include "Arduino.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"

typedef struct
{
    uint8_t detent_num = 2;  // number of points for clicks
    uint8_t detent_num_prev = 0;
    uint8_t detent_count = 0;

    float current_detent_center;
    float attract_angle = 0; // angle where PID will point to

    uint8_t pos = 0;
    uint8_t min_pos = 0; // in terms of position index
    uint8_t max_pos = 1; // in terms of position index
    float pos_width_rad = detent_num/6.28;

    float snap_point = 0.055;
    float snap_point_bias = 0;

    float click_strength = 5;
    float detent_strength_unit = 0;
    float endstop_strength_unit = 0;
} hapticState;

typedef struct
{
    float dead_zone_percent = 0.2;
    float dead_zone_rad = _PI/180;

    float idle_velocity_ewma_alpha = 0.001;
    float idle_velocity_rad_per_sec = 0.05;
    float idle_check_velocity_ewma = 0;
    float idle_correction_max_angle_rad = 5 * _PI/180;
    float idle_correction_rate_alpha = 0.0005;

    float last_idle_start = 0;
    uint32_t idle_correction_delay_millis = 500;

} hapticParms;

class HapticInterface
{
public:    
    hapticState* haptic_config;
    hapticParms* haptic_params;
    BLDCMotor* motor;
    PIDController* haptic_pid;

    // All the various constructors.
    HapticInterface(BLDCMotor* _motor);
    HapticInterface(BLDCMotor* _motor, PIDController* _pid);
    HapticInterface(BLDCMotor* _motor, hapticState* _config);
    HapticInterface(BLDCMotor* _motor, PIDController* _pid, hapticState* _config);
    HapticInterface(BLDCMotor* _motor, PIDController* _pid, hapticState* _config, hapticParms* _parms);


    void init(void);
    void change_haptic_mode(void);
    void haptic_loop(void);
    void haptic_click(void);

protected:
private:
    void find_detent(void);
    float haptic_target(void);
    void correct_pid(void);
    void correct_idle(void);
};

#endif