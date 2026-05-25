#ifndef FIRMWARE_SIM_BRIDGE_H
#define FIRMWARE_SIM_BRIDGE_H

#include <array>
#include <cstdint>

#include <QString>

class FirmwareSimBridge
{
public:
    static constexpr int kIrSensorCount = 6;

    enum class ControlMode
    {
        TelemetryOnly,
        StraightYawHold,
        WallFollowAdvance,
        SmoothTurnLeft,
        SmoothTurnRight,
        PivotLeft90,
        PivotRight90,
        Pivot180
    };

    struct SensorSnapshot
    {
        uint32_t dt_ms = 0;

        std::array<double, kIrSensorCount> ir_distance_mm = {};
        bool floor_front_black = false;
        bool floor_rear_black = false;

        double yaw_deg = 0.0;
        double yaw_rate_deg_s = 0.0;

        double left_motor_gain = 1.0;
        double right_motor_gain = 1.0;
    };

    struct Command
    {
        int left_pwm = 0;
        int right_pwm = 0;
    };

    struct Debug
    {
        QString state = QStringLiteral("STUB");
        QString reason = QStringLiteral("FirmwareSimBridge not connected");
        bool enabled = false;
        int left_pwm = 0;
        int right_pwm = 0;
        QString control_mode = QStringLiteral("TelemetryOnly");
        QString advance_state = QStringLiteral("n/a");
        QString pivot_state = QStringLiteral("n/a");
        QString smooth_state = QStringLiteral("n/a");
        uint16_t sim_config_left_base = 0;
        uint16_t sim_config_right_base = 0;
        uint8_t maze_x = 0;
        uint8_t maze_y = 0;
        uint8_t maze_heading = 0;
        uint8_t maze_cell = 0;
        bool maze_valid = false;

        bool floor_front_black = false;
        bool floor_rear_black = false;
        bool wall_front = false;
        bool wall_left = false;
        bool wall_right = false;
        bool wall_diag_left = false;
        bool wall_diag_right = false;

        uint16_t dist_front_left_mm = 0;
        uint16_t dist_front_right_mm = 0;
        uint16_t dist_left_lat_mm = 0;
        uint16_t dist_right_lat_mm = 0;
        uint16_t dist_diagonal_left_mm = 0;
        uint16_t dist_diagonal_right_mm = 0;
        uint16_t adc_floor_front = 0;
        uint16_t adc_floor_rear = 0;

        int recommended_action = 0;
        QString recommended_action_text = QStringLiteral("NONE/STUB");
        uint8_t available_options_mask = 0;
        uint8_t valid_option_count = 0;
        uint32_t decision_random_value = 0;
    };

    struct FirmwareConfig
    {
        uint16_t right_motor_base_speed = 0;
        uint16_t left_motor_base_speed = 0;
        uint16_t faster_motor_smooth_turn_speed = 0;
        uint16_t slower_motor_smooth_turn_speed = 0;
        uint16_t turn_target_dps = 0;
        uint16_t pivot_turn_target_dps = 0;
        uint16_t wall_target_mm = 0;
        uint16_t wall_threshold_mm_front = 0;
        uint16_t wall_threshold_mm_side = 0;
        uint16_t wall_threshold_mm_diagonal = 0;
        uint16_t wall_hysteresis_mm = 0;

        int32_t advance_pid_kp_q16 = 0;
        int32_t advance_pid_ki_q16 = 0;
        int32_t advance_pid_kd_q16 = 0;
        int32_t advance_pid_output_limit_pwm = 0;

        int32_t smooth_turn_pid_kp_q16 = 0;
        int32_t smooth_turn_pid_ki_q16 = 0;
        int32_t smooth_turn_pid_kd_q16 = 0;
        int32_t smooth_turn_pid_output_limit_pwm = 0;

        int32_t pivot_turn_pid_kp_q16 = 0;
        int32_t pivot_turn_pid_ki_q16 = 0;
        int32_t pivot_turn_pid_kd_q16 = 0;
        int32_t pivot_turn_pid_output_limit_pwm = 0;

        int32_t braking_pid_kp_q16 = 0;
        int32_t braking_pid_ki_q16 = 0;
        int32_t braking_pid_kd_q16 = 0;
        int32_t braking_pid_output_limit_pwm = 0;
        int16_t braking_min_speed_pwm = 0;
    };

    FirmwareSimBridge();

    void reset();
    void start();
    void stop();
    void startStraightYawHold(double current_yaw_deg);
    void startWallFollowAdvance();
    void startSmoothTurnLeft();
    void startSmoothTurnRight();
    void startPivotLeft90();
    void startPivotRight90();
    void startPivot180();
    void stopControl();

    Command tick(const SensorSnapshot &snapshot);
    Debug debug() const;
    bool isFirmwareControlActive() const;
    bool getFirmwareConfig(FirmwareConfig *out) const;
    bool setFirmwareConfig(const FirmwareConfig &config);
    bool resetFirmwareConfigToSimulationDefaults();

private:
    void ensureFirmwareCoreInitialized();
    void applySimulationFirmwareConfig(const SensorSnapshot &snapshot);
    void updateMazeDebug();

    bool enabled_ = false;
    bool firmware_initialized_ = false;
    ControlMode control_mode_ = ControlMode::TelemetryOnly;
    bool simulation_config_applied_ = false;
    double last_left_gain_ = 0.0;
    double last_right_gain_ = 0.0;
    uint16_t sim_config_left_base_ = 0;
    uint16_t sim_config_right_base_ = 0;
    double straight_yaw_target_deg_ = 0.0;
    bool advance_yaw_reference_valid_ = false;
    double advance_yaw_start_deg_ = 0.0;
    bool smooth_yaw_reference_valid_ = false;
    double smooth_yaw_start_deg_ = 0.0;
    bool pivot_yaw_reference_valid_ = false;
    double pivot_yaw_start_deg_ = 0.0;
    Debug debug_;
};

#endif // FIRMWARE_SIM_BRIDGE_H
