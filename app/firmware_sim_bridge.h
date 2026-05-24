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
        StraightYawHold
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
        uint16_t sim_config_left_base = 0;
        uint16_t sim_config_right_base = 0;

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

    FirmwareSimBridge();

    void reset();
    void start();
    void stop();
    void startStraightYawHold(double current_yaw_deg);
    void stopControl();

    Command tick(const SensorSnapshot &snapshot);
    Debug debug() const;
    bool isFirmwareControlActive() const;

private:
    void ensureFirmwareCoreInitialized();
    void applySimulationFirmwareConfig(const SensorSnapshot &snapshot);

    bool enabled_ = false;
    bool firmware_initialized_ = false;
    ControlMode control_mode_ = ControlMode::TelemetryOnly;
    bool simulation_config_applied_ = false;
    double last_left_gain_ = 0.0;
    double last_right_gain_ = 0.0;
    uint16_t sim_config_left_base_ = 0;
    uint16_t sim_config_right_base_ = 0;
    double straight_yaw_target_deg_ = 0.0;
    Debug debug_;
};

#endif // FIRMWARE_SIM_BRIDGE_H
