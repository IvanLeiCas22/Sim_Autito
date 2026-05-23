#ifndef FIRMWARE_SIM_BRIDGE_H
#define FIRMWARE_SIM_BRIDGE_H

#include <array>
#include <cstdint>

#include <QString>

class FirmwareSimBridge
{
public:
    static constexpr int kIrSensorCount = 6;

    struct SensorSnapshot
    {
        uint32_t dt_ms = 0;

        std::array<double, kIrSensorCount> ir_distance_mm = {};
        bool floor_front_black = false;
        bool floor_rear_black = false;

        double yaw_deg = 0.0;
        double yaw_rate_deg_s = 0.0;
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
    };

    FirmwareSimBridge();

    void reset();
    void start();
    void stop();

    Command tick(const SensorSnapshot &snapshot);
    Debug debug() const;

private:
    void ensureFirmwareCoreInitialized();

    bool enabled_ = false;
    bool firmware_initialized_ = false;
    Debug debug_;
};

#endif // FIRMWARE_SIM_BRIDGE_H
