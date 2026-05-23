#include "firmware_sim_bridge.h"

#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <cstdint>

#ifndef SIM_AUTITO_HAS_FIRMWARE_CORE
#define SIM_AUTITO_HAS_FIRMWARE_CORE 0
#endif

#if SIM_AUTITO_HAS_FIRMWARE_CORE
extern "C" {
#include "app_nav_types.h"
#include "app_nav_config.h"
#include "app_nav_debug.h"
#include "app_nav.h"
}
#endif

namespace {
#if SIM_AUTITO_HAS_FIRMWARE_CORE
constexpr int kAdcRightLatCh = 0;
constexpr int kAdcDiagonalRightCh = 1;
constexpr int kAdcFrontRightCh = 2;
constexpr int kAdcFloorFrontCh = 3;
constexpr int kAdcFrontLeftCh = 4;
constexpr int kAdcDiagonalLeftCh = 5;
constexpr int kAdcLeftLatCh = 6;
constexpr int kAdcFloorRearCh = 7;

constexpr uint16_t kFloorWhiteAdc = 4095;
constexpr uint16_t kFloorBlackAdc = 0;

uint16_t toFirmwareDistanceMm(double distance_mm)
{
    if (!std::isfinite(distance_mm) || distance_mm <= 0.0) {
        return 0;
    }

    const double rounded = std::round(distance_mm);
    return static_cast<uint16_t>(std::clamp(rounded, 0.0, 65535.0));
}

int16_t toFirmwareInt16(double value)
{
    if (!std::isfinite(value)) {
        return 0;
    }

    const double rounded = std::round(value);
    return static_cast<int16_t>(std::clamp(rounded, -32768.0, 32767.0));
}

int32_t toQ16Deg(double deg)
{
    if (!std::isfinite(deg)) {
        return 0;
    }

    const double q = std::round(deg * 65536.0);
    return static_cast<int32_t>(std::clamp(q, -2147483648.0, 2147483647.0));
}

AppNavInput buildAppNavInput(const FirmwareSimBridge::SensorSnapshot &snapshot)
{
    AppNavInput input = {};
    input.dt_ms = snapshot.dt_ms;

    input.dist_front_left_mm = toFirmwareDistanceMm(snapshot.ir_distance_mm[0]);
    input.dist_front_right_mm = toFirmwareDistanceMm(snapshot.ir_distance_mm[1]);
    input.dist_left_lat_mm = toFirmwareDistanceMm(snapshot.ir_distance_mm[2]);
    input.dist_right_lat_mm = toFirmwareDistanceMm(snapshot.ir_distance_mm[3]);
    input.dist_diagonal_left_mm = toFirmwareDistanceMm(snapshot.ir_distance_mm[4]);
    input.dist_diagonal_right_mm = toFirmwareDistanceMm(snapshot.ir_distance_mm[5]);

    // Distances are provided directly in mm; adc_filtered IR channels are
    // placeholders until the portable core requires raw ADC.
    input.adc_filtered[kAdcRightLatCh] = 0;
    input.adc_filtered[kAdcDiagonalRightCh] = 0;
    input.adc_filtered[kAdcFrontRightCh] = 0;
    input.adc_filtered[kAdcFrontLeftCh] = 0;
    input.adc_filtered[kAdcDiagonalLeftCh] = 0;
    input.adc_filtered[kAdcLeftLatCh] = 0;

    // The legacy firmware detects floor tape as a low ADC value.
    input.adc_filtered[kAdcFloorFrontCh] = snapshot.floor_front_black ? kFloorBlackAdc : kFloorWhiteAdc;
    input.adc_filtered[kAdcFloorRearCh] = snapshot.floor_rear_black ? kFloorBlackAdc : kFloorWhiteAdc;

    // Temporary: simulated yaw rate in deg/s is stored in gz until the portable
    // API defines an explicit yaw-rate field.
    input.gz = toFirmwareInt16(snapshot.yaw_rate_deg_s);
    input.yaw_q16_deg = toQ16Deg(snapshot.yaw_deg);

    return input;
}
#endif
}

FirmwareSimBridge::FirmwareSimBridge()
{
    ensureFirmwareCoreInitialized();
}

void FirmwareSimBridge::ensureFirmwareCoreInitialized()
{
#if SIM_AUTITO_HAS_FIRMWARE_CORE
    if (firmware_initialized_) {
        return;
    }

    App_Nav_Init(nullptr);
    firmware_initialized_ = true;
    debug_.state = QStringLiteral("FW: initialized");
    debug_.reason = QStringLiteral("Firmware core initialized");
#else
    firmware_initialized_ = true;
#endif
}

void FirmwareSimBridge::reset()
{
    ensureFirmwareCoreInitialized();

    enabled_ = false;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    App_Nav_Reset();

    debug_ = Debug{};
    debug_.enabled = enabled_;
    debug_.state = QStringLiteral("FW: reset");
    debug_.reason = QStringLiteral("Firmware core initialized and reset");
#else
    debug_ = Debug{};
#endif
}

void FirmwareSimBridge::start()
{
    ensureFirmwareCoreInitialized();

    enabled_ = true;
    debug_.enabled = enabled_;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    App_Nav_StartFindCells();
    debug_.state = QStringLiteral("FW: running");
    debug_.reason = QStringLiteral("Firmware core find-cells mode started");
#else
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Bridge enabled, firmware core not connected");
#endif
}

void FirmwareSimBridge::stop()
{
    ensureFirmwareCoreInitialized();

    enabled_ = false;
    debug_.enabled = enabled_;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    App_Nav_Stop();
    debug_.state = QStringLiteral("FW: stopped");
    debug_.reason = QStringLiteral("Firmware core stopped");
#else
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Bridge stopped");
#endif
}

FirmwareSimBridge::Command FirmwareSimBridge::tick(const SensorSnapshot &snapshot)
{
    Command command;
    command.left_pwm = 0;
    command.right_pwm = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    ensureFirmwareCoreInitialized();

    const AppNavInput input = buildAppNavInput(snapshot);
    AppNavOutput output = {};

    App_Nav_Tick(&input, &output);

    if (enabled_) {
        command.left_pwm = output.left_motor_pwm;
        command.right_pwm = output.right_motor_pwm;
    }

    AppNavDebug firmware_debug = {};
    App_Nav_GetDebug(&firmware_debug);

    debug_.state = QStringLiteral("FW: mode=%1 state=%2")
        .arg(static_cast<int>(firmware_debug.mode))
        .arg(static_cast<int>(firmware_debug.state));
    debug_.reason = QStringLiteral("last_transition_reason=%1 transition_sequence=%2")
        .arg(static_cast<int>(firmware_debug.last_transition_reason))
        .arg(static_cast<int>(firmware_debug.transition_sequence));
    debug_.floor_front_black = firmware_debug.floor_front_black != 0U;
    debug_.floor_rear_black = firmware_debug.floor_rear_black != 0U;
    debug_.wall_front = firmware_debug.wall_front != 0U;
    debug_.wall_left = firmware_debug.wall_left != 0U;
    debug_.wall_right = firmware_debug.wall_right != 0U;
    debug_.wall_diag_left = firmware_debug.wall_diag_left != 0U;
    debug_.wall_diag_right = firmware_debug.wall_diag_right != 0U;
    debug_.dist_front_left_mm = firmware_debug.dist_front_left_mm;
    debug_.dist_front_right_mm = firmware_debug.dist_front_right_mm;
    debug_.dist_left_lat_mm = firmware_debug.dist_left_lat_mm;
    debug_.dist_right_lat_mm = firmware_debug.dist_right_lat_mm;
    debug_.dist_diagonal_left_mm = firmware_debug.dist_diagonal_left_mm;
    debug_.dist_diagonal_right_mm = firmware_debug.dist_diagonal_right_mm;
    debug_.adc_floor_front = firmware_debug.floor_front_adc;
    debug_.adc_floor_rear = firmware_debug.floor_rear_adc;
#else
    Q_UNUSED(snapshot);

    debug_.enabled = enabled_;
    debug_.state = QStringLiteral("STUB");
    debug_.reason = enabled_
        ? QStringLiteral("FirmwareSimBridge tick executed without firmware core")
        : QStringLiteral("FirmwareSimBridge disabled");
#endif

    debug_.enabled = enabled_;
    debug_.left_pwm = command.left_pwm;
    debug_.right_pwm = command.right_pwm;

    return command;
}

FirmwareSimBridge::Debug FirmwareSimBridge::debug() const
{
    return debug_;
}
