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
QString controlModeText(FirmwareSimBridge::ControlMode mode)
{
    switch (mode) {
    case FirmwareSimBridge::ControlMode::TelemetryOnly:
        return QStringLiteral("TelemetryOnly");
    case FirmwareSimBridge::ControlMode::StraightYawHold:
        return QStringLiteral("StraightYawHold");
    }

    return QStringLiteral("Unknown");
}

double shortestDeltaDeg(double current_deg, double target_deg)
{
    if (!std::isfinite(current_deg) || !std::isfinite(target_deg)) {
        return 0.0;
    }

    double delta = std::fmod(current_deg - target_deg, 360.0);
    if (delta > 180.0) {
        delta -= 360.0;
    } else if (delta < -180.0) {
        delta += 360.0;
    }

    return delta;
}

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
constexpr uint16_t kSimLeftBasePwm = 3000;
constexpr uint32_t kDecisionRandomValue = 0U;

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

QString recommendedActionText(AppNavRecommendedAction action)
{
    switch (action) {
    case APP_NAV_ACTION_NONE:
        return QStringLiteral("NONE");
    case APP_NAV_ACTION_GO_BACK:
        return QStringLiteral("GO_BACK");
    case APP_NAV_ACTION_GO_FRONT_NAVIGATING:
        return QStringLiteral("GO_FRONT_NAVIGATING");
    case APP_NAV_ACTION_GO_FRONT_STRAIGHT:
        return QStringLiteral("GO_FRONT_STRAIGHT");
    case APP_NAV_ACTION_SMOOTH_LEFT:
        return QStringLiteral("SMOOTH_LEFT");
    case APP_NAV_ACTION_SMOOTH_RIGHT:
        return QStringLiteral("SMOOTH_RIGHT");
    }

    return QStringLiteral("UNKNOWN");
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

    // gx/gy/gz are optional legacy raw IMU channels. The portable yaw-rate
    // measurement consumed by the firmware core is yaw_rate_dps.
    input.gz = 0;
    input.yaw_rate_dps = toFirmwareInt16(snapshot.yaw_rate_deg_s);
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

void FirmwareSimBridge::applySimulationFirmwareConfig(const SensorSnapshot &snapshot)
{
#if SIM_AUTITO_HAS_FIRMWARE_CORE
    const double leftGain = std::isfinite(snapshot.left_motor_gain) && snapshot.left_motor_gain > 0.0
        ? snapshot.left_motor_gain
        : 1.0;
    const double rightGain = std::isfinite(snapshot.right_motor_gain) && snapshot.right_motor_gain > 0.0
        ? snapshot.right_motor_gain
        : 1.0;

    if (simulation_config_applied_
        && std::abs(leftGain - last_left_gain_) < 0.000001
        && std::abs(rightGain - last_right_gain_) < 0.000001) {
        return;
    }

    AppNavConfig config = {};
    App_Nav_GetConfig(&config);

    const double rightBase = std::round(static_cast<double>(kSimLeftBasePwm) * leftGain / rightGain);
    config.left_motor_base_speed = kSimLeftBasePwm;
    config.right_motor_base_speed = static_cast<uint16_t>(std::clamp(rightBase, 0.0, 65535.0));

    App_Nav_SetConfig(&config);

    simulation_config_applied_ = true;
    last_left_gain_ = leftGain;
    last_right_gain_ = rightGain;
    sim_config_left_base_ = config.left_motor_base_speed;
    sim_config_right_base_ = config.right_motor_base_speed;
    debug_.sim_config_left_base = sim_config_left_base_;
    debug_.sim_config_right_base = sim_config_right_base_;
#else
    Q_UNUSED(snapshot);
#endif
}

void FirmwareSimBridge::reset()
{
    ensureFirmwareCoreInitialized();

    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    simulation_config_applied_ = false;
    straight_yaw_target_deg_ = 0.0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    App_Nav_Reset();

    debug_ = Debug{};
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.sim_config_left_base = sim_config_left_base_;
    debug_.sim_config_right_base = sim_config_right_base_;
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
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);

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
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    App_Nav_Stop();
    debug_.state = QStringLiteral("FW: stopped");
    debug_.reason = QStringLiteral("Firmware core stopped");
#else
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Bridge stopped");
#endif
}

void FirmwareSimBridge::startStraightYawHold(double current_yaw_deg)
{
    ensureFirmwareCoreInitialized();

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    straight_yaw_target_deg_ = current_yaw_deg;
    const bool started = App_Nav_StartStraightDriveYawHold(toQ16Deg(straight_yaw_target_deg_));
    enabled_ = started;
    control_mode_ = started ? ControlMode::StraightYawHold : ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.state = started ? QStringLiteral("FW: straight yaw-hold") : QStringLiteral("FW: idle");
    debug_.reason = started
        ? QStringLiteral("Straight yaw-hold primitive started")
        : QStringLiteral("Straight yaw-hold primitive could not start");
#else
    Q_UNUSED(current_yaw_deg);
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = QStringLiteral("TelemetryOnly");
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Straight yaw-hold unsupported without firmware core");
#endif
}

void FirmwareSimBridge::stopControl()
{
    stop();
}

FirmwareSimBridge::Command FirmwareSimBridge::tick(const SensorSnapshot &snapshot)
{
    Command command;
    command.left_pwm = 0;
    command.right_pwm = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    ensureFirmwareCoreInitialized();
    applySimulationFirmwareConfig(snapshot);

    SensorSnapshot firmware_snapshot = snapshot;
    if (control_mode_ == ControlMode::StraightYawHold) {
        firmware_snapshot.yaw_deg = straight_yaw_target_deg_
            + shortestDeltaDeg(snapshot.yaw_deg, straight_yaw_target_deg_);
    }

    const AppNavInput input = buildAppNavInput(firmware_snapshot);
    AppNavOutput output = {};

    App_Nav_Tick(&input, &output);

    AppNavRecommendedAction recommended_action = APP_NAV_ACTION_NONE;
    App_Nav_RecommendAction(kDecisionRandomValue, &recommended_action);

    bool straight_yaw_hold_ok = true;
    if (control_mode_ == ControlMode::StraightYawHold) {
        AppNavOutput primitive_output = {};
        straight_yaw_hold_ok = App_Nav_ComputeStraightDrivePwm(&input, &primitive_output);
        if (straight_yaw_hold_ok) {
            command.left_pwm = primitive_output.left_motor_pwm;
            command.right_pwm = primitive_output.right_motor_pwm;
        }
    }

    AppNavDebug firmware_debug = {};
    App_Nav_GetDebug(&firmware_debug);

    debug_.state = QStringLiteral("FW: mode=%1 state=%2")
        .arg(static_cast<int>(firmware_debug.mode))
        .arg(static_cast<int>(firmware_debug.state));
    debug_.reason = QStringLiteral("last_transition_reason=%1 transition_sequence=%2")
        .arg(static_cast<int>(firmware_debug.last_transition_reason))
        .arg(static_cast<int>(firmware_debug.transition_sequence));
    if (control_mode_ == ControlMode::StraightYawHold && !straight_yaw_hold_ok) {
        debug_.reason += QStringLiteral(" straight_yaw_hold_pwm=false");
    }
    debug_.control_mode = controlModeText(control_mode_);
    debug_.sim_config_left_base = sim_config_left_base_;
    debug_.sim_config_right_base = sim_config_right_base_;
    debug_.recommended_action = static_cast<int>(recommended_action);
    debug_.recommended_action_text = recommendedActionText(recommended_action);
    debug_.available_options_mask = firmware_debug.available_options_mask;
    debug_.valid_option_count = firmware_debug.valid_option_count;
    debug_.decision_random_value = kDecisionRandomValue;
    if (static_cast<int>(firmware_debug.last_recommended_action) != debug_.recommended_action) {
        debug_.reason += QStringLiteral(" last_recommended_action=%1")
            .arg(static_cast<int>(firmware_debug.last_recommended_action));
    }
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
    debug_.control_mode = QStringLiteral("TelemetryOnly");
#endif

    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.left_pwm = command.left_pwm;
    debug_.right_pwm = command.right_pwm;

    return command;
}

FirmwareSimBridge::Debug FirmwareSimBridge::debug() const
{
    return debug_;
}

bool FirmwareSimBridge::isFirmwareControlActive() const
{
    return enabled_ && control_mode_ != ControlMode::TelemetryOnly;
}
