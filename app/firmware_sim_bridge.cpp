#include "firmware_sim_bridge.h"

#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <cstdint>

#ifndef SIM_AUTITO_HAS_FIRMWARE_CORE
#define SIM_AUTITO_HAS_FIRMWARE_CORE 0
#endif

#ifndef SIM_AUTITO_HAS_NAV_SUPERVISOR
#define SIM_AUTITO_HAS_NAV_SUPERVISOR 0
#endif

#if SIM_AUTITO_HAS_FIRMWARE_CORE
extern "C" {
#include "app_nav_types.h"
#include "app_nav_config.h"
#include "app_nav_debug.h"
#include "app_nav.h"
#include "app_maze.h"
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
#include "app_nav_supervisor.h"
#endif
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
    case FirmwareSimBridge::ControlMode::WallFollowAdvance:
        return QStringLiteral("WallFollowAdvance");
    case FirmwareSimBridge::ControlMode::SmoothTurnLeft:
        return QStringLiteral("SmoothTurnLeft");
    case FirmwareSimBridge::ControlMode::SmoothTurnRight:
        return QStringLiteral("SmoothTurnRight");
    case FirmwareSimBridge::ControlMode::PivotLeft90:
        return QStringLiteral("PivotLeft90");
    case FirmwareSimBridge::ControlMode::PivotRight90:
        return QStringLiteral("PivotRight90");
    case FirmwareSimBridge::ControlMode::Pivot180:
        return QStringLiteral("Pivot180");
    case FirmwareSimBridge::ControlMode::SupervisorV1:
        return QStringLiteral("SupervisorV1");
    }

    return QStringLiteral("Unknown");
}

bool isPivotControlMode(FirmwareSimBridge::ControlMode mode)
{
    return mode == FirmwareSimBridge::ControlMode::PivotLeft90
        || mode == FirmwareSimBridge::ControlMode::PivotRight90
        || mode == FirmwareSimBridge::ControlMode::Pivot180;
}

bool isSmoothControlMode(FirmwareSimBridge::ControlMode mode)
{
    return mode == FirmwareSimBridge::ControlMode::SmoothTurnLeft
        || mode == FirmwareSimBridge::ControlMode::SmoothTurnRight;
}

bool isAdvanceControlMode(FirmwareSimBridge::ControlMode mode)
{
    return mode == FirmwareSimBridge::ControlMode::WallFollowAdvance;
}

bool isSupervisorControlMode(FirmwareSimBridge::ControlMode mode)
{
    return mode == FirmwareSimBridge::ControlMode::SupervisorV1;
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
constexpr int kSimAdvancePidKpX100 = 700;
constexpr int kSimAdvancePidKdX100 = 380;
constexpr int32_t kSimAdvancePidOutputLimitPwm = 2000;
constexpr int kSimSmoothTurnPidKpX100 = 900;
constexpr int kSimSmoothTurnPidKiX100 = 11000;
constexpr int kSimSmoothTurnPidKdX100 = 0;
constexpr int32_t kSimSmoothTurnPidOutputLimitPwm = 20000;
constexpr int kSimPivotTurnPidKpX100 = 900;
constexpr int kSimPivotTurnPidKiX100 = 11000;
constexpr int kSimPivotTurnPidKdX100 = 0;
constexpr int32_t kSimPivotTurnPidOutputLimitPwm = 20000;
constexpr uint16_t kSimFasterMotorSmoothTurnSpeed = 3000;
constexpr uint16_t kSimSlowerMotorSmoothTurnSpeed = 3000;
constexpr uint16_t kSimTurnTargetDps = 120;
constexpr uint16_t kSimWallTargetMm = 62;
constexpr uint16_t kSimWallThresholdSideMm = 135;
constexpr uint16_t kSimWallThresholdDiagonalMm = 145;
constexpr uint16_t kSimWallThresholdFrontMm = 140;
constexpr uint32_t kDecisionRandomValue = 0U;

double effectiveMotorGain(double gain)
{
    return std::isfinite(gain) && gain > 0.0 ? gain : 1.0;
}

uint16_t simulationRightBasePwm(double left_gain, double right_gain)
{
    const double rightBase = std::round(
        static_cast<double>(kSimLeftBasePwm)
        * effectiveMotorGain(left_gain)
        / effectiveMotorGain(right_gain));
    return static_cast<uint16_t>(std::clamp(rightBase, 0.0, 65535.0));
}

int32_t hundredthsToQ16(int value_x100)
{
    const double q16 = std::round(static_cast<double>(value_x100) * 65536.0 / 100.0);
    return static_cast<int32_t>(std::clamp(q16, -2147483648.0, 2147483647.0));
}

void applySimulationBasePwmConfig(AppNavConfig *config, double left_gain, double right_gain)
{
    if (config == nullptr) {
        return;
    }

    config->left_motor_base_speed = kSimLeftBasePwm;
    config->right_motor_base_speed = simulationRightBasePwm(left_gain, right_gain);
}

void applySimulationFirmwareDefaults(AppNavConfig *config, double left_gain, double right_gain)
{
    if (config == nullptr) {
        return;
    }

    applySimulationBasePwmConfig(config, left_gain, right_gain);

    config->advance_pid_kp_q16 = hundredthsToQ16(kSimAdvancePidKpX100);
    if (config->advance_pid_ki_q16 == 0) {
        config->advance_pid_ki_q16 = hundredthsToQ16(0);
    }
    config->advance_pid_kd_q16 = hundredthsToQ16(kSimAdvancePidKdX100);
    config->advance_pid_output_limit_pwm = kSimAdvancePidOutputLimitPwm;

    config->smooth_turn_pid_kp_q16 = hundredthsToQ16(kSimSmoothTurnPidKpX100);
    config->smooth_turn_pid_ki_q16 = hundredthsToQ16(kSimSmoothTurnPidKiX100);
    config->smooth_turn_pid_kd_q16 = hundredthsToQ16(kSimSmoothTurnPidKdX100);
    config->smooth_turn_pid_output_limit_pwm = kSimSmoothTurnPidOutputLimitPwm;
    config->faster_motor_smooth_turn_speed = kSimFasterMotorSmoothTurnSpeed;
    config->slower_motor_smooth_turn_speed = kSimSlowerMotorSmoothTurnSpeed;
    config->turn_target_dps = kSimTurnTargetDps;

    config->pivot_turn_pid_kp_q16 = hundredthsToQ16(kSimPivotTurnPidKpX100);
    config->pivot_turn_pid_ki_q16 = hundredthsToQ16(kSimPivotTurnPidKiX100);
    config->pivot_turn_pid_kd_q16 = hundredthsToQ16(kSimPivotTurnPidKdX100);
    config->pivot_turn_pid_output_limit_pwm = kSimPivotTurnPidOutputLimitPwm;

    config->wall_target_mm = kSimWallTargetMm;
    config->wall_threshold_mm_side = kSimWallThresholdSideMm;
    config->wall_threshold_mm_diagonal = kSimWallThresholdDiagonalMm;
    config->wall_threshold_mm_front = kSimWallThresholdFrontMm;
}

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

QString advanceActionStateText(AppNavAdvanceActionState state)
{
    switch (state) {
    case APP_NAV_ADVANCE_ACTION_IDLE:
        return QStringLiteral("idle");
    case APP_NAV_ADVANCE_ACTION_WAIT_LEAVE_REAR_TAPE:
        return QStringLiteral("wait_leave_rear_tape");
    case APP_NAV_ADVANCE_ACTION_RUNNING_WALL_FOLLOW:
        return QStringLiteral("wall_follow");
    case APP_NAV_ADVANCE_ACTION_RUNNING_YAW_HOLD:
        return QStringLiteral("yaw_hold");
    case APP_NAV_ADVANCE_ACTION_DONE_REAR_TAPE:
        return QStringLiteral("done_rear_tape");
    case APP_NAV_ADVANCE_ACTION_TIMEOUT:
        return QStringLiteral("timeout");
    case APP_NAV_ADVANCE_ACTION_ERROR:
        return QStringLiteral("error");
    }

    return QStringLiteral("unknown");
}

QString smoothActionStateText(AppNavSmoothActionState state)
{
    switch (state) {
    case APP_NAV_SMOOTH_ACTION_IDLE:
        return QStringLiteral("idle");
    case APP_NAV_SMOOTH_ACTION_TURNING:
        return QStringLiteral("turning");
    case APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE:
        return QStringLiteral("post_yaw");
    case APP_NAV_SMOOTH_ACTION_DONE_REAR_TAPE:
        return QStringLiteral("done_rear_tape");
    case APP_NAV_SMOOTH_ACTION_DONE_WALL:
        return QStringLiteral("done_wall");
    case APP_NAV_SMOOTH_ACTION_DONE_POST_YAW_REAR_TAPE:
        return QStringLiteral("done_post_yaw_rear_tape");
    case APP_NAV_SMOOTH_ACTION_FRONT_WALL_SAFETY:
        return QStringLiteral("front_wall_safety");
    case APP_NAV_SMOOTH_ACTION_POST_YAW_TIMEOUT:
        return QStringLiteral("post_yaw_timeout");
    case APP_NAV_SMOOTH_ACTION_ERROR:
        return QStringLiteral("error");
    }

    return QStringLiteral("unknown");
}

QString pivotActionStateText(AppNavPivotActionState state)
{
    switch (state) {
    case APP_NAV_PIVOT_ACTION_IDLE:
        return QStringLiteral("idle");
    case APP_NAV_PIVOT_ACTION_RUNNING:
        return QStringLiteral("running");
    case APP_NAV_PIVOT_ACTION_DONE:
        return QStringLiteral("done");
    case APP_NAV_PIVOT_ACTION_TIMEOUT:
        return QStringLiteral("timeout");
    case APP_NAV_PIVOT_ACTION_ERROR:
        return QStringLiteral("error");
    }

    return QStringLiteral("unknown");
}

#if SIM_AUTITO_HAS_NAV_SUPERVISOR
QString supervisorStateText(AppNavSupervisorState state)
{
    switch (state) {
    case APP_NAV_SUPERVISOR_IDLE:
        return QStringLiteral("idle");
    case APP_NAV_SUPERVISOR_DECIDE:
        return QStringLiteral("decide");
    case APP_NAV_SUPERVISOR_RUN_ADVANCE:
        return QStringLiteral("run_advance");
    case APP_NAV_SUPERVISOR_RUN_APPROACH_FRONT_WALL_FOR_PIVOT:
        return QStringLiteral("run_approach_front_wall_for_pivot");
    case APP_NAV_SUPERVISOR_RUN_SMOOTH_LEFT:
        return QStringLiteral("run_smooth_left");
    case APP_NAV_SUPERVISOR_RUN_SMOOTH_RIGHT:
        return QStringLiteral("run_smooth_right");
    case APP_NAV_SUPERVISOR_RUN_PIVOT_180:
        return QStringLiteral("run_pivot_180");
    case APP_NAV_SUPERVISOR_ERROR:
        return QStringLiteral("error");
    }

    return QStringLiteral("unknown");
}

QString supervisorActionText(AppNavSupervisorAction action)
{
    switch (action) {
    case APP_NAV_SUPERVISOR_ACTION_NONE:
        return QStringLiteral("none");
    case APP_NAV_SUPERVISOR_ACTION_ADVANCE:
        return QStringLiteral("advance");
    case APP_NAV_SUPERVISOR_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT:
        return QStringLiteral("approach_front_wall_for_pivot");
    case APP_NAV_SUPERVISOR_ACTION_SMOOTH_LEFT:
        return QStringLiteral("smooth_left");
    case APP_NAV_SUPERVISOR_ACTION_SMOOTH_RIGHT:
        return QStringLiteral("smooth_right");
    case APP_NAV_SUPERVISOR_ACTION_PIVOT_180:
        return QStringLiteral("pivot_180");
    }

    return QStringLiteral("unknown");
}
#endif

FirmwareSimBridge::FirmwareConfig toBridgeConfig(const AppNavConfig &config)
{
    FirmwareSimBridge::FirmwareConfig out;
    out.right_motor_base_speed = config.right_motor_base_speed;
    out.left_motor_base_speed = config.left_motor_base_speed;
    out.faster_motor_smooth_turn_speed = config.faster_motor_smooth_turn_speed;
    out.slower_motor_smooth_turn_speed = config.slower_motor_smooth_turn_speed;
    out.turn_target_dps = config.turn_target_dps;
    out.pivot_turn_target_dps = config.pivot_turn_target_dps;
    out.wall_target_mm = config.wall_target_mm;
    out.approach_front_wall_target_mm = config.approach_front_wall_target_mm;
    out.wall_threshold_mm_front = config.wall_threshold_mm_front;
    out.wall_threshold_mm_side = config.wall_threshold_mm_side;
    out.wall_threshold_mm_diagonal = config.wall_threshold_mm_diagonal;
    out.wall_hysteresis_mm = config.wall_hysteresis_mm;

    out.advance_pid_kp_q16 = config.advance_pid_kp_q16;
    out.advance_pid_ki_q16 = config.advance_pid_ki_q16;
    out.advance_pid_kd_q16 = config.advance_pid_kd_q16;
    out.advance_pid_output_limit_pwm = config.advance_pid_output_limit_pwm;

    out.smooth_turn_pid_kp_q16 = config.smooth_turn_pid_kp_q16;
    out.smooth_turn_pid_ki_q16 = config.smooth_turn_pid_ki_q16;
    out.smooth_turn_pid_kd_q16 = config.smooth_turn_pid_kd_q16;
    out.smooth_turn_pid_output_limit_pwm = config.smooth_turn_pid_output_limit_pwm;

    out.pivot_turn_pid_kp_q16 = config.pivot_turn_pid_kp_q16;
    out.pivot_turn_pid_ki_q16 = config.pivot_turn_pid_ki_q16;
    out.pivot_turn_pid_kd_q16 = config.pivot_turn_pid_kd_q16;
    out.pivot_turn_pid_output_limit_pwm = config.pivot_turn_pid_output_limit_pwm;

    out.braking_pid_kp_q16 = config.braking_pid_kp_q16;
    out.braking_pid_ki_q16 = config.braking_pid_ki_q16;
    out.braking_pid_kd_q16 = config.braking_pid_kd_q16;
    out.braking_pid_output_limit_pwm = config.braking_pid_output_limit_pwm;
    out.braking_min_speed_pwm = config.braking_min_speed_pwm;
    return out;
}

void copyEditableConfigToFirmware(const FirmwareSimBridge::FirmwareConfig &input,
                                  AppNavConfig *config)
{
    if (config == nullptr) {
        return;
    }

    config->right_motor_base_speed = input.right_motor_base_speed;
    config->left_motor_base_speed = input.left_motor_base_speed;
    config->faster_motor_smooth_turn_speed = input.faster_motor_smooth_turn_speed;
    config->slower_motor_smooth_turn_speed = input.slower_motor_smooth_turn_speed;
    config->turn_target_dps = input.turn_target_dps;
    config->pivot_turn_target_dps = input.pivot_turn_target_dps;
    config->wall_target_mm = input.wall_target_mm;
    config->approach_front_wall_target_mm = input.approach_front_wall_target_mm;
    config->wall_threshold_mm_front = input.wall_threshold_mm_front;
    config->wall_threshold_mm_side = input.wall_threshold_mm_side;
    config->wall_threshold_mm_diagonal = input.wall_threshold_mm_diagonal;
    config->wall_hysteresis_mm = input.wall_hysteresis_mm;

    config->advance_pid_kp_q16 = input.advance_pid_kp_q16;
    config->advance_pid_ki_q16 = input.advance_pid_ki_q16;
    config->advance_pid_kd_q16 = input.advance_pid_kd_q16;
    config->advance_pid_output_limit_pwm = input.advance_pid_output_limit_pwm;

    config->smooth_turn_pid_kp_q16 = input.smooth_turn_pid_kp_q16;
    config->smooth_turn_pid_ki_q16 = input.smooth_turn_pid_ki_q16;
    config->smooth_turn_pid_kd_q16 = input.smooth_turn_pid_kd_q16;
    config->smooth_turn_pid_output_limit_pwm = input.smooth_turn_pid_output_limit_pwm;

    config->pivot_turn_pid_kp_q16 = input.pivot_turn_pid_kp_q16;
    config->pivot_turn_pid_ki_q16 = input.pivot_turn_pid_ki_q16;
    config->pivot_turn_pid_kd_q16 = input.pivot_turn_pid_kd_q16;
    config->pivot_turn_pid_output_limit_pwm = input.pivot_turn_pid_output_limit_pwm;

    config->braking_pid_kp_q16 = input.braking_pid_kp_q16;
    config->braking_pid_ki_q16 = input.braking_pid_ki_q16;
    config->braking_pid_kd_q16 = input.braking_pid_kd_q16;
    config->braking_pid_output_limit_pwm = input.braking_pid_output_limit_pwm;
    config->braking_min_speed_pwm = input.braking_min_speed_pwm;
}

AppNavInput buildAppNavInput(const FirmwareSimBridge::SensorSnapshot &snapshot)
{
    AppNavInput input = {};
    input.dt_ms = snapshot.dt_ms;
    input.floor_front_black = snapshot.floor_front_black ? 1U : 0U;
    input.floor_rear_black = snapshot.floor_rear_black ? 1U : 0U;

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
    // The simulator yaw-rate sign is opposite to the firmware convention;
    // firmware expects smooth-left/pivot-left as positive yaw rate.
    input.yaw_rate_dps = -toFirmwareInt16(snapshot.yaw_rate_deg_s);
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
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    App_NavSupervisor_Init();
#endif
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
    const double leftGain = effectiveMotorGain(snapshot.left_motor_gain);
    const double rightGain = effectiveMotorGain(snapshot.right_motor_gain);

    if (simulation_config_applied_
        && std::abs(leftGain - last_left_gain_) < 0.000001
        && std::abs(rightGain - last_right_gain_) < 0.000001) {
        return;
    }

    const bool firstApplication = !simulation_config_applied_;

    AppNavConfig config = {};
    App_Nav_GetConfig(&config);
    if (firstApplication) {
        applySimulationFirmwareDefaults(&config, leftGain, rightGain);
    } else {
        applySimulationBasePwmConfig(&config, leftGain, rightGain);
    }

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

void FirmwareSimBridge::updateMazeDebug()
{
#if SIM_AUTITO_HAS_FIRMWARE_CORE
    uint8_t buffer[APP_MAZE_CELL_UPDATE_PAYLOAD_SIZE] = {};
    const uint8_t payloadSize = App_Maze_WriteCurrentCellUpdatePayload(buffer);
    if (payloadSize == APP_MAZE_CELL_UPDATE_PAYLOAD_SIZE) {
        debug_.maze_x = buffer[0];
        debug_.maze_y = buffer[1];
        debug_.maze_cell = buffer[2];
        debug_.maze_heading = buffer[3];
        debug_.maze_valid = true;
    } else {
        debug_.maze_valid = false;
    }
#endif
}

void FirmwareSimBridge::updateFirmwareMazeMapDebug()
{
#if SIM_AUTITO_HAS_FIRMWARE_CORE
    debug_.fw_maze_map_valid = false;
    debug_.fw_maze_cells = FirmwareMazeCells{};

    if (MAZE_WIDTH > kFirmwareMazeWidth || MAZE_HEIGHT > kFirmwareMazeHeight) {
        return;
    }

    FirmwareMazeCells cells = {};
    uint8_t currentX = 0;
    uint8_t currentY = 0;
    uint8_t heading = 0;
    bool havePosition = false;

    for (uint8_t col = 0; col < MAZE_WIDTH; ++col) {
        uint8_t buffer[APP_MAZE_COLUMN_SYNC_PAYLOAD_SIZE] = {};
        const uint8_t payloadSize = App_Maze_WriteColumnSyncPayload(col, buffer);
        if (payloadSize != APP_MAZE_COLUMN_SYNC_PAYLOAD_SIZE || buffer[0] != col) {
            return;
        }

        for (uint8_t row = 0; row < MAZE_HEIGHT; ++row) {
            cells[col][row] = buffer[1U + row];
        }

        const uint8_t positionOffset = 1U + MAZE_HEIGHT;
        const uint8_t columnCurrentX = buffer[positionOffset];
        const uint8_t columnCurrentY = buffer[positionOffset + 1U];
        const uint8_t columnHeading = buffer[positionOffset + 2U];
        if (!havePosition) {
            currentX = columnCurrentX;
            currentY = columnCurrentY;
            heading = columnHeading;
            havePosition = true;
        } else if (currentX != columnCurrentX
                   || currentY != columnCurrentY
                   || heading != columnHeading) {
            return;
        }
    }

    debug_.fw_maze_cells = cells;
    debug_.fw_maze_current_x = currentX;
    debug_.fw_maze_current_y = currentY;
    debug_.fw_maze_heading = heading;
    debug_.fw_maze_map_valid = havePosition;
#else
    debug_.fw_maze_map_valid = false;
    debug_.fw_maze_cells = FirmwareMazeCells{};
    debug_.fw_maze_current_x = 0;
    debug_.fw_maze_current_y = 0;
    debug_.fw_maze_heading = 0;
#endif
}

void FirmwareSimBridge::updateSupervisorDebug()
{
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    AppNavSupervisorDebug supervisorDebug = {};
    App_NavSupervisor_GetDebug(&supervisorDebug);
    debug_.supervisor_state = supervisorStateText(supervisorDebug.state);
    debug_.supervisor_action = supervisorActionText(supervisorDebug.current_action);
    debug_.supervisor_result = supervisorDebug.last_result;
#else
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;
#endif
}

void FirmwareSimBridge::reset()
{
    ensureFirmwareCoreInitialized();

    const bool wasAdvanceControl = isAdvanceControlMode(control_mode_);
    const bool wasSmoothControl = isSmoothControlMode(control_mode_);
    const bool wasPivotControl = isPivotControlMode(control_mode_);
    const bool wasSupervisorControl = isSupervisorControlMode(control_mode_);
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    simulation_config_applied_ = false;
    straight_yaw_target_deg_ = 0.0;
    advance_yaw_reference_valid_ = false;
    advance_yaw_start_deg_ = 0.0;
    smooth_yaw_reference_valid_ = false;
    smooth_yaw_start_deg_ = 0.0;
    pivot_yaw_reference_valid_ = false;
    pivot_yaw_start_deg_ = 0.0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    if (wasAdvanceControl) {
        App_Nav_StopAdvanceAction();
    }
    if (wasSmoothControl) {
        App_Nav_StopSmoothAction();
    }
    if (wasPivotControl) {
        App_Nav_StopPivotAction();
    }
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (wasSupervisorControl) {
        App_NavSupervisor_Stop();
    }
#endif
    App_Nav_Reset();
    App_Maze_ResetState();
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    App_NavSupervisor_Reset();
#endif

    debug_ = Debug{};
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.sim_config_left_base = sim_config_left_base_;
    debug_.sim_config_right_base = sim_config_right_base_;
    debug_.state = QStringLiteral("FW: reset");
    debug_.reason = QStringLiteral("Firmware core initialized and reset");
    updateMazeDebug();
    updateFirmwareMazeMapDebug();
#else
    debug_ = Debug{};
#endif
}

void FirmwareSimBridge::start()
{
    ensureFirmwareCoreInitialized();

    const bool wasAdvanceControl = isAdvanceControlMode(control_mode_);
    const bool wasSmoothControl = isSmoothControlMode(control_mode_);
    const bool wasPivotControl = isPivotControlMode(control_mode_);
    const bool wasSupervisorControl = isSupervisorControlMode(control_mode_);
    enabled_ = true;
    control_mode_ = ControlMode::TelemetryOnly;
    advance_yaw_reference_valid_ = false;
    smooth_yaw_reference_valid_ = false;
    pivot_yaw_reference_valid_ = false;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.pivot_state = QStringLiteral("n/a");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    if (wasAdvanceControl) {
        App_Nav_StopAdvanceAction();
    }
    if (wasSmoothControl) {
        App_Nav_StopSmoothAction();
    }
    if (wasPivotControl) {
        App_Nav_StopPivotAction();
    }
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (wasSupervisorControl) {
        App_NavSupervisor_Stop();
    }
#endif
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

    const bool wasAdvanceControl = isAdvanceControlMode(control_mode_);
    const bool wasSmoothControl = isSmoothControlMode(control_mode_);
    const bool wasPivotControl = isPivotControlMode(control_mode_);
    const bool wasSupervisorControl = isSupervisorControlMode(control_mode_);
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    advance_yaw_reference_valid_ = false;
    smooth_yaw_reference_valid_ = false;
    pivot_yaw_reference_valid_ = false;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.pivot_state = QStringLiteral("n/a");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    if (wasAdvanceControl) {
        App_Nav_StopAdvanceAction();
    }
    if (wasSmoothControl) {
        App_Nav_StopSmoothAction();
    }
    if (wasPivotControl) {
        App_Nav_StopPivotAction();
    }
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (wasSupervisorControl) {
        App_NavSupervisor_Stop();
    }
#endif
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
    advance_yaw_reference_valid_ = false;
    smooth_yaw_reference_valid_ = false;
    pivot_yaw_reference_valid_ = false;
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.pivot_state = QStringLiteral("n/a");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (isSupervisorControlMode(control_mode_)) {
        App_NavSupervisor_Stop();
    }
#endif
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

void FirmwareSimBridge::startWallFollowAdvance()
{
    ensureFirmwareCoreInitialized();
    advance_yaw_reference_valid_ = false;
    advance_yaw_start_deg_ = 0.0;
    smooth_yaw_reference_valid_ = false;
    pivot_yaw_reference_valid_ = false;
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.pivot_state = QStringLiteral("n/a");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (isSupervisorControlMode(control_mode_)) {
        App_NavSupervisor_Stop();
    }
#endif
    const bool started = App_Nav_StartAdvanceAction(APP_NAV_ADVANCE_ACTION_WALL_FOLLOW_AUTO_YAW_HOLD);
    enabled_ = started;
    control_mode_ = started ? ControlMode::WallFollowAdvance : ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.advance_state = started ? QStringLiteral("wait_leave_rear_tape") : QStringLiteral("error");
    debug_.state = started ? QStringLiteral("FW: wall-follow advance") : QStringLiteral("FW: idle");
    debug_.reason = started
        ? QStringLiteral("Advance action started")
        : QStringLiteral("Advance action could not start");
#else
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = QStringLiteral("TelemetryOnly");
    debug_.advance_state = QStringLiteral("n/a");
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Wall-follow advance unsupported without firmware core");
#endif
}

void FirmwareSimBridge::startSmoothTurnLeft()
{
    ensureFirmwareCoreInitialized();
    advance_yaw_reference_valid_ = false;
    smooth_yaw_reference_valid_ = false;
    smooth_yaw_start_deg_ = 0.0;
    pivot_yaw_reference_valid_ = false;
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.pivot_state = QStringLiteral("n/a");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (isSupervisorControlMode(control_mode_)) {
        App_NavSupervisor_Stop();
    }
#endif
    const bool started = App_Nav_StartSmoothAction(APP_NAV_SMOOTH_ACTION_LEFT);
    enabled_ = started;
    control_mode_ = started ? ControlMode::SmoothTurnLeft : ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.smooth_state = started ? QStringLiteral("turning") : QStringLiteral("error");
    debug_.state = started ? QStringLiteral("FW: smooth turn left") : QStringLiteral("FW: idle");
    debug_.reason = started
        ? QStringLiteral("Smooth turn left action started")
        : QStringLiteral("Smooth turn left action could not start");
#else
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = QStringLiteral("TelemetryOnly");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Smooth turn left unsupported without firmware core");
#endif
}

void FirmwareSimBridge::startSmoothTurnRight()
{
    ensureFirmwareCoreInitialized();
    advance_yaw_reference_valid_ = false;
    smooth_yaw_reference_valid_ = false;
    smooth_yaw_start_deg_ = 0.0;
    pivot_yaw_reference_valid_ = false;
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.pivot_state = QStringLiteral("n/a");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (isSupervisorControlMode(control_mode_)) {
        App_NavSupervisor_Stop();
    }
#endif
    const bool started = App_Nav_StartSmoothAction(APP_NAV_SMOOTH_ACTION_RIGHT);
    enabled_ = started;
    control_mode_ = started ? ControlMode::SmoothTurnRight : ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.smooth_state = started ? QStringLiteral("turning") : QStringLiteral("error");
    debug_.state = started ? QStringLiteral("FW: smooth turn right") : QStringLiteral("FW: idle");
    debug_.reason = started
        ? QStringLiteral("Smooth turn right action started")
        : QStringLiteral("Smooth turn right action could not start");
#else
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = QStringLiteral("TelemetryOnly");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Smooth turn right unsupported without firmware core");
#endif
}

void FirmwareSimBridge::startPivotLeft90()
{
    ensureFirmwareCoreInitialized();
    advance_yaw_reference_valid_ = false;
    smooth_yaw_reference_valid_ = false;
    pivot_yaw_reference_valid_ = false;
    pivot_yaw_start_deg_ = 0.0;
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (isSupervisorControlMode(control_mode_)) {
        App_NavSupervisor_Stop();
    }
#endif
    const bool started = App_Nav_StartPivotAction(APP_NAV_PIVOT_LEFT_90);
    enabled_ = started;
    control_mode_ = started ? ControlMode::PivotLeft90 : ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.pivot_state = started ? QStringLiteral("running") : QStringLiteral("error");
    debug_.state = started ? QStringLiteral("FW: pivot left 90") : QStringLiteral("FW: idle");
    debug_.reason = started
        ? QStringLiteral("Pivot left 90 action started")
        : QStringLiteral("Pivot left 90 action could not start");
#else
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = QStringLiteral("TelemetryOnly");
    debug_.pivot_state = QStringLiteral("n/a");
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Pivot left 90 unsupported without firmware core");
#endif
}

void FirmwareSimBridge::startPivotRight90()
{
    ensureFirmwareCoreInitialized();
    advance_yaw_reference_valid_ = false;
    smooth_yaw_reference_valid_ = false;
    pivot_yaw_reference_valid_ = false;
    pivot_yaw_start_deg_ = 0.0;
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (isSupervisorControlMode(control_mode_)) {
        App_NavSupervisor_Stop();
    }
#endif
    const bool started = App_Nav_StartPivotAction(APP_NAV_PIVOT_RIGHT_90);
    enabled_ = started;
    control_mode_ = started ? ControlMode::PivotRight90 : ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.pivot_state = started ? QStringLiteral("running") : QStringLiteral("error");
    debug_.state = started ? QStringLiteral("FW: pivot right 90") : QStringLiteral("FW: idle");
    debug_.reason = started
        ? QStringLiteral("Pivot right 90 action started")
        : QStringLiteral("Pivot right 90 action could not start");
#else
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = QStringLiteral("TelemetryOnly");
    debug_.pivot_state = QStringLiteral("n/a");
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Pivot right 90 unsupported without firmware core");
#endif
}

void FirmwareSimBridge::startPivot180()
{
    ensureFirmwareCoreInitialized();
    advance_yaw_reference_valid_ = false;
    smooth_yaw_reference_valid_ = false;
    pivot_yaw_reference_valid_ = false;
    pivot_yaw_start_deg_ = 0.0;
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;

#if SIM_AUTITO_HAS_FIRMWARE_CORE
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (isSupervisorControlMode(control_mode_)) {
        App_NavSupervisor_Stop();
    }
#endif
    const bool started = App_Nav_StartPivotAction(APP_NAV_PIVOT_180_RIGHT);
    enabled_ = started;
    control_mode_ = started ? ControlMode::Pivot180 : ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    debug_.pivot_state = started ? QStringLiteral("running") : QStringLiteral("error");
    debug_.state = started ? QStringLiteral("FW: pivot 180") : QStringLiteral("FW: idle");
    debug_.reason = started
        ? QStringLiteral("Pivot 180 action started")
        : QStringLiteral("Pivot 180 action could not start");
#else
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = QStringLiteral("TelemetryOnly");
    debug_.pivot_state = QStringLiteral("n/a");
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Pivot 180 unsupported without firmware core");
#endif
}

bool FirmwareSimBridge::resetSupervisorWithInitialPose(uint8_t x, uint8_t y, uint8_t heading)
{
    ensureFirmwareCoreInitialized();

#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    const bool validPose =
        x < MAZE_WIDTH
        && y < MAZE_HEIGHT
        && heading <= static_cast<uint8_t>(HEADING_WEST);
    const bool resetOk = validPose
        && App_NavSupervisor_ResetWithInitialPose(x, y, static_cast<HeadingTypeDef>(heading));
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    if (resetOk) {
        updateSupervisorDebug();
        updateMazeDebug();
        updateFirmwareMazeMapDebug();
        debug_.state = QStringLiteral("FW: reset");
        debug_.reason = QStringLiteral("Supervisor V1 initial pose set");
        return true;
    }

    debug_.fw_maze_map_valid = false;
    debug_.fw_maze_cells = FirmwareMazeCells{};
    debug_.state = QStringLiteral("FW: reset");
    debug_.reason = QStringLiteral("Supervisor V1 initial pose invalid");
    return false;
#else
    Q_UNUSED(x);
    Q_UNUSED(y);
    Q_UNUSED(heading);
    debug_.fw_maze_map_valid = false;
    debug_.fw_maze_cells = FirmwareMazeCells{};
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Supervisor V1 initial pose unsupported without app_nav_supervisor");
    return false;
#endif
}

bool FirmwareSimBridge::startSupervisorV1()
{
    return startSupervisorV1Internal(false, 0U, 0U, 0U);
}

bool FirmwareSimBridge::startSupervisorV1(uint8_t x, uint8_t y, uint8_t heading)
{
    return startSupervisorV1Internal(true, x, y, heading);
}

bool FirmwareSimBridge::startSupervisorV1Internal(bool has_initial_pose,
                                                  uint8_t x,
                                                  uint8_t y,
                                                  uint8_t heading)
{
    ensureFirmwareCoreInitialized();
    advance_yaw_reference_valid_ = false;
    advance_yaw_start_deg_ = 0.0;
    smooth_yaw_reference_valid_ = false;
    smooth_yaw_start_deg_ = 0.0;
    pivot_yaw_reference_valid_ = false;
    pivot_yaw_start_deg_ = 0.0;
    debug_.advance_state = QStringLiteral("n/a");
    debug_.smooth_state = QStringLiteral("n/a");
    debug_.pivot_state = QStringLiteral("n/a");

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    App_Nav_StopAdvanceAction();
    App_Nav_StopSmoothAction();
    App_Nav_StopPivotAction();
#endif

#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    bool poseResetOk = true;
    if (has_initial_pose) {
        if (x >= MAZE_WIDTH || y >= MAZE_HEIGHT || heading > static_cast<uint8_t>(HEADING_WEST)) {
            poseResetOk = false;
        } else {
            poseResetOk = App_NavSupervisor_ResetWithInitialPose(x,
                                                                 y,
                                                                 static_cast<HeadingTypeDef>(heading));
        }
    } else {
        App_NavSupervisor_Reset();
    }

    if (!poseResetOk) {
        enabled_ = false;
        control_mode_ = ControlMode::TelemetryOnly;
        debug_.enabled = enabled_;
        debug_.control_mode = controlModeText(control_mode_);
        updateSupervisorDebug();
        debug_.fw_maze_map_valid = false;
        debug_.fw_maze_cells = FirmwareMazeCells{};
        debug_.state = QStringLiteral("FW: idle");
        debug_.reason = QStringLiteral("Supervisor V1 initial pose invalid");
        return false;
    }

    const bool started = App_NavSupervisor_Start();
    enabled_ = started;
    control_mode_ = started ? ControlMode::SupervisorV1 : ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = controlModeText(control_mode_);
    updateSupervisorDebug();
    debug_.state = started ? QStringLiteral("FW: supervisor V1") : QStringLiteral("FW: idle");
    debug_.reason = started
        ? QStringLiteral("Supervisor V1 started")
        : QStringLiteral("Supervisor V1 could not start");
    return started;
#else
    enabled_ = false;
    control_mode_ = ControlMode::TelemetryOnly;
    debug_.enabled = enabled_;
    debug_.control_mode = QStringLiteral("TelemetryOnly");
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Supervisor V1 unsupported without app_nav_supervisor");
    return false;
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
    } else if (isAdvanceControlMode(control_mode_)) {
        if (!advance_yaw_reference_valid_) {
            advance_yaw_start_deg_ = snapshot.yaw_deg;
            advance_yaw_reference_valid_ = true;
        }
        firmware_snapshot.yaw_deg = shortestDeltaDeg(snapshot.yaw_deg, advance_yaw_start_deg_);
    } else if (isSmoothControlMode(control_mode_)) {
        if (!smooth_yaw_reference_valid_) {
            smooth_yaw_start_deg_ = snapshot.yaw_deg;
            smooth_yaw_reference_valid_ = true;
        }
        firmware_snapshot.yaw_deg = shortestDeltaDeg(snapshot.yaw_deg, smooth_yaw_start_deg_);
    } else if (isPivotControlMode(control_mode_)) {
        if (!pivot_yaw_reference_valid_) {
            pivot_yaw_start_deg_ = snapshot.yaw_deg;
            pivot_yaw_reference_valid_ = true;
        }
        firmware_snapshot.yaw_deg = shortestDeltaDeg(snapshot.yaw_deg, pivot_yaw_start_deg_);
    }

    const AppNavInput input = buildAppNavInput(firmware_snapshot);
    AppNavOutput output = {};

    App_Nav_Tick(&input, &output);

    AppNavRecommendedAction recommended_action = APP_NAV_ACTION_NONE;
    if (!isSupervisorControlMode(control_mode_)) {
        App_Nav_RecommendAction(kDecisionRandomValue, &recommended_action);
    }

    bool straight_yaw_hold_ok = true;
    bool advance_action_ticked = false;
    AppNavAdvanceActionState advance_action_state = APP_NAV_ADVANCE_ACTION_IDLE;
    bool smooth_action_ticked = false;
    AppNavSmoothActionState smooth_action_state = APP_NAV_SMOOTH_ACTION_IDLE;
    bool pivot_action_ticked = false;
    AppNavPivotActionState pivot_action_state = APP_NAV_PIVOT_ACTION_IDLE;
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    bool supervisor_ticked = false;
    AppNavSupervisorState supervisor_state = APP_NAV_SUPERVISOR_IDLE;
#endif
    if (control_mode_ == ControlMode::StraightYawHold) {
        AppNavOutput primitive_output = {};
        straight_yaw_hold_ok = App_Nav_ComputeStraightDrivePwm(&input, &primitive_output);
        if (straight_yaw_hold_ok) {
            command.left_pwm = primitive_output.left_motor_pwm;
            command.right_pwm = primitive_output.right_motor_pwm;
        }
    }
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    else if (control_mode_ == ControlMode::SupervisorV1) {
        AppNavOutput supervisor_output = {};
        supervisor_ticked = true;
        supervisor_state = App_NavSupervisor_Tick(&input, &supervisor_output);
        if (supervisor_state == APP_NAV_SUPERVISOR_DECIDE
            || supervisor_state == APP_NAV_SUPERVISOR_RUN_ADVANCE
            || supervisor_state == APP_NAV_SUPERVISOR_RUN_APPROACH_FRONT_WALL_FOR_PIVOT
            || supervisor_state == APP_NAV_SUPERVISOR_RUN_SMOOTH_LEFT
            || supervisor_state == APP_NAV_SUPERVISOR_RUN_SMOOTH_RIGHT
            || supervisor_state == APP_NAV_SUPERVISOR_RUN_PIVOT_180) {
            command.left_pwm = supervisor_output.left_motor_pwm;
            command.right_pwm = supervisor_output.right_motor_pwm;
        } else {
            command.left_pwm = 0;
            command.right_pwm = 0;
        }
    }
#endif
    else if (control_mode_ == ControlMode::WallFollowAdvance) {
        AppNavOutput primitive_output = {};
        advance_action_ticked = true;
        advance_action_state = App_Nav_TickAdvanceAction(&input, &primitive_output);
        if (advance_action_state == APP_NAV_ADVANCE_ACTION_WAIT_LEAVE_REAR_TAPE
            || advance_action_state == APP_NAV_ADVANCE_ACTION_RUNNING_WALL_FOLLOW
            || advance_action_state == APP_NAV_ADVANCE_ACTION_RUNNING_YAW_HOLD) {
            command.left_pwm = primitive_output.left_motor_pwm;
            command.right_pwm = primitive_output.right_motor_pwm;
        } else {
            command.left_pwm = 0;
            command.right_pwm = 0;

            const bool advanceTerminalState =
                advance_action_state == APP_NAV_ADVANCE_ACTION_DONE_REAR_TAPE
                || advance_action_state == APP_NAV_ADVANCE_ACTION_TIMEOUT
                || advance_action_state == APP_NAV_ADVANCE_ACTION_ERROR;
            if (advanceTerminalState) {
                App_Nav_StopAdvanceAction();
                enabled_ = false;
                control_mode_ = ControlMode::TelemetryOnly;
                advance_yaw_reference_valid_ = false;
            }
        }
    } else if (control_mode_ == ControlMode::SmoothTurnLeft
               || control_mode_ == ControlMode::SmoothTurnRight) {
        AppNavOutput primitive_output = {};
        smooth_action_ticked = true;
        smooth_action_state = App_Nav_TickSmoothAction(&input, &primitive_output);
        if (smooth_action_state == APP_NAV_SMOOTH_ACTION_TURNING
            || smooth_action_state == APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE) {
            command.left_pwm = primitive_output.left_motor_pwm;
            command.right_pwm = primitive_output.right_motor_pwm;
        } else {
            command.left_pwm = 0;
            command.right_pwm = 0;

            const bool smoothTerminalState =
                smooth_action_state == APP_NAV_SMOOTH_ACTION_DONE_REAR_TAPE
                || smooth_action_state == APP_NAV_SMOOTH_ACTION_DONE_WALL
                || smooth_action_state == APP_NAV_SMOOTH_ACTION_DONE_POST_YAW_REAR_TAPE
                || smooth_action_state == APP_NAV_SMOOTH_ACTION_FRONT_WALL_SAFETY
                || smooth_action_state == APP_NAV_SMOOTH_ACTION_POST_YAW_TIMEOUT
                || smooth_action_state == APP_NAV_SMOOTH_ACTION_ERROR;
            if (smoothTerminalState) {
                App_Nav_StopSmoothAction();
                enabled_ = false;
                control_mode_ = ControlMode::TelemetryOnly;
                smooth_yaw_reference_valid_ = false;
            }
        }
    } else if (isPivotControlMode(control_mode_)) {
        AppNavOutput primitive_output = {};
        pivot_action_ticked = true;
        pivot_action_state = App_Nav_TickPivotAction(&input, &primitive_output);
        if (pivot_action_state == APP_NAV_PIVOT_ACTION_RUNNING) {
            command.left_pwm = primitive_output.left_motor_pwm;
            command.right_pwm = primitive_output.right_motor_pwm;
        } else {
            command.left_pwm = 0;
            command.right_pwm = 0;
            if (pivot_action_state == APP_NAV_PIVOT_ACTION_TIMEOUT
                || pivot_action_state == APP_NAV_PIVOT_ACTION_ERROR) {
                App_Nav_StopPivotAction();
            }
            if (pivot_action_state == APP_NAV_PIVOT_ACTION_DONE
                || pivot_action_state == APP_NAV_PIVOT_ACTION_TIMEOUT
                || pivot_action_state == APP_NAV_PIVOT_ACTION_ERROR) {
                enabled_ = false;
                control_mode_ = ControlMode::TelemetryOnly;
                pivot_yaw_reference_valid_ = false;
            }
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
    if (advance_action_ticked) {
        debug_.advance_state = advanceActionStateText(advance_action_state);
        if (advance_action_state == APP_NAV_ADVANCE_ACTION_TIMEOUT) {
            debug_.reason += QStringLiteral(" advance_action=timeout");
        } else if (advance_action_state == APP_NAV_ADVANCE_ACTION_ERROR) {
            debug_.reason += QStringLiteral(" advance_action=error");
        }
    } else if (!isAdvanceControlMode(control_mode_)) {
        debug_.advance_state = QStringLiteral("n/a");
    }
    if (smooth_action_ticked) {
        debug_.smooth_state = smoothActionStateText(smooth_action_state);
    } else if (!isSmoothControlMode(control_mode_)) {
        debug_.smooth_state = QStringLiteral("n/a");
    }
    if (pivot_action_ticked) {
        debug_.pivot_state = pivotActionStateText(pivot_action_state);
        if (pivot_action_state == APP_NAV_PIVOT_ACTION_TIMEOUT) {
            debug_.reason += QStringLiteral(" pivot_action=timeout");
        } else if (pivot_action_state == APP_NAV_PIVOT_ACTION_ERROR) {
            debug_.reason += QStringLiteral(" pivot_action=error");
        }
    } else if (!isPivotControlMode(control_mode_)) {
        debug_.pivot_state = QStringLiteral("n/a");
    }
#if SIM_AUTITO_HAS_NAV_SUPERVISOR
    if (supervisor_ticked) {
        updateSupervisorDebug();
        if (supervisor_state == APP_NAV_SUPERVISOR_ERROR) {
            debug_.reason += QStringLiteral(" supervisor=error");
        }
    } else if (!isSupervisorControlMode(control_mode_)) {
        debug_.supervisor_state = QStringLiteral("n/a");
        debug_.supervisor_action = QStringLiteral("n/a");
        debug_.supervisor_result = 0;
    }
#else
    debug_.supervisor_state = QStringLiteral("n/a");
    debug_.supervisor_action = QStringLiteral("n/a");
    debug_.supervisor_result = 0;
#endif
    debug_.control_mode = controlModeText(control_mode_);
    debug_.sim_config_left_base = sim_config_left_base_;
    debug_.sim_config_right_base = sim_config_right_base_;
    debug_.recommended_action = static_cast<int>(recommended_action);
    debug_.recommended_action_text = recommendedActionText(recommended_action);
    debug_.available_options_mask = firmware_debug.available_options_mask;
    debug_.valid_option_count = firmware_debug.valid_option_count;
    debug_.decision_random_value = kDecisionRandomValue;
    if (!isSupervisorControlMode(control_mode_)
        && static_cast<int>(firmware_debug.last_recommended_action) != debug_.recommended_action) {
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
    updateMazeDebug();
    updateFirmwareMazeMapDebug();
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

bool FirmwareSimBridge::getFirmwareConfig(FirmwareConfig *out) const
{
    if (out == nullptr) {
        return false;
    }

#if SIM_AUTITO_HAS_FIRMWARE_CORE
    AppNavConfig config = {};
    App_Nav_GetConfig(&config);
    *out = toBridgeConfig(config);
    return true;
#else
    return false;
#endif
}

bool FirmwareSimBridge::setFirmwareConfig(const FirmwareConfig &config)
{
#if SIM_AUTITO_HAS_FIRMWARE_CORE
    AppNavConfig firmwareConfig = {};
    App_Nav_GetConfig(&firmwareConfig);
    copyEditableConfigToFirmware(config, &firmwareConfig);
    App_Nav_SetConfig(&firmwareConfig);

    sim_config_left_base_ = firmwareConfig.left_motor_base_speed;
    sim_config_right_base_ = firmwareConfig.right_motor_base_speed;
    debug_.sim_config_left_base = sim_config_left_base_;
    debug_.sim_config_right_base = sim_config_right_base_;
    simulation_config_applied_ = true;
    return true;
#else
    Q_UNUSED(config);
    return false;
#endif
}

bool FirmwareSimBridge::resetFirmwareConfigToSimulationDefaults()
{
#if SIM_AUTITO_HAS_FIRMWARE_CORE
    AppNavConfig config = App_Nav_DefaultConfig();
    applySimulationFirmwareDefaults(&config, last_left_gain_, last_right_gain_);
    App_Nav_SetConfig(&config);

    simulation_config_applied_ = true;
    sim_config_left_base_ = config.left_motor_base_speed;
    sim_config_right_base_ = config.right_motor_base_speed;
    debug_.sim_config_left_base = sim_config_left_base_;
    debug_.sim_config_right_base = sim_config_right_base_;
    return true;
#else
    return false;
#endif
}
