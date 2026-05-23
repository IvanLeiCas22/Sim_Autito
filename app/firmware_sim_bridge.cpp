#include "firmware_sim_bridge.h"

#include <QtGlobal>

FirmwareSimBridge::FirmwareSimBridge() = default;

void FirmwareSimBridge::reset()
{
    enabled_ = false;
    debug_ = Debug{};
}

void FirmwareSimBridge::start()
{
    enabled_ = true;
    debug_.enabled = enabled_;
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Bridge enabled, firmware core not connected");
}

void FirmwareSimBridge::stop()
{
    enabled_ = false;
    debug_.enabled = enabled_;
    debug_.state = QStringLiteral("STUB");
    debug_.reason = QStringLiteral("Bridge stopped");
}

FirmwareSimBridge::Command FirmwareSimBridge::tick(const SensorSnapshot &snapshot)
{
    Q_UNUSED(snapshot);

    Command command;
    command.left_pwm = 0;
    command.right_pwm = 0;

    debug_.enabled = enabled_;
    debug_.state = QStringLiteral("STUB");
    debug_.reason = enabled_
        ? QStringLiteral("FirmwareSimBridge tick executed without firmware core")
        : QStringLiteral("FirmwareSimBridge disabled");
    debug_.left_pwm = command.left_pwm;
    debug_.right_pwm = command.right_pwm;

    return command;
}

FirmwareSimBridge::Debug FirmwareSimBridge::debug() const
{
    return debug_;
}
