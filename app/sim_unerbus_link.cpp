#include "sim_unerbus_link.h"

#include <QByteArray>
#include <QtGlobal>

#include <algorithm>
#include <cmath>

namespace {
constexpr char kHeader[] = {'U', 'N', 'E', 'R'};
constexpr char kToken = ':';
constexpr int kAlivePeriodMs = 5000;
constexpr int kSupervisorStatusPeriodMs = 200;
constexpr uint16_t kSimPwmPeriod = 10000U;

constexpr uint8_t kCmdAck = 0x0DU;
constexpr uint8_t kCmdGetAlive = 0xF0U;
constexpr uint8_t kCmdGetIrSensorSnapshot = 0xA0U;
constexpr uint8_t kCmdGetMotorPwm = 0xA6U;
constexpr uint8_t kCmdGetPwmPeriod = 0x51U;
constexpr uint8_t kCmdGetRobotStatus = 0x74U;
constexpr uint8_t kCmdGetYawAngle = 0x75U;
constexpr uint8_t kCmdSyncMazeColumn = 0x93U;
constexpr uint8_t kCmdSetSupervisorInitialPose = 0x98U;
constexpr uint8_t kCmdGetSupervisorInitialPose = 0x99U;
constexpr uint8_t kCmdStartSupervisorRun = 0x9AU;
constexpr uint8_t kCmdStopSupervisorRun = 0x9BU;
constexpr uint8_t kCmdGetSupervisorDebugStatus = 0x9CU;
constexpr uint8_t kCmdSetSupervisorGoalCell = 0x9DU;
constexpr uint8_t kCmdGetSupervisorGoalCell = 0x9EU;
constexpr uint8_t kCmdSupervisorStatusUpdate = 0x9FU;
constexpr uint8_t kCmdClearSupervisorLearnedMap = 0xA9U;

constexpr uint8_t kAppStateMenu = 0U;
constexpr uint8_t kAppStateRunning = 1U;
constexpr uint8_t kMenuModeIdle = 0U;
constexpr uint8_t kMenuModeFindCells = 1U;
constexpr uint8_t kMenuModeGoToB = 2U;
constexpr uint8_t kMissionGoToB = 1U;

constexpr uint8_t kSensorDetWallFront = 0x01U;
constexpr uint8_t kSensorDetWallLeft = 0x02U;
constexpr uint8_t kSensorDetWallRight = 0x04U;
constexpr uint8_t kSensorDetWallDiagLeft = 0x08U;
constexpr uint8_t kSensorDetWallDiagRight = 0x10U;
constexpr uint8_t kSensorDetFloorFront = 0x20U;
constexpr uint8_t kSensorDetFloorRear = 0x40U;

uint8_t toByte(char value)
{
    return static_cast<uint8_t>(static_cast<unsigned char>(value));
}

uint8_t buildDetectionFlags(const FirmwareSimBridge::Debug &debug)
{
    uint8_t flags = 0U;
    if (debug.wall_front) {
        flags |= kSensorDetWallFront;
    }
    if (debug.wall_left) {
        flags |= kSensorDetWallLeft;
    }
    if (debug.wall_right) {
        flags |= kSensorDetWallRight;
    }
    if (debug.wall_diag_left) {
        flags |= kSensorDetWallDiagLeft;
    }
    if (debug.wall_diag_right) {
        flags |= kSensorDetWallDiagRight;
    }
    if (debug.floor_front_black) {
        flags |= kSensorDetFloorFront;
    }
    if (debug.floor_rear_black) {
        flags |= kSensorDetFloorRear;
    }
    return flags;
}
}

SimUnerbusLink::SimUnerbusLink()
{
    remoteConfigured_ = configureRemote(remoteHost_, remotePort_);
    socket_.bind(QHostAddress::AnyIPv4, 0U, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    resetTiming();
}

bool SimUnerbusLink::configureRemote(const QString &host, quint16 port)
{
    if (host.trimmed().isEmpty() || port == 0U) {
        remoteConfigured_ = false;
        lastEvent_ = QStringLiteral("invalid remote endpoint");
        return false;
    }

    QHostAddress address;
    if (!address.setAddress(host.trimmed())) {
        remoteConfigured_ = false;
        lastEvent_ = QStringLiteral("invalid remote address");
        return false;
    }

    remoteAddress_ = address;
    remoteHost_ = host.trimmed();
    remotePort_ = port;
    remoteConfigured_ = true;
    lastEvent_ = QStringLiteral("remote configured");
    return true;
}

void SimUnerbusLink::setEnabled(bool enabled)
{
    enabled_ = enabled;
    resetTiming();
    lastEvent_ = enabled_ ? QStringLiteral("enabled") : QStringLiteral("disabled");
}

bool SimUnerbusLink::isEnabled() const
{
    return enabled_;
}

QString SimUnerbusLink::remoteHost() const
{
    return remoteHost_;
}

quint16 SimUnerbusLink::remotePort() const
{
    return remotePort_;
}

quint16 SimUnerbusLink::localPort() const
{
    return socket_.localPort();
}

QString SimUnerbusLink::statusText() const
{
    return QStringLiteral("enabled=%1 remote=%2:%3 local_port=%4 last=%5")
        .arg(enabled_ ? QStringLiteral("true") : QStringLiteral("false"))
        .arg(remoteConfigured_ ? remoteHost_ : QStringLiteral("invalid"))
        .arg(remoteConfigured_ ? QString::number(remotePort_) : QStringLiteral("-"))
        .arg(socket_.localPort())
        .arg(lastEvent_);
}

bool SimUnerbusLink::consumeStartSimulationRequest()
{
    const bool requested = startSimulationRequested_;
    startSimulationRequested_ = false;
    return requested;
}

bool SimUnerbusLink::consumeStopSimulationRequest()
{
    const bool requested = stopSimulationRequested_;
    stopSimulationRequested_ = false;
    return requested;
}

void SimUnerbusLink::resetTiming()
{
    aliveElapsedMs_ = kAlivePeriodMs;
    supervisorElapsedMs_ = kSupervisorStatusPeriodMs;
    lastSupervisorActive_ = false;
}

QByteArray SimUnerbusLink::buildPacket(uint8_t command, const QByteArray &payload)
{
    QByteArray packet;
    packet.reserve(7 + payload.size());
    packet.append(kHeader, 4);

    const uint8_t length = static_cast<uint8_t>(1 + payload.size() + 1);
    packet.append(static_cast<char>(length));
    packet.append(kToken);
    packet.append(static_cast<char>(command));
    packet.append(payload);

    uint8_t checksum = 0U;
    for (char byte : packet) {
        checksum ^= toByte(byte);
    }
    packet.append(static_cast<char>(checksum));
    return packet;
}

bool SimUnerbusLink::parsePacket(const QByteArray &datagram, ParsedPacket *packet_out)
{
    if (packet_out == nullptr || datagram.size() < 7) {
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (datagram.at(i) != kHeader[i]) {
            return false;
        }
    }

    const uint8_t length = toByte(datagram.at(4));
    if (length < 2U || datagram.at(5) != kToken) {
        return false;
    }

    const int expectedSize = 6 + static_cast<int>(length);
    if (datagram.size() < expectedSize) {
        return false;
    }

    uint8_t checksum = 0U;
    for (int i = 0; i < expectedSize - 1; ++i) {
        checksum ^= toByte(datagram.at(i));
    }
    if (checksum != toByte(datagram.at(expectedSize - 1))) {
        return false;
    }

    packet_out->command = toByte(datagram.at(6));
    packet_out->payload = datagram.mid(7, static_cast<int>(length) - 2);
    return true;
}

void SimUnerbusLink::appendUInt16LE(QByteArray *payload, uint16_t value)
{
    if (payload == nullptr) {
        return;
    }

    payload->append(static_cast<char>(value & 0xFFU));
    payload->append(static_cast<char>((value >> 8) & 0xFFU));
}

void SimUnerbusLink::appendInt32LE(QByteArray *payload, int32_t value)
{
    if (payload == nullptr) {
        return;
    }

    const uint32_t raw = static_cast<uint32_t>(value);
    payload->append(static_cast<char>(raw & 0xFFU));
    payload->append(static_cast<char>((raw >> 8) & 0xFFU));
    payload->append(static_cast<char>((raw >> 16) & 0xFFU));
    payload->append(static_cast<char>((raw >> 24) & 0xFFU));
}

uint16_t SimUnerbusLink::absPwmToUInt16(int pwm)
{
    const int absPwm = std::abs(pwm);
    return static_cast<uint16_t>(std::clamp(absPwm, 0, 65535));
}

void SimUnerbusLink::sendPacket(uint8_t command, const QByteArray &payload)
{
    if (!enabled_ || !remoteConfigured_) {
        return;
    }

    const QByteArray packet = buildPacket(command, payload);
    socket_.writeDatagram(packet, remoteAddress_, remotePort_);
    lastEvent_ = QStringLiteral("sent cmd=0x%1 size=%2")
        .arg(command, 2, 16, QLatin1Char('0'))
        .arg(packet.size());
}

void SimUnerbusLink::sendAlive()
{
    QByteArray payload;
    payload.append(static_cast<char>(kCmdAck));
    sendPacket(kCmdGetAlive, payload);
}

bool SimUnerbusLink::sendSupervisorStatus(const FirmwareSimBridge &bridge, bool force)
{
    Q_UNUSED(force);

    const QByteArray payload = buildSupervisorStatusPayload(bridge);
    if (payload.size() != FirmwareSimBridge::kFirmwareSupervisorDebugStatusSize) {
        return false;
    }

    sendPacket(kCmdSupervisorStatusUpdate, payload);
    return true;
}

void SimUnerbusLink::tick(int elapsed_ms,
                          FirmwareSimBridge &bridge,
                          const FirmwareSimBridge::SensorSnapshot &snapshot,
                          const FirmwareSimBridge::Command &command)
{
    if (!enabled_) {
        return;
    }

    processPendingDatagrams(bridge, snapshot, command);

    aliveElapsedMs_ += std::max(elapsed_ms, 0);
    if (aliveElapsedMs_ >= kAlivePeriodMs) {
        aliveElapsedMs_ = 0;
        sendAlive();
    }

    const QByteArray supervisorPayload = buildSupervisorStatusPayload(bridge);
    if (supervisorPayload.size() != FirmwareSimBridge::kFirmwareSupervisorDebugStatusSize) {
        return;
    }

    const bool supervisorActive = static_cast<uint8_t>(supervisorPayload.at(2)) != 0U;
    const bool activityChanged = supervisorActive != lastSupervisorActive_;
    supervisorElapsedMs_ += std::max(elapsed_ms, 0);

    if (activityChanged || (supervisorActive && supervisorElapsedMs_ >= kSupervisorStatusPeriodMs)) {
        supervisorElapsedMs_ = 0;
        sendPacket(kCmdSupervisorStatusUpdate, supervisorPayload);
    }

    lastSupervisorActive_ = supervisorActive;
}

void SimUnerbusLink::processPendingDatagrams(FirmwareSimBridge &bridge,
                                             const FirmwareSimBridge::SensorSnapshot &snapshot,
                                             const FirmwareSimBridge::Command &command)
{
    while (socket_.hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(socket_.pendingDatagramSize()));
        QHostAddress senderAddress;
        quint16 senderPort = 0U;
        socket_.readDatagram(datagram.data(), datagram.size(), &senderAddress, &senderPort);

        ParsedPacket packet;
        if (!parsePacket(datagram, &packet)) {
            lastEvent_ = QStringLiteral("ignored invalid datagram");
            continue;
        }

        if (!senderAddress.isNull() && senderPort != 0U) {
            remoteAddress_ = senderAddress;
            remoteHost_ = senderAddress.toString();
            if (remoteHost_.startsWith(QStringLiteral("::ffff:"))) {
                remoteHost_ = remoteHost_.mid(7);
            }
            remotePort_ = senderPort;
            remoteConfigured_ = true;
        }

        handleCommand(packet, bridge, snapshot, command);
    }
}

void SimUnerbusLink::handleCommand(const ParsedPacket &packet,
                                   FirmwareSimBridge &bridge,
                                   const FirmwareSimBridge::SensorSnapshot &snapshot,
                                   const FirmwareSimBridge::Command &command)
{
    switch (packet.command) {
    case kCmdGetAlive:
        sendAlive();
        break;

    case kCmdSetSupervisorInitialPose:
        if (packet.payload.size() >= 3) {
            const bool ok = bridge.setSupervisorInitialPose(toByte(packet.payload.at(0)),
                                                            toByte(packet.payload.at(1)),
                                                            toByte(packet.payload.at(2)));
            lastEvent_ = ok
                ? QStringLiteral("set supervisor initial pose")
                : QStringLiteral("rejected supervisor initial pose");
        } else {
            lastEvent_ = QStringLiteral("bad supervisor initial pose payload");
        }
        break;

    case kCmdGetSupervisorInitialPose:
        sendPacket(kCmdGetSupervisorInitialPose, buildSupervisorInitialPosePayload(bridge));
        break;

    case kCmdSetSupervisorGoalCell:
        if (packet.payload.size() >= 2) {
            const bool ok = bridge.setSupervisorGoalCell(toByte(packet.payload.at(0)),
                                                         toByte(packet.payload.at(1)));
            lastEvent_ = ok
                ? QStringLiteral("set supervisor goal cell")
                : QStringLiteral("rejected supervisor goal cell");
        } else {
            lastEvent_ = QStringLiteral("bad supervisor goal cell payload");
        }
        break;

    case kCmdGetSupervisorGoalCell:
        sendPacket(kCmdGetSupervisorGoalCell, buildSupervisorGoalCellPayload(bridge));
        break;

    case kCmdStartSupervisorRun:
        if (!packet.payload.isEmpty()) {
            const bool started = bridge.startSupervisorRun(toByte(packet.payload.at(0)));
            startSimulationRequested_ = started;
            stopSimulationRequested_ = !started;
            lastEvent_ = started
                ? QStringLiteral("started supervisor run")
                : QStringLiteral("supervisor run start failed");
        } else {
            stopSimulationRequested_ = true;
            lastEvent_ = QStringLiteral("bad supervisor run payload");
        }
        break;

    case kCmdStopSupervisorRun:
        bridge.stopControl();
        stopSimulationRequested_ = true;
        lastEvent_ = QStringLiteral("stopped supervisor run");
        break;

    case kCmdGetSupervisorDebugStatus:
        sendPacket(kCmdGetSupervisorDebugStatus, buildSupervisorStatusPayload(bridge));
        break;

    case kCmdClearSupervisorLearnedMap:
    {
        bridge.clearSupervisorLearnedMap();
        QByteArray payload;
        payload.append(static_cast<char>(kCmdAck));
        sendPacket(kCmdClearSupervisorLearnedMap, payload);
        break;
    }

    case kCmdSyncMazeColumn:
        if (!packet.payload.isEmpty()) {
            sendPacket(kCmdSyncMazeColumn,
                       buildMazeColumnPayload(bridge, toByte(packet.payload.at(0))));
        }
        break;

    case kCmdGetIrSensorSnapshot:
        sendPacket(kCmdGetIrSensorSnapshot, buildIrSensorSnapshotPayload(bridge));
        break;

    case kCmdGetYawAngle:
        sendPacket(kCmdGetYawAngle, buildYawAnglePayload(snapshot));
        break;

    case kCmdGetRobotStatus:
        sendPacket(kCmdGetRobotStatus, buildRobotStatusPayload(bridge));
        break;

    case kCmdGetPwmPeriod:
    {
        QByteArray payload;
        appendUInt16LE(&payload, kSimPwmPeriod);
        sendPacket(kCmdGetPwmPeriod, payload);
        break;
    }

    case kCmdGetMotorPwm:
        sendPacket(kCmdGetMotorPwm, buildMotorPwmPayload(command));
        break;

    default:
        lastEvent_ = QStringLiteral("ignored unsupported cmd=0x%1")
            .arg(packet.command, 2, 16, QLatin1Char('0'));
        break;
    }
}

QByteArray SimUnerbusLink::buildSupervisorStatusPayload(const FirmwareSimBridge &bridge) const
{
    FirmwareSimBridge::FirmwareSupervisorDebugStatusPayload rawPayload = {};
    if (!bridge.writeSupervisorDebugStatusPayload(&rawPayload)) {
        return QByteArray();
    }

    QByteArray payload;
    payload.reserve(static_cast<int>(rawPayload.size()));
    for (uint8_t byte : rawPayload) {
        payload.append(static_cast<char>(byte));
    }
    return payload;
}

QByteArray SimUnerbusLink::buildSupervisorInitialPosePayload(const FirmwareSimBridge &bridge) const
{
    const FirmwareSimBridge::SupervisorInitialPose pose = bridge.supervisorInitialPose();

    QByteArray payload;
    payload.reserve(3);
    payload.append(static_cast<char>(pose.valid ? pose.x : 0U));
    payload.append(static_cast<char>(pose.valid ? pose.y : 0U));
    payload.append(static_cast<char>(pose.valid ? pose.heading : FirmwareSimBridge::kFirmwareMazeHeadingNorth));
    return payload;
}

QByteArray SimUnerbusLink::buildSupervisorGoalCellPayload(const FirmwareSimBridge &bridge) const
{
    const FirmwareSimBridge::SupervisorGoalCell goal = bridge.supervisorGoalCell();

    QByteArray payload;
    payload.reserve(3);
    payload.append(static_cast<char>(goal.valid ? goal.x : 0U));
    payload.append(static_cast<char>(goal.valid ? goal.y : 0U));
    payload.append(static_cast<char>(goal.valid ? 1U : 0U));
    return payload;
}

QByteArray SimUnerbusLink::buildMazeColumnPayload(const FirmwareSimBridge &bridge, uint8_t column) const
{
    FirmwareSimBridge::FirmwareMazeColumnSyncPayload rawPayload = {};
    if (!bridge.writeFirmwareMazeColumnSyncPayload(column, &rawPayload)) {
        return QByteArray();
    }

    QByteArray payload;
    payload.reserve(static_cast<int>(rawPayload.size()));
    for (uint8_t byte : rawPayload) {
        payload.append(static_cast<char>(byte));
    }
    return payload;
}

QByteArray SimUnerbusLink::buildIrSensorSnapshotPayload(const FirmwareSimBridge &bridge) const
{
    const FirmwareSimBridge::Debug debug = bridge.debug();
    QByteArray payload;
    payload.reserve(17);

    appendUInt16LE(&payload, debug.dist_right_lat_mm);
    appendUInt16LE(&payload, debug.dist_diagonal_right_mm);
    appendUInt16LE(&payload, debug.dist_front_right_mm);
    appendUInt16LE(&payload, debug.adc_floor_front);
    appendUInt16LE(&payload, debug.dist_front_left_mm);
    appendUInt16LE(&payload, debug.dist_diagonal_left_mm);
    appendUInt16LE(&payload, debug.dist_left_lat_mm);
    appendUInt16LE(&payload, debug.adc_floor_rear);
    payload.append(static_cast<char>(buildDetectionFlags(debug)));
    return payload;
}

QByteArray SimUnerbusLink::buildMotorPwmPayload(const FirmwareSimBridge::Command &command) const
{
    QByteArray payload;
    payload.reserve(8);

    appendUInt16LE(&payload, command.right_pwm < 0 ? absPwmToUInt16(command.right_pwm) : 0U);
    appendUInt16LE(&payload, command.right_pwm > 0 ? absPwmToUInt16(command.right_pwm) : 0U);
    appendUInt16LE(&payload, command.left_pwm < 0 ? absPwmToUInt16(command.left_pwm) : 0U);
    appendUInt16LE(&payload, command.left_pwm > 0 ? absPwmToUInt16(command.left_pwm) : 0U);
    return payload;
}

QByteArray SimUnerbusLink::buildRobotStatusPayload(const FirmwareSimBridge &bridge) const
{
    QByteArray payload;
    payload.reserve(2);
    const bool running = bridge.isFirmwareControlActive();
    uint8_t menuMode = kMenuModeIdle;

    if (running) {
        const FirmwareSimBridge::Debug debug = bridge.debug();
        menuMode = (debug.supervisor_mission == kMissionGoToB) ? kMenuModeGoToB : kMenuModeFindCells;
    }

    payload.append(static_cast<char>(running ? kAppStateRunning : kAppStateMenu));
    payload.append(static_cast<char>(menuMode));
    return payload;
}

QByteArray SimUnerbusLink::buildYawAnglePayload(const FirmwareSimBridge::SensorSnapshot &snapshot) const
{
    QByteArray payload;
    payload.reserve(4);
    const int32_t yawDeg = static_cast<int32_t>(std::llround(snapshot.yaw_deg));
    appendInt32LE(&payload, yawDeg);
    return payload;
}
