#ifndef SIM_UNERBUS_LINK_H
#define SIM_UNERBUS_LINK_H

#include "firmware_sim_bridge.h"

#include <cstdint>

#include <QByteArray>
#include <QHostAddress>
#include <QString>
#include <QtNetwork/QUdpSocket>

class SimUnerbusLink
{
public:
    SimUnerbusLink();

    bool configureRemote(const QString &host, quint16 port);
    void setEnabled(bool enabled);
    bool isEnabled() const;
    QString remoteHost() const;
    quint16 remotePort() const;
    quint16 localPort() const;
    QString statusText() const;

    void resetTiming();
    void sendAlive();
    bool sendSupervisorStatus(const FirmwareSimBridge &bridge, bool force = false);
    void tick(int elapsed_ms,
              const FirmwareSimBridge &bridge,
              const FirmwareSimBridge::SensorSnapshot &snapshot,
              const FirmwareSimBridge::Command &command);

private:
    struct ParsedPacket
    {
        uint8_t command = 0U;
        QByteArray payload;
    };

    static QByteArray buildPacket(uint8_t command, const QByteArray &payload);
    static bool parsePacket(const QByteArray &datagram, ParsedPacket *packet_out);
    static void appendUInt16LE(QByteArray *payload, uint16_t value);
    static void appendInt32LE(QByteArray *payload, int32_t value);
    static uint16_t absPwmToUInt16(int pwm);

    void sendPacket(uint8_t command, const QByteArray &payload);
    void processPendingDatagrams(const FirmwareSimBridge &bridge,
                                 const FirmwareSimBridge::SensorSnapshot &snapshot,
                                 const FirmwareSimBridge::Command &command);
    void handleCommand(const ParsedPacket &packet,
                       const FirmwareSimBridge &bridge,
                       const FirmwareSimBridge::SensorSnapshot &snapshot,
                       const FirmwareSimBridge::Command &command);

    QByteArray buildSupervisorStatusPayload(const FirmwareSimBridge &bridge) const;
    QByteArray buildMazeColumnPayload(const FirmwareSimBridge &bridge, uint8_t column) const;
    QByteArray buildIrSensorSnapshotPayload(const FirmwareSimBridge &bridge) const;
    QByteArray buildMotorPwmPayload(const FirmwareSimBridge::Command &command) const;
    QByteArray buildRobotStatusPayload(const FirmwareSimBridge &bridge) const;
    QByteArray buildYawAnglePayload(const FirmwareSimBridge::SensorSnapshot &snapshot) const;

    bool enabled_ = false;
    bool remoteConfigured_ = false;
    QHostAddress remoteAddress_ = QHostAddress(QHostAddress::LocalHost);
    QString remoteHost_ = QStringLiteral("127.0.0.1");
    quint16 remotePort_ = 30010U;
    QUdpSocket socket_;
    int aliveElapsedMs_ = 0;
    int supervisorElapsedMs_ = 0;
    bool lastSupervisorActive_ = false;
    QString lastEvent_ = QStringLiteral("idle");
};

#endif // SIM_UNERBUS_LINK_H
