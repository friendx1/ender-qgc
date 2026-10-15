#pragma once

#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtGui/QQuaternion>
#include <QtGui/QVector3D>
#include <QtNetwork/QTcpSocket>
#include <QtQmlIntegration/QtQmlIntegration>

#include "MapProtocol.h"

/// Receives the obstacle map (SUPER inflate map) from the onboard computer (EMAP v1 over TCP).
///
/// The map is in the planner world frame and changes slowly. The vehicle pose arrives separately
/// and often, so the map is not rebuilt when the vehicle moves. The point cloud view draws the world scene:
/// the map as it is, the vehicle and the scan at the poses given here (vehiclePosition/vehicleRotation for the
/// latest pose, scanPosition/scanRotation for the pose at the moment of the scan).
///
/// While active, the receiver connects to the onboard computer and reconnects when the link breaks.
/// The pose flows as soon as it is connected. The map is only sent while wantMap is set. Switching it off
/// closes the connection, which stops the work on the vehicle.
class MapReceiver : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(bool active READ active WRITE setActive NOTIFY activeChanged)
    Q_PROPERTY(bool wantMap READ wantMap WRITE setWantMap NOTIFY wantMapChanged)
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY hostChanged)
    Q_PROPERTY(quint16 port READ port WRITE setPort NOTIFY portChanged)
    Q_PROPERTY(float radius READ radius WRITE setRadius NOTIFY radiusChanged)
    Q_PROPERTY(bool frozen READ frozen WRITE setFrozen NOTIFY frozenChanged)
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    Q_PROPERTY(bool receiving READ receiving NOTIFY receivingChanged)
    Q_PROPERTY(bool poseValid READ poseValid NOTIFY poseChanged)
    Q_PROPERTY(int pointCount READ pointCount NOTIFY mapChanged)
    Q_PROPERTY(QVector3D vehiclePosition READ vehiclePosition NOTIFY poseChanged)
    Q_PROPERTY(QQuaternion vehicleRotation READ vehicleRotation NOTIFY poseChanged)
    Q_PROPERTY(quint32 scanStamp READ scanStamp WRITE setScanStamp NOTIFY scanStampChanged)
    Q_PROPERTY(QVector3D scanPosition READ scanPosition NOTIFY scanPlacementChanged)
    Q_PROPERTY(QQuaternion scanRotation READ scanRotation NOTIFY scanPlacementChanged)
    Q_PROPERTY(QString errorString READ errorString NOTIFY errorStringChanged)

public:
    static constexpr quint16 kDefaultPort = 19902;
    static constexpr const char* kDefaultHost = "192.168.144.50";
    static constexpr float kDefaultRadius = 20.0f;    ///< meters around the vehicle that are kept and drawn
    static constexpr int kReconnectIntervalMs = 2000;
    static constexpr int kReceivingTimeoutMs = 4000;  ///< the map comes about once per second
    static constexpr int kPoseTimeoutMs = 2000;       ///< the pose comes about ten times per second

    explicit MapReceiver(QObject* parent = nullptr);
    ~MapReceiver() override;

    bool active() const { return _active; }

    void setActive(bool active);

    /// Whether the vehicle should send the obstacle map (and do the work for it). Sent to the vehicle
    /// when connected and again after every reconnect.
    bool wantMap() const { return _wantMap; }

    void setWantMap(bool wantMap);

    /// IP address of the onboard computer. Taken from the environment variable ENDER_GOAL_HOST if set,
    /// the same address that receives the flight goals.
    QString host() const { return _host; }

    void setHost(const QString& host);

    quint16 port() const { return _port; }

    void setPort(quint16 port);

    /// Points farther than this from the vehicle (meters) are dropped when a map arrives
    float radius() const { return _radius; }

    void setRadius(float radius);

    /// While frozen, the displayed map and pose stay as they are (the point cloud view freezes the scan
    /// while a goal is picked). Unfreezing shows the latest state.
    bool frozen() const { return _frozen; }

    void setFrozen(bool frozen);

    bool connected() const { return _connected; }

    bool receiving() const { return _receiving; }

    bool poseValid() const { return _poseValid; }

    int pointCount() const { return static_cast<int>(_map.points.size()); }

    /// Latest pose of the vehicle as placement of a node in the world scene (centimeters, scene axes).
    /// A node with this transform carries everything that is given in the vehicle body frame.
    QVector3D vehiclePosition() const { return _vehiclePosition; }

    QQuaternion vehicleRotation() const { return _vehicleRotation; }

    /// Stamp (milliseconds) of the scan on display. The scan is given in the vehicle body frame, so it is placed
    /// with the pose the vehicle had at that moment, which is scanPosition and scanRotation.
    quint32 scanStamp() const { return _scanStamp; }

    void setScanStamp(quint32 stampMs);

    QVector3D scanPosition() const { return _scanPosition; }

    QQuaternion scanRotation() const { return _scanRotation; }

    QString errorString() const { return _errorString; }

    /// Displayed map in world meters, valid until the next mapChanged()
    const EmapCloud& map() const { return _map; }

    /// Feeds received bytes, normally called from the socket. Public so tests need no network.
    void handleBytes(const QByteArray& bytes);

signals:
    void activeChanged();
    void wantMapChanged();
    void scanStampChanged();
    void scanPlacementChanged();
    void hostChanged();
    void portChanged();
    void radiusChanged();
    void frozenChanged();
    void connectedChanged();
    void receivingChanged();
    void poseChanged();
    void mapChanged();
    void errorStringChanged();

private slots:
    void _readSocket();
    void _onConnected();
    void _onDisconnected();
    void _onSocketError();
    void _reconnect();
    void _updateStatus();

private:
    void _open();
    void _close();
    void _applyPose(const EmapPose& pose);
    void _applyMap(EmapCloud cloud);
    void _publishPose();
    void _updateScanPlacement();
    void _sendControl();
    void _publishMap();
    void _clearMap();
    void _setConnected(bool connected);
    void _setReceiving(bool receiving);
    void _setPoseValid(bool valid);
    void _setErrorString(const QString& errorString);

    QTcpSocket* _socket = nullptr;
    QTimer _reconnectTimer;
    QTimer _statusTimer;
    QElapsedTimer _sinceLastMap;
    QElapsedTimer _sinceLastPose;
    EmapStreamDecoder _decoder;

    EmapCloud _latestMap;  ///< newest map as received, cut to the radius around the pose of its arrival
    EmapPose _latestPose;
    EmapPoseHistory _poseHistory;
    bool _hasPose = false;
    bool _mapNeedsPose = false;  ///< a map arrived before the first pose and is not cut yet

    EmapCloud _map;              ///< displayed
    QVector3D _vehiclePosition;
    QQuaternion _vehicleRotation;
    quint32 _scanStamp = 0;
    QVector3D _scanPosition;
    QQuaternion _scanRotation;

    bool _active = false;
    bool _wantMap = true;
    QString _host;
    quint16 _port = kDefaultPort;
    float _radius = kDefaultRadius;
    bool _frozen = false;
    bool _connected = false;
    bool _receiving = false;
    bool _poseValid = false;
    QString _errorString;
};
