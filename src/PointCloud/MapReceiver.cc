#include "MapReceiver.h"

#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(MapReceiverLog, "PointCloud.MapReceiver")

namespace {
constexpr int kStatusIntervalMs = 500;
}

MapReceiver::MapReceiver(QObject* parent) : QObject(parent)
{
    const QString envHost = qEnvironmentVariable("ENDER_GOAL_HOST");
    _host = envHost.isEmpty() ? QString::fromLatin1(kDefaultHost) : envHost;
    _modelRotation = QQuaternion();

    _reconnectTimer.setInterval(kReconnectIntervalMs);
    _reconnectTimer.setSingleShot(true);
    connect(&_reconnectTimer, &QTimer::timeout, this, &MapReceiver::_reconnect);

    _statusTimer.setInterval(kStatusIntervalMs);
    connect(&_statusTimer, &QTimer::timeout, this, &MapReceiver::_updateStatus);
}

MapReceiver::~MapReceiver()
{
    _close();
}

void MapReceiver::setActive(bool active)
{
    if (_active == active) {
        return;
    }
    _active = active;
    if (_active) {
        _open();
    } else {
        _close();
    }
    emit activeChanged();
}

void MapReceiver::setHost(const QString& host)
{
    if (_host == host) {
        return;
    }
    _host = host;
    emit hostChanged();
    if (_active) {
        _close();
        _open();
    }
}

void MapReceiver::setPort(quint16 port)
{
    if (_port == port) {
        return;
    }
    _port = port;
    emit portChanged();
    if (_active) {
        _close();
        _open();
    }
}

void MapReceiver::setRadius(float radius)
{
    if (qFuzzyCompare(_radius, radius) || radius <= 0.0f) {
        return;
    }
    _radius = radius;
    emit radiusChanged();
}

void MapReceiver::setFrozen(bool frozen)
{
    if (_frozen == frozen) {
        return;
    }
    _frozen = frozen;
    emit frozenChanged();
    if (!_frozen) {
        _publishPose();
        _publishMap();
    }
}

void MapReceiver::_open()
{
    if (_socket) {
        return;
    }
    _setErrorString(QString());
    _statusTimer.start();

    _socket = new QTcpSocket(this);
    connect(_socket, &QTcpSocket::readyRead, this, &MapReceiver::_readSocket);
    connect(_socket, &QTcpSocket::connected, this, &MapReceiver::_onConnected);
    connect(_socket, &QTcpSocket::disconnected, this, &MapReceiver::_onDisconnected);
    connect(_socket, &QTcpSocket::errorOccurred, this, &MapReceiver::_onSocketError);

    qCDebug(MapReceiverLog) << "Connecting to the obstacle map at" << _host << _port;
    _socket->connectToHost(_host, _port);
}

void MapReceiver::_close()
{
    _reconnectTimer.stop();
    _statusTimer.stop();
    if (_socket) {
        // Disconnect first: closing must not start a reconnect through the signals
        _socket->disconnect(this);
        _socket->abort();
        _socket->deleteLater();
        _socket = nullptr;
    }
    _decoder.reset();
    _setConnected(false);
    _setReceiving(false);
    _setPoseValid(false);
    _hasPose = false;
    _mapNeedsPose = false;
    _latestMap = EmapCloud();
    _clearMap();
}

void MapReceiver::_reconnect()
{
    if (!_active || !_socket || _socket->state() != QAbstractSocket::UnconnectedState) {
        return;
    }
    _socket->connectToHost(_host, _port);
}

void MapReceiver::_onConnected()
{
    qCDebug(MapReceiverLog) << "Obstacle map connected";
    _decoder.reset();
    _setErrorString(QString());
    _setConnected(true);
}

void MapReceiver::_onDisconnected()
{
    qCDebug(MapReceiverLog) << "Obstacle map connection closed";
    _setConnected(false);
    _setReceiving(false);
    if (_active) {
        _reconnectTimer.start();
    }
}

void MapReceiver::_onSocketError()
{
    if (!_socket) {
        return;
    }
    _setErrorString(_socket->errorString());
    // A failed connect does not always report disconnected()
    if (_socket->state() == QAbstractSocket::UnconnectedState) {
        _onDisconnected();
    }
}

void MapReceiver::_readSocket()
{
    if (_socket) {
        handleBytes(_socket->readAll());
    }
}

void MapReceiver::handleBytes(const QByteArray& bytes)
{
    const QList<EmapProtocol::Message> messages = _decoder.feed(bytes);

    // Poses are cheap and each one counts. Of several maps only the last is unpacked.
    const EmapProtocol::Message* lastMap = nullptr;
    for (const EmapProtocol::Message& message : messages) {
        if (message.type == EmapProtocol::kTypePose) {
            if (const auto pose = EmapProtocol::parsePose(message.payload)) {
                _applyPose(*pose);
            }
        } else if (message.type == EmapProtocol::kTypeMap) {
            lastMap = &message;
        }
    }
    if (lastMap) {
        if (auto cloud = EmapProtocol::parseMap(lastMap->payload)) {
            _applyMap(std::move(*cloud));
        }
    }
}

void MapReceiver::_applyPose(const EmapPose& pose)
{
    _latestPose = pose;
    _hasPose = true;
    _sinceLastPose.start();
    _setPoseValid(true);
    _publishPose();

    if (_mapNeedsPose) {
        // The first map arrived before the first pose and could not be cut yet
        _mapNeedsPose = false;
        _applyMap(std::move(_latestMap));
    }
}

void MapReceiver::_applyMap(EmapCloud cloud)
{
    _sinceLastMap.start();
    _setReceiving(true);

    if (!_hasPose) {
        _latestMap = std::move(cloud);
        _mapNeedsPose = true;
        return;
    }

    // Keep only what is near the vehicle: it is all that is drawn and the view is small
    const float radiusSquared = _radius * _radius;
    QList<QVector3D> kept;
    kept.reserve(cloud.points.size());
    for (const QVector3D& point : std::as_const(cloud.points)) {
        if ((point - _latestPose.position).lengthSquared() <= radiusSquared) {
            kept.append(point);
        }
    }
    cloud.points = std::move(kept);
    _latestMap = std::move(cloud);
    _publishMap();
}

void MapReceiver::_publishPose()
{
    if (_frozen || !_hasPose) {
        return;
    }
    EmapProtocol::sceneTransform(_latestPose, &_modelPosition, &_modelRotation);
    emit poseChanged();
}

void MapReceiver::_publishMap()
{
    if (_frozen || _mapNeedsPose) {
        return;
    }
    _map = _latestMap;
    emit mapChanged();
}

void MapReceiver::_clearMap()
{
    if (_map.points.isEmpty() && _map.stampMs == 0) {
        return;
    }
    _map = EmapCloud();
    emit mapChanged();
}

void MapReceiver::_updateStatus()
{
    if (_receiving && (!_sinceLastMap.isValid() || _sinceLastMap.hasExpired(kReceivingTimeoutMs))) {
        _setReceiving(false);
    }
    if (_poseValid && (!_sinceLastPose.isValid() || _sinceLastPose.hasExpired(kPoseTimeoutMs))) {
        _setPoseValid(false);
    }
}

void MapReceiver::_setConnected(bool connected)
{
    if (_connected == connected) {
        return;
    }
    _connected = connected;
    emit connectedChanged();
}

void MapReceiver::_setReceiving(bool receiving)
{
    if (_receiving == receiving) {
        return;
    }
    _receiving = receiving;
    emit receivingChanged();
}

void MapReceiver::_setPoseValid(bool valid)
{
    if (_poseValid == valid) {
        return;
    }
    _poseValid = valid;
    emit poseChanged();
}

void MapReceiver::_setErrorString(const QString& errorString)
{
    if (_errorString == errorString) {
        return;
    }
    _errorString = errorString;
    emit errorStringChanged();
}
