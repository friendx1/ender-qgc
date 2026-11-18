#include "LauncherClient.h"

#include <QtCore/QDateTime>
#include <QtCore/QVariantMap>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkDatagram>

#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(LauncherClientLog, "PointCloud.LauncherClient")

LauncherClient::LauncherClient(QObject* parent) : QObject(parent)
{
    // A time based start, so a restarted app does not reuse sequence numbers the launcher may still know
    _nextSequence = static_cast<quint32>(QDateTime::currentMSecsSinceEpoch() & 0xFFFFFFF);

    _pollTimer.setInterval(kPollIntervalMs);
    connect(&_pollTimer, &QTimer::timeout, this, &LauncherClient::_poll);
    _retryTimer.setInterval(kRetryIntervalMs);
    connect(&_retryTimer, &QTimer::timeout, this, &LauncherClient::_retry);
    _silenceTimer.setInterval(1000);
    connect(&_silenceTimer, &QTimer::timeout, this, &LauncherClient::_checkSilence);
}

void LauncherClient::setHost(const QString& host)
{
    if (_host == host) {
        return;
    }
    _host = host;
    // The answers of the old address say nothing about the new one
    _reply = LauncherReply();
    _message.clear();
    _setReachable(false);
    emit hostChanged();
    emit stateChanged();
    if (_active) {
        _poll();
    }
}

void LauncherClient::setActive(bool active)
{
    if (_active == active) {
        return;
    }
    _active = active;
    if (_active) {
        _lastReplyMs = QDateTime::currentMSecsSinceEpoch();
        _pollTimer.start();
        _silenceTimer.start();
        _poll();
    } else {
        _pollTimer.stop();
        _silenceTimer.stop();
        _retryTimer.stop();
        _commandPending = false;
    }
    emit activeChanged();
}

QVariantList LauncherClient::steps() const
{
    QVariantList list;
    for (const LauncherStepInfo& step : _reply.steps) {
        list.append(QVariantMap{
            {QStringLiteral("name"), step.name},
            {QStringLiteral("state"), step.state},
            {QStringLiteral("detail"), step.detail},
            {QStringLiteral("canStop"), step.canStop},
        });
    }
    return list;
}

void LauncherClient::startAll()
{
    _sendCommand(QStringLiteral("start"));
}

void LauncherClient::startStep(int index)
{
    if (index < 0 || index >= _reply.steps.size()) {
        return;
    }
    _sendCommand(QStringLiteral("step"), index);
}

void LauncherClient::stopStep(int index)
{
    if (index < 0 || index >= _reply.steps.size() || !_reply.steps[index].canStop) {
        return;
    }
    _sendCommand(QStringLiteral("stop_step"), index);
}

void LauncherClient::stopAll()
{
    _sendCommand(QStringLiteral("stop"));
}

bool LauncherClient::_ensureSocket()
{
    if (_socket) {
        return true;
    }
    auto* socket = new QUdpSocket(this);
    if (!socket->bind(QHostAddress::AnyIPv4, 0)) {
        qCWarning(LauncherClientLog) << "Cannot open UDP socket:" << socket->errorString();
        delete socket;
        return false;
    }
    _socket = socket;
    connect(_socket, &QUdpSocket::readyRead, this, &LauncherClient::_readPendingDatagrams);
    return true;
}

bool LauncherClient::_send(const QByteArray& datagram)
{
    if (!_ensureSocket()) {
        return false;
    }
    const QHostAddress address(_host);
    if (address.isNull()) {
        return false;
    }
    return _socket->writeDatagram(datagram, address, LauncherProtocol::kDefaultPort) == datagram.size();
}

void LauncherClient::_poll()
{
    if (!_active) {
        return;
    }
    if (!_send(LauncherProtocol::encodeRequest(QStringLiteral("status"), _nextSequence++))) {
        _message = tr("Ungültige Adresse: %1").arg(_host);
        emit stateChanged();
    }
}

void LauncherClient::_sendCommand(const QString& command, int stepIndex)
{
    if (!_active || _commandPending) {
        return;  // one command at a time, the next one waits for the answer to this one
    }
    _commandSequence = _nextSequence++;
    _commandDatagram = LauncherProtocol::encodeRequest(command, _commandSequence, stepIndex);
    _attempts = 0;
    _commandPending = true;
    qCDebug(LauncherClientLog) << "Command" << command << "step" << stepIndex << "seq" << _commandSequence;
    _retry();
}

void LauncherClient::_retry()
{
    if (!_commandPending) {
        _retryTimer.stop();
        return;
    }
    if (_attempts >= kMaxAttempts) {
        _commandPending = false;
        _retryTimer.stop();
        _message = tr("Keine Antwort vom Starter-Dienst");
        emit stateChanged();
        return;
    }
    _attempts++;
    if (!_send(_commandDatagram)) {
        _commandPending = false;
        _retryTimer.stop();
        _message = tr("Senden fehlgeschlagen");
        emit stateChanged();
        return;
    }
    _retryTimer.start();
}

void LauncherClient::_readPendingDatagrams()
{
    while (_socket && _socket->hasPendingDatagrams()) {
        const QNetworkDatagram datagram = _socket->receiveDatagram();
        const std::optional<LauncherReply> reply = LauncherProtocol::decodeReply(datagram.data());
        if (!reply) {
            continue;
        }
        if (_commandPending && reply->sequence == _commandSequence) {
            _commandPending = false;
            _retryTimer.stop();
        }
        _lastReplyMs = QDateTime::currentMSecsSinceEpoch();
        _reply = *reply;
        _message = reply->message;
        _setReachable(true);
        emit stateChanged();
    }
}

void LauncherClient::_checkSilence()
{
    if (_reachable && QDateTime::currentMSecsSinceEpoch() - _lastReplyMs > kSilenceMs) {
        _setReachable(false);
        _message = tr("Keine Verbindung zum Starter-Dienst");
        emit stateChanged();
    }
}

void LauncherClient::_setReachable(bool reachable)
{
    _reachable = reachable;
}
