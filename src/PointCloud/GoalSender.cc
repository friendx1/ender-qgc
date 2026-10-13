#include "GoalSender.h"

#include <QtCore/QDateTime>
#include <QtNetwork/QHostAddress>

#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(GoalSenderLog, "PointCloud.GoalSender")

GoalSender::GoalSender(QObject* parent) : QObject(parent)
{
    const QString envHost = qEnvironmentVariable("ENDER_GOAL_HOST");
    _host = envHost.isEmpty() ? QString::fromLatin1(kDefaultHost) : envHost;

    // Start at a time based number: the vehicle ignores a repeated sequence, and a restarted app
    // must not reuse the number of its last goal
    _nextSequence = static_cast<quint32>(QDateTime::currentMSecsSinceEpoch() & 0xFFFFFFFF);

    _retryTimer.setInterval(kRetryIntervalMs);
    connect(&_retryTimer, &QTimer::timeout, this, &GoalSender::_retry);
}

void GoalSender::setHost(const QString& host)
{
    if (_host == host) {
        return;
    }
    _host = host;
    emit hostChanged();
}

void GoalSender::setPort(quint16 port)
{
    if (_port == port) {
        return;
    }
    _port = port;
    emit portChanged();
}

bool GoalSender::_ensureSocket()
{
    if (_socket) {
        return true;
    }
    auto* socket = new QUdpSocket(this);
    if (!socket->bind(QHostAddress::AnyIPv4, 0)) {
        qCWarning(GoalSenderLog) << "Cannot open UDP socket:" << socket->errorString();
        delete socket;
        return false;
    }
    _socket = socket;
    connect(_socket, &QUdpSocket::readyRead, this, &GoalSender::_readPendingDatagrams);
    return true;
}

void GoalSender::sendGoal(double x, double y, quint32 scanStampMs)
{
    if (_pending) {
        // A new goal replaces one that is still waiting for its answer
        _retryTimer.stop();
    }

    _request.sequence = _nextSequence++;
    _request.scanStampMs = scanStampMs;
    _request.x = static_cast<float>(x);
    _request.y = static_cast<float>(y);
    _datagram = GoalProtocol::encodeGoal(_request);
    _attempts = 0;

    if (!_ensureSocket()) {
        _finish(false, tr("Senden nicht möglich"));
        return;
    }
    if (QHostAddress(_host).isNull()) {
        _finish(false, tr("Ungültige Adresse: %1").arg(_host));
        return;
    }

    if (!_pending) {
        _pending = true;
        emit pendingChanged();
    }
    _resultText.clear();
    _resultAccepted = false;
    emit resultChanged();

    qCDebug(GoalSenderLog) << "Goal" << _request.sequence << "x" << _request.x << "y" << _request.y << "scan"
                           << _request.scanStampMs << "to" << _host << _port;
    _retry();
}

void GoalSender::_retry()
{
    if (!_pending) {
        _retryTimer.stop();
        return;
    }
    if (_attempts >= kMaxAttempts) {
        _finish(false, tr("Keine Antwort von der Drohne"));
        return;
    }
    _attempts++;
    if (!_sendDatagram()) {
        _finish(false, tr("Senden fehlgeschlagen"));
        return;
    }
    _retryTimer.start();
}

bool GoalSender::_sendDatagram()
{
    const qint64 written = _socket->writeDatagram(_datagram, QHostAddress(_host), _port);
    if (written != _datagram.size()) {
        qCWarning(GoalSenderLog) << "Send failed:" << _socket->errorString();
        return false;
    }
    return true;
}

void GoalSender::_readPendingDatagrams()
{
    while (_socket && _socket->hasPendingDatagrams()) {
        const QByteArray datagram = _socket->receiveDatagram().data();
        const std::optional<GoalAck> ack = GoalProtocol::decodeAck(datagram);
        if (!ack || !_pending || ack->sequence != _request.sequence) {
            continue;  // not for the goal we are waiting for
        }
        _finish(ack->status == GoalProtocol::Ok, _statusText(ack->status));
    }
}

void GoalSender::_finish(bool accepted, const QString& text)
{
    _retryTimer.stop();
    if (_pending) {
        _pending = false;
        emit pendingChanged();
    }
    _resultAccepted = accepted;
    _resultText = text;
    emit resultChanged();
    emit goalResult(accepted, text);
}

QString GoalSender::_statusText(quint8 status)
{
    switch (status) {
        case GoalProtocol::Ok:
            return tr("Ziel angenommen");
        case GoalProtocol::Stale:
            return tr("Abgelehnt: Scan zu alt");
        case GoalProtocol::TooFar:
            return tr("Abgelehnt: Ziel zu weit weg");
        case GoalProtocol::NoPose:
            return tr("Abgelehnt: keine Position der Drohne");
        case GoalProtocol::BadValue:
            return tr("Abgelehnt: ungültiger Wert");
        default:
            return tr("Abgelehnt (Status %1)").arg(status);
    }
}
