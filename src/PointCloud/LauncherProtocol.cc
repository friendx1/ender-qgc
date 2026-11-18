#include "LauncherProtocol.h"

#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

QByteArray LauncherProtocol::encodeRequest(const QString& command, quint32 sequence, int stepIndex)
{
    QJsonObject request;
    request.insert(QStringLiteral("v"), kVersion);
    request.insert(QStringLiteral("seq"), static_cast<double>(sequence));
    request.insert(QStringLiteral("cmd"), command);
    if (stepIndex >= 0) {
        request.insert(QStringLiteral("step"), stepIndex);
    }
    return QJsonDocument(request).toJson(QJsonDocument::Compact);
}

std::optional<LauncherReply> LauncherProtocol::decodeReply(const QByteArray& datagram)
{
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(datagram, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        return std::nullopt;
    }
    const QJsonObject object = document.object();
    if (object.value(QStringLiteral("v")).toInt(-1) != kVersion || !object.value(QStringLiteral("seq")).isDouble()
        || !object.value(QStringLiteral("state")).isString() || !object.value(QStringLiteral("steps")).isArray()) {
        return std::nullopt;
    }

    LauncherReply reply;
    reply.sequence = static_cast<quint32>(object.value(QStringLiteral("seq")).toDouble());
    reply.profile = object.value(QStringLiteral("profile")).toString();
    reply.state = object.value(QStringLiteral("state")).toString();
    reply.message = object.value(QStringLiteral("message")).toString();

    const QJsonArray steps = object.value(QStringLiteral("steps")).toArray();
    for (const QJsonValue& value : steps) {
        const QJsonObject step = value.toObject();
        LauncherStepInfo info;
        info.name = step.value(QStringLiteral("name")).toString();
        info.state = step.value(QStringLiteral("state")).toString();
        info.detail = step.value(QStringLiteral("detail")).toString();
        info.canStop = step.value(QStringLiteral("can_stop")).toBool();
        reply.steps.append(info);
    }
    return reply;
}
