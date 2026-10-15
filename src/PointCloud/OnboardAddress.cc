#include "OnboardAddress.h"

#include <QtCore/QSettings>

namespace {
constexpr char kKey[] = "onboardHost";

QSettings openSettings()
{
    return QSettings(QSettings::IniFormat, QSettings::UserScope, QStringLiteral("Ender"), QStringLiteral("EnderGCS"));
}
}  // namespace

OnboardAddress::OnboardAddress(QObject* parent) : QObject(parent)
{
    const QString envHost = qEnvironmentVariable("ENDER_GOAL_HOST").trimmed();
    const QString savedHost = openSettings().value(QLatin1String(kKey)).toString().trimmed();

    if (!envHost.isEmpty()) {
        _host = envHost;
    } else if (!savedHost.isEmpty()) {
        _host = savedHost;
    } else {
        _host = QString::fromLatin1(kDefaultHost);
    }
}

void OnboardAddress::setHost(const QString& host)
{
    const QString trimmed = host.trimmed();
    QSettings settings = openSettings();
    QString newHost;
    if (trimmed.isEmpty()) {
        settings.remove(QLatin1String(kKey));
        newHost = QString::fromLatin1(kDefaultHost);
    } else {
        settings.setValue(QLatin1String(kKey), trimmed);
        newHost = trimmed;
    }
    settings.sync();

    if (_host == newHost) {
        return;
    }
    _host = newHost;
    emit hostChanged();
}
