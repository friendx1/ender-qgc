#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtQmlIntegration/QtQmlIntegration>

/// Address of the onboard computer, which the flight goals, the pose, the obstacle map and the trajectories use.
///
/// The radio has no way to set an environment variable, so the address can be typed in the point cloud view and is
/// stored. The environment variable ENDER_GOAL_HOST (desktop builds) wins over the stored address when the app
/// starts, the default 192.168.144.50 is the onboard computer in the Siyi network.
class OnboardAddress : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY hostChanged)

public:
    static constexpr const char* kDefaultHost = "192.168.144.50";

    explicit OnboardAddress(QObject* parent = nullptr);

    QString host() const { return _host; }

    /// Sets and stores the address. Surrounding blanks are cut, an empty text goes back to the default.
    void setHost(const QString& host);

signals:
    void hostChanged();

private:
    QString _host;
};
