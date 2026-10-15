#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QList>
#include <QtCore/QPointer>
#include <QtGui/QColor>
#include <QtGui/QVector3D>
#include <QtQmlIntegration/QtQmlIntegration>
#include <QtQuick3D/QQuick3DInstancing>

#include "MapReceiver.h"

/// Draws a planned trajectory (SUPER exp_traj or backup_traj) as a chain of small cubes along its lines.
///
/// The lines are in the world frame, like the obstacle map, so the model that carries this instancing sits in the
/// world scene without any transform. The buffer is rebuilt when a new trajectory arrives.
class TrajectoryInstancing : public QQuick3DInstancing
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(MapReceiver* receiver READ receiver WRITE setReceiver NOTIFY receiverChanged)
    Q_PROPERTY(int kind READ kind WRITE setKind NOTIFY kindChanged)
    Q_PROPERTY(int count READ count NOTIFY countChanged)
    Q_PROPERTY(float pointSize READ pointSize WRITE setPointSize NOTIFY pointSizeChanged)
    Q_PROPERTY(QColor color READ color WRITE setColor NOTIFY colorChanged)

    friend class TrajectoryInstancingTest;

public:
    static constexpr float kSpacingMeters = 0.04f;  ///< distance between the cubes along a line
    static constexpr int kMaxCubes = 8000;

    explicit TrajectoryInstancing(QQuick3DObject* parent = nullptr);

    MapReceiver* receiver() const { return _receiver; }

    void setReceiver(MapReceiver* receiver);

    /// Which trajectory to draw: EmapProtocol::kKindExp or kKindBackup
    int kind() const { return _kind; }

    void setKind(int kind);

    /// Number of cubes
    int count() const { return static_cast<int>(_positions.size()); }

    /// Edge length of each cube in centimeters
    float pointSize() const { return _pointSize; }

    void setPointSize(float pointSize);

    QColor color() const { return _color; }

    void setColor(const QColor& color);

    /// Replaces the displayed trajectory
    void setLines(const QList<QList<QVector3D>>& lines);

    /// Points along the lines, at most spacingMeters apart (more if that would give more than maxPoints in total).
    /// Every point of a line is kept, so corners stay corners, and so is its last point.
    static QList<QVector3D> resample(const QList<QList<QVector3D>>& lines, float spacingMeters, int maxPoints);

signals:
    void receiverChanged();
    void kindChanged();
    void countChanged();
    void pointSizeChanged();
    void colorChanged();

protected:
    QByteArray getInstanceBuffer(int* instanceCount) override;

private:
    void _onTrajectoryChanged(quint8 kind);
    void _reload();

    QPointer<MapReceiver> _receiver;
    int _kind = EmapProtocol::kKindExp;
    QList<QVector3D> _positions;  ///< world meters
    QByteArray _instanceData;
    bool _dirty = false;
    float _pointSize = 6.0f;
    QColor _color = QColor(QStringLiteral("#39ff14"));
};
