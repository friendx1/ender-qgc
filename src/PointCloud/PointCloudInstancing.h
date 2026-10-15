#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QHash>
#include <QtCore/QPointer>
#include <QtGui/QColor>
#include <QtGui/QVector3D>
#include <QtQmlIntegration/QtQmlIntegration>
#include <QtQuick3D/QQuick3DInstancing>

#include "EpclFrameAssembler.h"
#include "MapReceiver.h"
#include "PointCloudReceiver.h"

/// Draws the latest LiDAR scan as one small cube per point, colored by height.
///
/// Scene units are centimeters. The sensor body frame (x forward, y left, z up) is mapped to the
/// Qt Quick 3D frame (x right, y up, forward = -z), so the vehicle flies toward -z on screen.
///
/// Persistence: with a poseSource and persistenceMs above zero the points of the last persistenceMs
/// stay visible. Each scan is turned into world coordinates with the pose of the vehicle at the stamp of the scan
/// and collected in a grid of 10 cm cells: a surface that is seen again only refreshes its cell, so the number of
/// points does not grow while the vehicle hovers. Cells older than persistenceMs (by the stamps of the scans)
/// drop out. The buffer is then in the world frame (worldMode) and the model that carries it needs no transform.
/// Without a pose the buffer holds the latest scan in the body frame, as before.
class PointCloudInstancing : public QQuick3DInstancing
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(PointCloudReceiver* receiver READ receiver WRITE setReceiver NOTIFY receiverChanged)
    Q_PROPERTY(int count READ count NOTIFY countChanged)
    Q_PROPERTY(bool frozen READ frozen WRITE setFrozen NOTIFY frozenChanged)
    Q_PROPERTY(int persistenceMs READ persistenceMs WRITE setPersistenceMs NOTIFY persistenceMsChanged)
    Q_PROPERTY(MapReceiver* poseSource READ poseSource WRITE setPoseSource NOTIFY poseSourceChanged)
    Q_PROPERTY(bool worldMode READ worldMode NOTIFY worldModeChanged)
    Q_PROPERTY(quint32 stampMs READ stampMs NOTIFY stampMsChanged)
    Q_PROPERTY(float pointSize READ pointSize WRITE setPointSize NOTIFY pointSizeChanged)
    Q_PROPERTY(float heightMin READ heightMin WRITE setHeightMin NOTIFY heightRangeChanged)
    Q_PROPERTY(float heightMax READ heightMax WRITE setHeightMax NOTIFY heightRangeChanged)

    friend class PointCloudInstancingTest;

public:
    explicit PointCloudInstancing(QQuick3DObject* parent = nullptr);

    PointCloudReceiver* receiver() const { return _receiver; }

    void setReceiver(PointCloudReceiver* receiver);

    /// Number of points drawn
    int count() const { return static_cast<int>(_voxels.isEmpty() ? _frame.points.size() : _voxels.size()); }

    /// How long points stay visible, in milliseconds of scan time. 0 shows only the latest scan.
    int persistenceMs() const { return _persistenceMs; }

    void setPersistenceMs(int persistenceMs);

    /// Where the pose of the vehicle comes from. Needed to keep points in the world.
    MapReceiver* poseSource() const { return _poseSource; }

    void setPoseSource(MapReceiver* poseSource);

    /// True while the buffer is in the world frame (points kept over time), false while it is the latest scan in
    /// the body frame
    bool worldMode() const { return !_voxels.isEmpty(); }

    /// While frozen, new scans are kept back and the displayed scan stays as it is. Unfreezing shows the latest scan.
    bool frozen() const { return _frozen; }

    void setFrozen(bool frozen);

    /// Stamp of the displayed scan, as sent by the vehicle
    quint32 stampMs() const { return _frame.stampMs; }

    /// Edge length of each point cube in centimeters
    float pointSize() const { return _pointSize; }

    void setPointSize(float pointSize);

    /// Heights in meters that map to the cold and the warm end of the color ramp
    float heightMin() const { return _heightMin; }

    void setHeightMin(float heightMin);

    float heightMax() const { return _heightMax; }

    void setHeightMax(float heightMax);

    /// Replaces the displayed scan
    void setFrame(const EpclFrame& frame);

    /// Body frame (meters) to scene position (centimeters)
    static QVector3D toScenePosition(const QVector3D& bodyMeters);

signals:
    void receiverChanged();
    void countChanged();
    void frozenChanged();
    void stampMsChanged();
    void persistenceMsChanged();
    void poseSourceChanged();
    void worldModeChanged();
    void pointSizeChanged();
    void heightRangeChanged();

protected:
    QByteArray getInstanceBuffer(int* instanceCount) override;

private:
    struct Voxel
    {
        QVector3D world;  ///< meters
        float colorHeight = 0.0f;  ///< height in the body frame, the same colors as without persistence
        quint32 stampMs = 0;
    };

    void _onFrameReceived();
    void _onPoseChanged();
    void _accumulate(const EpclFrame& frame);
    void _clearVoxels();
    QColor _colorForHeight(float heightMeters) const;

    QPointer<PointCloudReceiver> _receiver;
    QPointer<MapReceiver> _poseSource;
    EpclFrame _frame;
    QHash<quint64, Voxel> _voxels;
    quint32 _newestStampMs = 0;
    int _persistenceMs = 5000;
    QByteArray _instanceData;
    bool _dirty = false;
    bool _frozen = false;
    float _pointSize = 4.0f;
    float _heightMin = -1.5f;
    float _heightMax = 2.5f;
};
