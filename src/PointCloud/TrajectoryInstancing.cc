#include "TrajectoryInstancing.h"

#include <algorithm>
#include <cmath>

#include "PointCloudInstancing.h"

namespace {
constexpr float kCubeEdgeUnits = 100.0f;  // Edge length of the built-in #Cube primitive
}

TrajectoryInstancing::TrajectoryInstancing(QQuick3DObject* parent) : QQuick3DInstancing(parent) {}

void TrajectoryInstancing::setReceiver(MapReceiver* receiver)
{
    if (_receiver == receiver) {
        return;
    }
    if (_receiver) {
        disconnect(_receiver, nullptr, this, nullptr);
    }
    _receiver = receiver;
    if (_receiver) {
        connect(_receiver, &MapReceiver::trajectoryChanged, this, &TrajectoryInstancing::_onTrajectoryChanged);
    }
    _reload();
    emit receiverChanged();
}

void TrajectoryInstancing::setKind(int kind)
{
    if (_kind == kind) {
        return;
    }
    _kind = kind;
    _reload();
    emit kindChanged();
}

void TrajectoryInstancing::setPointSize(float pointSize)
{
    if (qFuzzyCompare(_pointSize, pointSize) || pointSize <= 0.0f) {
        return;
    }
    _pointSize = pointSize;
    _dirty = true;
    markDirty();
    emit pointSizeChanged();
}

void TrajectoryInstancing::setColor(const QColor& color)
{
    if (_color == color) {
        return;
    }
    _color = color;
    _dirty = true;
    markDirty();
    emit colorChanged();
}

void TrajectoryInstancing::setLines(const QList<QList<QVector3D>>& lines)
{
    _positions = resample(lines, kSpacingMeters, kMaxCubes);
    _dirty = true;
    markDirty();
    emit countChanged();
}

void TrajectoryInstancing::_onTrajectoryChanged(quint8 kind)
{
    if (kind == _kind) {
        _reload();
    }
}

void TrajectoryInstancing::_reload()
{
    if (_receiver && _kind >= 0 && _kind < EmapProtocol::kKindCount) {
        setLines(_receiver->trajectory(static_cast<quint8>(_kind)));
    } else {
        setLines({});
    }
}

QList<QVector3D> TrajectoryInstancing::resample(const QList<QList<QVector3D>>& lines, float spacingMeters,
                                                int maxPoints)
{
    float totalLength = 0.0f;
    for (const QList<QVector3D>& line : lines) {
        for (qsizetype i = 1; i < line.size(); ++i) {
            totalLength += line[i].distanceToPoint(line[i - 1]);
        }
    }
    // Stay below the cap, a few extra points per line for its corners and its end
    const float spacing = std::max(spacingMeters, maxPoints > 0 ? totalLength / float(maxPoints) : spacingMeters);
    if (spacing <= 0.0f) {
        return {};
    }

    QList<QVector3D> out;
    for (const QList<QVector3D>& line : lines) {
        for (qsizetype i = 0; i + 1 < line.size(); ++i) {
            const QVector3D start = line[i];
            const QVector3D end = line[i + 1];
            const int steps = std::max(1, static_cast<int>(std::ceil(start.distanceToPoint(end) / spacing)));
            for (int s = 0; s < steps; ++s) {
                out.append(start + (end - start) * (float(s) / float(steps)));
            }
        }
        if (!line.isEmpty()) {
            out.append(line.last());
        }
    }
    return out;
}

QByteArray TrajectoryInstancing::getInstanceBuffer(int* instanceCount)
{
    if (_dirty) {
        _instanceData.clear();
        _instanceData.reserve(static_cast<qsizetype>(_positions.size() * sizeof(InstanceTableEntry)));

        const float cubeScale = _pointSize / kCubeEdgeUnits;
        const QVector3D scale(cubeScale, cubeScale, cubeScale);
        for (const QVector3D& point : std::as_const(_positions)) {
            const InstanceTableEntry entry =
                calculateTableEntry(PointCloudInstancing::toScenePosition(point), scale, QVector3D(0, 0, 0), _color);
            _instanceData.append(reinterpret_cast<const char*>(&entry), sizeof(entry));
        }
        _dirty = false;
    }

    if (instanceCount) {
        *instanceCount = static_cast<int>(_positions.size());
    }
    return _instanceData;
}
