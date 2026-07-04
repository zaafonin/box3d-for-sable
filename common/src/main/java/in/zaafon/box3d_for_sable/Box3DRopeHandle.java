package in.zaafon.box3d_for_sable;

import dev.ryanhcode.sable.api.physics.object.rope.RopeHandle;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import org.jetbrains.annotations.ApiStatus;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import java.util.List;

@ApiStatus.Internal
public record Box3DRopeHandle(long sceneHandle, long handle) implements RopeHandle {

    public static Box3DRopeHandle create(final long sceneHandle, final double pointRadius, final List<Vector3d> points) {
        if (points.size() < 2) {
            throw new IllegalArgumentException("A rope needs at least two points");
        }

        final double[] coordinates = new double[points.size() * 3];

        for (int i = 0; i < points.size(); i++) {
            final Vector3d point = points.get(i);
            coordinates[i * 3] = point.x;
            coordinates[i * 3 + 1] = point.y;
            coordinates[i * 3 + 2] = point.z;
        }

        final long handle = Box3DNative.createRope(sceneHandle, pointRadius, points.get(0).distance(points.get(1)), coordinates, points.size());
        if (handle == 0L) {
            throw new IllegalStateException("Box3D failed to create a rope");
        }

        return new Box3DRopeHandle(sceneHandle, handle);
    }

    @Override
    public void readPose(final List<Vector3d> dest) {
        final double[] coordinates = Box3DNative.queryRope(this.sceneHandle, this.handle);
        final int count = Math.min(dest.size(), coordinates.length / 3);

        for (int i = 0; i < count; i++) {
            dest.get(i).set(coordinates[i * 3], coordinates[i * 3 + 1], coordinates[i * 3 + 2]);
        }
    }

    @Override
    public void remove() {
        Box3DNative.removeRope(this.sceneHandle, this.handle);
    }

    @Override
    public void setFirstSegmentLength(final double length) {
        Box3DNative.setRopeFirstSegmentLength(this.sceneHandle, this.handle, length);
    }

    @Override
    public void removeFirstPoint() {
        Box3DNative.removeRopePointAtStart(this.sceneHandle, this.handle);
    }

    @Override
    public void addPoint(final Vector3dc position) {
        Box3DNative.addRopePointAtStart(this.sceneHandle, this.handle, position.x(), position.y(), position.z());
    }

    @Override
    public void setAttachment(final AttachmentPoint attachmentPoint, final Vector3dc location, final ServerSubLevel subLevel) {
        Box3DNative.setRopeAttachment(
                this.sceneHandle,
                this.handle,
                subLevel == null ? -1 : Box3DNative.getID(subLevel),
                location.x(),
                location.y(),
                location.z(),
                attachmentPoint == AttachmentPoint.END);
    }

    @Override
    public void wakeUp() {
        Box3DNative.wakeUpRope(this.sceneHandle, this.handle);
    }
}
