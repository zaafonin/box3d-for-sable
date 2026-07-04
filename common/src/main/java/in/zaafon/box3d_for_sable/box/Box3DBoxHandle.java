package in.zaafon.box3d_for_sable.box;

import dev.ryanhcode.sable.api.physics.object.box.BoxHandle;
import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import in.zaafon.box3d_for_sable.Box3DNative;
import org.jetbrains.annotations.ApiStatus;
import org.joml.Quaterniondc;
import org.joml.Vector3dc;

@ApiStatus.Internal
public record Box3DBoxHandle(long sceneHandle, int id, double[] poseCache) implements BoxHandle {

    public static Box3DBoxHandle create(final long sceneHandle, final Pose3dc pose, final Vector3dc halfExtents, final double mass) {
        final Vector3dc pos = pose.position();
        final Quaterniondc rot = pose.orientation();

        final int id = Box3DNative.nextBodyID();
        Box3DNative.createBox(sceneHandle, id, mass, halfExtents.x(), halfExtents.y(), halfExtents.z(), new double[]{pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()});
        return new Box3DBoxHandle(sceneHandle, id, new double[7]);
    }

    @Override
    public void readPose(final Pose3d dest) {
        Box3DNative.getPose(this.sceneHandle, this.id, this.poseCache);

        dest.position().set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        dest.orientation().set(this.poseCache[3], this.poseCache[4], this.poseCache[5], this.poseCache[6]);
    }

    @Override
    public void remove() {
        Box3DNative.removeBox(this.sceneHandle, this.id);
    }

    @Override
    public void wakeUp() {
        Box3DNative.wakeUpObject(this.sceneHandle, this.id);
    }

    @Override
    public int getRuntimeId() {
        return this.id;
    }
}
