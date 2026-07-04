package in.zaafon.box3d_for_sable.collider;

import dev.ryanhcode.sable.api.physics.collider.VoxelColliderData;
import in.zaafon.box3d_for_sable.Box3DNative;
import org.joml.Vector3dc;

/**
 * Represents a block physics data entry in the Box3D backend.
 *
 * @param handle the internal integer handle of the block physics data entry
 */
public record Box3DVoxelColliderData(int handle) implements VoxelColliderData {
    public static final Box3DVoxelColliderData EMPTY = new Box3DVoxelColliderData(-1);

    @Override
    public void addBox(final Vector3dc min, final Vector3dc max) {
        Box3DNative.addVoxelColliderBox(this.handle, new double[]{min.x(), min.y(), min.z(), max.x(), max.y(), max.z()});
    }

    @Override
    public void clearBoxes() {
        Box3DNative.clearVoxelColliderBoxes(this.handle);
    }
}
