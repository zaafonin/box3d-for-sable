package in.zaafon.box3d_for_sable;

import dev.ryanhcode.sable.api.physics.constraint.ConstraintJointAxis;
import dev.ryanhcode.sable.api.physics.constraint.FixedConstraintHandle;
import dev.ryanhcode.sable.api.physics.constraint.FreeConstraintHandle;
import dev.ryanhcode.sable.api.physics.constraint.GenericConstraintHandle;
import dev.ryanhcode.sable.api.physics.constraint.RotaryConstraintHandle;
import org.jetbrains.annotations.ApiStatus;
import org.jetbrains.annotations.NotNull;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

@ApiStatus.Internal
abstract class Box3DConstraintHandle {
    protected final long sceneHandle;
    protected final long handle;
    private final double[] impulseCache = new double[6];

    Box3DConstraintHandle(final long sceneHandle, final long handle) {
        this.sceneHandle = sceneHandle;
        this.handle = handle;
    }

    public void getJointImpulses(final Vector3d linearImpulseDest, final Vector3d angularImpulseDest) {
        this.assertValid();
        Box3DNative.getConstraintImpulses(this.sceneHandle, this.handle, this.impulseCache);
        linearImpulseDest.set(this.impulseCache[0], this.impulseCache[1], this.impulseCache[2]);
        angularImpulseDest.set(this.impulseCache[3], this.impulseCache[4], this.impulseCache[5]);
    }

    public void setContactsEnabled(final boolean enabled) {
        this.assertValid();
        Box3DNative.setConstraintContactsEnabled(this.sceneHandle, this.handle, enabled);
    }

    public void setMotor(final ConstraintJointAxis axis, final double target, final double stiffness, final double damping, final boolean hasMaxForce, final double maxForce) {
        this.assertValid();
        Box3DNative.setConstraintMotor(this.sceneHandle, this.handle, axis.ordinal(), target, stiffness, damping, hasMaxForce, maxForce);
    }

    public void remove() {
        Box3DNative.removeConstraint(this.sceneHandle, this.handle);
    }

    public boolean isValid() {
        return Box3DNative.isConstraintValid(this.sceneHandle, this.handle);
    }

    protected void assertValid() {
        if (!this.isValid()) {
            throw new RuntimeException("Attempted to mutate an invalid constraint");
        }
    }
}

@ApiStatus.Internal
final class Box3DRotaryConstraintHandle extends Box3DConstraintHandle implements RotaryConstraintHandle {
    Box3DRotaryConstraintHandle(final long sceneHandle, final long handle) {
        super(sceneHandle, handle);
    }
}

@ApiStatus.Internal
final class Box3DFixedConstraintHandle extends Box3DConstraintHandle implements FixedConstraintHandle {
    Box3DFixedConstraintHandle(final long sceneHandle, final long handle) {
        super(sceneHandle, handle);
    }
}

@ApiStatus.Internal
final class Box3DFreeConstraintHandle extends Box3DConstraintHandle implements FreeConstraintHandle {
    Box3DFreeConstraintHandle(final long sceneHandle, final long handle) {
        super(sceneHandle, handle);
    }
}

@ApiStatus.Internal
final class Box3DGenericConstraintHandle extends Box3DConstraintHandle implements GenericConstraintHandle {
    private static final int FRAME_SIDE_FIRST = 0;
    private static final int FRAME_SIDE_SECOND = 1;

    Box3DGenericConstraintHandle(final long sceneHandle, final long handle) {
        super(sceneHandle, handle);
    }

    @Override
    public void setFrame1(final Vector3dc localPosition, final Quaterniondc localOrientation) {
        this.assertValid();
        Box3DNative.setConstraintFrame(
                this.sceneHandle,
                this.handle,
                FRAME_SIDE_FIRST,
                localPosition.x(),
                localPosition.y(),
                localPosition.z(),
                localOrientation.x(),
                localOrientation.y(),
                localOrientation.z(),
                localOrientation.w());
    }

    @Override
    public void setFrame2(final Vector3dc localPosition, final Quaterniondc localOrientation) {
        this.assertValid();
        Box3DNative.setConstraintFrame(
                this.sceneHandle,
                this.handle,
                FRAME_SIDE_SECOND,
                localPosition.x(),
                localPosition.y(),
                localPosition.z(),
                localOrientation.x(),
                localOrientation.y(),
                localOrientation.z(),
                localOrientation.w());
    }

    @Override
    public void setLimit(final ConstraintJointAxis axis, final double min, final double max) {
        this.assertValid();
        Box3DNative.setConstraintLimit(this.sceneHandle, this.handle, axis.ordinal(), min, max);
    }

    @Override
    public void lockAxes(final ConstraintJointAxis @NotNull... axes) {
        int mask = 0;
        for (final ConstraintJointAxis axis : axes) {
            final int bit = 1 << axis.ordinal();
            if ((mask & bit) != 0) {
                throw new RuntimeException("Duplicate axis: " + axis);
            }

            mask |= bit;
        }

        this.assertValid();
        Box3DNative.lockConstraintAxes(this.sceneHandle, this.handle, mask);
    }

}
