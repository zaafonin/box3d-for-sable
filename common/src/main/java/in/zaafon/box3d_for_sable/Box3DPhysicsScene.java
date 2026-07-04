package in.zaafon.box3d_for_sable;

import org.jetbrains.annotations.ApiStatus;

/**
 * A physics scene, stored natively in {@link Box3DNative}.
 */
@ApiStatus.Internal
public final class Box3DPhysicsScene {
    private final long handle;

    Box3DPhysicsScene(final long handle) {
        this.handle = handle;
    }

    long handle() {
        return this.handle;
    }
}
