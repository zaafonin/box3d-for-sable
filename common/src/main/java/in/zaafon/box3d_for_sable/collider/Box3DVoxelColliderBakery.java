package in.zaafon.box3d_for_sable.collider;

import dev.ryanhcode.sable.api.block.BlockSubLevelCollisionShape;
import dev.ryanhcode.sable.api.block.BlockWithSubLevelCollisionCallback;
import dev.ryanhcode.sable.api.physics.callback.BlockSubLevelCollisionCallback;
import dev.ryanhcode.sable.api.physics.collider.SableCollisionContext;
import dev.ryanhcode.sable.companion.math.JOMLConversion;
import dev.ryanhcode.sable.physics.chunk.VoxelNeighborhoodState;
import dev.ryanhcode.sable.physics.config.block_properties.PhysicsBlockPropertyHelper;
import in.zaafon.box3d_for_sable.Box3DNative;
import net.minecraft.Util;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.BlockGetter;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.joml.Vector3d;

import java.util.Objects;
import java.util.function.Function;

/**
 * A collider bakery that creates and caches collision shapes for blocks in Box3D.
 */
public class Box3DVoxelColliderBakery {
    private final @NotNull PhysicsColliderBlockGetter level;
    private final Function<BlockState, Box3DVoxelColliderData> blockPhysicsDataBuilder = Util.memoize(this::buildPhysicsDataForBlock);

    public Box3DVoxelColliderBakery(@NotNull final BlockGetter blockGetter) {
        this.level = new PhysicsColliderBlockGetter(blockGetter);
    }

    public @NotNull BlockGetter getLevel() {
        return this.level;
    }

    private @NotNull Box3DVoxelColliderData buildPhysicsDataForBlock(final BlockState childState) {
        final boolean liquid = VoxelNeighborhoodState.isLiquid(childState);

        final double friction = PhysicsBlockPropertyHelper.getFriction(childState);
        final double volume = PhysicsBlockPropertyHelper.getVolume(childState);
        final double restitution = PhysicsBlockPropertyHelper.getRestitution(childState);
        final BlockSubLevelCollisionCallback callback = BlockWithSubLevelCollisionCallback.sable$getCallback(childState);
        final Box3DVoxelColliderData entry = Box3DNative.createVoxelColliderEntry(friction, volume, restitution, liquid, callback);

        if (liquid) {
            entry.addBox(JOMLConversion.ZERO, new Vector3d(1.0, 1.0, 1.0));
            return entry;
        }

        final VoxelShape shape;

        this.level.setup(childState);
        if (childState.getBlock() instanceof final BlockSubLevelCollisionShape extension) {
            shape = extension.getSubLevelCollisionShape(this.level, childState);
        } else {
            shape = childState.getCollisionShape(this.level, BlockPos.ZERO, SableCollisionContext.get());
        }
        this.level.setup(Blocks.AIR.defaultBlockState());

        if (shape.isEmpty()) {
            return Box3DVoxelColliderData.EMPTY;
        }

        shape.forAllBoxes((minX, minY, minZ, maxX, maxY, maxZ) -> entry.addBox(
                new Vector3d(Math.max(minX, 0.0), Math.max(minY, 0.0), Math.max(minZ, 0.0)),
                new Vector3d(Math.min(maxX, 1.0), Math.min(maxY, 1.0), Math.min(maxZ, 1.0))
        ));

        return entry;
    }

    public @Nullable Box3DVoxelColliderData getPhysicsDataForBlock(final BlockState state) {
        final Box3DVoxelColliderData data = this.blockPhysicsDataBuilder.apply(Objects.requireNonNull(state, "state"));
        return data == Box3DVoxelColliderData.EMPTY ? null : data;
    }
}
