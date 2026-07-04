package in.zaafon.box3d_for_sable;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.constraint.ConstraintJointAxis;
import dev.ryanhcode.sable.api.physics.constraint.FixedConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.constraint.FreeConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.constraint.GenericConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.constraint.PhysicsConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.constraint.PhysicsConstraintHandle;
import dev.ryanhcode.sable.api.physics.constraint.RotaryConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.mass.MassTracker;
import dev.ryanhcode.sable.api.physics.object.box.BoxHandle;
import dev.ryanhcode.sable.api.physics.object.box.BoxPhysicsObject;
import dev.ryanhcode.sable.api.physics.object.rope.RopeHandle;
import dev.ryanhcode.sable.api.physics.object.rope.RopePhysicsObject;
import dev.ryanhcode.sable.api.sublevel.KinematicContraption;
import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.companion.math.BoundingBox3i;
import dev.ryanhcode.sable.companion.math.BoundingBox3ic;
import dev.ryanhcode.sable.companion.math.JOMLConversion;
import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.physics.chunk.VoxelNeighborhoodState;
import dev.ryanhcode.sable.physics.config.PhysicsConfigData;
import in.zaafon.box3d_for_sable.box.Box3DBoxHandle;
import in.zaafon.box3d_for_sable.collider.Box3DVoxelColliderBakery;
import in.zaafon.box3d_for_sable.collider.Box3DVoxelColliderData;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import dev.ryanhcode.sable.sublevel.plot.LevelPlot;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import dev.ryanhcode.sable.util.LevelAccelerator;
import dev.ryanhcode.sable.util.SableMathUtils;
import it.unimi.dsi.fastutil.ints.Int2ObjectArrayMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2LongOpenHashMap;
import it.unimi.dsi.fastutil.longs.Long2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.Object2ObjectMap;
import it.unimi.dsi.fastutil.objects.Object2ObjectOpenHashMap;
import it.unimi.dsi.fastutil.objects.ReferenceArrayList;
import it.unimi.dsi.fastutil.objects.ReferenceList;
import net.minecraft.CrashReport;
import net.minecraft.CrashReportCategory;
import net.minecraft.ReportedException;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.core.SectionPos;
import net.minecraft.core.particles.BlockParticleOption;
import net.minecraft.core.particles.ParticleTypes;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.sounds.SoundSource;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.SoundType;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.chunk.LevelChunk;
import net.minecraft.world.level.chunk.LevelChunkSection;
import net.minecraft.world.phys.Vec3;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.joml.Quaterniond;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

/**
 * Experimental {@link PhysicsPipeline} implementation for Box3D.
 */
public class Box3DPhysicsPipeline implements PhysicsPipeline {

    private static final double DISTANCE_THRESHOLD = 1e-7;
    private static final double ANGULAR_THRESHOLD = 1e-7;

    private final ServerLevel level;
    private final LevelAccelerator accelerator;
    private final Box3DVoxelColliderBakery colliderBakery;
    private final Int2ObjectMap<ServerSubLevel> activeSubLevels = new Int2ObjectArrayMap<>();
    private final Object2ObjectMap<KinematicContraption, TrackedKinematicContraption> activeContraptions = new Object2ObjectOpenHashMap<>();
    private final Long2LongOpenHashMap recentCollisions = new Long2LongOpenHashMap();
    private final ReferenceList<PhysicsPipelineBody> queuedWakeUps = new ReferenceArrayList<>();
    private final double[] poseCache;
    private final long[] debugStatsCache = new long[10];
    private Box3DPhysicsScene scene;

    public Box3DPhysicsPipeline(final ServerLevel level) {
        this.level = level;
        this.accelerator = new LevelAccelerator(level);
        this.colliderBakery = new Box3DVoxelColliderBakery(this.accelerator);
        this.recentCollisions.defaultReturnValue(-1);
        this.poseCache = new double[7];
    }

    private static int packBlockState(final VoxelNeighborhoodState state, final int colliderID) {
        return ((int) state.byteRepresentation()) | (colliderID << 16);
    }

    long getSceneHandle() {
        if (this.scene == null) {
            throw new IllegalStateException("Physics scene is not initialized");
        }
        return this.scene.handle();
    }

    @Override
    public void init(@Nullable final Vector3dc gravity, final double universalDrag) {
        try {
            this.scene = new Box3DPhysicsScene(Box3DNative.initialize(gravity.x(), gravity.y(), gravity.z(), universalDrag));
        } catch (final UnsatisfiedLinkError e) {
            Sable.LOGGER.error("Sable has failed to link with the natives for its Box3D pipeline. Please report with system details to " + Sable.ISSUE_TRACKER_URL, e);
            final CrashReport crashReport = CrashReport.forThrowable(e.getCause(), "Sable linking with Box3D natives");
            final CrashReportCategory category = crashReport.addCategory("Natives");
            category.setDetail("Name", Box3DNative.NATIVE_NAME);
            throw new ReportedException(crashReport);
        }
    }

    @Override
    public void dispose() {
        if (this.scene != null) {
            Box3DNative.dispose(this.scene.handle());
            this.scene = null;
        }
    }

    @Override
    public void prePhysicsTicks() {
        Box3DNative.tick(this.scene.handle(), 1.0 / 20.0);
    }

    @Override
    public void physicsTick(final double timeStep) {
        this.updateContraptionPoses();
        Box3DNative.step(this.scene.handle(), timeStep);

        for (final PhysicsPipelineBody queuedWakeUp : this.queuedWakeUps) {
            if (queuedWakeUp.isRemoved()) {
                continue;
            }

            Box3DNative.wakeUpObject(this.scene.handle(), queuedWakeUp.getRuntimeId());
        }

        this.queuedWakeUps.clear();
    }

    @Override
    public void postPhysicsTicks() {
        this.processCollisionEffects();
        this.logDebugStats();
    }

    @Override
    public void tick() {
        this.accelerator.clearCache();
    }

    @Override
    public void add(final ServerSubLevel subLevel, final Pose3dc pose) {
        this.assertBodyValid(subLevel);
        final Vector3dc pos = pose.position();
        final Quaterniondc rot = pose.orientation();

        final int id = Box3DNative.getID(subLevel);
        Box3DNative.createSubLevel(this.scene.handle(), id, new double[]{pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()});

        subLevel.updateMergedMassData(1.0f);
        final Vector3dc centerOfMass = subLevel.getMassTracker().getCenterOfMass();

        if (centerOfMass != null) {
            subLevel.logicalPose().rotationPoint().set(centerOfMass);
            this.onStatsChanged(subLevel);
        }

        this.activeSubLevels.put(Box3DNative.getID(subLevel), subLevel);
    }

    @Override
    public void remove(final ServerSubLevel subLevel) {
        Box3DNative.removeSubLevel(this.scene.handle(), Box3DNative.getID(subLevel));
        this.activeSubLevels.remove(Box3DNative.getID(subLevel));
    }

    @Override
    public void add(final KinematicContraption contraption) {
        if (this.activeContraptions.containsKey(contraption)) {
            throw new IllegalStateException("Contraption " + contraption + " is already present in pipeline");
        }

        final int id = this.getNextRuntimeID();
        this.activeContraptions.put(contraption, new TrackedKinematicContraption(
                new Vector3d(Double.POSITIVE_INFINITY),
                new Quaterniond(),
                new Vector3d(Double.POSITIVE_INFINITY),
                new Vector3d(Double.POSITIVE_INFINITY),
                id));

        final Vector3dc worldPosition = contraption.sable$getPosition();
        final SubLevel mountSubLevel = Sable.HELPER.getContaining(this.level, worldPosition);
        final int mountId = mountSubLevel != null ? Box3DNative.getID((ServerSubLevel) mountSubLevel) : -1;
        final Vector3dc parentCenterOfMass = mountSubLevel != null ? ((ServerSubLevel) mountSubLevel).getMassTracker().getCenterOfMass() : JOMLConversion.ZERO;

        final BoundingBox3i localBounds = new BoundingBox3i();
        contraption.sable$getLocalBounds(localBounds);

        final Vector3d pos = new Vector3d(worldPosition).sub(parentCenterOfMass);
        final Quaterniond rot = contraption.sable$getOrientation();
        final double[] pose = {pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()};

        Box3DNative.createKinematicContraption(this.scene.handle(), mountId, id, pose);

        record UploadingContraptionChunk(int[] data) {
        }
        final Long2ObjectMap<UploadingContraptionChunk> chunks = new Long2ObjectOpenHashMap<>();

        final BlockPos.MutableBlockPos blockPos = new BlockPos.MutableBlockPos();
        for (int x = localBounds.minX(); x <= localBounds.maxX(); x++) {
            for (int z = localBounds.minZ(); z <= localBounds.maxZ(); z++) {
                for (int y = localBounds.minY(); y <= localBounds.maxY(); y++) {
                    final BlockState blockState = contraption.sable$blockGetter().getBlockState(blockPos.set(x, y, z));

                    if (blockState.isAir()) {
                        continue;
                    }

                    final SectionPos sectionPos = SectionPos.of(blockPos);
                    final UploadingContraptionChunk chunk = chunks.computeIfAbsent(sectionPos.asLong(), longPos -> new UploadingContraptionChunk(new int[LevelChunkSection.SECTION_SIZE]));

                    final VoxelNeighborhoodState state = VoxelNeighborhoodState.CORNER;
                    final Box3DVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(blockState);

                    final int index = (x & 15) + ((z & 15) << 4) + ((y & 15) << 8);

                    final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
                    chunk.data[index] = packBlockState(state, colliderValue);
                }
            }
        }

        if (contraption.sable$shouldCollide()) {
            for (final Long2ObjectMap.Entry<UploadingContraptionChunk> entry : chunks.long2ObjectEntrySet()) {
                final SectionPos sectionPos = SectionPos.of(entry.getLongKey());
                final UploadingContraptionChunk chunk = entry.getValue();
                Box3DNative.addKinematicContraptionChunkSection(this.scene.handle(), id, sectionPos.x(), sectionPos.y(), sectionPos.z(), chunk.data());
            }
        }

        this.updateContraptionPose(contraption, 1.0f);
        Box3DNative.setLocalBounds(this.scene.handle(), id, localBounds.minX, localBounds.minY, localBounds.minZ, localBounds.maxX, localBounds.maxY, localBounds.maxZ);
    }

    @Override
    public void remove(final KinematicContraption contraption) {
        final TrackedKinematicContraption removed = this.activeContraptions.remove(contraption);

        if (removed == null) {
            return;
        }

        Box3DNative.removeKinematicContraption(this.scene.handle(), removed.id());
    }

    @Override
    public Pose3d readPose(final ServerSubLevel subLevel, final Pose3d dest) {
        this.assertBodyValid(subLevel);
        Box3DNative.getPose(this.scene.handle(), Box3DNative.getID(subLevel), this.poseCache);

        dest.position().set(this.poseCache[0], this.poseCache[1], this.poseCache[2]);
        dest.orientation().set(this.poseCache[3], this.poseCache[4], this.poseCache[5], this.poseCache[6]);

        return dest;
    }

    @Override
    public RopeHandle addRope(final RopePhysicsObject rope) {
        return Box3DRopeHandle.create(this.scene.handle(), rope.getCollisionRadius(), rope.getPoints());
    }

    @Override
    public BoxHandle addBox(final BoxPhysicsObject box) {
        return Box3DBoxHandle.create(this.scene.handle(), box.getPose(), box.getHalfExtents(), box.getMass());
    }

    @Override
    public void handleChunkSectionAddition(final LevelChunkSection section, final int x, final int y, final int z, final boolean uploadDataIfGlobal) {
        this.accelerator.clearCache();

        final int[] array = new int[LevelChunkSection.SECTION_SIZE];
        final SectionPos sectionPos = SectionPos.of(x, y, z);

        if (!section.hasOnlyAir()) {
            final LevelChunk chunk = this.accelerator.getChunk(x, z);

            for (int bx = 0; bx < 16; bx++) {
                for (int bz = 0; bz < 16; bz++) {
                    for (int by = 0; by < 16; by++) {
                        final BlockPos globalPos = new BlockPos(bx, by, bz).offset(sectionPos.minBlockX(), sectionPos.minBlockY(), sectionPos.minBlockZ());
                        final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, globalPos, chunk);
                        final Box3DVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(this.accelerator.getBlockState(globalPos));

                        final int index = bx + (bz << 4) + (by << 8);

                        final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
                        array[index] = packBlockState(state, colliderValue);
                    }
                }
            }
        }

        final LevelPlot plot = SubLevelContainer.getContainer(this.level).getPlot(x, z);
        final boolean global = plot == null;
        int id = -1;

        if (plot != null) {
            // Rapier can keep plot chunks in its shared level chunk map. Box3D needs
            // body-owned shapes, so every plot chunk upload must be attached here.
            id = Box3DNative.getID((ServerSubLevel) plot.getSubLevel());
        }
        Box3DNative.addChunk(this.scene.handle(), x, y, z, array, global, id);
    }

    private void logDebugStats() {
        if (!Box3DForSableConfig.values(this.level).debugStats() || this.scene == null || this.level.getGameTime() % 100L != 0L) {
            return;
        }

        Box3DNative.getDebugStats(this.scene.handle(), this.debugStatsCache);
        Sable.LOGGER.info(
                "Box3D stats: trackedBodies={}, bodyChunks={}, bodyShapes={}, globalChunks={}, globalShapes={}, contacts={}, awakeContacts={}, nativeBodies={}, nativeShapes={}, pendingHits={}",
                this.debugStatsCache[0],
                this.debugStatsCache[1],
                this.debugStatsCache[2],
                this.debugStatsCache[3],
                this.debugStatsCache[4],
                this.debugStatsCache[5],
                this.debugStatsCache[6],
                this.debugStatsCache[7],
                this.debugStatsCache[8],
                this.debugStatsCache[9]);
    }

    @Override
    public void handleChunkSectionRemoval(final int x, final int y, final int z) {
        Box3DNative.removeChunk(this.scene.handle(), x, y, z, !SubLevelContainer.getContainer(this.level).inBounds(x, z));
    }

    @Override
    public void handleBlockChange(final SectionPos sectionPos, final LevelChunkSection chunk, int x, int y, int z, final BlockState oldState, final BlockState newState) {
        x = (sectionPos.x() << 4) + x;
        y = (sectionPos.y() << 4) + y;
        z = (sectionPos.z() << 4) + z;

        final BlockPos globalBlockPos = new BlockPos(x, y, z);

        for (final Direction dir : Direction.values()) {
            final BlockPos pos = globalBlockPos.relative(dir);
            final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, pos, null);
            final Box3DVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(this.level.getBlockState(pos));

            final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
            Box3DNative.changeBlock(this.scene.handle(), pos.getX(), pos.getY(), pos.getZ(), packBlockState(state, colliderValue));
        }

        final VoxelNeighborhoodState state = VoxelNeighborhoodState.getState(this.accelerator, globalBlockPos, null);
        final Box3DVoxelColliderData colliderData = this.colliderBakery.getPhysicsDataForBlock(newState);

        final int colliderValue = colliderData == null ? 0 : colliderData.handle() + 1;
        Box3DNative.changeBlock(this.scene.handle(), x, y, z, packBlockState(state, colliderValue));
    }

    @Override
    public void onStatsChanged(@NotNull final ServerSubLevel subLevel) {
        this.assertBodyValid(subLevel);

        final BoundingBox3ic plotBounds = subLevel.getPlot().getBoundingBox();
        final int id = Box3DNative.getID(subLevel);

        final Vector3dc centerOfMass = subLevel.getMassTracker().getCenterOfMass();
        if (centerOfMass != null) {
            Box3DNative.setCenterOfMass(this.scene.handle(), id, centerOfMass.x(), centerOfMass.y(), centerOfMass.z());
            Box3DNative.setMassPropertiesFrom(this.scene.handle(), id, subLevel.getMassTracker());
        }

        Box3DNative.setLocalBounds(this.scene.handle(), id, plotBounds.minX(), plotBounds.minY(), plotBounds.minZ(), plotBounds.maxX(), plotBounds.maxY(), plotBounds.maxZ());
    }

    @Override
    public void teleport(final PhysicsPipelineBody body, final Vector3dc position, final Quaterniondc orientation) {
        this.assertBodyValid(body);

        Box3DNative.teleportObject(this.scene.handle(), Box3DNative.getID(body), position.x(), position.y(), position.z(), orientation.x(), orientation.y(), orientation.z(), orientation.w());
        if (body instanceof final ServerSubLevel subLevel) {
            subLevel.logicalPose().position().set(position);
            subLevel.logicalPose().orientation().set(orientation);
        }
    }

    @Override
    public void applyImpulse(final PhysicsPipelineBody body, final Vector3dc position, final Vector3dc force) {
        this.assertBodyValid(body);

        final Vector3dc centerOfMass = body.getMassTracker().getCenterOfMass();
        Box3DNative.applyForce(this.scene.handle(), Box3DNative.getID(body), position.x() - centerOfMass.x(), position.y() - centerOfMass.y(), position.z() - centerOfMass.z(), force.x(), force.y(), force.z(), true);
    }

    @Override
    public void applyLinearAndAngularImpulse(final PhysicsPipelineBody body, final Vector3dc force, final Vector3dc torque, final boolean wakeUp) {
        this.assertBodyValid(body);
        Box3DNative.applyForceAndTorque(this.scene.handle(), Box3DNative.getID(body), force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z(), wakeUp);
    }

    @Override
    public void addLinearAndAngularVelocity(final PhysicsPipelineBody body, final Vector3dc linearVelocity, final Vector3dc angularVelocity) {
        this.assertBodyValid(body);
        Box3DNative.addLinearAngularVelocities(this.scene.handle(), Box3DNative.getID(body), linearVelocity.x(), linearVelocity.y(), linearVelocity.z(), angularVelocity.x(), angularVelocity.y(), angularVelocity.z(), true);
    }

    @Override
    public Vector3d getLinearVelocity(final PhysicsPipelineBody body, final Vector3d dest) {
        this.assertBodyValid(body);
        Box3DNative.getLinearVelocity(this.scene.handle(), Box3DNative.getID(body), this.poseCache);
        return dest.set(this.poseCache);
    }

    @Override
    public Vector3d getAngularVelocity(final PhysicsPipelineBody body, final Vector3d dest) {
        this.assertBodyValid(body);
        Box3DNative.getAngularVelocity(this.scene.handle(), Box3DNative.getID(body), this.poseCache);
        return dest.set(this.poseCache);
    }

    @Override
    public void wakeUp(final PhysicsPipelineBody body) {
        this.assertBodyValid(body);

        if (!SubLevelPhysicsSystem.IN_PHYSICS_STEP) {
            Box3DNative.wakeUpObject(this.scene.handle(), Box3DNative.getID(body));
        } else {
            this.queuedWakeUps.add(body);
        }
    }

    @SuppressWarnings("unchecked")
    @Override
    @Nullable
    public <T extends PhysicsConstraintHandle> T addConstraint(@Nullable final PhysicsPipelineBody bodyA, @Nullable final PhysicsPipelineBody bodyB, @NotNull final PhysicsConstraintConfiguration<T> configuration) {
        if (bodyA == null && bodyB == null) {
            throw new IllegalArgumentException("Cannot add a constraint between the static world and static world");
        }

        if (bodyA == bodyB) {
            throw new IllegalArgumentException("Cannot add a constraint between a body and itself");
        }

        try {
            configuration.validate(ServerSubLevelContainer.getContainer(this.level), bodyA, bodyB);
        } catch (final Exception e) {
            throw new IllegalArgumentException("Constraint validation failed", e);
        }

        final int idA = bodyA == null ? -1 : Box3DNative.getID(bodyA);
        final int idB = bodyB == null ? -1 : Box3DNative.getID(bodyB);
        final PhysicsConstraintHandle constraint = switch (configuration) {
            case final RotaryConstraintConfiguration config -> {
                final long handle = Box3DNative.addRotaryConstraint(
                    this.scene.handle(),
                    idA,
                    idB,
                    config.pos1().x(),
                    config.pos1().y(),
                    config.pos1().z(),
                    config.pos2().x(),
                    config.pos2().y(),
                    config.pos2().z(),
                    config.normal1().x(),
                    config.normal1().y(),
                    config.normal1().z(),
                    config.normal2().x(),
                    config.normal2().y(),
                    config.normal2().z());
                yield handle == 0L ? null : new Box3DRotaryConstraintHandle(this.scene.handle(), handle);
            }
            case final FixedConstraintConfiguration config -> {
                final long handle = Box3DNative.addFixedConstraint(
                    this.scene.handle(),
                    idA,
                    idB,
                    config.pos1().x(),
                    config.pos1().y(),
                    config.pos1().z(),
                    config.pos2().x(),
                    config.pos2().y(),
                    config.pos2().z(),
                    config.orientation().x(),
                    config.orientation().y(),
                    config.orientation().z(),
                    config.orientation().w());
                yield handle == 0L ? null : new Box3DFixedConstraintHandle(this.scene.handle(), handle);
            }
            case final FreeConstraintConfiguration config -> {
                final long handle = Box3DNative.addFreeConstraint(
                    this.scene.handle(),
                    idA,
                    idB,
                    config.pos1().x(),
                    config.pos1().y(),
                    config.pos1().z(),
                    config.pos2().x(),
                    config.pos2().y(),
                    config.pos2().z(),
                    config.orientation().x(),
                    config.orientation().y(),
                    config.orientation().z(),
                    config.orientation().w());
                yield handle == 0L ? null : new Box3DFreeConstraintHandle(this.scene.handle(), handle);
            }
            case final GenericConstraintConfiguration config -> {
                final long handle = Box3DNative.addGenericConstraint(
                    this.scene.handle(),
                    idA,
                    idB,
                    config.pos1().x(),
                    config.pos1().y(),
                    config.pos1().z(),
                    config.orientation1().x(),
                    config.orientation1().y(),
                    config.orientation1().z(),
                    config.orientation1().w(),
                    config.pos2().x(),
                    config.pos2().y(),
                    config.pos2().z(),
                    config.orientation2().x(),
                    config.orientation2().y(),
                    config.orientation2().z(),
                    config.orientation2().w(),
                    lockedAxesMask(config));
                yield handle == 0L ? null : new Box3DGenericConstraintHandle(this.scene.handle(), handle);
            }
        };

        if (constraint == null || !constraint.isValid()) {
            return null;
        }

        return (T) constraint;
    }

    @Override
    public void updateConfigFrom(final PhysicsConfigData data) {
        final Box3DForSableConfig.Values box3dConfig = Box3DForSableConfig.values(this.level);

        Box3DNative.configFrequencyAndDamping(this.scene.handle(), data.contactSpringFrequency, data.contactSpringDampingRatio, box3dConfig.contactCorrectionSpeed());
        Box3DNative.configSolverIterations(this.scene.handle(), data.solverIterations, data.pgsIterations, data.stabilizationIterations, box3dConfig.internalSubsteps());
        Box3DNative.configMinIslandSize(this.scene.handle(), data.minDynamicBodiesPerIsland);
    }

    @Override
    public int getNextRuntimeID() {
        return Box3DNative.nextBodyID();
    }

    private static int lockedAxesMask(final GenericConstraintConfiguration config) {
        int mask = 0;
        for (final ConstraintJointAxis axis : config.lockedAxes()) {
            mask |= 1 << axis.ordinal();
        }
        return mask;
    }

    private void assertBodyValid(final PhysicsPipelineBody body) {
        if (body.isRemoved()) {
            throw new RuntimeException("Body has been removed");
        }
    }

    private void updateContraptionPoses() {
        final SubLevelPhysicsSystem system = SubLevelPhysicsSystem.require(this.level);
        final double partialPhysicsTick = system.getPartialPhysicsTick();

        for (final KinematicContraption contraption : this.activeContraptions.keySet()) {
            this.updateContraptionPose(contraption, partialPhysicsTick);
        }
    }

    private void updateContraptionPose(final KinematicContraption contraption, final double partialPhysicsTick) {
        final TrackedKinematicContraption trackedContraption = this.activeContraptions.get(contraption);

        final SubLevel mountSubLevel = Sable.HELPER.getContaining(this.level, contraption.sable$getPosition());
        final Vector3dc parentCenterOfMass = mountSubLevel != null ? ((ServerSubLevel) mountSubLevel).getMassTracker().getCenterOfMass() : JOMLConversion.ZERO;

        final Vector3dc lastPosition = new Vector3d(contraption.sable$getPosition(partialPhysicsTick - 1.0f));
        final Quaterniondc lastOrientation = new Quaterniond(contraption.sable$getOrientation(partialPhysicsTick - 1.0f));

        final Vector3d pos = new Vector3d(contraption.sable$getPosition(partialPhysicsTick));
        final Quaterniondc rot = contraption.sable$getOrientation(partialPhysicsTick);

        final Vector3d linVel = pos.sub(lastPosition, new Vector3d());
        final Vector3d angVel = SableMathUtils.getAngularVelocity(lastOrientation, rot, new Vector3d());

        linVel.mul(20.0);
        angVel.mul(20.0);
        rot.transformInverse(linVel);
        rot.transformInverse(angVel);

        pos.sub(parentCenterOfMass);

        if (
                pos.distanceSquared(trackedContraption.lastUploadedPosition()) > DISTANCE_THRESHOLD * DISTANCE_THRESHOLD ||
                        linVel.distanceSquared(trackedContraption.lastUploadedLinVel()) > DISTANCE_THRESHOLD * DISTANCE_THRESHOLD ||
                        angVel.distanceSquared(trackedContraption.lastUploadedAngVel()) > DISTANCE_THRESHOLD * DISTANCE_THRESHOLD ||
                        rot.div(trackedContraption.lastUploadedOrientation(), new Quaterniond()).angle() > ANGULAR_THRESHOLD * ANGULAR_THRESHOLD
        ) {
            final MassTracker massTracker = contraption.sable$getMassTracker();
            final Vector3dc centerOfMass = massTracker.getCenterOfMass();

            final double[] centerOfMassArray = new double[]{centerOfMass.x(), centerOfMass.y(), centerOfMass.z()};
            final double[] poseArray = {pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()};
            final double[] velocityArray = {linVel.x(), linVel.y(), linVel.z(), angVel.x(), angVel.y(), angVel.z()};
            Box3DNative.setKinematicContraptionTransform(this.scene.handle(), trackedContraption.id(), centerOfMassArray, poseArray, velocityArray);

            trackedContraption.lastUploadedPosition().set(pos);
            trackedContraption.lastUploadedLinVel().set(linVel);
            trackedContraption.lastUploadedAngVel().set(angVel);
            trackedContraption.lastUploadedOrientation().set(rot);
        }
    }

    private void processCollisionEffects() {
        this.recentCollisions.long2LongEntrySet().removeIf(entry -> this.level.getGameTime() - entry.getLongValue() > 2);

        final Vector3d localPointA = new Vector3d();
        final Vector3d localPointB = new Vector3d();
        final Vector3d localNormalA = new Vector3d();
        final Vector3d localNormalB = new Vector3d();

        final Vector3d globalPointA = new Vector3d();
        final Vector3d globalPointB = new Vector3d();

        final double[] collisions = Box3DNative.clearCollisions(this.scene.handle());

        final BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        final BlockPos.MutableBlockPos cornerPos = new BlockPos.MutableBlockPos();

        for (int i = 0; i < collisions.length / 15; i++) {
            final int startIndex = i * 15;
            final int idA = (int) collisions[startIndex];
            final int idB = (int) collisions[startIndex + 1];

            final double forceAmount = collisions[startIndex + 2];
            localNormalA.set(collisions[startIndex + 3], collisions[startIndex + 4], collisions[startIndex + 5]);
            localNormalB.set(collisions[startIndex + 6], collisions[startIndex + 7], collisions[startIndex + 8]);
            localPointA.set(collisions[startIndex + 9], collisions[startIndex + 10], collisions[startIndex + 11]);
            localPointB.set(collisions[startIndex + 12], collisions[startIndex + 13], collisions[startIndex + 14]);

            final ServerSubLevel subLevelA = this.activeSubLevels.get(idA);
            final ServerSubLevel subLevelB = this.activeSubLevels.get(idB);

            final double minMass = Math.min(subLevelA != null ? subLevelA.getMassTracker().getMass() : Double.MAX_VALUE, subLevelB != null ? subLevelB.getMassTracker().getMass() : Double.MAX_VALUE);

            if (forceAmount > 25.0 * minMass) {
                BlockState stateA = Blocks.STONE.defaultBlockState();
                BlockState stateB = stateA;

                if (subLevelA != null) {
                    final Pose3d pose = subLevelA.logicalPose();
                    pos.set(localPointA.x + pose.rotationPoint().x, localPointA.y + pose.rotationPoint().y, localPointA.z + pose.rotationPoint().z);
                    cornerPos.set(localPointA.x + pose.rotationPoint().x + 0.5, localPointA.y + pose.rotationPoint().y + 0.5, localPointA.z + pose.rotationPoint().z + 0.5);

                    final long exists = this.recentCollisions.put(cornerPos.asLong(), this.level.getGameTime());

                    if (exists != -1) {
                        continue;
                    }

                    stateA = this.accelerator.getBlockState(pos);
                }

                if (subLevelB != null) {
                    final Pose3d pose = subLevelB.logicalPose();
                    pos.set(localPointB.x + pose.rotationPoint().x, localPointB.y + pose.rotationPoint().y, localPointB.z + pose.rotationPoint().z);
                    cornerPos.set(localPointB.x + pose.rotationPoint().x + 0.5, localPointB.y + pose.rotationPoint().y + 0.5, localPointB.z + pose.rotationPoint().z + 0.5);

                    final long exists = this.recentCollisions.put(cornerPos.asLong(), this.level.getGameTime());

                    if (exists != -1) {
                        continue;
                    }

                    stateB = this.accelerator.getBlockState(pos);
                }

                globalPointA.set(localPointA);
                globalPointB.set(localPointB);

                if (subLevelA != null) {
                    final Pose3d pose = subLevelA.logicalPose();
                    pose.orientation().transform(globalPointA).add(pose.position());
                }

                if (subLevelB != null) {
                    final Pose3d pose = subLevelB.logicalPose();
                    pose.orientation().transform(globalPointB).add(pose.position());
                }

                final BlockState state = stateB;
                this.level.sendParticles(new BlockParticleOption(ParticleTypes.BLOCK, state), globalPointA.x, globalPointA.y, globalPointA.z, 2, 0.0, 0.0, 0.0, 0.1);

                final Vec3 position = JOMLConversion.toMojang(globalPointA);
                final float volumeScale = 0.4f;
                final SoundType soundType = state.getSoundType();

                this.level.playSound(null, position.x, position.y, position.z, soundType.getStepSound(), SoundSource.BLOCKS, 0.2f * volumeScale, (float) (0.6 - 0.2 + Math.random() * 0.4));
                this.level.playSound(null, position.x, position.y, position.z, soundType.getHitSound(), SoundSource.BLOCKS, 0.2f * volumeScale, (float) (Math.random() * 0.4));
                this.level.playSound(null, position.x, position.y, position.z, soundType.getPlaceSound(), SoundSource.BLOCKS, 0.2f * volumeScale, (float) (0.5 - 0.2 + Math.random() * 0.4));
            }
        }
    }

    private record TrackedKinematicContraption(Vector3d lastUploadedPosition, Quaterniond lastUploadedOrientation,
                                               Vector3d lastUploadedLinVel, Vector3d lastUploadedAngVel, int id) {
    }
}
