package in.zaafon.box3d_for_sable;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineBody;
import dev.ryanhcode.sable.api.physics.callback.BlockSubLevelCollisionCallback;
import dev.ryanhcode.sable.api.physics.mass.MassData;
import in.zaafon.box3d_for_sable.collider.Box3DVoxelColliderData;
import dev.ryanhcode.sable.platform.SableLoaderPlatform;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import net.minecraft.CrashReport;
import net.minecraft.CrashReportCategory;
import net.minecraft.ReportedException;
import net.minecraft.Util;
import net.minecraft.Util.OS;
import net.minecraft.server.level.ServerLevel;
import org.jetbrains.annotations.ApiStatus;
import org.joml.Matrix3dc;
import org.joml.Vector3dc;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

/**
 * Java side of the Box3D for Sable JNI bridge.
 */
@ApiStatus.Internal
public final class Box3DNative {
    private static final Path NATIVE_DIR = resolveNativeDir();
    private static final String LIB_ZIP_NAME = "sable_box3d_binaries.zip";
    private static final String LIB_NAME = "sable_box3d";

    public static final String NATIVE_NAME = getNativeName();

    private static int countingObjectID = 0;

    static {
        loadLibrary();
    }

    private Box3DNative() {
    }

    private static Path resolveNativeDir() {
        final Path gameDir = getGameDirectory();
        if (gameDir != null) {
            final Path gameDirRelativeDir = gameDir.resolve(".sable").resolve("natives").normalize();
            Sable.LOGGER.info("Using game-dir-relative Box3D native directory {}", gameDirRelativeDir.toAbsolutePath());
            return gameDirRelativeDir;
        }

        final Path fallbackDir = Paths.get(System.getProperty("user.home", System.getProperty("user.dir")), ".sable", "natives");
        Sable.LOGGER.info("Using fallback Box3D native directory {}", fallbackDir.toAbsolutePath());
        return fallbackDir;
    }

    private static Path getGameDirectory() {
        try {
            final Object value = SableLoaderPlatform.INSTANCE.getClass().getMethod("getGameDirectory").invoke(SableLoaderPlatform.INSTANCE);
            return value instanceof Path path ? path : null;
        } catch (final ReflectiveOperationException ignored) {
            return null;
        }
    }

    private static String getNativeName() {
        final String arch;
        if (System.getProperty("os.arch").equals("arm") || System.getProperty("os.arch").startsWith("aarch64")) {
            arch = "aarch64";
        } else {
            arch = "x86_64";
        }

        final OS os = Util.getPlatform();
        if (os == OS.WINDOWS) {
            return LIB_NAME + "_" + arch + "_windows.dll";
        } else if (os == OS.OSX) {
            return LIB_NAME + "_" + arch + "_macos.dylib";
        } else {
            if (os != OS.LINUX) {
                Sable.LOGGER.error("Unknown platform '{}' detected, sable will attempt to use linux natives, this may or may not work.", System.getProperty("os.name"));
            }
            return LIB_NAME + "_" + arch + "_linux.so";
        }
    }

    private static void loadLibrary() {
        try {
            try (final InputStream archive = Box3DNative.class.getResourceAsStream("/natives/" + LIB_NAME + "/" + LIB_ZIP_NAME)) {
                if (archive != null) {
                    loadFromArchive(archive);
                    return;
                }
            }

            try (final InputStream direct = Box3DNative.class.getResourceAsStream("/natives/" + LIB_NAME + "/" + NATIVE_NAME)) {
                if (direct == null) {
                    throw new FileNotFoundException(LIB_ZIP_NAME + " or " + NATIVE_NAME);
                }
                loadFromStream(direct, NATIVE_NAME);
            }
        } catch (final Throwable t) {
            failLoad(t);
        }
    }

    private static void loadFromArchive(final InputStream archive) throws java.io.IOException {
        try (final ZipInputStream zip = new ZipInputStream(archive)) {
            ZipEntry entry;
            while ((entry = zip.getNextEntry()) != null) {
                if (entry.getName().equals(NATIVE_NAME)) {
                    loadFromStream(zip, NATIVE_NAME);
                    return;
                }
            }
        }

        throw new FileNotFoundException(NATIVE_NAME + " in " + LIB_ZIP_NAME);
    }

    private static void loadFromStream(final InputStream inputStream, final String fileName) throws java.io.IOException {
        if (!Files.exists(NATIVE_DIR)) {
            Files.createDirectories(NATIVE_DIR);
        }

        final Path tempFile = NATIVE_DIR.resolve(fileName);
        if (Files.exists(tempFile)) {
            Files.delete(tempFile);
        }
        Files.createFile(tempFile);
        Files.copy(inputStream, tempFile, StandardCopyOption.REPLACE_EXISTING);
        System.load(tempFile.toAbsolutePath().toString());
    }

    private static void failLoad(final Throwable t) {
        Sable.LOGGER.error(
                "Sable has failed to load the natives needed for its Box3D pipeline. Please report with system details and logs to {}",
                Sable.ISSUE_TRACKER_URL,
                t);
        final CrashReport crashReport = CrashReport.forThrowable(t instanceof UnsatisfiedLinkError && t.getCause() != null ? t.getCause() : t, "Sable linking with Box3D natives");
        final CrashReportCategory category = crashReport.addCategory("Natives");
        category.setDetail("Name", Box3DNative.NATIVE_NAME);
        category.setDetail("Native Directory", Box3DNative.NATIVE_DIR);
        throw new ReportedException(crashReport);
    }

    @ApiStatus.Internal
    public static int getID(final PhysicsPipelineBody body) {
        return body.getRuntimeId();
    }

    @ApiStatus.Internal
    public static synchronized int nextBodyID() {
        return countingObjectID++;
    }

    @ApiStatus.Internal
    public static long getSceneHandle(final ServerLevel level) {
        final PhysicsPipeline pipeline = SubLevelPhysicsSystem.require(level).getPipeline();

        if (!(pipeline instanceof final Box3DPhysicsPipeline box3DPipeline)) {
            throw new IllegalStateException("ServerLevel does not use the Box3D physics pipeline");
        }

        return box3DPipeline.getSceneHandle();
    }

    static native long initialize(double gravityX, double gravityY, double gravityZ, double universalDrag);

    static native void tick(long sceneHandle, double timeStep);

    static native void step(long sceneHandle, double timeStep);

    static native void dispose(long sceneHandle);

    static native void createSubLevel(long sceneHandle, int id, double[] pose);

    static native void removeSubLevel(long sceneHandle, int id);

    public static native void createBox(long sceneHandle, int id, double mass, double halfExtentsX, double halfExtentsY, double halfExtentsZ, double[] pose);

    public static native void removeBox(long sceneHandle, int id);

    public static native void getPose(long sceneHandle, int id, double[] store);

    static native void setCenterOfMass(long sceneHandle, int id, double x, double y, double z);

    static native void setLocalBounds(long sceneHandle, int id, int minX, int minY, int minZ, int maxX, int maxY, int maxZ);

    static native void addChunk(long sceneHandle, int x, int y, int z, int[] chunk, boolean global, int id);

    static native void removeChunk(long sceneHandle, int x, int y, int z, boolean global);

    static native void getDebugStats(long sceneHandle, long[] store);

    public static native void changeBlock(long sceneHandle, int x, int y, int z, int newState);

    private static native int newVoxelCollider(double frictionMultiplier, double volume, double restitution, boolean isFluid, BlockSubLevelCollisionCallback contactEvents);

    public static native void addVoxelColliderBox(int index, double[] bounds);

    public static native void clearVoxelColliderBoxes(int index);

    private static native void setMassProperties(long sceneHandle, int index, double mass, double[] centerOfMass, double[] inertiaTensor);

    public static Box3DVoxelColliderData createVoxelColliderEntry(final double frictionMultiplier, final double volume, final double restitution, final boolean isFluid, final BlockSubLevelCollisionCallback contactEvents) {
        return new Box3DVoxelColliderData(Box3DNative.newVoxelCollider(frictionMultiplier, volume, restitution, isFluid, contactEvents));
    }

    static native void teleportObject(long sceneHandle, int id, double x, double y, double z, double i, double j, double k, double r);

    public static native void wakeUpObject(long sceneHandle, int id);

    public static native void addLinearAngularVelocities(long sceneHandle, int bodyId, double linearX, double linearY, double linearZ, double angularX, double angularY, double angularZ, boolean wakeUp);

    static native double[] clearCollisions(long sceneHandle);

    static native void applyForce(long sceneHandle, int bodyID, double x, double y, double z, double fx, double fy, double fz, boolean wakeUp);

    static native void applyForceAndTorque(long sceneHandle, int bodyID, double fx, double fy, double fz, double tx, double ty, double tz, boolean wakeUp);

    static native void getLinearVelocity(long sceneHandle, int bodyID, double[] store);

    static native void getAngularVelocity(long sceneHandle, int bodyID, double[] store);

    static native long createRope(long sceneHandle, double pointRadius, double firstJointLength, double[] points, int pointCount);

    static native double[] queryRope(long sceneHandle, long ropeId);

    static native void removeRope(long sceneHandle, long ropeId);

    static native void setRopeAttachment(long sceneHandle, long ropeId, int subLevelId, double x, double y, double z, boolean end);

    static native void addRopePointAtStart(long sceneHandle, long ropeId, double x, double y, double z);

    static native void removeRopePointAtStart(long sceneHandle, long ropeId);

    static native void wakeUpRope(long sceneHandle, long ropeId);

    static native void setRopeFirstSegmentLength(long sceneHandle, long ropeId, double firstSegmentLength);

    static native void createKinematicContraption(long sceneHandle, int mountId, int id, double[] pose);

    static native void removeKinematicContraption(long sceneHandle, int id);

    static native void setKinematicContraptionTransform(long sceneHandle, int id, double[] centerOfMass, double[] pose, double[] velocities);

    static native void addKinematicContraptionChunkSection(long sceneHandle, int id, int x, int y, int z, int[] data);

    static native long addRotaryConstraint(
            long sceneHandle,
            int idA,
            int idB,
            double localXA,
            double localYA,
            double localZA,
            double localXB,
            double localYB,
            double localZB,
            double axisXA,
            double axisYA,
            double axisZA,
            double axisXB,
            double axisYB,
            double axisZB);

    static native long addFixedConstraint(
            long sceneHandle,
            int idA,
            int idB,
            double localXA,
            double localYA,
            double localZA,
            double localXB,
            double localYB,
            double localZB,
            double localQX,
            double localQY,
            double localQZ,
            double localQW);

    static native long addFreeConstraint(
            long sceneHandle,
            int idA,
            int idB,
            double localXA,
            double localYA,
            double localZA,
            double localXB,
            double localYB,
            double localZB,
            double localQX,
            double localQY,
            double localQZ,
            double localQW);

    static native long addGenericConstraint(
            long sceneHandle,
            int idA,
            int idB,
            double localXA,
            double localYA,
            double localZA,
            double localQXA,
            double localQYA,
            double localQZA,
            double localQWA,
            double localXB,
            double localYB,
            double localZB,
            double localQXB,
            double localQYB,
            double localQZB,
            double localQWB,
            int lockedAxesMask);

    static native void setConstraintFrame(
            long sceneHandle,
            long constraintHandle,
            int side,
            double localX,
            double localY,
            double localZ,
            double localQX,
            double localQY,
            double localQZ,
            double localQW);

    static native void setConstraintLimit(long sceneHandle, long constraintHandle, int axis, double min, double max);

    static native void lockConstraintAxes(long sceneHandle, long constraintHandle, int lockedAxesMask);

    static native void setConstraintMotor(long sceneHandle, long constraintHandle, int axis, double target, double stiffness, double damping, boolean hasMaxForce, double maxForce);

    static native void setConstraintContactsEnabled(long sceneHandle, long constraintHandle, boolean enabled);

    static native void getConstraintImpulses(long sceneHandle, long constraintHandle, double[] store);

    static native void removeConstraint(long sceneHandle, long constraintHandle);

    static native boolean isConstraintValid(long sceneHandle, long constraintHandle);

    static native void configFrequencyAndDamping(long sceneHandle, double contactNaturalFrequency, double contactDampingRatio, double contactCorrectionSpeed);

    static native void configSolverIterations(long sceneHandle, int solverIterations, int pgsIterations, int stabilizationIterations, int box3dInternalSubsteps);

    static native void configMinIslandSize(long sceneHandle, int islandSize);

    static void setMassPropertiesFrom(final long sceneHandle, final int id, final MassData massTracker) {
        final Matrix3dc inertiaTensor = massTracker.getInertiaTensor();
        final Vector3dc centerOfMass = massTracker.getCenterOfMass();
        final double mass = massTracker.getMass();

        final double[] centerOfMassArray = new double[]{centerOfMass.x(), centerOfMass.y(), centerOfMass.z()};
        final double[] inertiaTensorArray = new double[]{
                inertiaTensor.m00(), inertiaTensor.m01(), inertiaTensor.m02(),
                inertiaTensor.m10(), inertiaTensor.m11(), inertiaTensor.m12(),
                inertiaTensor.m20(), inertiaTensor.m21(), inertiaTensor.m22()
        };

        Box3DNative.setMassProperties(sceneHandle, id, mass, centerOfMassArray, inertiaTensorArray);
    }
}
