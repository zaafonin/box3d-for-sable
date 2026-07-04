package in.zaafon.box3d_for_sable;

import com.electronwill.nightconfig.core.EnumGetMethod;
import dev.ryanhcode.sable.Sable;
import net.minecraft.server.level.ServerLevel;
import net.neoforged.neoforge.common.ModConfigSpec;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Locale;

public final class Box3DForSableConfig {
    private static final String BACKEND_PROPERTY = "sable.physics.backend";
    private static final String BACKEND_ENVIRONMENT_VARIABLE = "SABLE_PHYSICS_BACKEND";
    private static final String BOX3D_BACKEND_PROPERTY = "box3d_for_sable.backend";
    private static final String BOX3D_BACKEND_ENVIRONMENT_VARIABLE = "BOX3D_FOR_SABLE_BACKEND";
    private static final String INTERNAL_SUBSTEPS_PROPERTY = "sable.physics.box3d.internalSubsteps";
    private static final String INTERNAL_SUBSTEPS_ENVIRONMENT_VARIABLE = "SABLE_BOX3D_INTERNAL_SUBSTEPS";
    private static final String CORRECTION_SPEED_PROPERTY = "sable.physics.box3d.contactCorrectionSpeed";
    private static final String CORRECTION_SPEED_ENVIRONMENT_VARIABLE = "SABLE_BOX3D_CONTACT_CORRECTION_SPEED";
    private static final String DEBUG_STATS_PROPERTY = "sable.physics.box3d.debugStats";
    private static final String DEBUG_STATS_ENVIRONMENT_VARIABLE = "SABLE_BOX3D_DEBUG_STATS";

    private static final int DEFAULT_INTERNAL_SUBSTEPS = 6;
    private static final double DEFAULT_CONTACT_CORRECTION_SPEED = 40.0;
    private static final boolean DEFAULT_DEBUG_STATS = false;

    public static final ModConfigSpec SPEC;

    private static final ModConfigSpec.EnumValue<PhysicsBackend> BACKEND;
    private static final ModConfigSpec.IntValue INTERNAL_SUBSTEPS;
    private static final ModConfigSpec.DoubleValue CONTACT_CORRECTION_SPEED;
    private static final ModConfigSpec.BooleanValue DEBUG_STATS;

    static {
        final ModConfigSpec.Builder builder = new ModConfigSpec.Builder();

        BACKEND = builder
                .comment("Physics backend to use for new Sable server physics pipelines. JVM property sable.physics.backend and SABLE_PHYSICS_BACKEND environment variable override this value.")
                .worldRestart()
                .defineEnum("backend", PhysicsBackend.BOX3D, EnumGetMethod.NAME_IGNORECASE);
        INTERNAL_SUBSTEPS = builder
                .comment("How many internal Box3D solver substeps are run for each Sable physics step. Higher values improve tall-stack stability but can significantly increase server tick time.")
                .defineInRange("internal_substeps", DEFAULT_INTERNAL_SUBSTEPS, 1, 50);
        CONTACT_CORRECTION_SPEED = builder
                .comment("Maximum Box3D contact correction speed in blocks per second. Higher values resist clipping from external forces better, but excessive values can increase jitter.")
                .defineInRange("contact_correction_speed", DEFAULT_CONTACT_CORRECTION_SPEED, 0.0, 500.0);
        DEBUG_STATS = builder
                .comment("Log Box3D native body, shape, contact, and chunk counts every 100 game ticks.")
                .define("debug_stats", DEFAULT_DEBUG_STATS);

        SPEC = builder.build();
    }

    private Box3DForSableConfig() {
    }

    public static @NotNull String backend(@Nullable final ServerLevel level) {
        final String systemBackend = stringOverride(BACKEND_PROPERTY, BACKEND_ENVIRONMENT_VARIABLE);
        if (systemBackend != null) {
            return normalizeBackend(systemBackend);
        }

        final String ownSystemBackend = stringOverride(BOX3D_BACKEND_PROPERTY, BOX3D_BACKEND_ENVIRONMENT_VARIABLE);
        if (ownSystemBackend != null) {
            return normalizeBackend(ownSystemBackend);
        }

        return configBackend().id();
    }

    public static @NotNull Values values(@Nullable final ServerLevel level) {
        final int internalSubsteps = intOverride(
                INTERNAL_SUBSTEPS_PROPERTY,
                INTERNAL_SUBSTEPS_ENVIRONMENT_VARIABLE,
                configInt(INTERNAL_SUBSTEPS),
                1,
                50);
        final double contactCorrectionSpeed = doubleOverride(
                CORRECTION_SPEED_PROPERTY,
                CORRECTION_SPEED_ENVIRONMENT_VARIABLE,
                configDouble(CONTACT_CORRECTION_SPEED),
                0.0,
                500.0);

        return new Values(internalSubsteps, contactCorrectionSpeed, booleanOverride(DEBUG_STATS_PROPERTY, DEBUG_STATS_ENVIRONMENT_VARIABLE, configBoolean(DEBUG_STATS)));
    }

    private static int intOverride(@NotNull final String propertyName, @NotNull final String environmentName, final int fallback, final int min, final int max) {
        final String value = stringOverride(propertyName, environmentName);
        return value == null ? fallback : parseInt(propertyName, value, fallback, min, max);
    }

    private static double doubleOverride(@NotNull final String propertyName, @NotNull final String environmentName, final double fallback, final double min, final double max) {
        final String value = stringOverride(propertyName, environmentName);
        return value == null ? fallback : parseDouble(propertyName, value, fallback, min, max);
    }

    private static boolean booleanOverride(@NotNull final String propertyName, @NotNull final String environmentName, final boolean fallback) {
        final String value = stringOverride(propertyName, environmentName);
        return value == null ? fallback : Boolean.parseBoolean(value);
    }

    private static @NotNull PhysicsBackend configBackend() {
        try {
            return BACKEND.get();
        } catch (final IllegalStateException e) {
            Sable.LOGGER.debug("Box3D for Sable server config is not loaded yet, using default backend", e);
            return BACKEND.getDefault();
        }
    }

    private static int configInt(@NotNull final ModConfigSpec.IntValue value) {
        try {
            return value.getAsInt();
        } catch (final IllegalStateException e) {
            Sable.LOGGER.debug("Box3D for Sable server config is not loaded yet, using default integer value for {}", String.join(".", value.getPath()), e);
            return value.getDefault();
        }
    }

    private static double configDouble(@NotNull final ModConfigSpec.DoubleValue value) {
        try {
            return value.getAsDouble();
        } catch (final IllegalStateException e) {
            Sable.LOGGER.debug("Box3D for Sable server config is not loaded yet, using default double value for {}", String.join(".", value.getPath()), e);
            return value.getDefault();
        }
    }

    private static boolean configBoolean(@NotNull final ModConfigSpec.BooleanValue value) {
        try {
            return value.getAsBoolean();
        } catch (final IllegalStateException e) {
            Sable.LOGGER.debug("Box3D for Sable server config is not loaded yet, using default boolean value for {}", String.join(".", value.getPath()), e);
            return value.getDefault();
        }
    }

    private static int parseInt(@NotNull final String key, @NotNull final String value, final int fallback, final int min, final int max) {
        try {
            return Math.clamp(Integer.parseInt(value.trim()), min, max);
        } catch (final NumberFormatException e) {
            Sable.LOGGER.warn("Ignoring invalid Box3D config {}={}", key, value);
            return fallback;
        }
    }

    private static double parseDouble(@NotNull final String key, @NotNull final String value, final double fallback, final double min, final double max) {
        try {
            return Math.clamp(Double.parseDouble(value.trim()), min, max);
        } catch (final NumberFormatException e) {
            Sable.LOGGER.warn("Ignoring invalid Box3D config {}={}", key, value);
            return fallback;
        }
    }

    public record Values(int internalSubsteps, double contactCorrectionSpeed, boolean debugStats) {
    }

    public enum PhysicsBackend {
        AUTO("auto"),
        RAPIER("rapier"),
        BOX3D("box3d"),
        STATIC("static");

        private final String id;

        PhysicsBackend(final String id) {
            this.id = id;
        }

        public @NotNull String id() {
            return this.id;
        }
    }

    private static @NotNull String normalizeBackend(@NotNull final String value) {
        return value.trim().replaceAll("[^A-Za-z0-9]", "").toLowerCase(Locale.ROOT);
    }

    private static @Nullable String stringOverride(@NotNull final String propertyName, @NotNull final String environmentName) {
        String value = System.getProperty(propertyName);
        if (value == null || value.isBlank()) {
            value = System.getenv(environmentName);
        }
        return value == null || value.isBlank() ? null : value.trim();
    }
}
