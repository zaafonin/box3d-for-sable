package in.zaafon.box3d_for_sable;

import dev.ryanhcode.sable.api.physics.PhysicsPipeline;
import dev.ryanhcode.sable.api.physics.PhysicsPipelineProvider;
import net.minecraft.server.level.ServerLevel;
import org.jetbrains.annotations.NotNull;

import java.lang.reflect.Method;
import java.util.Locale;
import java.util.ServiceLoader;
import java.util.stream.Collectors;

@PhysicsPipelineProvider.LoadPriority(1200)
public final class Box3DPhysicsPipelineProvider implements PhysicsPipelineProvider {
    private final PhysicsPipelineProvider fallbackProvider;

    public Box3DPhysicsPipelineProvider() {
        this(null);
    }

    private Box3DPhysicsPipelineProvider(final PhysicsPipelineProvider fallbackProvider) {
        this.fallbackProvider = fallbackProvider;
    }

    public static @NotNull PhysicsPipelineProvider replacing(final PhysicsPipelineProvider fallbackProvider) {
        if (fallbackProvider instanceof Box3DPhysicsPipelineProvider) {
            return fallbackProvider;
        }

        return new Box3DPhysicsPipelineProvider(fallbackProvider);
    }

    public @NotNull String id() {
        return "box3d";
    }

    @Override
    public @NotNull PhysicsPipeline createPipeline(@NotNull final ServerLevel level) {
        final String requestedBackend = Box3DForSableConfig.backend(level);
        if (requestedBackend.isEmpty() || requestedBackend.equals("auto") || requestedBackend.equals(this.id())) {
            return new Box3DPhysicsPipeline(level);
        }

        final PhysicsPipelineProvider provider = ServiceLoader.load(PhysicsPipelineProvider.class)
                .stream()
                .map(ServiceLoader.Provider::get)
                .filter(candidate -> !(candidate instanceof Box3DPhysicsPipelineProvider))
                .filter(candidate -> providerId(candidate).equals(requestedBackend))
                .findFirst()
                .or(() -> this.fallbackProvider != null && providerId(this.fallbackProvider).equals(requestedBackend)
                        ? java.util.Optional.of(this.fallbackProvider)
                        : java.util.Optional.empty())
                .orElseThrow(() -> new IllegalStateException("Requested physics backend '%s' was not found. Available backends: %s"
                        .formatted(requestedBackend, availableBackends())));
        return provider.createPipeline(level);
    }

    private static @NotNull String availableBackends() {
        return ServiceLoader.load(PhysicsPipelineProvider.class)
                .stream()
                .map(ServiceLoader.Provider::get)
                .map(Box3DPhysicsPipelineProvider::providerId)
                .sorted()
                .collect(Collectors.joining(", "));
    }

    private static @NotNull String providerId(@NotNull final PhysicsPipelineProvider provider) {
        try {
            final Method method = provider.getClass().getMethod("id");
            if (method.getReturnType() == String.class) {
                return normalize((String) method.invoke(provider));
            }
        } catch (final ReflectiveOperationException ignored) {
            // Upstream Sable 2.0.3 providers do not expose ids; derive one from the class name.
        }

        String simpleName = provider.getClass().getSimpleName();
        final String suffix = "PhysicsPipelineProvider";
        if (simpleName.endsWith(suffix)) {
            simpleName = simpleName.substring(0, simpleName.length() - suffix.length());
        }
        return normalize(simpleName);
    }

    private static @NotNull String normalize(@NotNull final String value) {
        return value.replaceAll("[^A-Za-z0-9]", "").toLowerCase(Locale.ROOT);
    }

    public static @NotNull PhysicsPipeline createBox3DPipeline(@NotNull final ServerLevel level) {
        return new Box3DPhysicsPipeline(level);
    }
}
