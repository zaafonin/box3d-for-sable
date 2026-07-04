package in.zaafon.box3d_for_sable.mixin;

import dev.ryanhcode.sable.api.physics.PhysicsPipelineProvider;
import in.zaafon.box3d_for_sable.Box3DPhysicsPipelineProvider;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Redirect;

@Mixin(SubLevelPhysicsSystem.class)
public abstract class SubLevelPhysicsSystemMixin {
    @Redirect(method = "<init>", at = @At(value = "FIELD", target = "Ldev/ryanhcode/sable/api/physics/PhysicsPipelineProvider;INSTANCE:Ldev/ryanhcode/sable/api/physics/PhysicsPipelineProvider;", ordinal = 0))
    private PhysicsPipelineProvider box3dForSable$providerForLog() {
        return Box3DPhysicsPipelineProvider.replacing(PhysicsPipelineProvider.INSTANCE);
    }

    @Redirect(method = "<init>", at = @At(value = "FIELD", target = "Ldev/ryanhcode/sable/api/physics/PhysicsPipelineProvider;INSTANCE:Ldev/ryanhcode/sable/api/physics/PhysicsPipelineProvider;", ordinal = 1))
    private PhysicsPipelineProvider box3dForSable$providerForCreate() {
        return Box3DPhysicsPipelineProvider.replacing(PhysicsPipelineProvider.INSTANCE);
    }
}
