package in.zaafon.box3d_for_sable.mixin;

import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import net.minecraft.ChatFormatting;
import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.components.DebugScreenOverlay;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.ModifyVariable;

import java.util.List;

@Mixin(DebugScreenOverlay.class)
public abstract class DebugScreenOverlayMixin {
    @Shadow
    protected abstract Level getLevel();

    @ModifyVariable(method = "getSystemInformation", at = @At(value = "INVOKE", target = "Lnet/minecraft/client/Minecraft;showOnlyReducedInfo()Z", shift = At.Shift.BEFORE), ordinal = 0)
    private List<String> box3dForSable$addPhysicsBackend(final List<String> lines) {
        final Minecraft minecraft = Minecraft.getInstance();
        final MinecraftServer server = minecraft.getSingleplayerServer();
        if (server == null || this.getLevel() == null) {
            return lines;
        }

        final ServerLevel serverLevel = server.getLevel(this.getLevel().dimension());
        final SubLevelPhysicsSystem physicsSystem = serverLevel == null ? null : SubLevelPhysicsSystem.get(serverLevel);
        if (physicsSystem == null) {
            return lines;
        }

        lines.add("");
        lines.add(ChatFormatting.UNDERLINE + "Box3D for Sable");
        lines.add("Physics Backend: " + physicsSystem.getPipeline().getClass().getSimpleName());
        return lines;
    }
}
