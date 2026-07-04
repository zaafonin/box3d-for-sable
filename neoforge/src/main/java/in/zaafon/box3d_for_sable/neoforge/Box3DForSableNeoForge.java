package in.zaafon.box3d_for_sable.neoforge;

import in.zaafon.box3d_for_sable.Box3DForSableConfig;
import net.neoforged.fml.ModContainer;
import net.neoforged.fml.common.Mod;
import net.neoforged.fml.config.ModConfig;

@Mod("box3d_for_sable")
public final class Box3DForSableNeoForge {
    public Box3DForSableNeoForge(final ModContainer modContainer) {
        modContainer.registerConfig(ModConfig.Type.SERVER, Box3DForSableConfig.SPEC);
    }
}
