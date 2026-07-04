package in.zaafon.box3d_for_sable.fabric;

import in.zaafon.box3d_for_sable.Box3DForSableConfig;
import fuzs.forgeconfigapiport.fabric.api.neoforge.v4.NeoForgeConfigRegistry;
import net.fabricmc.api.ModInitializer;
import net.neoforged.fml.config.ModConfig;

public final class Box3DForSableFabric implements ModInitializer {
    @Override
    public void onInitialize() {
        NeoForgeConfigRegistry.INSTANCE.register("box3d_for_sable", ModConfig.Type.SERVER, Box3DForSableConfig.SPEC);
    }
}
