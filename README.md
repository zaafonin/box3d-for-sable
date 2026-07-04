# Box3D for Sable

Standalone multi-loader mod that adds a Box3D physics backend for
[Sable](https://modrinth.com/mod/sable).

The mod registers a Sable `PhysicsPipelineProvider` and uses a small mixin hook so
upstream Sable can discover the backend without patching Sable itself.

## Requirements

- Minecraft 1.21.1
- Java 21
- Sable 2.0.3
- Sable Companion 1.6.0
- Box3D submodule initialized at `common/src/main/native/box3d`

## Building

```bash
git submodule update --init --recursive
./gradlew fabric:build neoforge:build
```

The build compiles the JNI native for the current host and packages it under:

```text
natives/sable_box3d/sable_box3d_<arch>_<os>.<ext>
```

Build just the native:

```bash
./gradlew common:buildBox3dNatives
```

Use a different Box3D checkout:

```bash
./gradlew common:buildBox3dNatives -Pbox3dSourceDir=/path/to/box3d
```

The Linux build fails if the native requires a newer glibc than `2.42` by
default. GitHub's Ubuntu runners are suitable for release builds; bleeding-edge
local distributions may need this override for local testing:

```bash
./gradlew common:buildBox3dNatives -Pbox3dAllowNewerGlibc=true
```

## Configuration

With the mod installed, Box3D is selected by default. The server config is:

```text
<world>/serverconfig/box3d_for_sable-server.toml
```

Common values:

```toml
backend = "BOX3D"
internal_substeps = 6
contact_correction_speed = 40.0
debug_stats = false
```

`backend` accepts `AUTO`, `RAPIER`, `BOX3D`, or `STATIC` when those providers are
available. Backend changes require a world restart because Sable creates physics
pipelines while server levels are being constructed.

JVM properties and environment variables override the server config for scripts
and test runs:

```text
-Dsable.physics.backend=box3d
-Dbox3d_for_sable.backend=box3d
SABLE_PHYSICS_BACKEND=box3d
BOX3D_FOR_SABLE_BACKEND=box3d
```

Box3D tuning overrides:

```text
-Dsable.physics.box3d.internalSubsteps=6
-Dsable.physics.box3d.contactCorrectionSpeed=40
-Dsable.physics.box3d.debugStats=false
SABLE_BOX3D_INTERNAL_SUBSTEPS=6
SABLE_BOX3D_CONTACT_CORRECTION_SPEED=40
SABLE_BOX3D_DEBUG_STATS=false
```

## Scope

The project intentionally does not bundle Fabric API, Create, Mod Menu, Cloth
Config, MixinExtras, Sable, or Sable Companion. Fabric declares ForgeConfigAPI
Port because Sable uses the same config compatibility layer on Fabric.

The mod package is `in.zaafon.box3d_for_sable`. Sable API references remain under
Sable's upstream `dev.ryanhcode.sable` packages.
