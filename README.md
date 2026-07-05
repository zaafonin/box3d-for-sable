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

Local builds compile the JNI native for the current host and package it into:

```text
natives/sable_box3d/sable_box3d_binaries.zip
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

GitHub Actions builds natives on each supported runner, then assembles the loader
jars in a final job using:

```bash
./gradlew fabric:build neoforge:build \
  -Pbox3dNativesDir=common/build/box3dNatives/imported \
  -Pbox3dRequiredNatives=sable_box3d_x86_64_linux.so,sable_box3d_aarch64_linux.so,sable_box3d_x86_64_windows.dll,sable_box3d_aarch64_macos.dylib
```

Those CI jars contain the Linux x86_64, Linux aarch64, Windows x86_64, and
macOS aarch64 natives in the same packed resource archive.

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
