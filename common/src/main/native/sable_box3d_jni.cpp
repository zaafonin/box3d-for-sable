#include <box3d/box3d.h>

#include <jni.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct VoxelBox {
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;
};

struct VoxelCollider {
    std::vector<VoxelBox> boxes;
    float friction = 0.5f;
    float volume = 1.0f;
    float restitution = 0.0f;
    bool isFluid = false;
    jobject contactEvents = nullptr;
    jmethodID contactMethod = nullptr;
};

struct ShapeUserData {
    int ownerId = -1;
    int blockX = 0;
    int blockY = 0;
    int blockZ = 0;
    int colliderIndex = -1;
    int voxelState = 0;
};

struct DirtyChunk {
    bool global = false;
    int bodyId = -1;
    int sectionX = 0;
    int sectionY = 0;
    int sectionZ = 0;
};

struct ChunkShapes {
    b3BodyId body = b3_nullBodyId;
    bool ownsBody = false;
    bool dirty = false;
    bool hasFluid = false;
    int sectionX = 0;
    int sectionY = 0;
    int sectionZ = 0;
    std::array<jint, 4096> states{};
    std::vector<b3ShapeId> shapes;
    std::deque<ShapeUserData> shapeData;
};

struct LocalOrigin {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct ConstraintFrameInfo {
    LocalOrigin position;
    b3Quat rotation = b3Quat{b3Vec3{0.0f, 0.0f, 0.0f}, 1.0f};
};

struct LocalBounds {
    bool valid = false;
    int minX = 0;
    int minY = 0;
    int minZ = 0;
    int maxX = 0;
    int maxY = 0;
    int maxZ = 0;
};

struct ColliderMaterialData {
    bool valid = false;
    bool fluid = false;
    float volume = 1.0f;
};

struct MotorSpringSettings {
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
};

struct BodyInfo {
    b3BodyId body = b3_nullBodyId;
    bool ownsBody = true;
    bool mountedContraption = false;
    int mountId = -1;
    LocalOrigin centerOfMass;
    b3Transform shapeRoot = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{0.0f, 0.0f, 0.0f}, 1.0f}};
    b3Vec3 localLinearVelocity = b3Vec3{0.0f, 0.0f, 0.0f};
    b3Vec3 localAngularVelocity = b3Vec3{0.0f, 0.0f, 0.0f};
    LocalBounds localBounds;
    std::unordered_map<int64_t, ChunkShapes> chunks;
};

enum class ConstraintKind {
    Fixed,
    Free,
    Rotary,
    GenericWeld,
    GenericSpherical,
    GenericRevolute,
    GenericPrismatic,
    GenericMotor,
};

struct ConstraintInfo {
    b3JointId joint = b3_nullJointId;
    ConstraintKind kind = ConstraintKind::Free;
    int primaryAxis = 0;
    int bodyIdA = -1;
    int bodyIdB = -1;
    ConstraintFrameInfo baseFrameA;
    ConstraintFrameInfo baseFrameB;
    ConstraintFrameInfo frameA;
    ConstraintFrameInfo frameB;
    b3Vec3 linearMotorTarget = b3Vec3{0.0f, 0.0f, 0.0f};
    b3Vec3 angularMotorTarget = b3Vec3{0.0f, 0.0f, 0.0f};
};

struct RopeSegmentInfo {
    b3JointId distanceJoint = b3_nullJointId;
    b3BodyId bodyA = b3_nullBodyId;
    b3BodyId bodyB = b3_nullBodyId;
};

struct RopeAttachmentInfo {
    b3JointId joint = b3_nullJointId;
    int bodyId = -1;
    ConstraintFrameInfo frame;
};

struct RopeInfo {
    std::vector<b3BodyId> points;
    std::vector<RopeSegmentInfo> segments;
    double pointRadius = 0.125;
    double firstSegmentLength = 1.0;
    bool hasStartAttachment = false;
    bool hasEndAttachment = false;
    RopeAttachmentInfo startAttachment;
    RopeAttachmentInfo endAttachment;
};

struct Box3DScene {
    b3WorldId world = b3_nullWorldId;
    b3BodyId groundBody = b3_nullBodyId;
    float universalDrag = 0.0f;
    float contactHertz = 30.0f;
    float contactDampingRatio = 5.0f;
    float contactSpeed = 40.0f;
    int subStepCount = 6;
    std::unordered_map<int, BodyInfo> bodies;
    std::unordered_map<int64_t, ChunkShapes> globalChunks;
    std::unordered_map<uint64_t, ConstraintInfo> constraints;
    std::unordered_map<uint64_t, RopeInfo> ropes;
    std::vector<DirtyChunk> dirtyChunks;
    std::vector<double> collisions;
    uint64_t nextRopeId = 1;
};

std::mutex gColliderMutex;
std::vector<VoxelCollider> gVoxelColliders;
JavaVM* gJvm = nullptr;

constexpr uint64_t LEVEL_COLLISION_CATEGORY = 1;
constexpr uint64_t ROPE_COLLISION_CATEGORY = 2;
constexpr float ROPE_POINT_MASS = 0.35f;
constexpr float ROPE_POINT_FRICTION = 0.15f;
constexpr float ROPE_RAPIER_DAMPING_STRENGTH = 18.0f;
constexpr float ROPE_RELATIVE_DAMPING = 0.5f * ROPE_POINT_MASS * ROPE_RAPIER_DAMPING_STRENGTH;

int64_t packSectionPos(int x, int y, int z) {
    int64_t value = 0;
    value |= (static_cast<int64_t>(x) & 4194303LL) << 42;
    value |= static_cast<int64_t>(y) & 1048575LL;
    value |= (static_cast<int64_t>(z) & 4194303LL) << 20;
    return value;
}

Box3DScene* sceneFromHandle(jlong handle) {
    return reinterpret_cast<Box3DScene*>(handle);
}

b3Vec3 vec3(float x, float y, float z) {
    return b3Vec3{x, y, z};
}

LocalOrigin origin(double x, double y, double z) {
    return LocalOrigin{x, y, z};
}

b3Quat quat(float x, float y, float z, float w) {
    return b3Quat{b3Vec3{x, y, z}, w};
}

b3Transform transform(b3Vec3 p, b3Quat q) {
    return b3Transform{p, q};
}

void throwRuntime(JNIEnv* env, const char* message) {
    env->ThrowNew(env->FindClass("java/lang/RuntimeException"), message);
}

std::array<double, 7> readPoseArray(JNIEnv* env, jdoubleArray pose) {
    std::array<double, 7> values{};
    env->GetDoubleArrayRegion(pose, 0, static_cast<jsize>(values.size()), values.data());
    return values;
}

b3Transform localTransformFromPose(const std::array<double, 7>& pose) {
    return transform(
        vec3(static_cast<float>(pose[0]), static_cast<float>(pose[1]), static_cast<float>(pose[2])),
        quat(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5]), static_cast<float>(pose[6])));
}

std::array<double, 3> readVector3Array(JNIEnv* env, jdoubleArray value) {
    std::array<double, 3> values{};
    env->GetDoubleArrayRegion(value, 0, static_cast<jsize>(values.size()), values.data());
    return values;
}

std::array<double, 9> readMatrix3Array(JNIEnv* env, jdoubleArray value) {
    std::array<double, 9> values{};
    env->GetDoubleArrayRegion(value, 0, static_cast<jsize>(values.size()), values.data());
    return values;
}

BodyInfo* findBody(Box3DScene* scene, int id) {
    auto it = scene->bodies.find(id);
    return it == scene->bodies.end() ? nullptr : &it->second;
}

b3BodyId bodyForId(Box3DScene* scene, int id) {
    BodyInfo* info = findBody(scene, id);
    return info == nullptr ? b3_nullBodyId : info->body;
}

b3BodyId constraintBodyForId(Box3DScene* scene, int id) {
    if (id == -1) {
        return scene->groundBody;
    }

    return bodyForId(scene, id);
}

LocalOrigin centerOfMassForId(Box3DScene* scene, int id) {
    BodyInfo* info = id == -1 ? nullptr : findBody(scene, id);
    return info == nullptr ? LocalOrigin{} : info->centerOfMass;
}

uint64_t jointKey(b3JointId joint) {
    return b3StoreJointId(joint);
}

int countBits(int value) {
    int count = 0;
    while (value != 0) {
        count += value & 1;
        value >>= 1;
    }
    return count;
}

b3Vec3 normalizeOr(b3Vec3 value, b3Vec3 fallback) {
    const float length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    if (length <= 1.0e-6f) {
        return fallback;
    }

    const float inv = 1.0f / length;
    return vec3(value.x * inv, value.y * inv, value.z * inv);
}

b3Quat normalizeQuat(b3Quat value) {
    const float length = std::sqrt(value.v.x * value.v.x + value.v.y * value.v.y + value.v.z * value.v.z + value.s * value.s);
    if (length <= 1.0e-6f) {
        return quat(0.0f, 0.0f, 0.0f, 1.0f);
    }

    const float inv = 1.0f / length;
    return quat(value.v.x * inv, value.v.y * inv, value.v.z * inv, value.s * inv);
}

b3Quat mulQuat(b3Quat a, b3Quat b) {
    return normalizeQuat(quat(
        a.s * b.v.x + a.v.x * b.s + a.v.y * b.v.z - a.v.z * b.v.y,
        a.s * b.v.y - a.v.x * b.v.z + a.v.y * b.s + a.v.z * b.v.x,
        a.s * b.v.z + a.v.x * b.v.y - a.v.y * b.v.x + a.v.z * b.s,
        a.s * b.s - a.v.x * b.v.x - a.v.y * b.v.y - a.v.z * b.v.z));
}

b3Quat rotationBetween(b3Vec3 from, b3Vec3 to) {
    from = normalizeOr(from, vec3(0.0f, 0.0f, 1.0f));
    to = normalizeOr(to, vec3(0.0f, 0.0f, 1.0f));

    const float dot = from.x * to.x + from.y * to.y + from.z * to.z;
    if (dot > 0.999999f) {
        return quat(0.0f, 0.0f, 0.0f, 1.0f);
    }

    if (dot < -0.999999f) {
        b3Vec3 axis = std::abs(from.x) < 0.9f
            ? vec3(1.0f, 0.0f, 0.0f)
            : vec3(0.0f, 1.0f, 0.0f);
        axis = normalizeOr(vec3(
            from.y * axis.z - from.z * axis.y,
            from.z * axis.x - from.x * axis.z,
            from.x * axis.y - from.y * axis.x), vec3(1.0f, 0.0f, 0.0f));
        return quat(axis.x, axis.y, axis.z, 0.0f);
    }

    const b3Vec3 cross = vec3(
        from.y * to.z - from.z * to.y,
        from.z * to.x - from.x * to.z,
        from.x * to.y - from.y * to.x);
    return normalizeQuat(quat(cross.x, cross.y, cross.z, 1.0f + dot));
}

b3Vec3 axisVector(int axis) {
    switch (axis) {
        case 0:
            return vec3(1.0f, 0.0f, 0.0f);
        case 1:
            return vec3(0.0f, 1.0f, 0.0f);
        default:
            return vec3(0.0f, 0.0f, 1.0f);
    }
}

b3Vec3 addVec3(b3Vec3 a, b3Vec3 b) {
    return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

b3Vec3 subVec3(b3Vec3 a, b3Vec3 b) {
    return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

b3Vec3 scaleVec3(b3Vec3 value, float scale) {
    return vec3(value.x * scale, value.y * scale, value.z * scale);
}

b3Vec3 crossVec3(b3Vec3 a, b3Vec3 b) {
    return vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x);
}

float dotVec3(b3Vec3 a, b3Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

bool sameVec3(b3Vec3 a, b3Vec3 b) {
    constexpr float epsilon = 1.0e-5f;
    return std::abs(a.x - b.x) <= epsilon && std::abs(a.y - b.y) <= epsilon && std::abs(a.z - b.z) <= epsilon;
}

bool sameQuat(b3Quat a, b3Quat b) {
    constexpr float epsilon = 1.0e-5f;
    return std::abs(a.v.x - b.v.x) <= epsilon
        && std::abs(a.v.y - b.v.y) <= epsilon
        && std::abs(a.v.z - b.v.z) <= epsilon
        && std::abs(a.s - b.s) <= epsilon;
}

bool sameTransform(b3Transform a, b3Transform b) {
    return sameVec3(a.p, b.p) && sameQuat(a.q, b.q);
}

void setBodyMassData(b3BodyId body, float mass, b3Vec3 center, b3Matrix3 inertia);
b3Matrix3 boxInertia(double mass, double hx, double hy, double hz);

b3Vec3 rotateVec3(b3Quat rotation, b3Vec3 value) {
    const b3Vec3 u = rotation.v;
    const float s = rotation.s;
    return addVec3(
        addVec3(scaleVec3(u, 2.0f * dotVec3(u, value)), scaleVec3(value, s * s - dotVec3(u, u))),
        scaleVec3(crossVec3(u, value), 2.0f * s));
}

b3Transform rootedLocalTransform(b3Transform root, b3Vec3 localPosition) {
    return transform(addVec3(root.p, rotateVec3(root.q, localPosition)), root.q);
}

b3Quat axisAngleQuat(b3Vec3 axis, float angle) {
    if (std::abs(angle) <= 1.0e-6f) {
        return quat(0.0f, 0.0f, 0.0f, 1.0f);
    }

    axis = normalizeOr(axis, vec3(1.0f, 0.0f, 0.0f));
    const float halfAngle = angle * 0.5f;
    const float sinHalfAngle = std::sin(halfAngle);
    return quat(axis.x * sinHalfAngle, axis.y * sinHalfAngle, axis.z * sinHalfAngle, std::cos(halfAngle));
}

b3Quat angularTargetQuat(b3Vec3 target) {
    b3Quat rotation = quat(0.0f, 0.0f, 0.0f, 1.0f);
    rotation = mulQuat(rotation, axisAngleQuat(axisVector(0), target.x));
    rotation = mulQuat(rotation, axisAngleQuat(axisVector(1), target.y));
    rotation = mulQuat(rotation, axisAngleQuat(axisVector(2), target.z));
    return rotation;
}

float& vectorComponent(b3Vec3& value, int axis) {
    switch (axis) {
        case 0:
            return value.x;
        case 1:
            return value.y;
        default:
            return value.z;
    }
}

bool isMotorJointConstraint(const ConstraintInfo& info) {
    return info.kind == ConstraintKind::Free || info.kind == ConstraintKind::GenericMotor;
}

MotorSpringSettings motorSpringSettings(jdouble stiffness, jdouble damping) {
    const float springStiffness = std::max(0.0f, static_cast<float>(stiffness));
    const float springDamping = std::max(0.0f, static_cast<float>(damping));
    if (springStiffness <= 1.0e-6f) {
        return MotorSpringSettings{0.0f, springDamping};
    }

    // Rapier's default joint motor model is acceleration-based:
    // acceleration = stiffness * error + damping * velocity_error.
    // Box3D's soft joint model takes natural frequency and damping ratio.
    const float angularFrequency = std::sqrt(springStiffness);
    float hertz = angularFrequency / (2.0f * B3_PI);
    float dampingRatio = springDamping / (2.0f * angularFrequency);
    if (!std::isfinite(hertz)) {
        hertz = 0.0f;
    }
    if (!std::isfinite(dampingRatio)) {
        dampingRatio = 0.0f;
    }

    return MotorSpringSettings{std::clamp(hertz, 0.0f, 120.0f), std::clamp(dampingRatio, 0.0f, 20.0f)};
}

ConstraintFrameInfo applyMotorTargetToFrame(const ConstraintFrameInfo& baseFrame, b3Vec3 linearTarget, b3Vec3 angularTarget) {
    ConstraintFrameInfo frame = baseFrame;
    const b3Vec3 rotatedLinearTarget = rotateVec3(baseFrame.rotation, linearTarget);
    frame.position.x += rotatedLinearTarget.x;
    frame.position.y += rotatedLinearTarget.y;
    frame.position.z += rotatedLinearTarget.z;
    frame.rotation = mulQuat(baseFrame.rotation, angularTargetQuat(angularTarget));
    return frame;
}

void updateMotorTargetFrames(ConstraintInfo& info) {
    info.frameA = info.baseFrameA;
    info.frameB = info.baseFrameB;

    if (isMotorJointConstraint(info)) {
        info.frameA = applyMotorTargetToFrame(info.baseFrameA, info.linearMotorTarget, info.angularMotorTarget);
    }
}

float unboundedMotorLimit() {
    return 1.0e12f;
}

float motorLimit(jboolean hasMaxForce, jdouble maxForce) {
    if (hasMaxForce != 0) {
        return std::max(0.0f, static_cast<float>(maxForce));
    }

    return unboundedMotorLimit();
}

float dampingOnlyMotorLimit(jboolean hasMaxForce, jdouble maxForce, jdouble damping) {
    if (hasMaxForce != 0) {
        return std::max(0.0f, static_cast<float>(maxForce));
    }

    const float dampingForce = std::max(0.0f, static_cast<float>(damping));
    return std::isfinite(dampingForce) ? dampingForce : 0.0f;
}

b3Quat box3dFrameRotation(const ConstraintInfo& info, b3Quat publicRotation) {
    if (info.kind == ConstraintKind::GenericRevolute && info.primaryAxis >= 3) {
        const int axis = info.primaryAxis - 3;
        return mulQuat(publicRotation, rotationBetween(vec3(0.0f, 0.0f, 1.0f), axisVector(axis)));
    }

    if (info.kind == ConstraintKind::GenericPrismatic && info.primaryAxis < 3) {
        return mulQuat(publicRotation, rotationBetween(vec3(1.0f, 0.0f, 0.0f), axisVector(info.primaryAxis)));
    }

    return publicRotation;
}

b3Transform constraintFrame(Box3DScene* scene, int bodyId, double x, double y, double z, b3Quat rotation) {
    const LocalOrigin center = centerOfMassForId(scene, bodyId);
    return transform(
        vec3(
            static_cast<float>(x - center.x),
            static_cast<float>(y - center.y),
            static_cast<float>(z - center.z)),
        normalizeQuat(rotation));
}

ConstraintFrameInfo constraintFrameInfo(double x, double y, double z, b3Quat rotation) {
    return ConstraintFrameInfo{origin(x, y, z), normalizeQuat(rotation)};
}

b3Transform constraintFrame(Box3DScene* scene, int bodyId, const ConstraintFrameInfo& frame) {
    return constraintFrame(scene, bodyId, frame.position.x, frame.position.y, frame.position.z, frame.rotation);
}

void configureJointBase(
    b3JointDef& base,
    b3BodyId bodyA,
    b3BodyId bodyB,
    b3Transform frameA,
    b3Transform frameB,
    bool collideConnected) {
    base.bodyIdA = bodyA;
    base.bodyIdB = bodyB;
    base.localFrameA = frameA;
    base.localFrameB = frameB;
    base.constraintHertz = 550.0f;
    base.constraintDampingRatio = 4.0f;
    base.collideConnected = collideConnected;
}

jlong storeConstraint(Box3DScene* scene, b3JointId joint, ConstraintKind kind, int primaryAxis, int bodyIdA, int bodyIdB, ConstraintFrameInfo frameA, ConstraintFrameInfo frameB) {
    if (B3_IS_NULL(joint) || !b3Joint_IsValid(joint)) {
        return 0;
    }

    const uint64_t key = jointKey(joint);
    ConstraintInfo info{};
    info.joint = joint;
    info.kind = kind;
    info.primaryAxis = primaryAxis;
    info.bodyIdA = bodyIdA;
    info.bodyIdB = bodyIdB;
    info.baseFrameA = frameA;
    info.baseFrameB = frameB;
    updateMotorTargetFrames(info);
    scene->constraints[key] = info;
    return static_cast<jlong>(key);
}

ConstraintInfo* findConstraint(Box3DScene* scene, jlong handle) {
    auto it = scene->constraints.find(static_cast<uint64_t>(handle));
    if (it == scene->constraints.end()) {
        return nullptr;
    }

    if (B3_IS_NULL(it->second.joint) || !b3Joint_IsValid(it->second.joint)) {
        scene->constraints.erase(it);
        return nullptr;
    }

    return &it->second;
}

void refreshConstraint(Box3DScene* scene, ConstraintInfo& info) {
    b3Joint_SetLocalFrameA(info.joint, constraintFrame(scene, info.bodyIdA, info.frameA));
    b3Joint_SetLocalFrameB(info.joint, constraintFrame(scene, info.bodyIdB, info.frameB));
}

void refreshConstraints(Box3DScene* scene) {
    for (auto it = scene->constraints.begin(); it != scene->constraints.end();) {
        ConstraintInfo& info = it->second;
        if (B3_IS_NULL(info.joint) || !b3Joint_IsValid(info.joint)) {
            it = scene->constraints.erase(it);
            continue;
        }

        refreshConstraint(scene, info);
        ++it;
    }
}

void markChunkDirty(Box3DScene* scene, ChunkShapes& chunk, bool global, int bodyId, int sectionX, int sectionY, int sectionZ) {
    if (chunk.dirty) {
        return;
    }

    chunk.dirty = true;
    scene->dirtyChunks.push_back(DirtyChunk{global, bodyId, sectionX, sectionY, sectionZ});
}

void markBodyChunksDirty(Box3DScene* scene, BodyInfo& info, int bodyId) {
    for (auto& entry : info.chunks) {
        ChunkShapes& chunk = entry.second;
        markChunkDirty(scene, chunk, false, bodyId, chunk.sectionX, chunk.sectionY, chunk.sectionZ);
    }
}

int chunkIndex(int x, int y, int z) {
    return x + (z << 4) + (y << 8);
}

int colliderIndexForPackedState(int packedState) {
    const int colliderValue = packedState >> 16;
    return colliderValue <= 0 ? -1 : colliderValue - 1;
}

ColliderMaterialData colliderMaterialForPackedState(int packedState, std::unordered_map<int, ColliderMaterialData>& cache) {
    const int colliderIndex = colliderIndexForPackedState(packedState);
    if (colliderIndex < 0) {
        return ColliderMaterialData{};
    }

    auto cached = cache.find(colliderIndex);
    if (cached != cache.end()) {
        return cached->second;
    }

    ColliderMaterialData data{};
    std::lock_guard<std::mutex> lock(gColliderMutex);
    if (colliderIndex >= 0 && colliderIndex < static_cast<int>(gVoxelColliders.size())) {
        const VoxelCollider& collider = gVoxelColliders[colliderIndex];
        data.valid = true;
        data.fluid = collider.isFluid;
        data.volume = std::isfinite(collider.volume) ? std::max(0.0f, collider.volume) : 1.0f;
    }

    cache.emplace(colliderIndex, data);
    return data;
}

void refreshChunkFluidFlag(ChunkShapes& chunk) {
    std::unordered_map<int, ColliderMaterialData> colliderCache;
    colliderCache.reserve(16);
    chunk.hasFluid = false;

    for (jint packedState : chunk.states) {
        if ((packedState >> 16) <= 0) {
            continue;
        }

        const ColliderMaterialData material = colliderMaterialForPackedState(packedState, colliderCache);
        if (material.valid && material.fluid) {
            chunk.hasFluid = true;
            return;
        }
    }
}

void setAwakeIfRequested(b3BodyId body, bool wake) {
    if (wake && B3_IS_NON_NULL(body) && b3Body_IsValid(body) && b3Body_GetType(body) != b3_staticBody) {
        b3Body_SetAwake(body, true);
    }
}

void wakeConstraintBodies(Box3DScene* scene, const ConstraintInfo& info) {
    setAwakeIfRequested(constraintBodyForId(scene, info.bodyIdA), true);
    setAwakeIfRequested(constraintBodyForId(scene, info.bodyIdB), true);
}

void destroyChunkShapes(ChunkShapes& chunk) {
    for (b3ShapeId shape : chunk.shapes) {
        if (B3_IS_NON_NULL(shape) && b3Shape_IsValid(shape)) {
            b3DestroyShape(shape, false);
        }
    }

    chunk.shapes.clear();
    chunk.shapeData.clear();
}

void destroyChunk(ChunkShapes& chunk) {
    destroyChunkShapes(chunk);
    chunk.dirty = false;

    if (chunk.ownsBody && B3_IS_NON_NULL(chunk.body) && b3Body_IsValid(chunk.body)) {
        b3DestroyBody(chunk.body);
    }

    chunk.body = b3_nullBodyId;
    chunk.ownsBody = false;
}

void destroyBodyInfo(BodyInfo& info) {
    for (auto& entry : info.chunks) {
        destroyChunk(entry.second);
    }
    info.chunks.clear();

    if (info.ownsBody && B3_IS_NON_NULL(info.body) && b3Body_IsValid(info.body)) {
        b3DestroyBody(info.body);
    }

    info.body = b3_nullBodyId;
}

bool isInteriorVoxel(int packedState) {
    return (packedState & 0xFFFF) == 4;
}

bool isFullUnitBox(const VoxelBox& box) {
    constexpr float epsilon = 1.0e-5f;
    return std::abs(box.minX) <= epsilon
        && std::abs(box.minY) <= epsilon
        && std::abs(box.minZ) <= epsilon
        && std::abs(box.maxX - 1.0f) <= epsilon
        && std::abs(box.maxY - 1.0f) <= epsilon
        && std::abs(box.maxZ - 1.0f) <= epsilon;
}

bool isMergeableFullBlock(int packedState) {
    int colliderValue = packedState >> 16;
    if (colliderValue <= 0 || isInteriorVoxel(packedState)) {
        return false;
    }

    const int colliderIndex = colliderValue - 1;
    std::lock_guard<std::mutex> lock(gColliderMutex);
    if (colliderIndex < 0 || colliderIndex >= static_cast<int>(gVoxelColliders.size())) {
        return false;
    }

    const VoxelCollider& collider = gVoxelColliders[colliderIndex];
    return !collider.isFluid
        && collider.contactEvents == nullptr
        && collider.boxes.size() == 1
        && isFullUnitBox(collider.boxes.front());
}

bool isMergeableFullBlock(int packedState, std::unordered_map<int, bool>& cache) {
    auto it = cache.find(packedState);
    if (it != cache.end()) {
        return it->second;
    }

    const bool mergeable = isMergeableFullBlock(packedState);
    cache.emplace(packedState, mergeable);
    return mergeable;
}

bool hasAnyShapeCandidate(const ChunkShapes& chunk) {
    for (jint packedState : chunk.states) {
        if ((packedState >> 16) > 0 && !isInteriorVoxel(packedState)) {
            return true;
        }
    }

    return false;
}

void createBoxShapeForRegion(
    ChunkShapes& chunk,
    b3BodyId body,
    int ownerId,
    int minX,
    int minY,
    int minZ,
    int maxX,
    int maxY,
    int maxZ,
    int packedState,
    LocalOrigin localOrigin,
    b3Transform rootTransform) {
    int colliderValue = packedState >> 16;
    if (colliderValue <= 0 || isInteriorVoxel(packedState)) {
        return;
    }

    const int colliderIndex = colliderValue - 1;
    std::lock_guard<std::mutex> lock(gColliderMutex);
    if (colliderIndex < 0 || colliderIndex >= static_cast<int>(gVoxelColliders.size())) {
        return;
    }

    const VoxelCollider& collider = gVoxelColliders[colliderIndex];
    if (collider.isFluid) {
        return;
    }

    const float hx = static_cast<float>(maxX - minX) * 0.5f;
    const float hy = static_cast<float>(maxY - minY) * 0.5f;
    const float hz = static_cast<float>(maxZ - minZ) * 0.5f;
    if (hx <= 0.0f || hy <= 0.0f || hz <= 0.0f) {
        return;
    }

    const float cx = static_cast<float>((static_cast<double>(minX) + static_cast<double>(maxX)) * 0.5 - localOrigin.x);
    const float cy = static_cast<float>((static_cast<double>(minY) + static_cast<double>(maxY)) * 0.5 - localOrigin.y);
    const float cz = static_cast<float>((static_cast<double>(minZ) + static_cast<double>(maxZ)) * 0.5 - localOrigin.z);

    b3BoxHull hull = b3MakeBoxHull(hx, hy, hz);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = collider.friction;
    shapeDef.baseMaterial.restitution = collider.restitution;
    shapeDef.filter.categoryBits = LEVEL_COLLISION_CATEGORY;
    shapeDef.filter.maskBits = LEVEL_COLLISION_CATEGORY | ROPE_COLLISION_CATEGORY;
    shapeDef.enableHitEvents = true;
    shapeDef.enableContactEvents = true;
    shapeDef.enablePreSolveEvents = ownerId < 0 && collider.contactEvents != nullptr;
    shapeDef.invokeContactCreation = ownerId >= 0;
    shapeDef.updateBodyMass = false;

    ShapeUserData& data = chunk.shapeData.emplace_back();
    data.ownerId = ownerId;
    data.blockX = minX;
    data.blockY = minY;
    data.blockZ = minZ;
    data.colliderIndex = colliderIndex;
    data.voxelState = packedState & 0xFFFF;
    shapeDef.userData = &data;

    b3Transform localTransform = rootedLocalTransform(rootTransform, vec3(cx, cy, cz));
    b3ShapeId shape = b3CreateTransformedHullShape(body, &shapeDef, &hull.base, localTransform, vec3(1.0f, 1.0f, 1.0f));
    if (B3_IS_NON_NULL(shape)) {
        chunk.shapes.push_back(shape);
    } else {
        chunk.shapeData.pop_back();
    }
}

void createBoxShapeForBlock(Box3DScene* scene, ChunkShapes& chunk, b3BodyId body, int ownerId, int blockX, int blockY, int blockZ, int packedState, LocalOrigin localOrigin, b3Transform rootTransform) {
    int colliderValue = packedState >> 16;
    if (colliderValue <= 0) {
        return;
    }

    const int voxelState = packedState & 0xFFFF;
    // Match Rapier's LevelCollider path: fully interior blocks should not produce ordinary
    // body-vs-body contacts. Keeping them as Box3D shapes is both expensive and noisy.
    if (isInteriorVoxel(packedState)) {
        return;
    }

    const int colliderIndex = colliderValue - 1;
    std::lock_guard<std::mutex> lock(gColliderMutex);
    if (colliderIndex < 0 || colliderIndex >= static_cast<int>(gVoxelColliders.size())) {
        return;
    }

    const VoxelCollider& collider = gVoxelColliders[colliderIndex];
    if (collider.isFluid || collider.boxes.empty()) {
        return;
    }

    for (const VoxelBox& box : collider.boxes) {
        const float hx = (box.maxX - box.minX) * 0.5f;
        const float hy = (box.maxY - box.minY) * 0.5f;
        const float hz = (box.maxZ - box.minZ) * 0.5f;

        if (hx <= 0.0f || hy <= 0.0f || hz <= 0.0f) {
            continue;
        }

        const float cx = static_cast<float>(static_cast<double>(blockX) + (static_cast<double>(box.minX) + static_cast<double>(box.maxX)) * 0.5 - localOrigin.x);
        const float cy = static_cast<float>(static_cast<double>(blockY) + (static_cast<double>(box.minY) + static_cast<double>(box.maxY)) * 0.5 - localOrigin.y);
        const float cz = static_cast<float>(static_cast<double>(blockZ) + (static_cast<double>(box.minZ) + static_cast<double>(box.maxZ)) * 0.5 - localOrigin.z);

        b3BoxHull hull = b3MakeBoxHull(hx, hy, hz);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.friction = collider.friction;
        shapeDef.baseMaterial.restitution = collider.restitution;
        shapeDef.filter.categoryBits = LEVEL_COLLISION_CATEGORY;
        shapeDef.filter.maskBits = LEVEL_COLLISION_CATEGORY | ROPE_COLLISION_CATEGORY;
        shapeDef.enableHitEvents = true;
        shapeDef.enableContactEvents = true;
        shapeDef.enablePreSolveEvents = ownerId < 0 && collider.contactEvents != nullptr;
        shapeDef.invokeContactCreation = ownerId >= 0;
        shapeDef.updateBodyMass = false;

        ShapeUserData& data = chunk.shapeData.emplace_back();
        data.ownerId = ownerId;
        data.blockX = blockX;
        data.blockY = blockY;
        data.blockZ = blockZ;
        data.colliderIndex = colliderIndex;
        data.voxelState = voxelState;
        shapeDef.userData = &data;

        b3Transform localTransform = rootedLocalTransform(rootTransform, vec3(cx, cy, cz));
        b3ShapeId shape = b3CreateTransformedHullShape(body, &shapeDef, &hull.base, localTransform, vec3(1.0f, 1.0f, 1.0f));
        if (B3_IS_NON_NULL(shape)) {
            chunk.shapes.push_back(shape);
        } else {
            chunk.shapeData.pop_back();
        }
    }
}

void rebuildChunkShapes(Box3DScene* scene, ChunkShapes& chunk, int ownerId, int sectionX, int sectionY, int sectionZ, LocalOrigin localOrigin, b3Transform rootTransform) {
    destroyChunkShapes(chunk);
    chunk.dirty = false;

    if (B3_IS_NULL(chunk.body) || !b3Body_IsValid(chunk.body)) {
        return;
    }

    if (!hasAnyShapeCandidate(chunk)) {
        return;
    }

    const int baseX = sectionX << 4;
    const int baseY = sectionY << 4;
    const int baseZ = sectionZ << 4;

    std::unordered_map<int, bool> mergeableStateCache;
    mergeableStateCache.reserve(16);
    std::array<bool, 4096> mergeable{};
    std::array<bool, 4096> consumed{};
    for (int bx = 0; bx < 16; ++bx) {
        for (int bz = 0; bz < 16; ++bz) {
            for (int by = 0; by < 16; ++by) {
                const int index = chunkIndex(bx, by, bz);
                mergeable[index] = isMergeableFullBlock(chunk.states[index], mergeableStateCache);
            }
        }
    }

    auto canMerge = [&](int bx, int by, int bz, int packedState) {
        const int index = chunkIndex(bx, by, bz);
        return !consumed[index] && mergeable[index] && chunk.states[index] == packedState;
    };

    auto canMergeZRange = [&](int bx0, int bx1, int by, int bz, int packedState) {
        for (int bx = bx0; bx < bx1; ++bx) {
            if (!canMerge(bx, by, bz, packedState)) {
                return false;
            }
        }
        return true;
    };

    auto canMergeYRange = [&](int bx0, int bx1, int by, int bz0, int bz1, int packedState) {
        for (int bz = bz0; bz < bz1; ++bz) {
            for (int bx = bx0; bx < bx1; ++bx) {
                if (!canMerge(bx, by, bz, packedState)) {
                    return false;
                }
            }
        }
        return true;
    };

    for (int by = 0; by < 16; ++by) {
        for (int bz = 0; bz < 16; ++bz) {
            for (int bx = 0; bx < 16; ++bx) {
                const int index = chunkIndex(bx, by, bz);
                if (consumed[index] || !mergeable[index]) {
                    continue;
                }

                const int packedState = chunk.states[index];
                int bx1 = bx + 1;
                while (bx1 < 16 && canMerge(bx1, by, bz, packedState)) {
                    ++bx1;
                }

                int bz1 = bz + 1;
                while (bz1 < 16 && canMergeZRange(bx, bx1, by, bz1, packedState)) {
                    ++bz1;
                }

                int by1 = by + 1;
                while (by1 < 16 && canMergeYRange(bx, bx1, by1, bz, bz1, packedState)) {
                    ++by1;
                }

                for (int my = by; my < by1; ++my) {
                    for (int mz = bz; mz < bz1; ++mz) {
                        for (int mx = bx; mx < bx1; ++mx) {
                            consumed[chunkIndex(mx, my, mz)] = true;
                        }
                    }
                }

                createBoxShapeForRegion(
                    chunk,
                    chunk.body,
                    ownerId,
                    baseX + bx,
                    baseY + by,
                    baseZ + bz,
                    baseX + bx1,
                    baseY + by1,
                    baseZ + bz1,
                    packedState,
                    localOrigin,
                    rootTransform);
            }
        }
    }

    for (int bx = 0; bx < 16; ++bx) {
        for (int bz = 0; bz < 16; ++bz) {
            for (int by = 0; by < 16; ++by) {
                const int index = chunkIndex(bx, by, bz);
                if (consumed[index]) {
                    continue;
                }

                createBoxShapeForBlock(
                    scene,
                    chunk,
                    chunk.body,
                    ownerId,
                    baseX + bx,
                    baseY + by,
                    baseZ + bz,
                    chunk.states[index],
                    localOrigin,
                    rootTransform);
            }
        }
    }
}

void rebuildBodyChunks(Box3DScene* scene, BodyInfo& info, int ownerId) {
    for (auto& entry : info.chunks) {
        ChunkShapes& chunk = entry.second;
        rebuildChunkShapes(scene, chunk, ownerId, chunk.sectionX, chunk.sectionY, chunk.sectionZ, info.centerOfMass, info.shapeRoot);
    }
}

void flushDirtyChunks(Box3DScene* scene) {
    if (scene->dirtyChunks.empty()) {
        return;
    }

    std::vector<DirtyChunk> dirtyChunks;
    dirtyChunks.swap(scene->dirtyChunks);

    for (const DirtyChunk& dirty : dirtyChunks) {
        const int64_t key = packSectionPos(dirty.sectionX, dirty.sectionY, dirty.sectionZ);
        if (dirty.global) {
            auto chunkIt = scene->globalChunks.find(key);
            if (chunkIt == scene->globalChunks.end() || !chunkIt->second.dirty) {
                continue;
            }

            rebuildChunkShapes(
                scene,
                chunkIt->second,
                -1,
                dirty.sectionX,
                dirty.sectionY,
                dirty.sectionZ,
                origin(
                    static_cast<double>(dirty.sectionX << 4),
                    static_cast<double>(dirty.sectionY << 4),
                    static_cast<double>(dirty.sectionZ << 4)),
                transform(vec3(0.0f, 0.0f, 0.0f), quat(0.0f, 0.0f, 0.0f, 1.0f)));
            continue;
        }

        auto bodyIt = scene->bodies.find(dirty.bodyId);
        if (bodyIt == scene->bodies.end()) {
            continue;
        }

        auto chunkIt = bodyIt->second.chunks.find(key);
        if (chunkIt == bodyIt->second.chunks.end() || !chunkIt->second.dirty) {
            continue;
        }

        rebuildChunkShapes(
            scene,
            chunkIt->second,
            dirty.bodyId,
            dirty.sectionX,
            dirty.sectionY,
            dirty.sectionZ,
            bodyIt->second.centerOfMass,
            bodyIt->second.shapeRoot);
    }
}

void addChunkToBody(Box3DScene* scene, BodyInfo& info, int ownerId, int sectionX, int sectionY, int sectionZ, jintArray chunkArray, JNIEnv* env) {
    const int64_t key = packSectionPos(sectionX, sectionY, sectionZ);
    auto existing = info.chunks.find(key);
    if (existing != info.chunks.end()) {
        destroyChunk(existing->second);
        info.chunks.erase(existing);
    }

    ChunkShapes chunk;
    chunk.body = info.body;
    chunk.ownsBody = false;
    chunk.sectionX = sectionX;
    chunk.sectionY = sectionY;
    chunk.sectionZ = sectionZ;
    env->GetIntArrayRegion(chunkArray, 0, static_cast<jsize>(chunk.states.size()), chunk.states.data());
    refreshChunkFluidFlag(chunk);

    rebuildChunkShapes(scene, chunk, ownerId, sectionX, sectionY, sectionZ, info.centerOfMass, info.shapeRoot);
    info.chunks[key] = std::move(chunk);
}

void addGlobalChunk(Box3DScene* scene, int sectionX, int sectionY, int sectionZ, jintArray chunkArray, JNIEnv* env) {
    const int64_t key = packSectionPos(sectionX, sectionY, sectionZ);

    auto existing = scene->globalChunks.find(key);
    if (existing != scene->globalChunks.end()) {
        destroyChunk(existing->second);
        scene->globalChunks.erase(existing);
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_staticBody;
    const LocalOrigin localOrigin = origin(
        static_cast<double>(sectionX << 4),
        static_cast<double>(sectionY << 4),
        static_cast<double>(sectionZ << 4));
    bodyDef.position = b3Pos{localOrigin.x, localOrigin.y, localOrigin.z};
    bodyDef.rotation = quat(0.0f, 0.0f, 0.0f, 1.0f);

    ChunkShapes chunk;
    chunk.body = b3CreateBody(scene->world, &bodyDef);
    chunk.ownsBody = true;
    chunk.sectionX = sectionX;
    chunk.sectionY = sectionY;
    chunk.sectionZ = sectionZ;
    env->GetIntArrayRegion(chunkArray, 0, static_cast<jsize>(chunk.states.size()), chunk.states.data());
    refreshChunkFluidFlag(chunk);

    rebuildChunkShapes(scene, chunk, -1, sectionX, sectionY, sectionZ, localOrigin, transform(vec3(0.0f, 0.0f, 0.0f), quat(0.0f, 0.0f, 0.0f, 1.0f)));
    scene->globalChunks[key] = std::move(chunk);
}

void setBodyMassData(b3BodyId body, float mass, b3Vec3 center, b3Matrix3 inertia) {
    if (B3_IS_NULL(body) || !b3Body_IsValid(body)) {
        return;
    }

    if (mass <= 0.0f || !std::isfinite(mass)) {
        return;
    }

    b3MassData massData;
    massData.mass = mass;
    massData.center = center;
    massData.inertia = inertia;
    b3Body_SetMassData(body, massData);
}

bool sameOrigin(LocalOrigin a, LocalOrigin b) {
    constexpr double epsilon = 1.0e-9;
    return std::abs(a.x - b.x) <= epsilon && std::abs(a.y - b.y) <= epsilon && std::abs(a.z - b.z) <= epsilon;
}

b3Matrix3 boxInertia(double mass, double hx, double hy, double hz) {
    const float x = static_cast<float>(2.0 * hx);
    const float y = static_cast<float>(2.0 * hy);
    const float z = static_cast<float>(2.0 * hz);
    const float m = static_cast<float>(mass);

    b3Matrix3 inertia{};
    inertia.cx = vec3((m / 12.0f) * (y * y + z * z), 0.0f, 0.0f);
    inertia.cy = vec3(0.0f, (m / 12.0f) * (x * x + z * z), 0.0f);
    inertia.cz = vec3(0.0f, 0.0f, (m / 12.0f) * (x * x + y * y));
    return inertia;
}

void createBody(Box3DScene* scene, int id, b3BodyType type, const std::array<double, 7>& pose) {
    BodyInfo info;

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = type;
    bodyDef.position = b3Pos{pose[0], pose[1], pose[2]};
    bodyDef.rotation = quat(static_cast<float>(pose[3]), static_cast<float>(pose[4]), static_cast<float>(pose[5]), static_cast<float>(pose[6]));
    bodyDef.linearDamping = scene->universalDrag;
    bodyDef.angularDamping = scene->universalDrag;
    bodyDef.userData = reinterpret_cast<void*>(static_cast<intptr_t>(id));
    bodyDef.enableSleep = true;
    bodyDef.isAwake = true;
    bodyDef.isBullet = type == b3_dynamicBody;
    bodyDef.enableContactRecycling = true;

    info.body = b3CreateBody(scene->world, &bodyDef);
    scene->bodies[id] = std::move(info);
}

b3Transform identityFrame() {
    return transform(vec3(0.0f, 0.0f, 0.0f), quat(0.0f, 0.0f, 0.0f, 1.0f));
}

bool isValidBody(b3BodyId body) {
    return B3_IS_NON_NULL(body) && b3Body_IsValid(body);
}

bool isValidJoint(b3JointId joint) {
    return B3_IS_NON_NULL(joint) && b3Joint_IsValid(joint);
}

b3BodyId createRopePointBody(Box3DScene* scene, double x, double y, double z, double pointRadius) {
    const float radius = std::max(B3_LINEAR_SLOP, static_cast<float>(pointRadius));

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{x, y, z};
    bodyDef.rotation = quat(0.0f, 0.0f, 0.0f, 1.0f);
    bodyDef.linearDamping = scene->universalDrag;
    bodyDef.angularDamping = scene->universalDrag;
    bodyDef.motionLocks.angularX = true;
    bodyDef.motionLocks.angularY = true;
    bodyDef.motionLocks.angularZ = true;
    bodyDef.enableSleep = true;
    bodyDef.isAwake = true;
    bodyDef.isBullet = true;
    bodyDef.enableContactRecycling = true;

    b3BodyId body = b3CreateBody(scene->world, &bodyDef);
    if (!isValidBody(body)) {
        return b3_nullBodyId;
    }

    b3BoxHull hull = b3MakeBoxHull(radius, radius, radius);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = ROPE_POINT_FRICTION;
    shapeDef.filter.categoryBits = ROPE_COLLISION_CATEGORY;
    shapeDef.filter.maskBits = LEVEL_COLLISION_CATEGORY;
    shapeDef.updateBodyMass = false;
    shapeDef.enableHitEvents = false;
    shapeDef.enableContactEvents = false;

    b3ShapeId shape = b3CreateHullShape(body, &shapeDef, &hull.base);
    if (B3_IS_NULL(shape)) {
        b3DestroyBody(body);
        return b3_nullBodyId;
    }

    setBodyMassData(body, ROPE_POINT_MASS, vec3(0.0f, 0.0f, 0.0f), boxInertia(ROPE_POINT_MASS, radius, radius, radius));
    return body;
}

float ropeSegmentLength(double length) {
    const float value = static_cast<float>(length);
    if (!std::isfinite(value) || value <= 0.0f) {
        return B3_LINEAR_SLOP;
    }

    return std::max(B3_LINEAR_SLOP, value);
}

RopeSegmentInfo createRopeSegment(Box3DScene* scene, b3BodyId bodyA, b3BodyId bodyB, double length) {
    RopeSegmentInfo segment{};
    if (!isValidBody(bodyA) || !isValidBody(bodyB)) {
        return segment;
    }

    segment.bodyA = bodyA;
    segment.bodyB = bodyB;

    const float maxLength = ropeSegmentLength(length);
    b3DistanceJointDef distanceDef = b3DefaultDistanceJointDef();
    configureJointBase(distanceDef.base, bodyA, bodyB, identityFrame(), identityFrame(), false);
    distanceDef.length = maxLength;
    // Box3D only honors distance limits when spring mode is enabled. With hertz zero
    // this behaves as a one-sided max-length rope limit, not a rest-length spring.
    distanceDef.enableSpring = true;
    distanceDef.hertz = 0.0f;
    distanceDef.dampingRatio = 0.0f;
    distanceDef.enableLimit = true;
    distanceDef.minLength = B3_LINEAR_SLOP;
    distanceDef.maxLength = maxLength;
    segment.distanceJoint = b3CreateDistanceJoint(scene->world, &distanceDef);

    return segment;
}

void setRopeSegmentLength(const RopeSegmentInfo& segment, double length) {
    if (!isValidJoint(segment.distanceJoint)) {
        return;
    }

    const float maxLength = ropeSegmentLength(length);
    b3DistanceJoint_SetLength(segment.distanceJoint, maxLength);
    b3DistanceJoint_EnableLimit(segment.distanceJoint, true);
    b3DistanceJoint_SetLengthRange(segment.distanceJoint, B3_LINEAR_SLOP, maxLength);
}

void destroyJointIfValid(b3JointId joint) {
    if (isValidJoint(joint)) {
        b3DestroyJoint(joint, true);
    }
}

void destroyRopeSegment(const RopeSegmentInfo& segment) {
    destroyJointIfValid(segment.distanceJoint);
}

void destroyRopeAttachment(RopeAttachmentInfo& attachment) {
    destroyJointIfValid(attachment.joint);
    attachment.joint = b3_nullJointId;
}

RopeAttachmentInfo createRopeAttachment(Box3DScene* scene, int bodyId, ConstraintFrameInfo frame, b3BodyId ropeBody) {
    RopeAttachmentInfo attachment{};
    attachment.bodyId = bodyId;
    attachment.frame = frame;

    b3BodyId anchorBody = constraintBodyForId(scene, bodyId);
    if (!isValidBody(anchorBody) || !isValidBody(ropeBody)) {
        return attachment;
    }

    b3DistanceJointDef def = b3DefaultDistanceJointDef();
    configureJointBase(def.base, anchorBody, ropeBody, constraintFrame(scene, bodyId, frame), identityFrame(), false);
    def.length = B3_LINEAR_SLOP;
    def.enableSpring = false;
    attachment.joint = b3CreateDistanceJoint(scene->world, &def);
    return attachment;
}

bool refreshRopeAttachment(Box3DScene* scene, RopeAttachmentInfo& attachment, b3BodyId ropeBody) {
    if (!isValidJoint(attachment.joint) || !isValidBody(ropeBody)) {
        attachment.joint = b3_nullJointId;
        return false;
    }

    b3BodyId anchorBody = constraintBodyForId(scene, attachment.bodyId);
    if (!isValidBody(anchorBody)) {
        destroyRopeAttachment(attachment);
        return false;
    }

    b3Joint_SetLocalFrameA(attachment.joint, constraintFrame(scene, attachment.bodyId, attachment.frame));
    b3Joint_SetLocalFrameB(attachment.joint, identityFrame());
    return true;
}

void refreshRopeAttachments(Box3DScene* scene) {
    for (auto& entry : scene->ropes) {
        RopeInfo& rope = entry.second;
        if (rope.hasStartAttachment) {
            if (rope.points.empty() || !refreshRopeAttachment(scene, rope.startAttachment, rope.points.front())) {
                rope.hasStartAttachment = false;
            }
        }

        if (rope.hasEndAttachment) {
            if (rope.points.empty() || !refreshRopeAttachment(scene, rope.endAttachment, rope.points.back())) {
                rope.hasEndAttachment = false;
            }
        }
    }
}

void applyRopeSegmentDamping(Box3DScene* scene) {
    for (auto& entry : scene->ropes) {
        RopeInfo& rope = entry.second;
        for (const RopeSegmentInfo& segment : rope.segments) {
            if (!isValidBody(segment.bodyA) || !isValidBody(segment.bodyB)) {
                continue;
            }

            const b3Vec3 velocityA = b3Body_GetLinearVelocity(segment.bodyA);
            const b3Vec3 velocityB = b3Body_GetLinearVelocity(segment.bodyB);
            const b3Vec3 relativeVelocity = subVec3(velocityB, velocityA);
            const b3Vec3 forceOnA = scaleVec3(relativeVelocity, ROPE_RELATIVE_DAMPING);
            if (dotVec3(forceOnA, forceOnA) <= 1.0e-8f) {
                continue;
            }

            b3Body_ApplyForceToCenter(segment.bodyA, forceOnA, true);
            b3Body_ApplyForceToCenter(segment.bodyB, scaleVec3(forceOnA, -1.0f), true);
        }
    }
}

void destroyRopeInfo(RopeInfo& rope) {
    if (rope.hasStartAttachment) {
        destroyRopeAttachment(rope.startAttachment);
        rope.hasStartAttachment = false;
    }
    if (rope.hasEndAttachment) {
        destroyRopeAttachment(rope.endAttachment);
        rope.hasEndAttachment = false;
    }

    for (const RopeSegmentInfo& segment : rope.segments) {
        destroyRopeSegment(segment);
    }
    rope.segments.clear();

    for (b3BodyId body : rope.points) {
        if (isValidBody(body)) {
            b3DestroyBody(body);
        }
    }
    rope.points.clear();
}

RopeInfo* findRope(Box3DScene* scene, jlong ropeHandle) {
    auto it = scene->ropes.find(static_cast<uint64_t>(ropeHandle));
    return it == scene->ropes.end() ? nullptr : &it->second;
}

ShapeUserData* shapeUserData(b3ShapeId shape) {
    if (B3_IS_NULL(shape) || !b3Shape_IsValid(shape)) {
        return nullptr;
    }

    return static_cast<ShapeUserData*>(b3Shape_GetUserData(shape));
}

b3Vec3 localPointForShape(b3ShapeId shape, b3Pos point) {
    b3BodyId body = b3Shape_GetBody(shape);
    if (B3_IS_NULL(body) || !b3Body_IsValid(body)) {
        return vec3(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z));
    }

    return b3Body_GetLocalPoint(body, point);
}

b3Vec3 localVectorForShape(b3ShapeId shape, b3Vec3 vector) {
    b3BodyId body = b3Shape_GetBody(shape);
    if (B3_IS_NULL(body) || !b3Body_IsValid(body)) {
        return vector;
    }

    return b3Body_GetLocalVector(body, vector);
}

struct JniEnvScope {
    JNIEnv* env = nullptr;
    bool attached = false;

    JniEnvScope() {
        if (gJvm == nullptr) {
            return;
        }

        void* rawEnv = nullptr;
        const jint status = gJvm->GetEnv(&rawEnv, JNI_VERSION_1_8);
        if (status == JNI_OK) {
            env = static_cast<JNIEnv*>(rawEnv);
            return;
        }

        if (status == JNI_EDETACHED && gJvm->AttachCurrentThread(&rawEnv, nullptr) == JNI_OK) {
            env = static_cast<JNIEnv*>(rawEnv);
            attached = true;
        }
    }

    ~JniEnvScope() {
        if (attached && gJvm != nullptr) {
            gJvm->DetachCurrentThread();
        }
    }
};

double contactImpactVelocity(b3ShapeId shapeA, b3ShapeId shapeB, b3Pos point, b3Vec3 normal) {
    const b3BodyId bodyA = b3Shape_GetBody(shapeA);
    const b3BodyId bodyB = b3Shape_GetBody(shapeB);
    const b3Vec3 velocityA = isValidBody(bodyA) ? b3Body_GetWorldPointVelocity(bodyA, point) : vec3(0.0f, 0.0f, 0.0f);
    const b3Vec3 velocityB = isValidBody(bodyB) ? b3Body_GetWorldPointVelocity(bodyB, point) : vec3(0.0f, 0.0f, 0.0f);
    return std::abs(static_cast<double>(dotVec3(subVec3(velocityA, velocityB), normal)));
}

bool invokeBlockCollisionCallback(
    Box3DScene*,
    const ShapeUserData* data,
    const ShapeUserData* otherData,
    b3Pos point,
    double impactVelocity) {
    // Rapier skips block callbacks for level colliders that own chunk data. Match
    // that behavior by only invoking callbacks on terrain/global block shapes.
    if (data == nullptr || data->ownerId >= 0 || data->colliderIndex < 0) {
        return false;
    }

    jobject contactEvents = nullptr;
    jmethodID contactMethod = nullptr;
    {
        std::lock_guard<std::mutex> lock(gColliderMutex);
        if (data->colliderIndex >= static_cast<int>(gVoxelColliders.size())) {
            return false;
        }

        const VoxelCollider& collider = gVoxelColliders[data->colliderIndex];
        contactEvents = collider.contactEvents;
        contactMethod = collider.contactMethod;
    }

    if (contactEvents == nullptr || contactMethod == nullptr) {
        return false;
    }

    JniEnvScope scope;
    JNIEnv* env = scope.env;
    if (env == nullptr) {
        return false;
    }

    const bool hasOtherBlock = otherData != nullptr && otherData->colliderIndex >= 0;
    jobject result = env->CallObjectMethod(
        contactEvents,
        contactMethod,
        static_cast<jint>(data->blockX),
        static_cast<jint>(data->blockY),
        static_cast<jint>(data->blockZ),
        static_cast<jint>(hasOtherBlock ? otherData->blockX : 0),
        static_cast<jint>(hasOtherBlock ? otherData->blockY : 0),
        static_cast<jint>(hasOtherBlock ? otherData->blockZ : 0),
        static_cast<jdouble>(point.x),
        static_cast<jdouble>(point.y),
        static_cast<jdouble>(point.z),
        static_cast<jdouble>(impactVelocity),
        static_cast<jboolean>(hasOtherBlock ? JNI_TRUE : JNI_FALSE));

    if (env->ExceptionCheck()) {
        env->ExceptionDescribe();
        env->ExceptionClear();
        return false;
    }

    if (result == nullptr) {
        return false;
    }

    jdouble values[4] = {0.0, 0.0, 0.0, 0.0};
    auto resultArray = static_cast<jdoubleArray>(result);
    const jsize length = env->GetArrayLength(resultArray);
    if (length > 0) {
        env->GetDoubleArrayRegion(resultArray, 0, std::min<jsize>(length, 4), values);
    }
    env->DeleteLocalRef(result);

    return values[3] > 0.0;
}

bool preSolveContact(b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Pos point, b3Vec3 normal, void* context) {
    auto* scene = static_cast<Box3DScene*>(context);
    if (scene == nullptr) {
        return true;
    }

    const ShapeUserData* dataA = shapeUserData(shapeIdA);
    const ShapeUserData* dataB = shapeUserData(shapeIdB);
    if (dataA == nullptr && dataB == nullptr) {
        return true;
    }

    const double impactVelocity = contactImpactVelocity(shapeIdA, shapeIdB, point, normal);
    const bool removeA = invokeBlockCollisionCallback(scene, dataA, dataB, point, impactVelocity);
    const bool removeB = invokeBlockCollisionCallback(scene, dataB, dataA, point, impactVelocity);
    return !(removeA || removeB);
}

void appendCollision(Box3DScene* scene, const b3ContactHitEvent& hit) {
    ShapeUserData* dataA = shapeUserData(hit.shapeIdA);
    ShapeUserData* dataB = shapeUserData(hit.shapeIdB);

    const int idA = dataA == nullptr ? -1 : dataA->ownerId;
    const int idB = dataB == nullptr ? -1 : dataB->ownerId;

    b3Vec3 localNormalA = localVectorForShape(hit.shapeIdA, hit.normal);
    b3Vec3 localNormalB = localVectorForShape(hit.shapeIdB, vec3(-hit.normal.x, -hit.normal.y, -hit.normal.z));
    b3Vec3 localPointA = localPointForShape(hit.shapeIdA, hit.point);
    b3Vec3 localPointB = localPointForShape(hit.shapeIdB, hit.point);

    scene->collisions.insert(scene->collisions.end(), {
        static_cast<double>(idA),
        static_cast<double>(idB),
        static_cast<double>(hit.approachSpeed),
        localNormalA.x,
        localNormalA.y,
        localNormalA.z,
        localNormalB.x,
        localNormalB.y,
        localNormalB.z,
        localPointA.x,
        localPointA.y,
        localPointA.z,
        localPointB.x,
        localPointB.y,
        localPointB.z,
    });
}

void collectCollisions(Box3DScene* scene) {
    b3ContactEvents events = b3World_GetContactEvents(scene->world);
    for (int i = 0; i < events.hitCount; ++i) {
        appendCollision(scene, events.hitEvents[i]);
    }
}

bool hasGlobalFluid(Box3DScene* scene) {
    for (const auto& entry : scene->globalChunks) {
        if (entry.second.hasFluid) {
            return true;
        }
    }

    return false;
}

float axisOverlap(double minA, double maxA, double minB, double maxB) {
    const double overlap = std::min(maxA, maxB) - std::max(minA, minB);
    return overlap <= 0.0 ? 0.0f : static_cast<float>(overlap);
}

float overlapVolume(
    double sampleMinX,
    double sampleMinY,
    double sampleMinZ,
    double sampleMaxX,
    double sampleMaxY,
    double sampleMaxZ,
    int blockX,
    int blockY,
    int blockZ) {
    const float x = axisOverlap(sampleMinX, sampleMaxX, static_cast<double>(blockX), static_cast<double>(blockX + 1));
    const float y = axisOverlap(sampleMinY, sampleMaxY, static_cast<double>(blockY), static_cast<double>(blockY + 1));
    const float z = axisOverlap(sampleMinZ, sampleMaxZ, static_cast<double>(blockZ), static_cast<double>(blockZ + 1));
    return x * y * z;
}

bool isGlobalFluidBlock(Box3DScene* scene, int blockX, int blockY, int blockZ, std::unordered_map<int, ColliderMaterialData>& colliderCache) {
    const int sectionX = blockX >> 4;
    const int sectionY = blockY >> 4;
    const int sectionZ = blockZ >> 4;
    auto chunkIt = scene->globalChunks.find(packSectionPos(sectionX, sectionY, sectionZ));
    if (chunkIt == scene->globalChunks.end() || !chunkIt->second.hasFluid) {
        return false;
    }

    const int index = chunkIndex(blockX & 15, blockY & 15, blockZ & 15);
    const ColliderMaterialData material = colliderMaterialForPackedState(chunkIt->second.states[index], colliderCache);
    return material.valid && material.fluid;
}

void applyFluidForSample(
    Box3DScene* scene,
    b3BodyId body,
    b3Pos point,
    float sampleHalfExtent,
    float blockVolume,
    std::unordered_map<int, ColliderMaterialData>& colliderCache) {
    const double sampleMinX = point.x - static_cast<double>(sampleHalfExtent);
    const double sampleMinY = point.y - static_cast<double>(sampleHalfExtent);
    const double sampleMinZ = point.z - static_cast<double>(sampleHalfExtent);
    const double sampleMaxX = point.x + static_cast<double>(sampleHalfExtent);
    const double sampleMaxY = point.y + static_cast<double>(sampleHalfExtent);
    const double sampleMaxZ = point.z + static_cast<double>(sampleHalfExtent);

    const int minX = static_cast<int>(std::floor(sampleMinX));
    const int minY = static_cast<int>(std::floor(sampleMinY));
    const int minZ = static_cast<int>(std::floor(sampleMinZ));
    const int maxX = static_cast<int>(std::ceil(sampleMaxX) - 1.0);
    const int maxY = static_cast<int>(std::ceil(sampleMaxY) - 1.0);
    const int maxZ = static_cast<int>(std::ceil(sampleMaxZ) - 1.0);

    const b3Vec3 pointVelocity = b3Body_GetWorldPointVelocity(body, point);
    for (int y = minY; y <= maxY; ++y) {
        for (int z = minZ; z <= maxZ; ++z) {
            for (int x = minX; x <= maxX; ++x) {
                if (!isGlobalFluidBlock(scene, x, y, z, colliderCache)) {
                    continue;
                }

                const float volume = overlapVolume(sampleMinX, sampleMinY, sampleMinZ, sampleMaxX, sampleMaxY, sampleMaxZ, x, y, z);
                if (volume <= 0.0f) {
                    continue;
                }

                const b3Vec3 drag = scaleVec3(pointVelocity, -1.7f * volume);
                if (dotVec3(drag, drag) > 1.0e-12f) {
                    b3Body_ApplyForce(body, drag, point, false);
                }

                const b3Vec3 lift = vec3(0.0f, 10.5f * volume * blockVolume, 0.0f);
                if (lift.y > 0.0f) {
                    b3Body_ApplyForce(body, lift, point, false);
                }
            }
        }
    }
}

bool usesComplexFluidSampling(const BodyInfo& info) {
    if (!info.localBounds.valid) {
        return false;
    }

    const int span = (info.localBounds.maxX - info.localBounds.minX)
        + (info.localBounds.maxY - info.localBounds.minY)
        + (info.localBounds.maxZ - info.localBounds.minZ);
    return span < 10;
}

bool bodyBoundsOverlapGlobalFluid(Box3DScene* scene, const BodyInfo& info) {
    if (!info.localBounds.valid) {
        return true;
    }

    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double minZ = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    double maxZ = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < 8; ++i) {
        const double x = (i & 1) == 0 ? static_cast<double>(info.localBounds.minX) : static_cast<double>(info.localBounds.maxX + 1);
        const double y = (i & 2) == 0 ? static_cast<double>(info.localBounds.minY) : static_cast<double>(info.localBounds.maxY + 1);
        const double z = (i & 4) == 0 ? static_cast<double>(info.localBounds.minZ) : static_cast<double>(info.localBounds.maxZ + 1);
        const b3Vec3 localPoint = vec3(
            static_cast<float>(x - info.centerOfMass.x),
            static_cast<float>(y - info.centerOfMass.y),
            static_cast<float>(z - info.centerOfMass.z));
        const b3Pos worldPoint = b3Body_GetWorldPoint(info.body, localPoint);

        minX = std::min(minX, worldPoint.x);
        minY = std::min(minY, worldPoint.y);
        minZ = std::min(minZ, worldPoint.z);
        maxX = std::max(maxX, worldPoint.x);
        maxY = std::max(maxY, worldPoint.y);
        maxZ = std::max(maxZ, worldPoint.z);
    }

    const int sectionMinX = static_cast<int>(std::floor(minX - 1.0)) >> 4;
    const int sectionMinY = static_cast<int>(std::floor(minY - 1.0)) >> 4;
    const int sectionMinZ = static_cast<int>(std::floor(minZ - 1.0)) >> 4;
    const int sectionMaxX = static_cast<int>(std::floor(maxX + 1.0)) >> 4;
    const int sectionMaxY = static_cast<int>(std::floor(maxY + 1.0)) >> 4;
    const int sectionMaxZ = static_cast<int>(std::floor(maxZ + 1.0)) >> 4;

    for (int y = sectionMinY; y <= sectionMaxY; ++y) {
        for (int z = sectionMinZ; z <= sectionMaxZ; ++z) {
            for (int x = sectionMinX; x <= sectionMaxX; ++x) {
                auto chunkIt = scene->globalChunks.find(packSectionPos(x, y, z));
                if (chunkIt != scene->globalChunks.end() && chunkIt->second.hasFluid) {
                    return true;
                }
            }
        }
    }

    return false;
}

void applyBuoyancyForBlock(
    Box3DScene* scene,
    BodyInfo& info,
    int blockX,
    int blockY,
    int blockZ,
    float blockVolume,
    bool complex,
    std::unordered_map<int, ColliderMaterialData>& colliderCache) {
    const b3Vec3 localCenter = vec3(
        static_cast<float>(static_cast<double>(blockX) + 0.5 - info.centerOfMass.x),
        static_cast<float>(static_cast<double>(blockY) + 0.5 - info.centerOfMass.y),
        static_cast<float>(static_cast<double>(blockZ) + 0.5 - info.centerOfMass.z));

    if (complex) {
        for (int i = 0; i < 8; ++i) {
            const float x = (i & 1) == 0 ? -0.25f : 0.25f;
            const float y = (i & 2) == 0 ? -0.25f : 0.25f;
            const float z = (i & 4) == 0 ? -0.25f : 0.25f;
            const b3Vec3 localPoint = addVec3(localCenter, vec3(x, y, z));
            applyFluidForSample(scene, info.body, b3Body_GetWorldPoint(info.body, localPoint), 0.25f, blockVolume, colliderCache);
        }
        return;
    }

    applyFluidForSample(scene, info.body, b3Body_GetWorldPoint(info.body, localCenter), 0.5f, blockVolume, colliderCache);
}

void applyBuoyancy(Box3DScene* scene) {
    if (!hasGlobalFluid(scene)) {
        return;
    }

    std::unordered_map<int, ColliderMaterialData> colliderCache;
    colliderCache.reserve(64);

    for (auto& bodyEntry : scene->bodies) {
        BodyInfo& info = bodyEntry.second;
        if (!isValidBody(info.body) || b3Body_GetType(info.body) != b3_dynamicBody || !b3Body_IsAwake(info.body)) {
            continue;
        }
        if (!bodyBoundsOverlapGlobalFluid(scene, info)) {
            continue;
        }

        const bool complex = usesComplexFluidSampling(info);
        for (const auto& chunkEntry : info.chunks) {
            const ChunkShapes& chunk = chunkEntry.second;
            const int baseX = chunk.sectionX << 4;
            const int baseY = chunk.sectionY << 4;
            const int baseZ = chunk.sectionZ << 4;

            for (int bx = 0; bx < 16; ++bx) {
                for (int bz = 0; bz < 16; ++bz) {
                    for (int by = 0; by < 16; ++by) {
                        const int index = chunkIndex(bx, by, bz);
                        const int packedState = chunk.states[index];
                        if ((packedState >> 16) <= 0 || isInteriorVoxel(packedState)) {
                            continue;
                        }

                        const ColliderMaterialData material = colliderMaterialForPackedState(packedState, colliderCache);
                        if (!material.valid || material.volume <= 0.0f) {
                            continue;
                        }

                        applyBuoyancyForBlock(
                            scene,
                            info,
                            baseX + bx,
                            baseY + by,
                            baseZ + bz,
                            material.volume,
                            complex,
                            colliderCache);
                    }
                }
            }
        }
    }
}

} // namespace

extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*) {
    gJvm = vm;
    return JNI_VERSION_1_8;
}

extern "C" JNIEXPORT jlong JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_initialize(
    JNIEnv*,
    jclass,
    jdouble gravityX,
    jdouble gravityY,
    jdouble gravityZ,
    jdouble universalDrag) {
    auto* scene = new Box3DScene();

    b3WorldDef worldDef = b3DefaultWorldDef();
    worldDef.gravity = vec3(static_cast<float>(gravityX), static_cast<float>(gravityY), static_cast<float>(gravityZ));
    worldDef.enableContinuous = true;
    worldDef.enableSleep = true;
    worldDef.workerCount = 1;
    worldDef.hitEventThreshold = 0.1f;
    worldDef.contactHertz = scene->contactHertz;
    worldDef.contactDampingRatio = scene->contactDampingRatio;
    worldDef.contactSpeed = scene->contactSpeed;

    scene->universalDrag = static_cast<float>(universalDrag);
    scene->world = b3CreateWorld(&worldDef);
    b3World_SetPreSolveCallback(scene->world, preSolveContact, scene);

    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.type = b3_staticBody;
    groundDef.position = b3Pos{0.0, 0.0, 0.0};
    groundDef.rotation = quat(0.0f, 0.0f, 0.0f, 1.0f);
    scene->groundBody = b3CreateBody(scene->world, &groundDef);

    return reinterpret_cast<jlong>(scene);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_tick(
    JNIEnv*,
    jclass,
    jlong,
    jdouble) {
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_step(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jdouble timeStep) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    flushDirtyChunks(scene);
    applyBuoyancy(scene);
    refreshConstraints(scene);
    refreshRopeAttachments(scene);
    applyRopeSegmentDamping(scene);
    b3World_Step(scene->world, static_cast<float>(timeStep), scene->subStepCount);
    collectCollisions(scene);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_dispose(
    JNIEnv*,
    jclass,
    jlong sceneHandle) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    for (auto& entry : scene->globalChunks) {
        destroyChunk(entry.second);
    }
    scene->globalChunks.clear();

    for (auto& entry : scene->ropes) {
        destroyRopeInfo(entry.second);
    }
    scene->ropes.clear();

    for (auto& entry : scene->bodies) {
        destroyBodyInfo(entry.second);
    }
    scene->bodies.clear();
    scene->constraints.clear();
    scene->dirtyChunks.clear();

    if (B3_IS_NON_NULL(scene->world)) {
        b3DestroyWorld(scene->world);
    }

    delete scene;
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_createSubLevel(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint id,
    jdoubleArray pose) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    createBody(scene, id, b3_dynamicBody, readPoseArray(env, pose));
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_removeSubLevel(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint id) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    auto it = scene->bodies.find(id);
    if (it == scene->bodies.end()) {
        return;
    }

    destroyBodyInfo(it->second);
    scene->bodies.erase(it);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_createBox(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint id,
    jdouble mass,
    jdouble halfExtentsX,
    jdouble halfExtentsY,
    jdouble halfExtentsZ,
    jdoubleArray pose) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    createBody(scene, id, b3_dynamicBody, readPoseArray(env, pose));
    BodyInfo* info = findBody(scene, id);
    if (info == nullptr) {
        return;
    }

    VoxelCollider collider;
    collider.boxes.push_back(VoxelBox{
        static_cast<float>(-halfExtentsX),
        static_cast<float>(-halfExtentsY),
        static_cast<float>(-halfExtentsZ),
        static_cast<float>(halfExtentsX),
        static_cast<float>(halfExtentsY),
        static_cast<float>(halfExtentsZ),
    });

    b3BoxHull hull = b3MakeBoxHull(static_cast<float>(halfExtentsX), static_cast<float>(halfExtentsY), static_cast<float>(halfExtentsZ));
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = mass > 0.0 ? static_cast<float>(mass / (8.0 * halfExtentsX * halfExtentsY * halfExtentsZ)) : 1.0f;
    shapeDef.filter.categoryBits = LEVEL_COLLISION_CATEGORY;
    shapeDef.filter.maskBits = LEVEL_COLLISION_CATEGORY | ROPE_COLLISION_CATEGORY;
    shapeDef.enableHitEvents = true;
    shapeDef.enableContactEvents = true;

    ChunkShapes chunk;
    chunk.body = info->body;
    ShapeUserData& data = chunk.shapeData.emplace_back();
    data.ownerId = id;
    shapeDef.userData = &data;

    b3ShapeId shape = b3CreateHullShape(info->body, &shapeDef, &hull.base);
    if (B3_IS_NON_NULL(shape)) {
        chunk.shapes.push_back(shape);
    } else {
        chunk.shapeData.pop_back();
    }
    info->chunks[-1] = std::move(chunk);

    setBodyMassData(info->body, static_cast<float>(mass), vec3(0.0f, 0.0f, 0.0f), boxInertia(mass, halfExtentsX, halfExtentsY, halfExtentsZ));
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_removeBox(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint id) {
    Java_in_zaafon_box3d_1for_1sable_Box3DNative_removeSubLevel(nullptr, nullptr, sceneHandle, id);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_getPose(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint id,
    jdoubleArray store) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    b3BodyId body = scene == nullptr ? b3_nullBodyId : bodyForId(scene, id);
    std::array<jdouble, 7> values{};

    if (B3_IS_NON_NULL(body) && b3Body_IsValid(body)) {
        b3Pos pos = b3Body_GetPosition(body);
        b3Quat rot = b3Body_GetRotation(body);
        values = {pos.x, pos.y, pos.z, rot.v.x, rot.v.y, rot.v.z, rot.s};
    }

    env->SetDoubleArrayRegion(store, 0, static_cast<jsize>(values.size()), values.data());
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setCenterOfMass(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint id,
    jdouble x,
    jdouble y,
    jdouble z) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    BodyInfo* info = scene == nullptr ? nullptr : findBody(scene, id);
    if (info == nullptr) {
        return;
    }

    const LocalOrigin newCenterOfMass = origin(x, y, z);
    const bool centerChanged = !sameOrigin(info->centerOfMass, newCenterOfMass);
    info->centerOfMass = newCenterOfMass;
    if (centerChanged) {
        markBodyChunksDirty(scene, *info, id);
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setLocalBounds(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint id,
    jint minX,
    jint minY,
    jint minZ,
    jint maxX,
    jint maxY,
    jint maxZ) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    BodyInfo* info = scene == nullptr ? nullptr : findBody(scene, id);
    if (info == nullptr) {
        return;
    }

    info->localBounds = LocalBounds{true, minX, minY, minZ, maxX, maxY, maxZ};
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_addChunk(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint x,
    jint y,
    jint z,
    jintArray chunk,
    jboolean global,
    jint id) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    if (global) {
        addGlobalChunk(scene, x, y, z, chunk, env);
        return;
    }

    BodyInfo* info = findBody(scene, id);
    if (info == nullptr) {
        return;
    }

    addChunkToBody(scene, *info, id, x, y, z, chunk, env);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_removeChunk(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint x,
    jint y,
    jint z,
    jboolean global) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    const int64_t key = packSectionPos(x, y, z);
    if (global) {
        auto it = scene->globalChunks.find(key);
        if (it != scene->globalChunks.end()) {
            destroyChunk(it->second);
            scene->globalChunks.erase(it);
        }
        return;
    }

    for (auto& entry : scene->bodies) {
        auto chunkIt = entry.second.chunks.find(key);
        if (chunkIt != entry.second.chunks.end()) {
            destroyChunk(chunkIt->second);
            entry.second.chunks.erase(chunkIt);
            return;
        }
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_getDebugStats(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jlongArray store) {
    std::array<jlong, 10> stats{};
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene != nullptr) {
        jlong bodyChunkCount = 0;
        jlong bodyShapeCount = 0;
        for (const auto& entry : scene->bodies) {
            bodyChunkCount += static_cast<jlong>(entry.second.chunks.size());
            for (const auto& chunkEntry : entry.second.chunks) {
                bodyShapeCount += static_cast<jlong>(chunkEntry.second.shapes.size());
            }
        }

        jlong globalShapeCount = 0;
        for (const auto& entry : scene->globalChunks) {
            globalShapeCount += static_cast<jlong>(entry.second.shapes.size());
        }

        b3Counters counters = b3World_GetCounters(scene->world);
        stats[0] = static_cast<jlong>(scene->bodies.size());
        stats[1] = bodyChunkCount;
        stats[2] = bodyShapeCount;
        stats[3] = static_cast<jlong>(scene->globalChunks.size());
        stats[4] = globalShapeCount;
        stats[5] = static_cast<jlong>(counters.contactCount);
        stats[6] = static_cast<jlong>(counters.awakeContactCount);
        stats[7] = static_cast<jlong>(counters.bodyCount);
        stats[8] = static_cast<jlong>(counters.shapeCount);
        stats[9] = static_cast<jlong>(scene->collisions.size());
    }

    const jsize length = env->GetArrayLength(store);
    const jsize count = std::min<jsize>(length, static_cast<jsize>(stats.size()));
    env->SetLongArrayRegion(store, 0, count, stats.data());
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_changeBlock(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint x,
    jint y,
    jint z,
    jint newState) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    const int sectionX = x >> 4;
    const int sectionY = y >> 4;
    const int sectionZ = z >> 4;
    const int index = (x & 15) + ((z & 15) << 4) + ((y & 15) << 8);
    const int64_t key = packSectionPos(sectionX, sectionY, sectionZ);

    auto globalIt = scene->globalChunks.find(key);
    if (globalIt != scene->globalChunks.end()) {
        if (globalIt->second.states[index] != newState) {
            globalIt->second.states[index] = newState;
            refreshChunkFluidFlag(globalIt->second);
            markChunkDirty(scene, globalIt->second, true, -1, sectionX, sectionY, sectionZ);
        }
    }

    for (auto& entry : scene->bodies) {
        auto chunkIt = entry.second.chunks.find(key);
        if (chunkIt == entry.second.chunks.end()) {
            continue;
        }

        if (chunkIt->second.states[index] != newState) {
            chunkIt->second.states[index] = newState;
            refreshChunkFluidFlag(chunkIt->second);
            markChunkDirty(scene, chunkIt->second, false, entry.first, sectionX, sectionY, sectionZ);
        }
        break;
    }
}

extern "C" JNIEXPORT jint JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_newVoxelCollider(
    JNIEnv* env,
    jclass,
    jdouble friction,
    jdouble volume,
    jdouble restitution,
    jboolean isFluid,
    jobject contactEvents) {
    std::lock_guard<std::mutex> lock(gColliderMutex);

    VoxelCollider collider;
    collider.friction = static_cast<float>(friction);
    collider.volume = static_cast<float>(volume);
    collider.restitution = static_cast<float>(restitution);
    collider.isFluid = isFluid;
    if (contactEvents != nullptr) {
        collider.contactEvents = env->NewGlobalRef(contactEvents);
        jclass contactClass = env->GetObjectClass(contactEvents);
        collider.contactMethod = env->GetMethodID(contactClass, "onCollision", "(IIIIIIDDDDZ)[D");
        env->DeleteLocalRef(contactClass);
        if (env->ExceptionCheck()) {
            env->ExceptionDescribe();
            env->ExceptionClear();
            collider.contactMethod = nullptr;
        }
    }

    gVoxelColliders.push_back(collider);
    return static_cast<jint>(gVoxelColliders.size() - 1);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_addVoxelColliderBox(
    JNIEnv* env,
    jclass,
    jint index,
    jdoubleArray boundsArray) {
    std::array<double, 6> bounds{};
    env->GetDoubleArrayRegion(boundsArray, 0, static_cast<jsize>(bounds.size()), bounds.data());

    std::lock_guard<std::mutex> lock(gColliderMutex);
    if (index < 0 || index >= static_cast<jint>(gVoxelColliders.size())) {
        return;
    }

    gVoxelColliders[index].boxes.push_back(VoxelBox{
        static_cast<float>(bounds[0]),
        static_cast<float>(bounds[1]),
        static_cast<float>(bounds[2]),
        static_cast<float>(bounds[3]),
        static_cast<float>(bounds[4]),
        static_cast<float>(bounds[5]),
    });
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_clearVoxelColliderBoxes(
    JNIEnv*,
    jclass,
    jint index) {
    std::lock_guard<std::mutex> lock(gColliderMutex);
    if (index < 0 || index >= static_cast<jint>(gVoxelColliders.size())) {
        return;
    }

    gVoxelColliders[index].boxes.clear();
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setMassProperties(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint id,
    jdouble mass,
    jdoubleArray centerOfMassArray,
    jdoubleArray inertiaTensorArray) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    BodyInfo* info = scene == nullptr ? nullptr : findBody(scene, id);
    if (info == nullptr) {
        return;
    }

    std::array<double, 3> center = readVector3Array(env, centerOfMassArray);
    std::array<double, 9> inertiaArray = readMatrix3Array(env, inertiaTensorArray);

    const LocalOrigin newCenterOfMass = origin(center[0], center[1], center[2]);
    const bool centerChanged = !sameOrigin(info->centerOfMass, newCenterOfMass);
    info->centerOfMass = newCenterOfMass;

    b3Matrix3 inertia{};
    inertia.cx = vec3(static_cast<float>(inertiaArray[0]), static_cast<float>(inertiaArray[3]), static_cast<float>(inertiaArray[6]));
    inertia.cy = vec3(static_cast<float>(inertiaArray[1]), static_cast<float>(inertiaArray[4]), static_cast<float>(inertiaArray[7]));
    inertia.cz = vec3(static_cast<float>(inertiaArray[2]), static_cast<float>(inertiaArray[5]), static_cast<float>(inertiaArray[8]));

    // Sable's pose position is already the center of mass. Rapier receives mass properties
    // with a zero local COM and stores the block-space COM separately for collider lookup.
    setBodyMassData(info->body, static_cast<float>(mass), vec3(0.0f, 0.0f, 0.0f), inertia);

    if (centerChanged) {
        markBodyChunksDirty(scene, *info, id);
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_teleportObject(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint id,
    jdouble x,
    jdouble y,
    jdouble z,
    jdouble qx,
    jdouble qy,
    jdouble qz,
    jdouble qw) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    b3BodyId body = scene == nullptr ? b3_nullBodyId : bodyForId(scene, id);
    if (B3_IS_NULL(body) || !b3Body_IsValid(body)) {
        return;
    }

    b3Body_SetTransform(body, b3Pos{x, y, z}, quat(static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(qz), static_cast<float>(qw)));
    b3Body_SetAwake(body, true);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_wakeUpObject(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint id) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    b3BodyId body = scene == nullptr ? b3_nullBodyId : bodyForId(scene, id);
    setAwakeIfRequested(body, true);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_addLinearAngularVelocities(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint bodyId,
    jdouble linearX,
    jdouble linearY,
    jdouble linearZ,
    jdouble angularX,
    jdouble angularY,
    jdouble angularZ,
    jboolean wakeUp) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    b3BodyId body = scene == nullptr ? b3_nullBodyId : bodyForId(scene, bodyId);
    if (B3_IS_NULL(body) || !b3Body_IsValid(body)) {
        return;
    }

    b3Vec3 linear = b3Body_GetLinearVelocity(body);
    b3Vec3 angular = b3Body_GetAngularVelocity(body);
    b3Body_SetLinearVelocity(body, vec3(linear.x + static_cast<float>(linearX), linear.y + static_cast<float>(linearY), linear.z + static_cast<float>(linearZ)));
    b3Body_SetAngularVelocity(body, vec3(angular.x + static_cast<float>(angularX), angular.y + static_cast<float>(angularY), angular.z + static_cast<float>(angularZ)));
    setAwakeIfRequested(body, wakeUp);
}

extern "C" JNIEXPORT jdoubleArray JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_clearCollisions(
    JNIEnv* env,
    jclass,
    jlong sceneHandle) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr || scene->collisions.empty()) {
        return env->NewDoubleArray(0);
    }

    jdoubleArray array = env->NewDoubleArray(static_cast<jsize>(scene->collisions.size()));
    env->SetDoubleArrayRegion(array, 0, static_cast<jsize>(scene->collisions.size()), scene->collisions.data());
    scene->collisions.clear();
    return array;
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_applyForce(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint bodyID,
    jdouble x,
    jdouble y,
    jdouble z,
    jdouble fx,
    jdouble fy,
    jdouble fz,
    jboolean wakeUp) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    b3BodyId body = scene == nullptr ? b3_nullBodyId : bodyForId(scene, bodyID);
    if (B3_IS_NULL(body) || !b3Body_IsValid(body)) {
        return;
    }

    const b3Vec3 localPoint = vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    const b3Vec3 localImpulse = vec3(static_cast<float>(fx), static_cast<float>(fy), static_cast<float>(fz));
    const b3Pos point = b3Body_GetWorldPoint(body, localPoint);
    const b3Vec3 impulse = b3Body_GetWorldVector(body, localImpulse);
    b3Body_ApplyLinearImpulse(body, impulse, point, wakeUp);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_applyForceAndTorque(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint bodyID,
    jdouble fx,
    jdouble fy,
    jdouble fz,
    jdouble tx,
    jdouble ty,
    jdouble tz,
    jboolean wakeUp) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    b3BodyId body = scene == nullptr ? b3_nullBodyId : bodyForId(scene, bodyID);
    if (B3_IS_NULL(body) || !b3Body_IsValid(body)) {
        return;
    }

    const b3Vec3 impulse = b3Body_GetWorldVector(body, vec3(static_cast<float>(fx), static_cast<float>(fy), static_cast<float>(fz)));
    const b3Vec3 torque = b3Body_GetWorldVector(body, vec3(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz)));
    b3Body_ApplyLinearImpulseToCenter(body, impulse, wakeUp);
    b3Body_ApplyAngularImpulse(body, torque, wakeUp);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_getLinearVelocity(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint bodyID,
    jdoubleArray store) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    b3BodyId body = scene == nullptr ? b3_nullBodyId : bodyForId(scene, bodyID);
    std::array<jdouble, 3> values{};
    if (B3_IS_NON_NULL(body) && b3Body_IsValid(body)) {
        b3Vec3 value = b3Body_GetLinearVelocity(body);
        values = {value.x, value.y, value.z};
    }
    env->SetDoubleArrayRegion(store, 0, static_cast<jsize>(values.size()), values.data());
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_getAngularVelocity(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint bodyID,
    jdoubleArray store) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    b3BodyId body = scene == nullptr ? b3_nullBodyId : bodyForId(scene, bodyID);
    std::array<jdouble, 3> values{};
    if (B3_IS_NON_NULL(body) && b3Body_IsValid(body)) {
        b3Vec3 value = b3Body_GetAngularVelocity(body);
        values = {value.x, value.y, value.z};
    }
    env->SetDoubleArrayRegion(store, 0, static_cast<jsize>(values.size()), values.data());
}

extern "C" JNIEXPORT jlong JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_createRope(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jdouble pointRadius,
    jdouble firstJointLength,
    jdoubleArray points,
    jint pointCount) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr || pointCount <= 0) {
        return 0;
    }

    std::vector<jdouble> coordinates(static_cast<size_t>(pointCount) * 3);
    env->GetDoubleArrayRegion(points, 0, static_cast<jsize>(coordinates.size()), coordinates.data());

    RopeInfo rope{};
    rope.pointRadius = pointRadius;
    rope.firstSegmentLength = firstJointLength;
    rope.points.reserve(static_cast<size_t>(pointCount));
    rope.segments.reserve(pointCount > 0 ? static_cast<size_t>(pointCount - 1) : 0);

    for (int i = 0; i < pointCount; ++i) {
        b3BodyId body = createRopePointBody(scene, coordinates[i * 3], coordinates[i * 3 + 1], coordinates[i * 3 + 2], pointRadius);
        if (!isValidBody(body)) {
            destroyRopeInfo(rope);
            return 0;
        }

        rope.points.push_back(body);
    }

    for (int i = 1; i < pointCount; ++i) {
        const double segmentLength = i == 1 ? firstJointLength : 1.0;
        RopeSegmentInfo segment = createRopeSegment(scene, rope.points[static_cast<size_t>(i - 1)], rope.points[static_cast<size_t>(i)], segmentLength);
        if (!isValidJoint(segment.distanceJoint)) {
            destroyRopeInfo(rope);
            return 0;
        }

        rope.segments.push_back(segment);
    }

    const uint64_t handle = scene->nextRopeId++;
    scene->ropes.emplace(handle, std::move(rope));
    return static_cast<jlong>(handle);
}

extern "C" JNIEXPORT jdoubleArray JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_queryRope(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jlong ropeHandle) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    RopeInfo* rope = scene == nullptr ? nullptr : findRope(scene, ropeHandle);
    if (rope == nullptr || rope->points.empty()) {
        return env->NewDoubleArray(0);
    }

    std::vector<jdouble> coordinates(rope->points.size() * 3);
    for (size_t i = 0; i < rope->points.size(); ++i) {
        b3BodyId body = rope->points[i];
        if (!isValidBody(body)) {
            continue;
        }

        b3Pos position = b3Body_GetPosition(body);
        coordinates[i * 3] = position.x;
        coordinates[i * 3 + 1] = position.y;
        coordinates[i * 3 + 2] = position.z;
    }

    jdoubleArray array = env->NewDoubleArray(static_cast<jsize>(coordinates.size()));
    env->SetDoubleArrayRegion(array, 0, static_cast<jsize>(coordinates.size()), coordinates.data());
    return array;
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_removeRope(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong ropeHandle) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    auto it = scene->ropes.find(static_cast<uint64_t>(ropeHandle));
    if (it == scene->ropes.end()) {
        return;
    }

    destroyRopeInfo(it->second);
    scene->ropes.erase(it);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setRopeAttachment(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong ropeHandle,
    jint subLevelId,
    jdouble x,
    jdouble y,
    jdouble z,
    jboolean end) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    RopeInfo* rope = scene == nullptr ? nullptr : findRope(scene, ropeHandle);
    if (rope == nullptr || rope->points.empty()) {
        return;
    }

    RopeAttachmentInfo& attachment = end ? rope->endAttachment : rope->startAttachment;
    bool& hasAttachment = end ? rope->hasEndAttachment : rope->hasStartAttachment;
    if (hasAttachment) {
        destroyRopeAttachment(attachment);
        hasAttachment = false;
    }

    const ConstraintFrameInfo frame = constraintFrameInfo(
        x,
        y,
        z,
        quat(0.0f, 0.0f, 0.0f, 1.0f));
    const b3BodyId ropeBody = end ? rope->points.back() : rope->points.front();
    attachment = createRopeAttachment(scene, subLevelId, frame, ropeBody);
    hasAttachment = isValidJoint(attachment.joint);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_addRopePointAtStart(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong ropeHandle,
    jdouble x,
    jdouble y,
    jdouble z) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    RopeInfo* rope = scene == nullptr ? nullptr : findRope(scene, ropeHandle);
    if (rope == nullptr) {
        return;
    }

    if (rope->hasStartAttachment) {
        destroyRopeAttachment(rope->startAttachment);
        rope->hasStartAttachment = false;
    }

    b3BodyId newBody = createRopePointBody(scene, x, y, z, rope->pointRadius);
    if (!isValidBody(newBody)) {
        return;
    }

    if (rope->points.empty()) {
        rope->points.push_back(newBody);
        return;
    }

    if (!rope->segments.empty()) {
        setRopeSegmentLength(rope->segments.front(), 1.0);
    }

    RopeSegmentInfo firstSegment = createRopeSegment(scene, newBody, rope->points.front(), rope->firstSegmentLength);
    if (!isValidJoint(firstSegment.distanceJoint)) {
        b3DestroyBody(newBody);
        return;
    }

    rope->points.insert(rope->points.begin(), newBody);
    rope->segments.insert(rope->segments.begin(), firstSegment);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_removeRopePointAtStart(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong ropeHandle) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    RopeInfo* rope = scene == nullptr ? nullptr : findRope(scene, ropeHandle);
    if (rope == nullptr || rope->points.empty()) {
        return;
    }

    if (rope->hasStartAttachment) {
        destroyRopeAttachment(rope->startAttachment);
        rope->hasStartAttachment = false;
    }

    if (rope->points.size() == 1 && rope->hasEndAttachment) {
        destroyRopeAttachment(rope->endAttachment);
        rope->hasEndAttachment = false;
    }

    if (!rope->segments.empty()) {
        destroyRopeSegment(rope->segments.front());
        rope->segments.erase(rope->segments.begin());
    }

    b3BodyId removed = rope->points.front();
    if (isValidBody(removed)) {
        b3DestroyBody(removed);
    }
    rope->points.erase(rope->points.begin());

    if (!rope->segments.empty()) {
        setRopeSegmentLength(rope->segments.front(), rope->firstSegmentLength);
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_wakeUpRope(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong ropeHandle) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    RopeInfo* rope = scene == nullptr ? nullptr : findRope(scene, ropeHandle);
    if (rope == nullptr) {
        return;
    }

    for (b3BodyId body : rope->points) {
        setAwakeIfRequested(body, true);
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setRopeFirstSegmentLength(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong ropeHandle,
    jdouble firstSegmentLength) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    RopeInfo* rope = scene == nullptr ? nullptr : findRope(scene, ropeHandle);
    if (rope == nullptr) {
        return;
    }

    rope->firstSegmentLength = firstSegmentLength;
    if (!rope->segments.empty()) {
        setRopeSegmentLength(rope->segments.front(), firstSegmentLength);
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_createKinematicContraption(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint mountId,
    jint id,
    jdoubleArray pose) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    const std::array<double, 7> poseValues = readPoseArray(env, pose);
    BodyInfo* mountInfo = mountId == -1 ? nullptr : findBody(scene, mountId);
    if (mountInfo != nullptr && isValidBody(mountInfo->body)) {
        BodyInfo info;
        info.body = mountInfo->body;
        info.ownsBody = false;
        info.mountedContraption = true;
        info.mountId = mountId;
        info.shapeRoot = localTransformFromPose(poseValues);
        scene->bodies[id] = std::move(info);
        setAwakeIfRequested(mountInfo->body, true);
        return;
    }

    createBody(scene, id, b3_kinematicBody, poseValues);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_removeKinematicContraption(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint id) {
    Java_in_zaafon_box3d_1for_1sable_Box3DNative_removeSubLevel(nullptr, nullptr, sceneHandle, id);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setKinematicContraptionTransform(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint id,
    jdoubleArray centerOfMass,
    jdoubleArray pose,
    jdoubleArray velocities) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    BodyInfo* info = scene == nullptr ? nullptr : findBody(scene, id);
    if (info == nullptr || B3_IS_NULL(info->body) || !b3Body_IsValid(info->body)) {
        return;
    }

    std::array<double, 3> center = readVector3Array(env, centerOfMass);
    std::array<double, 7> poseValues = readPoseArray(env, pose);
    std::array<double, 6> velocityValues{};
    env->GetDoubleArrayRegion(velocities, 0, static_cast<jsize>(velocityValues.size()), velocityValues.data());

    const LocalOrigin newCenterOfMass = origin(center[0], center[1], center[2]);
    const bool centerChanged = !sameOrigin(info->centerOfMass, newCenterOfMass);
    info->centerOfMass = newCenterOfMass;

    const b3Vec3 newLinearVelocity = vec3(static_cast<float>(velocityValues[0]), static_cast<float>(velocityValues[1]), static_cast<float>(velocityValues[2]));
    const b3Vec3 newAngularVelocity = vec3(static_cast<float>(velocityValues[3]), static_cast<float>(velocityValues[4]), static_cast<float>(velocityValues[5]));
    if (info->mountedContraption) {
        const b3Transform newShapeRoot = localTransformFromPose(poseValues);
        const bool rootChanged = !sameTransform(info->shapeRoot, newShapeRoot);
        info->shapeRoot = newShapeRoot;
        info->localLinearVelocity = newLinearVelocity;
        info->localAngularVelocity = newAngularVelocity;

        if (centerChanged || rootChanged) {
            markBodyChunksDirty(scene, *info, id);
        }

        setAwakeIfRequested(info->body, true);
        return;
    }

    b3Body_SetTransform(info->body, b3Pos{poseValues[0], poseValues[1], poseValues[2]}, quat(static_cast<float>(poseValues[3]), static_cast<float>(poseValues[4]), static_cast<float>(poseValues[5]), static_cast<float>(poseValues[6])));
    b3Body_SetLinearVelocity(info->body, newLinearVelocity);
    b3Body_SetAngularVelocity(info->body, newAngularVelocity);
    b3Body_SetAwake(info->body, true);

    if (centerChanged) {
        markBodyChunksDirty(scene, *info, id);
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_addKinematicContraptionChunkSection(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jint id,
    jint x,
    jint y,
    jint z,
    jintArray data) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    BodyInfo* info = scene == nullptr ? nullptr : findBody(scene, id);
    if (info == nullptr) {
        return;
    }

    addChunkToBody(scene, *info, id, x, y, z, data, env);
}

extern "C" JNIEXPORT jlong JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_addRotaryConstraint(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint idA,
    jint idB,
    jdouble localXA,
    jdouble localYA,
    jdouble localZA,
    jdouble localXB,
    jdouble localYB,
    jdouble localZB,
    jdouble axisXA,
    jdouble axisYA,
    jdouble axisZA,
    jdouble axisXB,
    jdouble axisYB,
    jdouble axisZB) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return 0;
    }

    const b3BodyId bodyA = constraintBodyForId(scene, idA);
    const b3BodyId bodyB = constraintBodyForId(scene, idB);
    if (B3_IS_NULL(bodyA) || B3_IS_NULL(bodyB) || !b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) {
        return 0;
    }

    b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
    const b3Quat frameA = rotationBetween(vec3(0.0f, 0.0f, 1.0f), vec3(static_cast<float>(axisXA), static_cast<float>(axisYA), static_cast<float>(axisZA)));
    const b3Quat frameB = rotationBetween(vec3(0.0f, 0.0f, 1.0f), vec3(static_cast<float>(axisXB), static_cast<float>(axisYB), static_cast<float>(axisZB)));
    const ConstraintFrameInfo localFrameA = constraintFrameInfo(localXA, localYA, localZA, frameA);
    const ConstraintFrameInfo localFrameB = constraintFrameInfo(localXB, localYB, localZB, frameB);
    configureJointBase(
        def.base,
        bodyA,
        bodyB,
        constraintFrame(scene, idA, localFrameA),
        constraintFrame(scene, idB, localFrameB),
        true);

    return storeConstraint(scene, b3CreateRevoluteJoint(scene->world, &def), ConstraintKind::Rotary, 3, idA, idB, localFrameA, localFrameB);
}

extern "C" JNIEXPORT jlong JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_addFixedConstraint(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint idA,
    jint idB,
    jdouble localXA,
    jdouble localYA,
    jdouble localZA,
    jdouble localXB,
    jdouble localYB,
    jdouble localZB,
    jdouble localQX,
    jdouble localQY,
    jdouble localQZ,
    jdouble localQW) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return 0;
    }

    const b3BodyId bodyA = constraintBodyForId(scene, idA);
    const b3BodyId bodyB = constraintBodyForId(scene, idB);
    if (B3_IS_NULL(bodyA) || B3_IS_NULL(bodyB) || !b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) {
        return 0;
    }

    b3WeldJointDef def = b3DefaultWeldJointDef();
    def.linearHertz = 550.0f;
    def.angularHertz = 550.0f;
    def.linearDampingRatio = 4.0f;
    def.angularDampingRatio = 4.0f;
    const ConstraintFrameInfo localFrameA = constraintFrameInfo(localXA, localYA, localZA, quat(static_cast<float>(localQX), static_cast<float>(localQY), static_cast<float>(localQZ), static_cast<float>(localQW)));
    const ConstraintFrameInfo localFrameB = constraintFrameInfo(localXB, localYB, localZB, quat(0.0f, 0.0f, 0.0f, 1.0f));
    configureJointBase(
        def.base,
        bodyA,
        bodyB,
        constraintFrame(scene, idA, localFrameA),
        constraintFrame(scene, idB, localFrameB),
        false);

    return storeConstraint(scene, b3CreateWeldJoint(scene->world, &def), ConstraintKind::Fixed, -1, idA, idB, localFrameA, localFrameB);
}

extern "C" JNIEXPORT jlong JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_addFreeConstraint(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint idA,
    jint idB,
    jdouble localXA,
    jdouble localYA,
    jdouble localZA,
    jdouble localXB,
    jdouble localYB,
    jdouble localZB,
    jdouble localQX,
    jdouble localQY,
    jdouble localQZ,
    jdouble localQW) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return 0;
    }

    const b3BodyId bodyA = constraintBodyForId(scene, idA);
    const b3BodyId bodyB = constraintBodyForId(scene, idB);
    if (B3_IS_NULL(bodyA) || B3_IS_NULL(bodyB) || !b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) {
        return 0;
    }

    b3MotorJointDef def = b3DefaultMotorJointDef();
    const ConstraintFrameInfo localFrameA = constraintFrameInfo(localXA, localYA, localZA, quat(static_cast<float>(localQX), static_cast<float>(localQY), static_cast<float>(localQZ), static_cast<float>(localQW)));
    const ConstraintFrameInfo localFrameB = constraintFrameInfo(localXB, localYB, localZB, quat(0.0f, 0.0f, 0.0f, 1.0f));
    configureJointBase(
        def.base,
        bodyA,
        bodyB,
        constraintFrame(scene, idA, localFrameA),
        constraintFrame(scene, idB, localFrameB),
        true);

    return storeConstraint(scene, b3CreateMotorJoint(scene->world, &def), ConstraintKind::Free, -1, idA, idB, localFrameA, localFrameB);
}

extern "C" JNIEXPORT jlong JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_addGenericConstraint(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint idA,
    jint idB,
    jdouble localXA,
    jdouble localYA,
    jdouble localZA,
    jdouble localQXA,
    jdouble localQYA,
    jdouble localQZA,
    jdouble localQWA,
    jdouble localXB,
    jdouble localYB,
    jdouble localZB,
    jdouble localQXB,
    jdouble localQYB,
    jdouble localQZB,
    jdouble localQWB,
    jint lockedAxesMask) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return 0;
    }

    const b3BodyId bodyA = constraintBodyForId(scene, idA);
    const b3BodyId bodyB = constraintBodyForId(scene, idB);
    if (B3_IS_NULL(bodyA) || B3_IS_NULL(bodyB) || !b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) {
        return 0;
    }

    const int linearMask = lockedAxesMask & 0x7;
    const int angularMask = (lockedAxesMask >> 3) & 0x7;
    const b3Quat rotationA = quat(static_cast<float>(localQXA), static_cast<float>(localQYA), static_cast<float>(localQZA), static_cast<float>(localQWA));
    const b3Quat rotationB = quat(static_cast<float>(localQXB), static_cast<float>(localQYB), static_cast<float>(localQZB), static_cast<float>(localQWB));
    const ConstraintFrameInfo localFrameA = constraintFrameInfo(localXA, localYA, localZA, rotationA);
    const ConstraintFrameInfo localFrameB = constraintFrameInfo(localXB, localYB, localZB, rotationB);

    if (lockedAxesMask == 0) {
        b3MotorJointDef def = b3DefaultMotorJointDef();
        configureJointBase(
            def.base,
            bodyA,
            bodyB,
            constraintFrame(scene, idA, localFrameA),
            constraintFrame(scene, idB, localFrameB),
            true);
        return storeConstraint(scene, b3CreateMotorJoint(scene->world, &def), ConstraintKind::GenericMotor, -1, idA, idB, localFrameA, localFrameB);
    }

    if ((lockedAxesMask & 0x3f) == 0x3f) {
        b3WeldJointDef def = b3DefaultWeldJointDef();
        def.linearHertz = 550.0f;
        def.angularHertz = 550.0f;
        def.linearDampingRatio = 4.0f;
        def.angularDampingRatio = 4.0f;
        configureJointBase(
            def.base,
            bodyA,
            bodyB,
            constraintFrame(scene, idA, localFrameA),
            constraintFrame(scene, idB, localFrameB),
            true);
        return storeConstraint(scene, b3CreateWeldJoint(scene->world, &def), ConstraintKind::GenericWeld, -1, idA, idB, localFrameA, localFrameB);
    }

    if (linearMask == 0x7 && angularMask == 0) {
        b3SphericalJointDef def = b3DefaultSphericalJointDef();
        configureJointBase(
            def.base,
            bodyA,
            bodyB,
            constraintFrame(scene, idA, localFrameA),
            constraintFrame(scene, idB, localFrameB),
            true);
        return storeConstraint(scene, b3CreateSphericalJoint(scene->world, &def), ConstraintKind::GenericSpherical, -1, idA, idB, localFrameA, localFrameB);
    }

    if (linearMask == 0x7 && countBits(angularMask) == 2) {
        const int freeAxis = (~angularMask) & 0x7;
        const int axis = freeAxis == 1 ? 0 : freeAxis == 2 ? 1 : 2;
        const b3Quat axisRotation = rotationBetween(vec3(0.0f, 0.0f, 1.0f), axisVector(axis));
        const ConstraintFrameInfo axisFrameA = constraintFrameInfo(localXA, localYA, localZA, mulQuat(rotationA, axisRotation));
        const ConstraintFrameInfo axisFrameB = constraintFrameInfo(localXB, localYB, localZB, mulQuat(rotationB, axisRotation));
        b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
        configureJointBase(
            def.base,
            bodyA,
            bodyB,
            constraintFrame(scene, idA, axisFrameA),
            constraintFrame(scene, idB, axisFrameB),
            true);
        return storeConstraint(scene, b3CreateRevoluteJoint(scene->world, &def), ConstraintKind::GenericRevolute, axis + 3, idA, idB, axisFrameA, axisFrameB);
    }

    if (angularMask == 0x7 && countBits(linearMask) == 2) {
        const int freeAxis = (~linearMask) & 0x7;
        const int axis = freeAxis == 1 ? 0 : freeAxis == 2 ? 1 : 2;
        const b3Quat axisRotation = rotationBetween(vec3(1.0f, 0.0f, 0.0f), axisVector(axis));
        const ConstraintFrameInfo axisFrameA = constraintFrameInfo(localXA, localYA, localZA, mulQuat(rotationA, axisRotation));
        const ConstraintFrameInfo axisFrameB = constraintFrameInfo(localXB, localYB, localZB, mulQuat(rotationB, axisRotation));
        b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
        configureJointBase(
            def.base,
            bodyA,
            bodyB,
            constraintFrame(scene, idA, axisFrameA),
            constraintFrame(scene, idB, axisFrameB),
            true);
        return storeConstraint(scene, b3CreatePrismaticJoint(scene->world, &def), ConstraintKind::GenericPrismatic, axis, idA, idB, axisFrameA, axisFrameB);
    }

    return 0;
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setConstraintFrame(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong constraintHandle,
    jint side,
    jdouble localX,
    jdouble localY,
    jdouble localZ,
    jdouble localQX,
    jdouble localQY,
    jdouble localQZ,
    jdouble localQW) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    ConstraintInfo* info = scene == nullptr ? nullptr : findConstraint(scene, constraintHandle);
    if (info == nullptr) {
        return;
    }
    if (side != 0 && side != 1) {
        return;
    }

    ConstraintFrameInfo& storedFrame = side == 0 ? info->baseFrameA : info->baseFrameB;
    storedFrame = constraintFrameInfo(
        localX,
        localY,
        localZ,
        box3dFrameRotation(*info, quat(static_cast<float>(localQX), static_cast<float>(localQY), static_cast<float>(localQZ), static_cast<float>(localQW))));
    updateMotorTargetFrames(*info);
    refreshConstraint(scene, *info);
    wakeConstraintBodies(scene, *info);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setConstraintLimit(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong constraintHandle,
    jint axis,
    jdouble min,
    jdouble max) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    ConstraintInfo* info = scene == nullptr ? nullptr : findConstraint(scene, constraintHandle);
    if (info == nullptr) {
        return;
    }

    if ((info->kind == ConstraintKind::Rotary || info->kind == ConstraintKind::GenericRevolute) && axis == info->primaryAxis) {
        b3RevoluteJoint_EnableLimit(info->joint, true);
        b3RevoluteJoint_SetLimits(info->joint, static_cast<float>(min), static_cast<float>(max));
    } else if (info->kind == ConstraintKind::GenericPrismatic && axis == info->primaryAxis) {
        b3PrismaticJoint_EnableLimit(info->joint, true);
        b3PrismaticJoint_SetLimits(info->joint, static_cast<float>(min), static_cast<float>(max));
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_lockConstraintAxes(
    JNIEnv*,
    jclass,
    jlong,
    jlong,
    jint) {
    // Box3D joint type cannot be changed in-place. Generic constraints are mapped
    // at creation time for the masks Box3D can represent directly.
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setConstraintMotor(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong constraintHandle,
    jint axis,
    jdouble target,
    jdouble stiffness,
    jdouble damping,
    jboolean hasMaxForce,
    jdouble maxForce) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    ConstraintInfo* info = scene == nullptr ? nullptr : findConstraint(scene, constraintHandle);
    if (info == nullptr) {
        return;
    }

    if (isMotorJointConstraint(*info) && axis >= 0 && axis < 6) {
        const float maxMotorForce = motorLimit(hasMaxForce, maxForce);
        const float maxDampingForce = dampingOnlyMotorLimit(hasMaxForce, maxForce, damping);
        if (axis < 3) {
            const MotorSpringSettings spring = motorSpringSettings(stiffness, damping);
            vectorComponent(info->linearMotorTarget, axis) = static_cast<float>(target);
            b3MotorJoint_SetLinearHertz(info->joint, spring.hertz);
            b3MotorJoint_SetLinearDampingRatio(info->joint, spring.dampingRatio);
            b3MotorJoint_SetMaxSpringForce(info->joint, spring.hertz > 0.0f ? maxMotorForce : 0.0f);
            b3MotorJoint_SetLinearVelocity(info->joint, vec3(0.0f, 0.0f, 0.0f));
            b3MotorJoint_SetMaxVelocityForce(info->joint, spring.hertz <= 0.0f && spring.dampingRatio > 0.0f ? maxDampingForce : 0.0f);
        } else {
            const MotorSpringSettings spring = motorSpringSettings(stiffness, damping);
            vectorComponent(info->angularMotorTarget, axis - 3) = static_cast<float>(target);
            b3MotorJoint_SetAngularHertz(info->joint, spring.hertz);
            b3MotorJoint_SetAngularDampingRatio(info->joint, spring.dampingRatio);
            b3MotorJoint_SetMaxSpringTorque(info->joint, spring.hertz > 0.0f ? maxMotorForce : 0.0f);
            b3MotorJoint_SetAngularVelocity(info->joint, vec3(0.0f, 0.0f, 0.0f));
            b3MotorJoint_SetMaxVelocityTorque(info->joint, spring.hertz <= 0.0f && spring.dampingRatio > 0.0f ? maxDampingForce : 0.0f);
        }

        updateMotorTargetFrames(*info);
        refreshConstraint(scene, *info);
        wakeConstraintBodies(scene, *info);
    } else if ((info->kind == ConstraintKind::Rotary || info->kind == ConstraintKind::GenericRevolute) && axis == info->primaryAxis) {
        const MotorSpringSettings spring = motorSpringSettings(stiffness, damping);
        b3RevoluteJoint_EnableSpring(info->joint, true);
        b3RevoluteJoint_SetSpringHertz(info->joint, spring.hertz);
        b3RevoluteJoint_SetSpringDampingRatio(info->joint, spring.dampingRatio);
        b3RevoluteJoint_SetTargetAngle(info->joint, static_cast<float>(target));
        if (hasMaxForce) {
            b3RevoluteJoint_SetMaxMotorTorque(info->joint, static_cast<float>(maxForce));
        }
        wakeConstraintBodies(scene, *info);
    } else if (info->kind == ConstraintKind::GenericPrismatic && axis == info->primaryAxis) {
        const MotorSpringSettings spring = motorSpringSettings(stiffness, damping);
        b3PrismaticJoint_EnableSpring(info->joint, true);
        b3PrismaticJoint_SetSpringHertz(info->joint, spring.hertz);
        b3PrismaticJoint_SetSpringDampingRatio(info->joint, spring.dampingRatio);
        b3PrismaticJoint_SetTargetTranslation(info->joint, static_cast<float>(target));
        if (hasMaxForce) {
            b3PrismaticJoint_SetMaxMotorForce(info->joint, static_cast<float>(maxForce));
        }
        wakeConstraintBodies(scene, *info);
    } else if (info->kind == ConstraintKind::GenericSpherical && axis >= 3) {
        const MotorSpringSettings spring = motorSpringSettings(stiffness, damping);
        const b3Vec3 localAxis = axisVector(axis - 3);
        const float halfAngle = static_cast<float>(target) * 0.5f;
        b3SphericalJoint_EnableSpring(info->joint, true);
        b3SphericalJoint_SetSpringHertz(info->joint, spring.hertz);
        b3SphericalJoint_SetSpringDampingRatio(info->joint, spring.dampingRatio);
        b3SphericalJoint_SetTargetRotation(info->joint, quat(
            localAxis.x * std::sin(halfAngle),
            localAxis.y * std::sin(halfAngle),
            localAxis.z * std::sin(halfAngle),
            std::cos(halfAngle)));
        if (hasMaxForce) {
            b3SphericalJoint_SetMaxMotorTorque(info->joint, static_cast<float>(maxForce));
        }
        wakeConstraintBodies(scene, *info);
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_setConstraintContactsEnabled(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong constraintHandle,
    jboolean enabled) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    ConstraintInfo* info = scene == nullptr ? nullptr : findConstraint(scene, constraintHandle);
    if (info != nullptr) {
        b3Joint_SetCollideConnected(info->joint, enabled);
    }
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_getConstraintImpulses(
    JNIEnv* env,
    jclass,
    jlong sceneHandle,
    jlong constraintHandle,
    jdoubleArray store) {
    std::array<jdouble, 6> values{};
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    ConstraintInfo* info = scene == nullptr ? nullptr : findConstraint(scene, constraintHandle);
    if (info != nullptr) {
        const b3Vec3 force = b3Joint_GetConstraintForce(info->joint);
        const b3Vec3 torque = b3Joint_GetConstraintTorque(info->joint);
        values = {force.x, force.y, force.z, torque.x, torque.y, torque.z};
    }

    env->SetDoubleArrayRegion(store, 0, static_cast<jsize>(values.size()), values.data());
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_removeConstraint(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong constraintHandle) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    ConstraintInfo* info = scene == nullptr ? nullptr : findConstraint(scene, constraintHandle);
    if (info == nullptr) {
        return;
    }

    b3DestroyJoint(info->joint, true);
    scene->constraints.erase(static_cast<uint64_t>(constraintHandle));
}

extern "C" JNIEXPORT jboolean JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_isConstraintValid(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jlong constraintHandle) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    return scene != nullptr && findConstraint(scene, constraintHandle) != nullptr;
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_configFrequencyAndDamping(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jdouble contactNaturalFrequency,
    jdouble contactDampingRatio,
    jdouble contactCorrectionSpeed) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    scene->contactHertz = static_cast<float>(contactNaturalFrequency);
    scene->contactDampingRatio = static_cast<float>(contactDampingRatio);
    scene->contactSpeed = static_cast<float>(contactCorrectionSpeed);
    b3World_SetContactTuning(scene->world, scene->contactHertz, scene->contactDampingRatio, scene->contactSpeed);
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_configSolverIterations(
    JNIEnv*,
    jclass,
    jlong sceneHandle,
    jint,
    jint pgsIterations,
    jint stabilizationIterations,
    jint box3dInternalSubsteps) {
    Box3DScene* scene = sceneFromHandle(sceneHandle);
    if (scene == nullptr) {
        return;
    }

    const int fallbackSubsteps = std::max(1, static_cast<int>(pgsIterations + stabilizationIterations));
    scene->subStepCount = box3dInternalSubsteps > 0 ? static_cast<int>(box3dInternalSubsteps) : fallbackSubsteps;
}

extern "C" JNIEXPORT void JNICALL Java_in_zaafon_box3d_1for_1sable_Box3DNative_configMinIslandSize(
    JNIEnv*,
    jclass,
    jlong,
    jint) {
}
