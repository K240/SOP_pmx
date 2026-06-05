#ifndef PMX_PARSER_H
#define PMX_PARSER_H

// Self-contained MikuMikuDance PMX (2.0 / 2.1) binary parser.
// Houdini-independent: pure C++17 + STL. Strings are normalized to UTF-8.
//
// Reference layout: signed reference indices (bone/material/morph/rigidbody/
// texture) use -1 to mean "none"; vertex indices (faces, morph targets, soft
// body anchors/pins) are unsigned. Multi-byte values are little-endian.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace pmx
{

enum class Encoding : uint8_t { UTF16LE = 0, UTF8 = 1 };

enum class WeightType : uint8_t { BDEF1 = 0, BDEF2 = 1, BDEF4 = 2, SDEF = 3, QDEF = 4 };

struct Vec2 { float x = 0, y = 0; };
struct Vec3 { float x = 0, y = 0, z = 0; };
struct Vec4 { float x = 0, y = 0, z = 0, w = 0; };

struct Header
{
    float version = 0.0f;                 // 2.0 or 2.1
    Encoding encoding = Encoding::UTF16LE;
    int additional_uv = 0;                // 0..4
    int vertex_index_size = 1;            // 1/2/4 (unsigned)
    int texture_index_size = 1;           // 1/2/4 (signed, -1 = none)
    int material_index_size = 1;
    int bone_index_size = 1;
    int morph_index_size = 1;
    int rigidbody_index_size = 1;
    std::string model_name_local;
    std::string model_name_universal;
    std::string comment_local;
    std::string comment_universal;
};

struct Vertex
{
    Vec3 position;
    Vec3 normal;
    Vec2 uv;
    Vec4 additional_uv[4];                // only [0, additional_uv) are valid
    WeightType weight_type = WeightType::BDEF1;
    int32_t bones[4] = { -1, -1, -1, -1 };
    float weights[4] = { 0, 0, 0, 0 };
    Vec3 sdef_c;                          // SDEF only
    Vec3 sdef_r0;
    Vec3 sdef_r1;
    float edge_scale = 1.0f;
};

// Material draw flag bits.
enum MaterialFlag : uint8_t
{
    MAT_NO_CULL        = 0x01,
    MAT_GROUND_SHADOW  = 0x02,
    MAT_CAST_SHADOW    = 0x04,
    MAT_RECEIVE_SHADOW = 0x08,
    MAT_HAS_EDGE       = 0x10,
    MAT_VERTEX_COLOR   = 0x20,            // 2.1
    MAT_POINT_DRAW     = 0x40,            // 2.1
    MAT_LINE_DRAW      = 0x80,            // 2.1
};

struct Material
{
    std::string name_local;
    std::string name_universal;
    Vec4 diffuse;                         // RGBA
    Vec3 specular;
    float specular_strength = 0.0f;
    Vec3 ambient;
    uint8_t draw_flags = 0;
    Vec4 edge_color;
    float edge_scale = 0.0f;
    int32_t texture_index = -1;
    int32_t environment_index = -1;       // sphere map
    uint8_t environment_blend = 0;        // 0 disabled, 1 multiply, 2 additive, 3 additional vec4
    uint8_t toon_flag = 0;                // 0 = texture reference, 1 = internal shared toon
    int32_t toon_index = -1;              // texture index (flag 0) or internal toon 0..9 (flag 1)
    std::string memo;
    int32_t surface_count = 0;            // number of indices (multiple of 3)
};

struct IkLink
{
    int32_t bone = -1;
    bool has_limit = false;
    Vec3 limit_min;
    Vec3 limit_max;
};

enum BoneFlag : uint16_t
{
    BONE_INDEXED_TAIL         = 0x0001,
    BONE_ROTATABLE            = 0x0002,
    BONE_TRANSLATABLE         = 0x0004,
    BONE_VISIBLE              = 0x0008,
    BONE_ENABLED              = 0x0010,
    BONE_IK                   = 0x0020,
    BONE_INHERIT_LOCAL        = 0x0080,
    BONE_INHERIT_ROTATION     = 0x0100,
    BONE_INHERIT_TRANSLATION  = 0x0200,
    BONE_FIXED_AXIS           = 0x0400,
    BONE_LOCAL_AXIS           = 0x0800,
    BONE_PHYSICS_AFTER_DEFORM = 0x1000,
    BONE_EXTERNAL_PARENT      = 0x2000,
};

struct Bone
{
    std::string name_local;
    std::string name_universal;
    Vec3 position;
    int32_t parent = -1;
    int32_t layer = 0;
    uint16_t flags = 0;
    Vec3 tail_offset;                     // when !(flags & BONE_INDEXED_TAIL)
    int32_t tail_bone = -1;               // when  (flags & BONE_INDEXED_TAIL)
    int32_t inherit_parent = -1;
    float inherit_weight = 0.0f;
    Vec3 fixed_axis;
    Vec3 local_axis_x;
    Vec3 local_axis_z;
    int32_t external_parent_key = -1;
    int32_t ik_target = -1;
    int32_t ik_loop_count = 0;
    float ik_limit_radian = 0.0f;
    std::vector<IkLink> ik_links;
};

enum class MorphType : uint8_t
{
    Group = 0, Vertex = 1, Bone = 2, UV = 3, UV1 = 4, UV2 = 5, UV3 = 6, UV4 = 7,
    Material = 8, Flip = 9, Impulse = 10
};

struct GroupMorphOffset    { int32_t morph = -1; float influence = 0; };
struct VertexMorphOffset   { int32_t vertex = 0; Vec3 offset; };
struct BoneMorphOffset     { int32_t bone = -1; Vec3 translation; Vec4 rotation; };
struct UVMorphOffset       { int32_t vertex = 0; Vec4 offset; };
struct MaterialMorphOffset
{
    int32_t material = -1;                // -1 = all materials
    uint8_t operation = 0;                // 0 multiply, 1 add
    Vec4 diffuse;
    Vec3 specular;
    float specular_strength = 0;
    Vec3 ambient;
    Vec4 edge_color;
    float edge_scale = 0;
    Vec4 texture_tint;
    Vec4 environment_tint;
    Vec4 toon_tint;
};
struct FlipMorphOffset     { int32_t morph = -1; float influence = 0; };            // 2.1
struct ImpulseMorphOffset  { int32_t rigidbody = -1; uint8_t local_flag = 0; Vec3 velocity; Vec3 torque; }; // 2.1

struct Morph
{
    std::string name_local;
    std::string name_universal;
    uint8_t panel = 0;                    // 0 system, 1 eyebrow, 2 eye, 3 mouth, 4 other
    MorphType type = MorphType::Group;
    std::vector<GroupMorphOffset>    group;
    std::vector<VertexMorphOffset>   vertex;
    std::vector<BoneMorphOffset>     bone;
    std::vector<UVMorphOffset>       uv;  // UV and UV1..UV4 (channel = type)
    std::vector<MaterialMorphOffset> material;
    std::vector<FlipMorphOffset>     flip;
    std::vector<ImpulseMorphOffset>  impulse;
};

struct DisplayElement { uint8_t type = 0; int32_t index = -1; }; // type 0 bone, 1 morph
struct DisplayFrame
{
    std::string name_local;
    std::string name_universal;
    uint8_t special_flag = 0;             // 0 normal, 1 special (Root / 表情)
    std::vector<DisplayElement> elements;
};

enum class RigidShape : uint8_t { Sphere = 0, Box = 1, Capsule = 2 };
enum class PhysicsMode : uint8_t { FollowBone = 0, Physics = 1, PhysicsAndBone = 2 };

struct RigidBody
{
    std::string name_local;
    std::string name_universal;
    int32_t bone = -1;
    uint8_t group = 0;
    uint16_t non_collision_mask = 0;
    RigidShape shape = RigidShape::Sphere;
    Vec3 size;
    Vec3 position;
    Vec3 rotation;                        // euler radians
    float mass = 1.0f;
    float linear_damping = 0.0f;
    float angular_damping = 0.0f;
    float restitution = 0.0f;
    float friction = 0.0f;
    PhysicsMode physics_mode = PhysicsMode::FollowBone;
};

enum class JointType : uint8_t { Spring6DOF = 0, SixDOF = 1, P2P = 2, ConeTwist = 3, Slider = 4, Hinge = 5 };

struct Joint
{
    std::string name_local;
    std::string name_universal;
    JointType type = JointType::Spring6DOF;
    int32_t rigidbody_a = -1;
    int32_t rigidbody_b = -1;
    Vec3 position;
    Vec3 rotation;                        // euler radians
    Vec3 position_limit_lower;
    Vec3 position_limit_upper;
    Vec3 rotation_limit_lower;
    Vec3 rotation_limit_upper;
    Vec3 spring_position;
    Vec3 spring_rotation;
};

enum class SoftBodyShape : uint8_t { TriMesh = 0, Rope = 1 };
struct SoftBodyAnchor { int32_t rigidbody = -1; int32_t vertex = 0; uint8_t near_mode = 0; };

struct SoftBody                            // PMX 2.1 only
{
    std::string name_local;
    std::string name_universal;
    SoftBodyShape shape = SoftBodyShape::TriMesh;
    int32_t material = -1;
    uint8_t group = 0;
    uint16_t non_collision_mask = 0;
    uint8_t flags = 0;                    // bit0 b-link create, bit1 cluster create, bit2 link crossing
    int32_t b_link_distance = 0;
    int32_t cluster_count = 0;
    float total_mass = 0;
    float collision_margin = 0;
    int32_t aero_model = 0;
    float vcf = 0, dp = 0, dg = 0, lf = 0, pr = 0, vc = 0, df = 0, mt = 0, chr = 0, khr = 0, shr = 0, ahr = 0;
    float srhr_cl = 0, skhr_cl = 0, sshr_cl = 0, sr_splt_cl = 0, sk_splt_cl = 0, ss_splt_cl = 0;
    int32_t v_it = 0, p_it = 0, d_it = 0, c_it = 0;
    float lst = 0, ast = 0, vst = 0;
    std::vector<SoftBodyAnchor> anchors;
    std::vector<int32_t> pins;
};

struct Model
{
    Header header;
    std::vector<Vertex> vertices;
    std::vector<int32_t> indices;         // triangle vertex indices (unsigned values)
    std::vector<std::string> textures;    // relative paths (as stored in file)
    std::vector<Material> materials;
    std::vector<Bone> bones;
    std::vector<Morph> morphs;
    std::vector<DisplayFrame> display_frames;
    std::vector<RigidBody> rigid_bodies;
    std::vector<Joint> joints;
    std::vector<SoftBody> soft_bodies;    // empty for 2.0
};

struct LoadResult
{
    bool ok = false;
    std::string error;
    explicit operator bool() const { return ok; }
};

// Parse a PMX file (path is UTF-8; CJK paths are supported on Windows).
LoadResult load(const char *path, Model &out);

// Parse PMX bytes already in memory.
LoadResult load_from_memory(const uint8_t *data, size_t size, Model &out);

} // namespace pmx

#endif // PMX_PARSER_H
