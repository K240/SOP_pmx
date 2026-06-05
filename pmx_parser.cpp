#include "pmx_parser.h"

#include <cstring>
#include <filesystem>
#include <fstream>

namespace pmx
{
namespace
{

// Convert a UTF-16LE byte run to a UTF-8 std::string (handles surrogate pairs).
std::string
utf16le_to_utf8(const uint8_t *p, size_t byte_len)
{
    std::string out;
    out.reserve(byte_len);
    const size_t units = byte_len / 2;
    auto unit = [&](size_t i) -> uint32_t {
        return (uint32_t)p[i * 2] | ((uint32_t)p[i * 2 + 1] << 8);
    };
    for (size_t i = 0; i < units; ++i)
    {
        uint32_t cp = unit(i);
        if (cp >= 0xD800 && cp <= 0xDBFF && i + 1 < units)
        {
            const uint32_t lo = unit(i + 1);
            if (lo >= 0xDC00 && lo <= 0xDFFF)
            {
                cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                ++i;
            }
        }
        if (cp < 0x80)
        {
            out.push_back((char)cp);
        }
        else if (cp < 0x800)
        {
            out.push_back((char)(0xC0 | (cp >> 6)));
            out.push_back((char)(0x80 | (cp & 0x3F)));
        }
        else if (cp < 0x10000)
        {
            out.push_back((char)(0xE0 | (cp >> 12)));
            out.push_back((char)(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back((char)(0x80 | (cp & 0x3F)));
        }
        else
        {
            out.push_back((char)(0xF0 | (cp >> 18)));
            out.push_back((char)(0x80 | ((cp >> 12) & 0x3F)));
            out.push_back((char)(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back((char)(0x80 | (cp & 0x3F)));
        }
    }
    return out;
}

// Bounds-checked little-endian byte cursor. After any out-of-range read the
// cursor latches an error; subsequent reads return zero/empty.
struct Reader
{
    const uint8_t *data = nullptr;
    size_t size = 0;
    size_t pos = 0;
    bool error = false;
    std::string err;
    Header header;                         // index sizes / encoding used by helpers

    Reader(const uint8_t *d, size_t s) : data(d), size(s) {}

    size_t remaining() const { return pos <= size ? size - pos : 0; }

    bool need(size_t n)
    {
        if (error)
            return false;
        if (pos + n > size)
        {
            error = true;
            if (err.empty())
                err = "Unexpected end of file while reading PMX data.";
            return false;
        }
        return true;
    }

    void fail(const char *msg)
    {
        if (!error)
        {
            error = true;
            err = msg;
        }
    }

    uint8_t u8()
    {
        if (!need(1)) return 0;
        return data[pos++];
    }
    uint16_t u16()
    {
        if (!need(2)) return 0;
        uint16_t v;
        std::memcpy(&v, data + pos, 2);
        pos += 2;
        return v;
    }
    uint32_t u32()
    {
        if (!need(4)) return 0;
        uint32_t v;
        std::memcpy(&v, data + pos, 4);
        pos += 4;
        return v;
    }
    int32_t i32() { return (int32_t)u32(); }
    float f32()
    {
        if (!need(4)) return 0.0f;
        float v;
        std::memcpy(&v, data + pos, 4);
        pos += 4;
        return v;
    }
    Vec2 vec2() { Vec2 v; v.x = f32(); v.y = f32(); return v; }
    Vec3 vec3() { Vec3 v; v.x = f32(); v.y = f32(); v.z = f32(); return v; }
    Vec4 vec4() { Vec4 v; v.x = f32(); v.y = f32(); v.z = f32(); v.w = f32(); return v; }

    // Unsigned vertex index (faces, morph targets, soft body refs).
    int32_t vertex_index()
    {
        switch (header.vertex_index_size)
        {
        case 1: return (int32_t)u8();
        case 2: return (int32_t)u16();
        default: return (int32_t)u32();
        }
    }

    // Signed reference index (-1 = none) of the given byte width.
    int32_t index(int byte_size)
    {
        switch (byte_size)
        {
        case 1: return (int32_t)(int8_t)u8();
        case 2: return (int32_t)(int16_t)u16();
        default: return (int32_t)u32();
        }
    }

    std::string text()
    {
        const int32_t len = i32();
        if (error || len <= 0)
            return std::string();
        if (!need((size_t)len))
            return std::string();
        const uint8_t *p = data + pos;
        pos += (size_t)len;
        if (header.encoding == Encoding::UTF16LE)
            return utf16le_to_utf8(p, (size_t)len);
        return std::string((const char *)p, (size_t)len);
    }
};

// Guard a section element count against corruption / truncation. Each element
// needs at least one byte, so count may not exceed the remaining bytes.
bool
valid_count(Reader &r, int32_t count)
{
    if (r.error)
        return false;
    if (count < 0)
    {
        r.fail("Negative element count in PMX file.");
        return false;
    }
    if ((size_t)count > r.remaining() + 1)
    {
        r.fail("Element count exceeds remaining PMX data.");
        return false;
    }
    return true;
}

void
read_vertex(Reader &r, Vertex &v)
{
    v.position = r.vec3();
    v.normal = r.vec3();
    v.uv = r.vec2();
    for (int i = 0; i < r.header.additional_uv; ++i)
        v.additional_uv[i] = r.vec4();

    const int bs = r.header.bone_index_size;
    v.weight_type = (WeightType)r.u8();
    switch (v.weight_type)
    {
    case WeightType::BDEF1:
        v.bones[0] = r.index(bs);
        v.weights[0] = 1.0f;
        break;
    case WeightType::BDEF2:
        v.bones[0] = r.index(bs);
        v.bones[1] = r.index(bs);
        v.weights[0] = r.f32();
        v.weights[1] = 1.0f - v.weights[0];
        break;
    case WeightType::SDEF:
        v.bones[0] = r.index(bs);
        v.bones[1] = r.index(bs);
        v.weights[0] = r.f32();
        v.weights[1] = 1.0f - v.weights[0];
        v.sdef_c = r.vec3();
        v.sdef_r0 = r.vec3();
        v.sdef_r1 = r.vec3();
        break;
    case WeightType::BDEF4:
    case WeightType::QDEF:
        v.bones[0] = r.index(bs);
        v.bones[1] = r.index(bs);
        v.bones[2] = r.index(bs);
        v.bones[3] = r.index(bs);
        v.weights[0] = r.f32();
        v.weights[1] = r.f32();
        v.weights[2] = r.f32();
        v.weights[3] = r.f32();
        break;
    default:
        r.fail("Unknown vertex weight type in PMX file.");
        break;
    }
    v.edge_scale = r.f32();
}

void
read_material(Reader &r, Material &m)
{
    m.name_local = r.text();
    m.name_universal = r.text();
    m.diffuse = r.vec4();
    m.specular = r.vec3();
    m.specular_strength = r.f32();
    m.ambient = r.vec3();
    m.draw_flags = r.u8();
    m.edge_color = r.vec4();
    m.edge_scale = r.f32();
    m.texture_index = r.index(r.header.texture_index_size);
    m.environment_index = r.index(r.header.texture_index_size);
    m.environment_blend = r.u8();
    m.toon_flag = r.u8();
    if (m.toon_flag == 0)
        m.toon_index = r.index(r.header.texture_index_size);
    else
        m.toon_index = (int32_t)r.u8();   // internal shared toon 0..9
    m.memo = r.text();
    m.surface_count = r.i32();
}

void
read_bone(Reader &r, Bone &b)
{
    const int bs = r.header.bone_index_size;
    b.name_local = r.text();
    b.name_universal = r.text();
    b.position = r.vec3();
    b.parent = r.index(bs);
    b.layer = r.i32();
    b.flags = r.u16();

    if (b.flags & BONE_INDEXED_TAIL)
        b.tail_bone = r.index(bs);
    else
        b.tail_offset = r.vec3();

    if (b.flags & (BONE_INHERIT_ROTATION | BONE_INHERIT_TRANSLATION))
    {
        b.inherit_parent = r.index(bs);
        b.inherit_weight = r.f32();
    }
    if (b.flags & BONE_FIXED_AXIS)
        b.fixed_axis = r.vec3();
    if (b.flags & BONE_LOCAL_AXIS)
    {
        b.local_axis_x = r.vec3();
        b.local_axis_z = r.vec3();
    }
    if (b.flags & BONE_EXTERNAL_PARENT)
        b.external_parent_key = r.i32();
    if (b.flags & BONE_IK)
    {
        b.ik_target = r.index(bs);
        b.ik_loop_count = r.i32();
        b.ik_limit_radian = r.f32();
        const int32_t link_count = r.i32();
        if (valid_count(r, link_count))
        {
            b.ik_links.resize((size_t)link_count);
            for (auto &link : b.ik_links)
            {
                link.bone = r.index(bs);
                link.has_limit = r.u8() != 0;
                if (link.has_limit)
                {
                    link.limit_min = r.vec3();
                    link.limit_max = r.vec3();
                }
                if (r.error)
                    break;
            }
        }
    }
}

void
read_morph(Reader &r, Morph &mo)
{
    mo.name_local = r.text();
    mo.name_universal = r.text();
    mo.panel = r.u8();
    mo.type = (MorphType)r.u8();
    const int32_t count = r.i32();
    if (!valid_count(r, count))
        return;

    for (int32_t i = 0; i < count && !r.error; ++i)
    {
        switch (mo.type)
        {
        case MorphType::Group:
        {
            GroupMorphOffset o;
            o.morph = r.index(r.header.morph_index_size);
            o.influence = r.f32();
            mo.group.push_back(o);
            break;
        }
        case MorphType::Vertex:
        {
            VertexMorphOffset o;
            o.vertex = r.vertex_index();
            o.offset = r.vec3();
            mo.vertex.push_back(o);
            break;
        }
        case MorphType::Bone:
        {
            BoneMorphOffset o;
            o.bone = r.index(r.header.bone_index_size);
            o.translation = r.vec3();
            o.rotation = r.vec4();
            mo.bone.push_back(o);
            break;
        }
        case MorphType::UV:
        case MorphType::UV1:
        case MorphType::UV2:
        case MorphType::UV3:
        case MorphType::UV4:
        {
            UVMorphOffset o;
            o.vertex = r.vertex_index();
            o.offset = r.vec4();
            mo.uv.push_back(o);
            break;
        }
        case MorphType::Material:
        {
            MaterialMorphOffset o;
            o.material = r.index(r.header.material_index_size);
            o.operation = r.u8();
            o.diffuse = r.vec4();
            o.specular = r.vec3();
            o.specular_strength = r.f32();
            o.ambient = r.vec3();
            o.edge_color = r.vec4();
            o.edge_scale = r.f32();
            o.texture_tint = r.vec4();
            o.environment_tint = r.vec4();
            o.toon_tint = r.vec4();
            mo.material.push_back(o);
            break;
        }
        case MorphType::Flip:
        {
            FlipMorphOffset o;
            o.morph = r.index(r.header.morph_index_size);
            o.influence = r.f32();
            mo.flip.push_back(o);
            break;
        }
        case MorphType::Impulse:
        {
            ImpulseMorphOffset o;
            o.rigidbody = r.index(r.header.rigidbody_index_size);
            o.local_flag = r.u8();
            o.velocity = r.vec3();
            o.torque = r.vec3();
            mo.impulse.push_back(o);
            break;
        }
        default:
            r.fail("Unknown morph type in PMX file.");
            break;
        }
    }
}

void
read_rigidbody(Reader &r, RigidBody &rb)
{
    rb.name_local = r.text();
    rb.name_universal = r.text();
    rb.bone = r.index(r.header.bone_index_size);
    rb.group = r.u8();
    rb.non_collision_mask = r.u16();
    rb.shape = (RigidShape)r.u8();
    rb.size = r.vec3();
    rb.position = r.vec3();
    rb.rotation = r.vec3();
    rb.mass = r.f32();
    rb.linear_damping = r.f32();
    rb.angular_damping = r.f32();
    rb.restitution = r.f32();
    rb.friction = r.f32();
    rb.physics_mode = (PhysicsMode)r.u8();
}

void
read_joint(Reader &r, Joint &j)
{
    const int rs = r.header.rigidbody_index_size;
    j.name_local = r.text();
    j.name_universal = r.text();
    j.type = (JointType)r.u8();
    j.rigidbody_a = r.index(rs);
    j.rigidbody_b = r.index(rs);
    j.position = r.vec3();
    j.rotation = r.vec3();
    j.position_limit_lower = r.vec3();
    j.position_limit_upper = r.vec3();
    j.rotation_limit_lower = r.vec3();
    j.rotation_limit_upper = r.vec3();
    j.spring_position = r.vec3();
    j.spring_rotation = r.vec3();
}

void
read_softbody(Reader &r, SoftBody &sb)
{
    sb.name_local = r.text();
    sb.name_universal = r.text();
    sb.shape = (SoftBodyShape)r.u8();
    sb.material = r.index(r.header.material_index_size);
    sb.group = r.u8();
    sb.non_collision_mask = r.u16();
    sb.flags = r.u8();
    sb.b_link_distance = r.i32();
    sb.cluster_count = r.i32();
    sb.total_mass = r.f32();
    sb.collision_margin = r.f32();
    sb.aero_model = r.i32();
    sb.vcf = r.f32(); sb.dp = r.f32(); sb.dg = r.f32(); sb.lf = r.f32();
    sb.pr = r.f32(); sb.vc = r.f32(); sb.df = r.f32(); sb.mt = r.f32();
    sb.chr = r.f32(); sb.khr = r.f32(); sb.shr = r.f32(); sb.ahr = r.f32();
    sb.srhr_cl = r.f32(); sb.skhr_cl = r.f32(); sb.sshr_cl = r.f32();
    sb.sr_splt_cl = r.f32(); sb.sk_splt_cl = r.f32(); sb.ss_splt_cl = r.f32();
    sb.v_it = r.i32(); sb.p_it = r.i32(); sb.d_it = r.i32(); sb.c_it = r.i32();
    sb.lst = r.f32(); sb.ast = r.f32(); sb.vst = r.f32();

    const int32_t anchor_count = r.i32();
    if (valid_count(r, anchor_count))
    {
        sb.anchors.resize((size_t)anchor_count);
        for (auto &a : sb.anchors)
        {
            a.rigidbody = r.index(r.header.rigidbody_index_size);
            a.vertex = r.vertex_index();
            a.near_mode = r.u8();
            if (r.error)
                break;
        }
    }
    const int32_t pin_count = r.i32();
    if (valid_count(r, pin_count))
    {
        sb.pins.resize((size_t)pin_count);
        for (auto &p : sb.pins)
        {
            p = r.vertex_index();
            if (r.error)
                break;
        }
    }
}

} // namespace

LoadResult
load_from_memory(const uint8_t *data, size_t size, Model &out)
{
    LoadResult res;
    Reader r(data, size);

    // Signature + version.
    if (size < 4 || std::memcmp(data, "PMX ", 4) != 0)
    {
        res.error = "Not a PMX file (bad signature; expected 'PMX ').";
        return res;
    }
    r.pos = 4;
    out = Model();
    out.header.version = r.f32();

    const uint8_t globals_count = r.u8();
    if (globals_count < 8)
    {
        res.error = "PMX globals block is too small (expected at least 8).";
        return res;
    }
    uint8_t globals[8] = { 0 };
    for (int i = 0; i < globals_count; ++i)
    {
        const uint8_t g = r.u8();
        if (i < 8)
            globals[i] = g;
    }
    Header &h = out.header;
    h.encoding = (Encoding)globals[0];
    h.additional_uv = globals[1];
    h.vertex_index_size = globals[2];
    h.texture_index_size = globals[3];
    h.material_index_size = globals[4];
    h.bone_index_size = globals[5];
    h.morph_index_size = globals[6];
    h.rigidbody_index_size = globals[7];
    if (h.additional_uv < 0 || h.additional_uv > 4)
    {
        res.error = "PMX additional UV count out of range (0..4).";
        return res;
    }
    r.header = h;                          // reader uses index sizes / encoding

    h.model_name_local = r.text();
    h.model_name_universal = r.text();
    h.comment_local = r.text();
    h.comment_universal = r.text();
    r.header = h;

    // Vertices.
    {
        const int32_t count = r.i32();
        if (!valid_count(r, count)) { res.error = r.err; return res; }
        out.vertices.resize((size_t)count);
        for (auto &v : out.vertices)
        {
            read_vertex(r, v);
            if (r.error) break;
        }
    }

    // Faces (triangle vertex indices).
    {
        const int32_t count = r.i32();
        if (!valid_count(r, count)) { res.error = r.err; return res; }
        out.indices.resize((size_t)count);
        for (auto &idx : out.indices)
        {
            idx = r.vertex_index();
            if (r.error) break;
        }
    }

    // Textures.
    {
        const int32_t count = r.i32();
        if (!valid_count(r, count)) { res.error = r.err; return res; }
        out.textures.resize((size_t)count);
        for (auto &t : out.textures)
        {
            t = r.text();
            if (r.error) break;
        }
    }

    // Materials.
    {
        const int32_t count = r.i32();
        if (!valid_count(r, count)) { res.error = r.err; return res; }
        out.materials.resize((size_t)count);
        for (auto &m : out.materials)
        {
            read_material(r, m);
            if (r.error) break;
        }
    }

    // Bones.
    {
        const int32_t count = r.i32();
        if (!valid_count(r, count)) { res.error = r.err; return res; }
        out.bones.resize((size_t)count);
        for (auto &b : out.bones)
        {
            read_bone(r, b);
            if (r.error) break;
        }
    }

    // Morphs.
    {
        const int32_t count = r.i32();
        if (!valid_count(r, count)) { res.error = r.err; return res; }
        out.morphs.resize((size_t)count);
        for (auto &mo : out.morphs)
        {
            read_morph(r, mo);
            if (r.error) break;
        }
    }

    // Display frames.
    {
        const int32_t count = r.i32();
        if (!valid_count(r, count)) { res.error = r.err; return res; }
        out.display_frames.resize((size_t)count);
        for (auto &dframe : out.display_frames)
        {
            dframe.name_local = r.text();
            dframe.name_universal = r.text();
            dframe.special_flag = r.u8();
            const int32_t elem_count = r.i32();
            if (!valid_count(r, elem_count)) break;
            dframe.elements.resize((size_t)elem_count);
            for (auto &e : dframe.elements)
            {
                e.type = r.u8();
                e.index = (e.type == 0) ? r.index(h.bone_index_size)
                                        : r.index(h.morph_index_size);
                if (r.error) break;
            }
            if (r.error) break;
        }
    }

    // Rigid bodies.
    {
        const int32_t count = r.i32();
        if (!valid_count(r, count)) { res.error = r.err; return res; }
        out.rigid_bodies.resize((size_t)count);
        for (auto &rb : out.rigid_bodies)
        {
            read_rigidbody(r, rb);
            if (r.error) break;
        }
    }

    // Joints.
    {
        const int32_t count = r.i32();
        if (!valid_count(r, count)) { res.error = r.err; return res; }
        out.joints.resize((size_t)count);
        for (auto &j : out.joints)
        {
            read_joint(r, j);
            if (r.error) break;
        }
    }

    // Soft bodies (PMX 2.1+ only). 2.0 files have no soft body section.
    if (!r.error && out.header.version > 2.05f && r.remaining() >= 4)
    {
        const int32_t count = r.i32();
        if (valid_count(r, count))
        {
            out.soft_bodies.resize((size_t)count);
            for (auto &sb : out.soft_bodies)
            {
                read_softbody(r, sb);
                if (r.error) break;
            }
        }
    }

    if (r.error)
    {
        res.error = r.err.empty() ? "Failed to parse PMX file." : r.err;
        return res;
    }
    res.ok = true;
    return res;
}

LoadResult
load(const char *path, Model &out)
{
    LoadResult res;
    if (!path || !*path)
    {
        res.error = "Empty PMX path.";
        return res;
    }

    std::error_code ec;
    // u8path constructs a wide path on Windows, so CJK paths open correctly.
    const std::filesystem::path fs_path = std::filesystem::u8path(path);
    const uintmax_t fsize = std::filesystem::file_size(fs_path, ec);
    if (ec)
    {
        res.error = std::string("Cannot stat PMX file: ") + path;
        return res;
    }

    std::ifstream in(fs_path, std::ios::binary);
    if (!in)
    {
        res.error = std::string("Cannot open PMX file: ") + path;
        return res;
    }

    std::vector<uint8_t> bytes((size_t)fsize);
    if (fsize > 0)
        in.read((char *)bytes.data(), (std::streamsize)fsize);
    if (!in && !in.eof())
    {
        res.error = std::string("Failed to read PMX file: ") + path;
        return res;
    }

    return load_from_memory(bytes.data(), bytes.size(), out);
}

} // namespace pmx
