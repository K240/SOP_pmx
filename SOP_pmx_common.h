#ifndef __SOP_pmx_common_h__
#define __SOP_pmx_common_h__

// Shared helpers bridging the Houdini-independent pmx::Model to Houdini types:
// coordinate/unit conversion, UTF-8 name handling, and texture path resolution.

#include <UT/UT_StringHolder.h>
#include <UT/UT_String.h>
#include <UT/UT_VarEncode.h>
#include <UT/UT_Vector3.h>

#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

#include "pmx_parser.h"

namespace sop_pmx
{

// MMD is left-handed (Y-up); Houdini is right-handed (Y-up). Converting flips Z
// on positions/normals and reverses triangle winding. Scale applies to lengths
// (positions) only, never to normals.
struct Xform
{
    bool convert = true;   // left-handed -> right-handed
    float scale = 1.0f;    // PMX unit -> Houdini unit (positions only)

    UT_Vector3 pos(const pmx::Vec3 &v) const
    {
        const float z = convert ? -v.z : v.z;
        return UT_Vector3(v.x * scale, v.y * scale, z * scale);
    }
    UT_Vector3 nml(const pmx::Vec3 &v) const
    {
        const float z = convert ? -v.z : v.z;
        UT_Vector3 n(v.x, v.y, z);
        n.normalize();
        return n;
    }
    // Direction vector (axes): Z flip only, no scaling, no renormalization.
    UT_Vector3 dir(const pmx::Vec3 &v) const
    {
        return UT_Vector3(v.x, v.y, convert ? -v.z : v.z);
    }
    // PMX is left-handed with clockwise front faces. When we negate Z on
    // positions/normals (convert), the same vertex order already yields outward,
    // normal-consistent faces, so winding is kept. In Original mode (no Z flip)
    // the winding must be reversed to agree with the un-negated normals.
    bool reverseWinding() const { return !convert; }
};

// PMX names are already UTF-8. Optionally encode to a valid Houdini attribute /
// group token. The returned holder references `s` unless sanitized, so keep the
// source pmx::Model alive while using it.
inline UT_StringHolder
pmxName(const std::string &s, bool sanitize)
{
    const UT_StringHolder raw(s.c_str(), (exint)s.length());
    if (sanitize && raw.isstring())
        return UT_VarEncode::encodeVar(raw);
    return raw;
}

inline UT_StringHolder
utf8(const std::string &s)
{
    return UT_StringHolder(s.c_str(), (exint)s.length());
}

// Directory containing the model file (UTF-8), used to resolve relative
// texture paths. Empty if `path` has no parent.
inline std::string
modelDirectory(const std::string &path)
{
    std::error_code ec;
    const std::filesystem::path p = std::filesystem::u8path(path);
    const std::filesystem::path dir = p.parent_path();
    if (dir.empty())
        return std::string();
    return dir.u8string();
}

// Resolve a PMX texture reference (relative, possibly using '\\' separators and
// CJK characters) against the model directory. Returns an absolute UTF-8 path
// when possible; the file is not required to exist.
inline std::string
resolveTexture(const std::string &model_dir, const std::string &rel)
{
    if (rel.empty())
        return std::string();

    std::string norm = rel;
    for (char &c : norm)
        if (c == '\\')
            c = '/';

    std::error_code ec;
    std::filesystem::path rp = std::filesystem::u8path(norm);
    if (rp.is_absolute() || model_dir.empty())
        return rp.lexically_normal().u8string();

    std::filesystem::path full = std::filesystem::u8path(model_dir) / rp;
    return full.lexically_normal().u8string();
}

inline bool
fileExists(const std::string &path)
{
    if (path.empty())
        return false;
    std::error_code ec;
    return std::filesystem::exists(std::filesystem::u8path(path), ec) && !ec;
}

// Optionally encode a UTF-8 name to a valid Houdini token.
inline std::string
encodeName(const std::string &s, bool sanitize)
{
    if (!sanitize || s.empty())
        return s;
    const UT_StringHolder enc =
        UT_VarEncode::encodeVar(UT_StringHolder(s.c_str(), (exint)s.length()));
    return std::string(enc.c_str(), enc.length());
}

// Make a list of names unique and non-empty (deterministically), so they can be
// used as point names / capture region names. Empty names get fallback_prefix+i.
inline std::vector<std::string>
deduplicate(std::vector<std::string> names, const char *fallback_prefix)
{
    std::unordered_set<std::string> used;
    for (size_t i = 0; i < names.size(); ++i)
    {
        std::string base = names[i].empty()
            ? (std::string(fallback_prefix) + std::to_string(i))
            : names[i];
        std::string name = base;
        if (used.count(name))
        {
            name = base + "_" + std::to_string(i);
            int s = 1;
            while (used.count(name))
                name = base + "_" + std::to_string(i) + "_" + std::to_string(s++);
        }
        used.insert(name);
        names[i] = std::move(name);
    }
    return names;
}

// Deterministic, collision-free bone names (UTF-8). Used for both bone capture
// region names (mesh output) and skeleton point names so they match exactly,
// which is what Bone/Skin Deform needs. Prefers the local (Japanese) name.
inline std::vector<std::string>
computeBoneNames(const pmx::Model &model, bool sanitize)
{
    std::vector<std::string> bases;
    bases.reserve(model.bones.size());
    for (const pmx::Bone &b : model.bones)
        bases.push_back(encodeName(!b.name_local.empty() ? b.name_local : b.name_universal, sanitize));
    return deduplicate(std::move(bases), "bone_");
}

// Deterministic, collision-free rigid body names (used by the joints output to
// reference bodies by name).
inline std::vector<std::string>
computeRigidNames(const pmx::Model &model, bool sanitize)
{
    std::vector<std::string> bases;
    bases.reserve(model.rigid_bodies.size());
    for (const pmx::RigidBody &r : model.rigid_bodies)
        bases.push_back(encodeName(!r.name_local.empty() ? r.name_local : r.name_universal, sanitize));
    return deduplicate(std::move(bases), "rigidbody_");
}

} // namespace sop_pmx

#endif
