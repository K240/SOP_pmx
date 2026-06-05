#include "SOP_pmx_reader.h"

#include "SOP_pmx_reader.proto.h"

#include "SOP_pmx_common.h"
#include "pmx_parser.h"

#include <CH/CH_Manager.h>
#include <GA/GA_ElementGroup.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Types.h>
#include <GEO/GEO_Detail.h>
#include <GU/GU_Detail.h>
#include <GU/GU_DetailHandle.h>
#include <GU/GU_PackedGeometry.h>
#include <GU/GU_PrimPacked.h>
#include <GU/GU_PrimPoly.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_TemplateBuilder.h>
#include <SYS/SYS_Math.h>
#include <SYS/SYS_Types.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Options.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_String.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_Vector4.h>
#include <UT/UT_XformOrder.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "pmx_parser.h"

using namespace UT::Literal;

const UT_StringHolder SOP_pmx_reader::theSOPTypeName("pmx_import"_sh);

void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        SOP_pmx_reader::theSOPTypeName,   // Internal name
        "pmx_import",                     // UI name
        SOP_pmx_reader::myConstructor,    // How to build the SOP
        SOP_pmx_reader::buildTemplates(), // My parameters
        0,                                // Min # of sources
        0,                                // Max # of sources
        nullptr,                          // Custom local variables (none)
        OP_FLAG_GENERATOR,                // Flag it as generator
        nullptr,                          // input labels
        5));                              // outputs: mesh, skeleton, rigidbodies, joints, morphs
}

static const char *theDsFile = R"THEDSFILE(
{
    name        parameters
    parm {
        name    "reimport"
        label   "Reimport"
        type    button
        default { "0" }
    }
    parm {
        name    "file"
        label   "pmx"
        type    file
        parmtag { filechooser_pattern "*.pmx" }
        parmtag { filechooser_mode "read" }
        default { "" }
    }
    parm {
        name    "coordinate"
        label   "Coordinate"
        type    ordinal
        default { "1" }
        menu {
            "original"  "Original"
            "houdini"   "Houdini"
        }
    }
    parm {
        name    "unitmeters"
        label   "Unit (meters per PMX unit)"
        type    float
        default { "0.08" }
        range   { 0.001! 1 }
        disablewhen "{ coordinate == original }"
    }
    parm {
        name    "sanitizenames"
        label   "Sanitize Names"
        type    toggle
        default { "0" }
    }
    parm {
        name    "skinning"
        label   "Import Skinning"
        type    toggle
        default { "1" }
    }
    parm {
        name    "blendshapes"
        label   "Import Blend Shapes"
        type    toggle
        default { "1" }
    }
}
)THEDSFILE";

static int
sopReimportWrapper(void *data, int /*index*/, fpreal /*t*/, const PRM_Template *)
{
    SOP_pmx_reader *me = static_cast<SOP_pmx_reader *>(data);
    if (!me)
        return 0;
    me->forceRecook();
    return 1;
}

PRM_Template *
SOP_pmx_reader::buildTemplates()
{
    static PRM_TemplateBuilder templ("SOP_pmx_reader.C"_sh, theDsFile);
    if (templ.justBuilt())
        templ.setCallback("reimport", &sopReimportWrapper);
    return templ.templates();
}

class SOP_pmx_readerVerb : public SOP_NodeVerb
{
public:
    SOP_pmx_readerVerb() {}
    ~SOP_pmx_readerVerb() override {}

    SOP_NodeParms *allocParms() const override { return new SOP_pmx_readerParms(); }
    UT_StringHolder name() const override { return SOP_pmx_reader::theSOPTypeName; }

    CookMode cookMode(const SOP_NodeParms *) const override { return COOK_GENERIC; }

    void cook(const CookParms &cookparms) const override;

    static const SOP_NodeVerb::Register<SOP_pmx_readerVerb> theVerb;
};

const SOP_NodeVerb::Register<SOP_pmx_readerVerb> SOP_pmx_readerVerb::theVerb;

const SOP_NodeVerb *
SOP_pmx_reader::cookVerb() const
{
    return SOP_pmx_readerVerb::theVerb.get();
}

// Defined later; appends the KineFX blend-shape packed prims to the mesh detail.
static void buildBlendshapes(GU_Detail *dst, const pmx::Model &model,
    const sop_pmx::Xform &xf, bool sanitize);

// Build the coordinate / unit transform. `convert` enables left-handed ->
// right-handed (Z flip) plus unit scaling; Original keeps raw data.
static sop_pmx::Xform
makeXform(bool convert, float unit_meters)
{
    sop_pmx::Xform xf;
    xf.convert = convert;
    if (convert)
    {
        float hip_unit = (float)CHgetManager()->getUnitLength();
        if (hip_unit <= 0.0f)
            hip_unit = 1.0f;
        xf.scale = unit_meters / hip_unit;
    }
    else
    {
        xf.scale = 1.0f;
    }
    return xf;
}

// Pick a material's display name (prefer the local/Japanese name).
static const std::string &
materialName(const pmx::Material &m)
{
    if (!m.name_local.empty())
        return m.name_local;
    return m.name_universal;
}

// Output 0: triangle mesh with normals, UVs, edge scale, per-material primitive
// attributes and a `pmx` detail dictionary.
void
SOP_pmx_readerVerb::cook(const SOP_NodeVerb::CookParms &cookparms) const
{
    auto &&sopparms = cookparms.parms<SOP_pmx_readerParms>();
    GU_Detail *detail = cookparms.gdh().gdpNC();

    UT_String path;
    path = sopparms.getFile();
    if (!path.length())
        return;

    using Coordinate = SOP_pmx_readerParms::Coordinate;
    const Coordinate coord = sopparms.opCoordinate(cookparms);
    const bool sanitize = sopparms.getSanitizenames();
    const float unit_meters = (float)sopparms.getUnitmeters();
    const bool do_skin = sopparms.getSkinning();
    const bool do_blendshapes = sopparms.getBlendshapes();

    UT_AutoInterrupt progress("Importing PMX mesh");

    pmx::Model model;
    const pmx::LoadResult res = pmx::load(path.c_str(), model);
    if (!res.ok)
    {
        cookparms.sopAddError(SOP_MESSAGE, res.error.c_str());
        return;
    }

    const sop_pmx::Xform xf = makeXform(coord == Coordinate::HOUDINI, unit_meters);

    // Resolve texture references (relative, CJK, '\\' separators) once per material.
    const std::string model_dir = sop_pmx::modelDirectory(std::string(path.c_str()));
    const size_t num_mats = model.materials.size();
    std::vector<std::string> mat_tex(num_mats), mat_sphere(num_mats), mat_toon(num_mats);
    auto resolveTex = [&](int ti) -> std::string {
        if (ti >= 0 && ti < (int)model.textures.size())
            return sop_pmx::resolveTexture(model_dir, model.textures[(size_t)ti]);
        return std::string();
    };
    for (size_t mi = 0; mi < num_mats; ++mi)
    {
        const pmx::Material &mat = model.materials[mi];
        mat_tex[mi] = resolveTex(mat.texture_index);
        mat_sphere[mi] = resolveTex(mat.environment_index);
        if (mat.toon_flag == 0)
            mat_toon[mi] = resolveTex(mat.toon_index);
    }

    detail->clearAndDestroy();

    // Point attributes.
    GA_RWHandleV3 n_h(detail->addNormalAttribute(GA_ATTRIB_POINT, GA_STORE_REAL32));
    GA_RWHandleV3 uv_h(detail->addFloatTuple(GA_ATTRIB_POINT, "uv", 3));
    GA_RWHandleF edge_h(detail->addFloatTuple(GA_ATTRIB_POINT, "pmx_edgescale", 1));
    // Stable point id (= PMX vertex index) for blendshape / morph correspondence
    // (the Morphs output references it via `id`; blendshapes ptidattr = "id").
    GA_RWHandleI id_h(detail->addIntTuple(GA_ATTRIB_POINT, "id", 1));

    const int auv = model.header.additional_uv;
    UT_Array<GA_RWHandleV4> auv_h;
    auv_h.setSize(auv);
    for (int i = 0; i < auv; ++i)
    {
        const std::string nm = "uv" + std::to_string(i + 2);
        auv_h[i] = GA_RWHandleV4(detail->addFloatTuple(GA_ATTRIB_POINT, nm.c_str(), 4));
    }

    // SDEF parameters (only when the model uses SDEF). Weights are still applied
    // as LBS in the bone capture; these preserve C/R0/R1 for an accurate SDEF
    // deformer downstream.
    bool has_sdef = false;
    for (const pmx::Vertex &v : model.vertices)
        if (v.weight_type == pmx::WeightType::SDEF) { has_sdef = true; break; }
    GA_RWHandleV3 sdefc_h, sdefr0_h, sdefr1_h;
    if (has_sdef)
    {
        sdefc_h = GA_RWHandleV3(detail->addFloatTuple(GA_ATTRIB_POINT, "pmx_sdef_c", 3));
        sdefr0_h = GA_RWHandleV3(detail->addFloatTuple(GA_ATTRIB_POINT, "pmx_sdef_r0", 3));
        sdefr1_h = GA_RWHandleV3(detail->addFloatTuple(GA_ATTRIB_POINT, "pmx_sdef_r1", 3));
    }

    // Primitive attributes.
    GA_RWHandleS mat_h(detail->addStringTuple(GA_ATTRIB_PRIMITIVE, "pmx_material", 1));
    GA_RWHandleI matidx_h(detail->addIntTuple(GA_ATTRIB_PRIMITIVE, "material_index", 1));
    GA_RWHandleS shoppath_h(detail->addStringTuple(GA_ATTRIB_PRIMITIVE, "shop_materialpath", 1));
    GA_RWHandleS tex_h(detail->addStringTuple(GA_ATTRIB_PRIMITIVE, "pmx_texture", 1));

    // Points: one Houdini point per PMX vertex, contiguous, so the point offset
    // for vertex v is (start + v).
    const exint nv = (exint)model.vertices.size();
    const GA_Offset start = detail->appendPointBlock(nv);
    for (exint i = 0; i < nv; ++i)
    {
        if (i % 4096 == 0 && progress.wasInterrupted())
            return;
        const GA_Offset off = start + i;
        const pmx::Vertex &v = model.vertices[(size_t)i];
        detail->setPos3(off, xf.pos(v.position));
        if (id_h.isValid())
            id_h.set(off, (int)i);
        if (n_h.isValid())
            n_h.set(off, xf.nml(v.normal));
        if (uv_h.isValid())
            uv_h.set(off, UT_Vector3(v.uv.x, 1.0f - v.uv.y, 0.0f));
        if (edge_h.isValid())
            edge_h.set(off, v.edge_scale);
        for (int a = 0; a < auv; ++a)
        {
            if (!auv_h[a].isValid())
                continue;
            const pmx::Vec4 &uvv = v.additional_uv[a];
            auv_h[a].set(off, UT_Vector4F(uvv.x, uvv.y, uvv.z, uvv.w));
        }
        if (has_sdef && v.weight_type == pmx::WeightType::SDEF)
        {
            sdefc_h.set(off, xf.pos(v.sdef_c));
            sdefr0_h.set(off, xf.pos(v.sdef_r0));
            sdefr1_h.set(off, xf.pos(v.sdef_r1));
        }
    }

    // Faces grouped by material: each material consumes a consecutive run of
    // surface_count indices (multiple of 3).
    const std::vector<int32_t> &idx = model.indices;
    const bool reverse = xf.reverseWinding();
    size_t cursor = 0;
    for (size_t mi = 0; mi < model.materials.size(); ++mi)
    {
        const pmx::Material &mat = model.materials[mi];
        const std::string &mname_src = materialName(mat);
        // pmx_material keeps the raw material name (exempt from Sanitize Names);
        // shop_materialpath is a usable token so it still honors the option.
        const UT_StringHolder mname_raw = sop_pmx::utf8(mname_src);
        const UT_StringHolder mpath = sanitize
            ? sop_pmx::pmxName(mname_src, true)
            : mname_raw;
        const UT_StringHolder texh = sop_pmx::utf8(mat_tex[mi]);

        const long tris = mat.surface_count / 3;
        for (long t = 0; t < tris; ++t)
        {
            if (cursor + 3 > idx.size())
                break;
            const int a = idx[cursor];
            const int b = idx[cursor + 1];
            const int c = idx[cursor + 2];
            cursor += 3;
            if (a < 0 || b < 0 || c < 0 || a >= nv || b >= nv || c >= nv)
                continue;

            GU_PrimPoly *poly = GU_PrimPoly::build(detail, 3, GU_POLY_CLOSED, 0);
            if (!poly)
                continue;
            poly->setVertexPoint(0, start + a);
            poly->setVertexPoint(1, start + (reverse ? c : b));
            poly->setVertexPoint(2, start + (reverse ? b : c));

            const GA_Offset primoff = poly->getMapOffset();
            if (mat_h.isValid())
                mat_h.set(primoff, mname_raw);
            if (matidx_h.isValid())
                matidx_h.set(primoff, (int)mi);
            if (shoppath_h.isValid())
                shoppath_h.set(primoff, mpath);
            if (tex_h.isValid() && texh.isstring())
                tex_h.set(primoff, texh);
        }
    }

    // Bone capture (skinning). Create every bone region up front so the per-point
    // capture arrays size correctly, then accumulate per-vertex weights.
    if (do_skin && !model.bones.empty())
    {
        const std::vector<std::string> bone_names = sop_pmx::computeBoneNames(model, sanitize);
        UT_Array<int> bone_region;
        bone_region.setSize((exint)model.bones.size());
        for (size_t bi = 0; bi < model.bones.size(); ++bi)
        {
            UT_String rname(bone_names[bi].c_str());
            UT_Matrix4 xform(1.0);
            xform.setTranslates(xf.pos(model.bones[bi].position));
            bone_region[(exint)bi] = detail->addCaptureRegion(rname, xform, 1.0f, 1.0f, 1.0f, 1.0f);
        }

        bool warned_sdef = false;
        for (exint i = 0; i < nv; ++i)
        {
            const pmx::Vertex &v = model.vertices[(size_t)i];
            if (v.weight_type == pmx::WeightType::SDEF && !warned_sdef)
            {
                cookparms.sopAddWarning(SOP_MESSAGE,
                    "SDEF skinning is imported as linear blend (LBS).");
                warned_sdef = true;
            }
            int b[4];
            float w[4];
            int cnt = 0;
            for (int k = 0; k < 4; ++k)
            {
                if (v.bones[k] >= 0 && v.bones[k] < (int)model.bones.size() && v.weights[k] > 0.0f)
                {
                    b[cnt] = v.bones[k];
                    w[cnt] = v.weights[k];
                    ++cnt;
                }
            }
            if (cnt == 0)
                continue;
            float sum = 0.0f;
            for (int k = 0; k < cnt; ++k)
                sum += w[k];
            if (sum <= 0.0f)
                continue;
            const GA_Offset off = start + i;
            for (int k = 0; k < cnt; ++k)
            {
                const int ridx = bone_region[(exint)b[k]];
                if (ridx < 0)
                    continue;
                const bool last = (k == cnt - 1);
                detail->setCaptureWeight(ridx, w[k] / sum, off, nullptr,
                    /*add_weight*/ true, /*clamp_negative*/ false,
                    /*normalizeweight*/ last, GEO_Detail::CAPTURE_BONE);
            }
        }
    }

    // Per-material catalog as a detail dict-array `pmx_materials`.
    if (num_mats > 0)
    {
        UT_Array<UT_OptionsHolder> mat_dicts;
        mat_dicts.setCapacity((exint)num_mats);
        for (size_t mi = 0; mi < num_mats; ++mi)
        {
            const pmx::Material &mat = model.materials[mi];
            UT_Options mo;
            mo.setOptionS("name", sop_pmx::utf8(mat.name_local));
            mo.setOptionS("name_en", sop_pmx::utf8(mat.name_universal));
            mo.setOptionV4("diffuse", UT_Vector4(mat.diffuse.x, mat.diffuse.y, mat.diffuse.z, mat.diffuse.w));
            mo.setOptionV3("specular", UT_Vector3(mat.specular.x, mat.specular.y, mat.specular.z));
            mo.setOptionF("specular_strength", (fpreal64)mat.specular_strength);
            mo.setOptionV3("ambient", UT_Vector3(mat.ambient.x, mat.ambient.y, mat.ambient.z));
            mo.setOptionV4("edge_color", UT_Vector4(mat.edge_color.x, mat.edge_color.y, mat.edge_color.z, mat.edge_color.w));
            mo.setOptionF("edge_scale", (fpreal64)mat.edge_scale);
            mo.setOptionI("draw_flags", mat.draw_flags);
            mo.setOptionB("no_cull", (mat.draw_flags & pmx::MAT_NO_CULL) != 0);
            mo.setOptionB("ground_shadow", (mat.draw_flags & pmx::MAT_GROUND_SHADOW) != 0);
            mo.setOptionB("cast_shadow", (mat.draw_flags & pmx::MAT_CAST_SHADOW) != 0);
            mo.setOptionB("receive_shadow", (mat.draw_flags & pmx::MAT_RECEIVE_SHADOW) != 0);
            mo.setOptionB("has_edge", (mat.draw_flags & pmx::MAT_HAS_EDGE) != 0);
            mo.setOptionS("texture", sop_pmx::utf8(mat_tex[mi]));
            mo.setOptionS("sphere", sop_pmx::utf8(mat_sphere[mi]));
            mo.setOptionI("sphere_mode", mat.environment_blend);
            mo.setOptionI("toon_flag", mat.toon_flag);
            if (mat.toon_flag == 0)
                mo.setOptionS("toon", sop_pmx::utf8(mat_toon[mi]));
            else
                mo.setOptionI("toon_internal", mat.toon_index);
            mo.setOptionI("num_triangles", mat.surface_count / 3);
            mat_dicts.append(UT_OptionsHolder(&mo));
        }
        GA_RWHandleDictA mats_h(detail->addDictArray(GA_ATTRIB_DETAIL, "pmx_materials", 1));
        if (mats_h.isValid())
            mats_h.set(GA_Offset(0), mat_dicts);
    }

    // Detail dictionary `pmx` with import metadata.
    UT_Options info;
    info.setOptionF("version", (fpreal64)model.header.version);
    info.setOptionI("encoding", (int)model.header.encoding);
    info.setOptionS("model_name_local", sop_pmx::utf8(model.header.model_name_local));
    info.setOptionS("model_name_universal", sop_pmx::utf8(model.header.model_name_universal));
    info.setOptionS("comment", sop_pmx::utf8(model.header.comment_local));
    info.setOptionI("additional_uv", model.header.additional_uv);
    info.setOptionI("num_vertices", (int)model.vertices.size());
    info.setOptionI("num_triangles", (int)(model.indices.size() / 3));
    info.setOptionI("num_textures", (int)model.textures.size());
    info.setOptionI("num_materials", (int)model.materials.size());
    info.setOptionI("num_bones", (int)model.bones.size());
    info.setOptionI("num_morphs", (int)model.morphs.size());
    info.setOptionI("num_rigidbodies", (int)model.rigid_bodies.size());
    info.setOptionI("num_joints", (int)model.joints.size());
    info.setOptionI("num_softbodies", (int)model.soft_bodies.size());
    info.setOptionS("coordinate", SOP_pmx_readerEnums::getToken(coord));
    info.setOptionF("unit_meters", (fpreal64)unit_meters);
    info.setOptionF("scale", (fpreal64)xf.scale);
    info.setOptionS("source_file", UT_StringHolder(path.c_str()));

    GA_RWHandleDict dict_h(detail->addDictTuple(GA_ATTRIB_DETAIL, "pmx", 1));
    if (dict_h.isValid())
        dict_h.set(GA_Offset(0), UT_OptionsHolder(&info));

    // Merge KineFX Character Blend Shapes into the mesh as hidden packed prims,
    // so output 0 feeds straight into `Character Blend Shapes` (Rest Geometry).
    if (do_blendshapes)
        buildBlendshapes(detail, model, xf, sanitize);

    detail->bumpAllDataIds();
}

// Output 1: KineFX-style skeleton. One point per bone (name + identity
// transform, matching the mesh's bone-capture bind), parent->child polylines for
// the hierarchy, and a `pmx_bone` point dictionary carrying the full bone record.
static void
buildSkeleton(GU_Detail *dst, const pmx::Model &model, const sop_pmx::Xform &xf, bool sanitize)
{
    dst->clearAndDestroy();
    if (model.bones.empty())
        return;

    const std::vector<std::string> names = sop_pmx::computeBoneNames(model, sanitize);

    GA_RWHandleS name_h(dst->addStringTuple(GA_ATTRIB_POINT, "name", 1));
    GA_RWHandleM3 xform_h(dst->addFloatTuple(GA_ATTRIB_POINT, "transform", 9));
    GA_RWHandleI bidx_h(dst->addIntTuple(GA_ATTRIB_POINT, "pmx_bone_index", 1));
    GA_RWHandleI parent_h(dst->addIntTuple(GA_ATTRIB_POINT, "pmx_parent", 1));
    GA_RWHandleI layer_h(dst->addIntTuple(GA_ATTRIB_POINT, "pmx_layer", 1));
    GA_RWHandleDict info_h(dst->addDictTuple(GA_ATTRIB_POINT, "pmx_bone", 1));

    const exint nb = (exint)model.bones.size();
    const GA_Offset start = dst->appendPointBlock(nb);
    const UT_Matrix3F ident(1.0f);

    for (exint i = 0; i < nb; ++i)
    {
        const pmx::Bone &b = model.bones[(size_t)i];
        const GA_Offset off = start + i;
        dst->setPos3(off, xf.pos(b.position));
        if (name_h.isValid())
            name_h.set(off, UT_StringHolder(names[(size_t)i].c_str(), (exint)names[(size_t)i].length()));
        if (xform_h.isValid())
            xform_h.set(off, ident);
        if (bidx_h.isValid())
            bidx_h.set(off, (int)i);
        if (parent_h.isValid())
            parent_h.set(off, b.parent);
        if (layer_h.isValid())
            layer_h.set(off, b.layer);

        if (!info_h.isValid())
            continue;

        UT_Options o;
        o.setOptionS("name_en", sop_pmx::utf8(b.name_universal));
        o.setOptionI("parent", b.parent);
        o.setOptionI("layer", b.layer);
        o.setOptionI("flags", b.flags);
        o.setOptionB("rotatable", (b.flags & pmx::BONE_ROTATABLE) != 0);
        o.setOptionB("translatable", (b.flags & pmx::BONE_TRANSLATABLE) != 0);
        o.setOptionB("visible", (b.flags & pmx::BONE_VISIBLE) != 0);
        o.setOptionB("enabled", (b.flags & pmx::BONE_ENABLED) != 0);
        o.setOptionB("ik", (b.flags & pmx::BONE_IK) != 0);
        o.setOptionB("inherit_rotation", (b.flags & pmx::BONE_INHERIT_ROTATION) != 0);
        o.setOptionB("inherit_translation", (b.flags & pmx::BONE_INHERIT_TRANSLATION) != 0);
        o.setOptionB("fixed_axis", (b.flags & pmx::BONE_FIXED_AXIS) != 0);
        o.setOptionB("local_axis", (b.flags & pmx::BONE_LOCAL_AXIS) != 0);
        o.setOptionB("physics_after_deform", (b.flags & pmx::BONE_PHYSICS_AFTER_DEFORM) != 0);

        const bool tail_is_bone = (b.flags & pmx::BONE_INDEXED_TAIL) != 0;
        o.setOptionB("tail_is_bone", tail_is_bone);
        if (tail_is_bone)
        {
            o.setOptionI("tail_bone", b.tail_bone);
            if (b.tail_bone >= 0 && b.tail_bone < (int)model.bones.size())
                o.setOptionV3("tail_pos", xf.pos(model.bones[(size_t)b.tail_bone].position));
        }
        else
        {
            const pmx::Vec3 tw{ b.position.x + b.tail_offset.x,
                                b.position.y + b.tail_offset.y,
                                b.position.z + b.tail_offset.z };
            o.setOptionV3("tail_pos", xf.pos(tw));
        }

        if (b.flags & (pmx::BONE_INHERIT_ROTATION | pmx::BONE_INHERIT_TRANSLATION))
        {
            o.setOptionI("inherit_parent", b.inherit_parent);
            o.setOptionF("inherit_weight", (fpreal64)b.inherit_weight);
        }
        if (b.flags & pmx::BONE_FIXED_AXIS)
            o.setOptionV3("fixed_axis", xf.dir(b.fixed_axis));
        if (b.flags & pmx::BONE_LOCAL_AXIS)
        {
            o.setOptionV3("local_axis_x", xf.dir(b.local_axis_x));
            o.setOptionV3("local_axis_z", xf.dir(b.local_axis_z));
        }
        if (b.flags & pmx::BONE_EXTERNAL_PARENT)
            o.setOptionI("external_parent_key", b.external_parent_key);
        if (b.flags & pmx::BONE_IK)
        {
            o.setOptionI("ik_target", b.ik_target);
            o.setOptionI("ik_loop", b.ik_loop_count);
            o.setOptionF("ik_limit_rad", (fpreal64)b.ik_limit_radian);
            std::vector<int64> link_bones, link_haslimit;
            std::vector<fpreal32> link_min, link_max;
            for (const pmx::IkLink &lk : b.ik_links)
            {
                link_bones.push_back(lk.bone);
                link_haslimit.push_back(lk.has_limit ? 1 : 0);
                link_min.push_back(lk.limit_min.x);
                link_min.push_back(lk.limit_min.y);
                link_min.push_back(lk.limit_min.z);
                link_max.push_back(lk.limit_max.x);
                link_max.push_back(lk.limit_max.y);
                link_max.push_back(lk.limit_max.z);
            }
            if (!link_bones.empty())
            {
                o.setOptionIArray("ik_link_bones", link_bones.data(), link_bones.size());
                o.setOptionIArray("ik_link_has_limit", link_haslimit.data(), link_haslimit.size());
                o.setOptionFArray("ik_link_min", link_min.data(), link_min.size());
                o.setOptionFArray("ik_link_max", link_max.data(), link_max.size());
            }
        }
        info_h.set(off, UT_OptionsHolder(&o));
    }

    // Parent -> child open polylines define the KineFX hierarchy.
    for (exint i = 0; i < nb; ++i)
    {
        const int p = model.bones[(size_t)i].parent;
        if (p < 0 || p >= nb)
            continue;
        GU_PrimPoly *line = GU_PrimPoly::build(dst, 2, GU_POLY_OPEN, 0);
        if (!line)
            continue;
        line->setVertexPoint(0, start + p);
        line->setVertexPoint(1, start + i);
    }
}

// --- PMX -> Jolt enum mapping (Jolt menu order from the sibling jolt plugin) ---
static int joltShapeType(pmx::RigidShape s)
{
    switch (s)
    {
    case pmx::RigidShape::Box: return 0;       // box
    case pmx::RigidShape::Sphere: return 1;    // sphere
    case pmx::RigidShape::Capsule: return 2;   // capsule
    default: return 1;
    }
}
static const char *pmxShapeName(pmx::RigidShape s)
{
    switch (s)
    {
    case pmx::RigidShape::Box: return "box";
    case pmx::RigidShape::Sphere: return "sphere";
    case pmx::RigidShape::Capsule: return "capsule";
    default: return "sphere";
    }
}
// FollowBone bodies are animated by their bone (kinematic); physics modes are dynamic.
static int joltBodyType(pmx::PhysicsMode m)
{
    return (m == pmx::PhysicsMode::FollowBone) ? 2 /*kinematic*/ : 1 /*dynamic*/;
}
static int joltConstraintType(pmx::JointType t)
{
    switch (t)
    {
    case pmx::JointType::P2P: return 1;        // point
    case pmx::JointType::ConeTwist: return 5;  // cone
    case pmx::JointType::Slider: return 4;     // slider
    case pmx::JointType::Hinge: return 2;      // hinge
    default: return 6;                         // spring6dof / 6dof -> sixdof
    }
}
static const char *pmxJointTypeName(pmx::JointType t)
{
    switch (t)
    {
    case pmx::JointType::Spring6DOF: return "spring6dof";
    case pmx::JointType::SixDOF: return "6dof";
    case pmx::JointType::P2P: return "p2p";
    case pmx::JointType::ConeTwist: return "conetwist";
    case pmx::JointType::Slider: return "slider";
    case pmx::JointType::Hinge: return "hinge";
    default: return "spring6dof";
    }
}

// Helpers to set int/float array options (skip when empty).
static void
setIArray(UT_Options &o, const char *key, const std::vector<int64> &v)
{
    if (!v.empty())
        o.setOptionIArray(key, v.data(), v.size());
}
static void
setFArray(UT_Options &o, const char *key, const std::vector<fpreal32> &v)
{
    if (!v.empty())
        o.setOptionFArray(key, v.data(), v.size());
}

// Output 2: one point per rigid body with shape/mass/physics attributes. Stores
// both lossless `pmx_*` data and best-effort `jolt_*` convenience attributes.
static void
buildRigidBodies(GU_Detail *dst, const pmx::Model &model, const sop_pmx::Xform &xf, bool sanitize)
{
    dst->clearAndDestroy();
    if (model.rigid_bodies.empty() && model.soft_bodies.empty())
        return;

    const std::vector<std::string> rnames = sop_pmx::computeRigidNames(model, sanitize);
    const std::vector<std::string> bnames = sop_pmx::computeBoneNames(model, sanitize);

    GA_RWHandleS name_h(dst->addStringTuple(GA_ATTRIB_POINT, "name", 1));
    GA_RWHandleQ orient_h(dst->addFloatTuple(GA_ATTRIB_POINT, "orient", 4));
    GA_RWHandleS shape_h(dst->addStringTuple(GA_ATTRIB_POINT, "pmx_shape", 1));
    GA_RWHandleV3 size_h(dst->addFloatTuple(GA_ATTRIB_POINT, "size", 3));
    GA_RWHandleF pscale_h(dst->addFloatTuple(GA_ATTRIB_POINT, "pscale", 1));
    GA_RWHandleS bone_h(dst->addStringTuple(GA_ATTRIB_POINT, "pmx_bone", 1));
    GA_RWHandleI boneidx_h(dst->addIntTuple(GA_ATTRIB_POINT, "pmx_bone_index", 1));
    GA_RWHandleF mass_h(dst->addFloatTuple(GA_ATTRIB_POINT, "pmx_mass", 1));
    GA_RWHandleI group_h(dst->addIntTuple(GA_ATTRIB_POINT, "pmx_group", 1));
    GA_RWHandleI mask_h(dst->addIntTuple(GA_ATTRIB_POINT, "pmx_noncollision_mask", 1));
    GA_RWHandleI mode_h(dst->addIntTuple(GA_ATTRIB_POINT, "pmx_physics_mode", 1));
    GA_RWHandleV3 rot_h(dst->addFloatTuple(GA_ATTRIB_POINT, "pmx_rotation", 3));
    GA_RWHandleF lind_h(dst->addFloatTuple(GA_ATTRIB_POINT, "pmx_linear_damping", 1));
    GA_RWHandleF angd_h(dst->addFloatTuple(GA_ATTRIB_POINT, "pmx_angular_damping", 1));
    GA_RWHandleF rest_h(dst->addFloatTuple(GA_ATTRIB_POINT, "pmx_restitution", 1));
    GA_RWHandleF fric_h(dst->addFloatTuple(GA_ATTRIB_POINT, "pmx_friction", 1));
    GA_RWHandleI jshape_h(dst->addIntTuple(GA_ATTRIB_POINT, "jolt_shapetype", 1));
    GA_RWHandleI jbody_h(dst->addIntTuple(GA_ATTRIB_POINT, "jolt_bodytype", 1));
    GA_RWHandleF jfric_h(dst->addFloatTuple(GA_ATTRIB_POINT, "jolt_friction", 1));
    GA_RWHandleF jrest_h(dst->addFloatTuple(GA_ATTRIB_POINT, "jolt_restitution", 1));
    GA_RWHandleF jlind_h(dst->addFloatTuple(GA_ATTRIB_POINT, "jolt_lineardamping", 1));
    GA_RWHandleF jangd_h(dst->addFloatTuple(GA_ATTRIB_POINT, "jolt_angulardamping", 1));

    const exint nr = (exint)model.rigid_bodies.size();
    const GA_Offset start = dst->appendPointBlock(nr);
    const UT_XformOrder order(UT_XformOrder::TRS, UT_XformOrder::YXZ);

    for (exint i = 0; i < nr; ++i)
    {
        const pmx::RigidBody &rb = model.rigid_bodies[(size_t)i];
        const GA_Offset off = start + i;
        dst->setPos3(off, xf.pos(rb.position));
        name_h.set(off, UT_StringHolder(rnames[(size_t)i].c_str(), (exint)rnames[(size_t)i].length()));

        // Orientation: build from PMX euler (best-effort order), then mirror
        // about Z to match the right-handed conversion. Raw euler kept below.
        UT_QuaternionF q;
        q.updateFromEuler(UT_Vector3F(rb.rotation.x, rb.rotation.y, rb.rotation.z), order);
        if (xf.convert)
            q = UT_QuaternionF(-q.x(), -q.y(), q.z(), q.w());
        orient_h.set(off, q);

        shape_h.set(off, UT_StringHolder(pmxShapeName(rb.shape)));
        size_h.set(off, UT_Vector3(rb.size.x * xf.scale, rb.size.y * xf.scale, rb.size.z * xf.scale));
        pscale_h.set(off, SYSmax(rb.size.x, SYSmax(rb.size.y, rb.size.z)) * xf.scale);
        if (rb.bone >= 0 && rb.bone < (int)model.bones.size())
            bone_h.set(off, UT_StringHolder(bnames[(size_t)rb.bone].c_str(), (exint)bnames[(size_t)rb.bone].length()));
        boneidx_h.set(off, rb.bone);
        mass_h.set(off, rb.mass);
        group_h.set(off, rb.group);
        mask_h.set(off, rb.non_collision_mask);
        mode_h.set(off, (int)rb.physics_mode);
        rot_h.set(off, UT_Vector3(rb.rotation.x, rb.rotation.y, rb.rotation.z));
        lind_h.set(off, rb.linear_damping);
        angd_h.set(off, rb.angular_damping);
        rest_h.set(off, rb.restitution);
        fric_h.set(off, rb.friction);
        jshape_h.set(off, joltShapeType(rb.shape));
        jbody_h.set(off, joltBodyType(rb.physics_mode));
        jfric_h.set(off, rb.friction);
        jrest_h.set(off, rb.restitution);
        jlind_h.set(off, rb.linear_damping);
        jangd_h.set(off, rb.angular_damping);
    }

    // Soft bodies (PMX 2.1) catalogued as a detail dict-array on this output.
    if (!model.soft_bodies.empty())
    {
        UT_Array<UT_OptionsHolder> dicts;
        dicts.setCapacity((exint)model.soft_bodies.size());
        for (const pmx::SoftBody &sb : model.soft_bodies)
        {
            UT_Options o;
            o.setOptionS("name", sop_pmx::utf8(sb.name_local));
            o.setOptionS("name_en", sop_pmx::utf8(sb.name_universal));
            o.setOptionI("shape", (int)sb.shape);          // 0 tri-mesh, 1 rope
            o.setOptionI("material", sb.material);
            o.setOptionI("group", sb.group);
            o.setOptionI("non_collision_mask", sb.non_collision_mask);
            o.setOptionI("flags", sb.flags);
            o.setOptionI("b_link_distance", sb.b_link_distance);
            o.setOptionI("cluster_count", sb.cluster_count);
            o.setOptionF("total_mass", (fpreal64)sb.total_mass);
            o.setOptionF("collision_margin", (fpreal64)sb.collision_margin);
            o.setOptionI("aero_model", sb.aero_model);
            const fpreal32 cfg[12] = { sb.vcf, sb.dp, sb.dg, sb.lf, sb.pr, sb.vc,
                                       sb.df, sb.mt, sb.chr, sb.khr, sb.shr, sb.ahr };
            o.setOptionFArray("config", cfg, 12);
            const fpreal32 cl[6] = { sb.srhr_cl, sb.skhr_cl, sb.sshr_cl,
                                     sb.sr_splt_cl, sb.sk_splt_cl, sb.ss_splt_cl };
            o.setOptionFArray("cluster", cl, 6);
            const int64 it[4] = { sb.v_it, sb.p_it, sb.d_it, sb.c_it };
            o.setOptionIArray("iteration", it, 4);
            const fpreal32 mat[3] = { sb.lst, sb.ast, sb.vst };
            o.setOptionFArray("material_coeff", mat, 3);
            std::vector<int64> anchor_rb, anchor_vtx, anchor_near, pins;
            for (const pmx::SoftBodyAnchor &a : sb.anchors)
            {
                anchor_rb.push_back(a.rigidbody);
                anchor_vtx.push_back(a.vertex);
                anchor_near.push_back(a.near_mode);
            }
            for (int32_t p : sb.pins)
                pins.push_back(p);
            setIArray(o, "anchor_rigidbodies", anchor_rb);
            setIArray(o, "anchor_vertices", anchor_vtx);
            setIArray(o, "anchor_near", anchor_near);
            setIArray(o, "pins", pins);
            dicts.append(UT_OptionsHolder(&o));
        }
        GA_RWHandleDictA h(dst->addDictArray(GA_ATTRIB_DETAIL, "pmx_soft_bodies", 1));
        if (h.isValid())
            h.set(GA_Offset(0), dicts);
    }
}

// Output 3: one open 2-point polyline per joint (rigid body A -> B), with the
// constraint type, body references and full PMX limit/spring data.
static void
buildJoints(GU_Detail *dst, const pmx::Model &model, const sop_pmx::Xform &xf, bool sanitize)
{
    dst->clearAndDestroy();
    if (model.joints.empty())
        return;

    const std::vector<std::string> rnames = sop_pmx::computeRigidNames(model, sanitize);
    std::vector<std::string> jbases;
    jbases.reserve(model.joints.size());
    for (const pmx::Joint &j : model.joints)
        jbases.push_back(sop_pmx::encodeName(!j.name_local.empty() ? j.name_local : j.name_universal, sanitize));
    const std::vector<std::string> jnames = sop_pmx::deduplicate(std::move(jbases), "joint_");

    GA_RWHandleS name_h(dst->addStringTuple(GA_ATTRIB_PRIMITIVE, "name", 1));
    GA_RWHandleI jtype_h(dst->addIntTuple(GA_ATTRIB_PRIMITIVE, "pmx_joint_type", 1));
    GA_RWHandleS jtypename_h(dst->addStringTuple(GA_ATTRIB_PRIMITIVE, "pmx_joint_type_name", 1));
    GA_RWHandleI jct_h(dst->addIntTuple(GA_ATTRIB_PRIMITIVE, "jolt_constraint_type", 1));
    GA_RWHandleS ba_h(dst->addStringTuple(GA_ATTRIB_PRIMITIVE, "jolt_body_a", 1));
    GA_RWHandleS bb_h(dst->addStringTuple(GA_ATTRIB_PRIMITIVE, "jolt_body_b", 1));
    GA_RWHandleI ai_h(dst->addIntTuple(GA_ATTRIB_PRIMITIVE, "pmx_rigid_a", 1));
    GA_RWHandleI bi_h(dst->addIntTuple(GA_ATTRIB_PRIMITIVE, "pmx_rigid_b", 1));
    GA_RWHandleV3 jpos_h(dst->addFloatTuple(GA_ATTRIB_PRIMITIVE, "pmx_joint_pos", 3));
    GA_RWHandleV3 jrot_h(dst->addFloatTuple(GA_ATTRIB_PRIMITIVE, "pmx_joint_rot", 3));
    GA_RWHandleV3 plo_h(dst->addFloatTuple(GA_ATTRIB_PRIMITIVE, "pmx_pos_limit_lower", 3));
    GA_RWHandleV3 phi_h(dst->addFloatTuple(GA_ATTRIB_PRIMITIVE, "pmx_pos_limit_upper", 3));
    GA_RWHandleV3 rlo_h(dst->addFloatTuple(GA_ATTRIB_PRIMITIVE, "pmx_rot_limit_lower", 3));
    GA_RWHandleV3 rhi_h(dst->addFloatTuple(GA_ATTRIB_PRIMITIVE, "pmx_rot_limit_upper", 3));
    GA_RWHandleV3 sp_h(dst->addFloatTuple(GA_ATTRIB_PRIMITIVE, "pmx_spring_pos", 3));
    GA_RWHandleV3 sr_h(dst->addFloatTuple(GA_ATTRIB_PRIMITIVE, "pmx_spring_rot", 3));

    const int nr = (int)model.rigid_bodies.size();
    auto bodyPos = [&](int idx, const pmx::Joint &j) -> UT_Vector3 {
        if (idx >= 0 && idx < nr)
            return xf.pos(model.rigid_bodies[(size_t)idx].position);
        return xf.pos(j.position);
    };
    const float s = xf.scale;

    for (size_t i = 0; i < model.joints.size(); ++i)
    {
        const pmx::Joint &j = model.joints[i];
        const GA_Offset p0 = dst->appendPoint();
        dst->setPos3(p0, bodyPos(j.rigidbody_a, j));
        const GA_Offset p1 = dst->appendPoint();
        dst->setPos3(p1, bodyPos(j.rigidbody_b, j));

        GU_PrimPoly *line = GU_PrimPoly::build(dst, 2, GU_POLY_OPEN, 0);
        if (!line)
            continue;
        line->setVertexPoint(0, p0);
        line->setVertexPoint(1, p1);
        const GA_Offset prim = line->getMapOffset();

        name_h.set(prim, UT_StringHolder(jnames[i].c_str(), (exint)jnames[i].length()));
        jtype_h.set(prim, (int)j.type);
        jtypename_h.set(prim, UT_StringHolder(pmxJointTypeName(j.type)));
        jct_h.set(prim, joltConstraintType(j.type));
        if (j.rigidbody_a >= 0 && j.rigidbody_a < nr)
            ba_h.set(prim, UT_StringHolder(rnames[(size_t)j.rigidbody_a].c_str(), (exint)rnames[(size_t)j.rigidbody_a].length()));
        if (j.rigidbody_b >= 0 && j.rigidbody_b < nr)
            bb_h.set(prim, UT_StringHolder(rnames[(size_t)j.rigidbody_b].c_str(), (exint)rnames[(size_t)j.rigidbody_b].length()));
        ai_h.set(prim, j.rigidbody_a);
        bi_h.set(prim, j.rigidbody_b);
        jpos_h.set(prim, xf.pos(j.position));
        jrot_h.set(prim, UT_Vector3(j.rotation.x, j.rotation.y, j.rotation.z));
        // Position limits are lengths (scaled); rotation limits/springs are kept
        // in PMX joint-local convention (radians / raw).
        plo_h.set(prim, UT_Vector3(j.position_limit_lower.x * s, j.position_limit_lower.y * s, j.position_limit_lower.z * s));
        phi_h.set(prim, UT_Vector3(j.position_limit_upper.x * s, j.position_limit_upper.y * s, j.position_limit_upper.z * s));
        rlo_h.set(prim, UT_Vector3(j.rotation_limit_lower.x, j.rotation_limit_lower.y, j.rotation_limit_lower.z));
        rhi_h.set(prim, UT_Vector3(j.rotation_limit_upper.x, j.rotation_limit_upper.y, j.rotation_limit_upper.z));
        sp_h.set(prim, UT_Vector3(j.spring_position.x * s, j.spring_position.y * s, j.spring_position.z * s));
        sr_h.set(prim, UT_Vector3(j.spring_rotation.x, j.spring_rotation.y, j.spring_rotation.z));
    }
}

static const char *morphTypeName(pmx::MorphType t)
{
    switch (t)
    {
    case pmx::MorphType::Group: return "group";
    case pmx::MorphType::Vertex: return "vertex";
    case pmx::MorphType::Bone: return "bone";
    case pmx::MorphType::UV: return "uv";
    case pmx::MorphType::UV1: return "uv1";
    case pmx::MorphType::UV2: return "uv2";
    case pmx::MorphType::UV3: return "uv3";
    case pmx::MorphType::UV4: return "uv4";
    case pmx::MorphType::Material: return "material";
    case pmx::MorphType::Flip: return "flip";
    case pmx::MorphType::Impulse: return "impulse";
    default: return "unknown";
    }
}

// KineFX Character Blend Shapes, merged into the Mesh output (0). Matches the
// stock format (verified against the SimpleCharacterBlendShapes example): the
// base mesh primitives carry `name` = the skin id; each vertex morph becomes a
// packed primitive of LOOSE POINTS (point `id` = base vertex index, `P` = the
// absolute morphed position) with `name` = the same skin id, `blendshape_channel`
// = "<morph>", `blendshape_name` = "<skin>.<morph>", placed in the
// `_3d_hidden_primitives` group. The Character Blend Shapes channels node links a
// blend shape to its skin by the shared `name`. Appends to `dst` (does NOT clear).
static void
buildBlendshapes(GU_Detail *dst, const pmx::Model &model, const sop_pmx::Xform &xf, bool sanitize)
{
    if (model.morphs.empty())
        return;

    std::vector<std::string> mbases;
    mbases.reserve(model.morphs.size());
    for (const pmx::Morph &mo : model.morphs)
        mbases.push_back(sop_pmx::encodeName(!mo.name_local.empty() ? mo.name_local : mo.name_universal, sanitize));
    const std::vector<std::string> mnames = sop_pmx::deduplicate(std::move(mbases), "morph_");

    const std::string skin = sop_pmx::skinName(model);
    const UT_StringHolder skin_h(skin.c_str(), (exint)skin.length());
    const int nv = (int)model.vertices.size();

    // Base mesh primitives identify their skin via `name`.
    GA_RWHandleS name_h(dst->addStringTuple(GA_ATTRIB_PRIMITIVE, "name", 1));
    if (name_h.isValid())
        for (GA_Iterator it(dst->getPrimitiveRange()); !it.atEnd(); ++it)
            name_h.set(*it, skin_h);

    GA_RWHandleS chan_h(dst->addStringTuple(GA_ATTRIB_PRIMITIVE, "blendshape_channel", 1));
    GA_RWHandleS bsname_h(dst->addStringTuple(GA_ATTRIB_PRIMITIVE, "blendshape_name", 1));
    GA_RWHandleF weight_h(dst->addFloatTuple(GA_ATTRIB_PRIMITIVE, "weight", 1));
    GA_PrimitiveGroup *hidden = dst->newPrimitiveGroup("_3d_hidden_primitives");

    for (size_t mi = 0; mi < model.morphs.size(); ++mi)
    {
        const pmx::Morph &mo = model.morphs[mi];
        if (mo.vertex.empty())   // only vertex morphs map to P blendshapes
            continue;

        // Embedded blend target: loose points at the absolute morphed position.
        GU_Detail *emb = new GU_Detail();
        GA_RWHandleI eid_h(emb->addIntTuple(GA_ATTRIB_POINT, "id", 1));
        for (const pmx::VertexMorphOffset &vo : mo.vertex)
        {
            if (vo.vertex < 0 || vo.vertex >= nv)
                continue;
            const GA_Offset pt = emb->appendPoint();
            const pmx::Vec3 &b = model.vertices[(size_t)vo.vertex].position;
            emb->setPos3(pt, xf.pos(pmx::Vec3{ b.x + vo.offset.x, b.y + vo.offset.y, b.z + vo.offset.z }));
            if (eid_h.isValid())
                eid_h.set(pt, vo.vertex);
        }
        if (emb->getNumPoints() == 0)
        {
            delete emb;
            continue;
        }

        GU_DetailHandle emb_gdh;
        emb_gdh.allocateAndSet(emb, true);
        GU_PrimPacked *pack = GU_PackedGeometry::packGeometry(*dst, GU_ConstDetailHandle(emb_gdh));
        if (!pack)
            continue;
        const GA_Offset primoff = pack->getMapOffset();
        const std::string &shape = mnames[mi];
        const std::string channel = shape;             // shape name only
        const std::string bsname = skin + "." + shape; // "<skin>.<shape>"
        if (name_h.isValid())
            name_h.set(primoff, skin_h);
        if (chan_h.isValid())
            chan_h.set(primoff, UT_StringHolder(channel.c_str(), (exint)channel.length()));
        if (bsname_h.isValid())
            bsname_h.set(primoff, UT_StringHolder(bsname.c_str(), (exint)bsname.length()));
        if (weight_h.isValid())
            weight_h.set(primoff, 0.0f);
        if (hidden)
            hidden->addOffset(primoff);
    }
}

// Output 4 (Morphs): vertex/UV morph deltas as a point cloud (one point per
// (morph, affected vertex): `morph_name`/`morph_index`/`morph_type`/`vtx`/`delta`/
// `uv_delta`). Non-deform morphs (group/bone/material/flip/impulse) and display
// frames are catalogued in detail dict arrays `pmx_morphs` / `pmx_display_frames`.
static void
buildMorphs(GU_Detail *dst, const pmx::Model &model, const sop_pmx::Xform &xf, bool sanitize)
{
    dst->clearAndDestroy();
    if (model.morphs.empty() && model.display_frames.empty())
        return;

    std::vector<std::string> mbases;
    mbases.reserve(model.morphs.size());
    for (const pmx::Morph &mo : model.morphs)
        mbases.push_back(sop_pmx::encodeName(!mo.name_local.empty() ? mo.name_local : mo.name_universal, sanitize));
    const std::vector<std::string> mnames = sop_pmx::deduplicate(std::move(mbases), "morph_");

    const int nv = (int)model.vertices.size();
    GA_RWHandleS pmname_h(dst->addStringTuple(GA_ATTRIB_POINT, "morph_name", 1));
    GA_RWHandleI pmidx_h(dst->addIntTuple(GA_ATTRIB_POINT, "morph_index", 1));
    GA_RWHandleI ptype_h(dst->addIntTuple(GA_ATTRIB_POINT, "morph_type", 1));
    GA_RWHandleI pvtx_h(dst->addIntTuple(GA_ATTRIB_POINT, "vtx", 1));
    GA_RWHandleV3 delta_h(dst->addFloatTuple(GA_ATTRIB_POINT, "delta", 3));
    GA_RWHandleV4 uvdelta_h(dst->addFloatTuple(GA_ATTRIB_POINT, "uv_delta", 4));

    for (size_t mi = 0; mi < model.morphs.size(); ++mi)
    {
        const pmx::Morph &mo = model.morphs[mi];
        const UT_StringHolder mname(mnames[mi].c_str(), (exint)mnames[mi].length());

        for (const pmx::VertexMorphOffset &vo : mo.vertex)
        {
            if (vo.vertex < 0 || vo.vertex >= nv)
                continue;
            const GA_Offset pt = dst->appendPoint();
            dst->setPos3(pt, xf.pos(model.vertices[(size_t)vo.vertex].position));
            pmname_h.set(pt, mname);
            pmidx_h.set(pt, (int)mi);
            ptype_h.set(pt, (int)mo.type);
            pvtx_h.set(pt, vo.vertex);
            delta_h.set(pt, xf.pos(vo.offset));   // position delta: Z flip + scale
        }
        for (const pmx::UVMorphOffset &uo : mo.uv)
        {
            if (uo.vertex < 0 || uo.vertex >= nv)
                continue;
            const GA_Offset pt = dst->appendPoint();
            dst->setPos3(pt, xf.pos(model.vertices[(size_t)uo.vertex].position));
            pmname_h.set(pt, mname);
            pmidx_h.set(pt, (int)mi);
            ptype_h.set(pt, (int)mo.type);
            pvtx_h.set(pt, uo.vertex);
            // UV V is flipped on import, so the V delta flips sign too.
            uvdelta_h.set(pt, UT_Vector4F(uo.offset.x,
                xf.convert ? -uo.offset.y : uo.offset.y, uo.offset.z, uo.offset.w));
        }
    }

    // Morph catalog (all morphs; deform morphs carry only metadata here).
    {
        UT_Array<UT_OptionsHolder> dicts;
        dicts.setCapacity((exint)model.morphs.size());
        for (size_t mi = 0; mi < model.morphs.size(); ++mi)
        {
            const pmx::Morph &mo = model.morphs[mi];
            UT_Options o;
            o.setOptionS("name", sop_pmx::utf8(mo.name_local));
            o.setOptionS("name_en", sop_pmx::utf8(mo.name_universal));
            o.setOptionI("panel", mo.panel);
            o.setOptionI("type", (int)mo.type);
            o.setOptionS("type_name", morphTypeName(mo.type));

            std::vector<int64> ia, ib;
            std::vector<fpreal32> fa, fb;
            switch (mo.type)
            {
            case pmx::MorphType::Group:
                for (const auto &g : mo.group) { ia.push_back(g.morph); fa.push_back(g.influence); }
                o.setOptionI("count", (int)mo.group.size());
                setIArray(o, "group_morphs", ia);
                setFArray(o, "group_influences", fa);
                break;
            case pmx::MorphType::Flip:
                for (const auto &f : mo.flip) { ia.push_back(f.morph); fa.push_back(f.influence); }
                o.setOptionI("count", (int)mo.flip.size());
                setIArray(o, "flip_morphs", ia);
                setFArray(o, "flip_influences", fa);
                break;
            case pmx::MorphType::Bone:
                for (const auto &b : mo.bone)
                {
                    ia.push_back(b.bone);
                    const UT_Vector3 t = xf.pos(b.translation);
                    fa.push_back(t.x()); fa.push_back(t.y()); fa.push_back(t.z());
                    // quaternion mirror about Z for handedness
                    fb.push_back(xf.convert ? -b.rotation.x : b.rotation.x);
                    fb.push_back(xf.convert ? -b.rotation.y : b.rotation.y);
                    fb.push_back(b.rotation.z);
                    fb.push_back(b.rotation.w);
                }
                o.setOptionI("count", (int)mo.bone.size());
                setIArray(o, "bone_bones", ia);
                setFArray(o, "bone_translations", fa);
                setFArray(o, "bone_rotations", fb);
                break;
            case pmx::MorphType::Material:
                for (const auto &m : mo.material) { ia.push_back(m.material); ib.push_back(m.operation); }
                o.setOptionI("count", (int)mo.material.size());
                setIArray(o, "material_targets", ia);
                setIArray(o, "material_ops", ib);
                break;
            case pmx::MorphType::Impulse:
                for (const auto &im : mo.impulse)
                {
                    ia.push_back(im.rigidbody); ib.push_back(im.local_flag);
                    fa.push_back(im.velocity.x); fa.push_back(im.velocity.y); fa.push_back(im.velocity.z);
                    fb.push_back(im.torque.x); fb.push_back(im.torque.y); fb.push_back(im.torque.z);
                }
                o.setOptionI("count", (int)mo.impulse.size());
                setIArray(o, "impulse_bodies", ia);
                setIArray(o, "impulse_local", ib);
                setFArray(o, "impulse_velocity", fa);
                setFArray(o, "impulse_torque", fb);
                break;
            default: // vertex / uv: deltas live in the point cloud
                o.setOptionI("count", (int)(mo.vertex.size() + mo.uv.size()));
                break;
            }
            dicts.append(UT_OptionsHolder(&o));
        }
        GA_RWHandleDictA h(dst->addDictArray(GA_ATTRIB_DETAIL, "pmx_morphs", 1));
        if (h.isValid())
            h.set(GA_Offset(0), dicts);
    }

    // Display frames (UI grouping of bones / morphs).
    if (!model.display_frames.empty())
    {
        UT_Array<UT_OptionsHolder> dicts;
        dicts.setCapacity((exint)model.display_frames.size());
        for (const pmx::DisplayFrame &df : model.display_frames)
        {
            UT_Options o;
            o.setOptionS("name", sop_pmx::utf8(df.name_local));
            o.setOptionS("name_en", sop_pmx::utf8(df.name_universal));
            o.setOptionB("special", df.special_flag != 0);
            std::vector<int64> types, indices;
            for (const pmx::DisplayElement &e : df.elements)
            {
                types.push_back(e.type);
                indices.push_back(e.index);
            }
            setIArray(o, "element_types", types);     // 0 bone, 1 morph
            setIArray(o, "element_indices", indices);
            dicts.append(UT_OptionsHolder(&o));
        }
        GA_RWHandleDictA h(dst->addDictArray(GA_ATTRIB_DETAIL, "pmx_display_frames", 1));
        if (h.isValid())
            h.set(GA_Offset(0), dicts);
    }
}

GU_DetailHandle
SOP_pmx_reader::cookMySopOutput(OP_Context &context, int outputidx, SOP_Node *interests)
{
    // Output 0 (Mesh) is produced by the cook verb.
    if (outputidx == 0)
        return SOP_Node::cookMySopOutput(context, outputidx, interests);

    GU_DetailHandle result;
    GU_Detail *dst = new GU_Detail();
    result.allocateAndSet(dst);

    const fpreal t = context.getTime();
    UT_String path;
    evalString(path, "file", 0, t);
    if (!path.length())
    {
        dst->bumpAllDataIds();
        return result;
    }

    const bool convert = evalInt("coordinate", 0, t) == 1;     // 0 original, 1 houdini
    const bool sanitize = evalInt("sanitizenames", 0, t) != 0;
    const float unit_meters = (float)evalFloat("unitmeters", 0, t);

    pmx::Model model;
    const pmx::LoadResult res = pmx::load(path.c_str(), model);
    if (!res.ok)
    {
        dst->bumpAllDataIds();
        return result;
    }

    const sop_pmx::Xform xf = makeXform(convert, unit_meters);

    switch (outputidx)
    {
    case 1:
        buildSkeleton(dst, model, xf, sanitize);
        break;
    case 2:
        buildRigidBodies(dst, model, xf, sanitize);
        break;
    case 3:
        buildJoints(dst, model, xf, sanitize);
        break;
    case 4:
        buildMorphs(dst, model, xf, sanitize);
        break;
    default:
        break;
    }

    dst->bumpAllDataIds();
    return result;
}

const char *
SOP_pmx_reader::outputLabel(OP_OutputIdx idx) const
{
    switch (idx)
    {
    case 0: return "Mesh";
    case 1: return "Skeleton";
    case 2: return "RigidBodies";
    case 3: return "Joints";
    case 4: return "Morphs";
    default: return SOP_Node::outputLabel(idx);
    }
}
