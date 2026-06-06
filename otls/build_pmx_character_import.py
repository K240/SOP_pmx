# -*- coding: utf-8 -*-
# Regenerates otls/pmx_character_import.hda.
#
# The HDA wraps the C++ `pmx_import` SOP and routes each of its 5 outputs through
# a null -> output node. That makes every output a normal, gdp-backed SOP output,
# which the Cache SOP (used internally by KineFX Rig Pose) accepts -- the raw
# cookMySopOutput outputs of the C++ node are dropped by the Cache SOP, so the
# Skeleton output otherwise needs a manual Null before Rig Pose.
#
# Requires the pmx_import DSO to be installed. Run with hython:
#   hython otls/build_pmx_character_import.py
import hou, os

HDANAME  = "pmx_character_import"
HDALABEL = "PMX Character Import"
HDAFILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "pmx_character_import.hda")
LABELS   = ["Mesh", "Skeleton", "RigidBodies", "Joints", "Morphs"]
PARMS    = ["reimport", "file", "coordinate", "unitmeters",
            "sanitizenames", "skinning", "blendshapes"]
STRINGP  = {"file"}

if "pmx_import" not in [t for t in hou.sopNodeTypeCategory().nodeTypes()]:
    raise SystemExit("pmx_import DSO not installed; build/install it first.")

try:
    hou.hda.uninstallFile(HDAFILE)
except Exception:
    pass
if os.path.exists(HDAFILE):
    os.remove(HDAFILE)

geo = hou.node("/obj").createNode("geo", "hdabuild")
sub = geo.createNode("subnet", HDANAME)
inner = sub.createNode("pmx_import", "import")

# inner:i -> null -> output(outputidx=i)
for i in range(5):
    nl = sub.createNode("null", "OUT_%s" % LABELS[i])
    nl.setInput(0, inner, i)
    op = sub.createNode("output", "output%d_%s" % (i, LABELS[i]))
    op.parm("outputidx").set(i)
    op.setInput(0, nl, 0)

# Clean promoted interface; reimport forwards to the inner node.
src = inner.parmTemplateGroup()
ptg = hou.ParmTemplateGroup()
for name in PARMS:
    t = src.find(name)
    if t is None:
        print("  WARN: missing parm template", name); continue
    if name == "reimport":
        t.setScriptCallback("kwargs['node'].node('import').parm('reimport').pressButton()")
        t.setScriptCallbackLanguage(hou.scriptLanguage.Python)
    ptg.append(t)
sub.setParmTemplateGroup(ptg)

# Link inner parms to the promoted HDA parms.
for name in PARMS:
    if name == "reimport":
        continue
    p = inner.parm(name)
    if p is not None:
        fn = "chs" if name in STRINGP else "ch"
        p.setExpression('%s("../%s")' % (fn, name), hou.exprLanguage.Hscript)

sub.layoutChildren()

asset = sub.createDigitalAsset(
    name=HDANAME, hda_file_name=HDAFILE, description=HDALABEL,
    min_num_inputs=0, max_num_inputs=0, ignore_external_references=True)
defn = asset.type().definition()
defn.setParmTemplateGroup(ptg)      # clean interface on the definition
defn.save(HDAFILE)

print("Wrote", HDAFILE)
print("Outputs (in order):", ", ".join("%d=%s" % (i, l) for i, l in enumerate(LABELS)))
