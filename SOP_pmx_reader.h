#ifndef __SOP_pmx_reader_h__
#define __SOP_pmx_reader_h__

#include <SOP/SOP_Node.h>
#include <UT/UT_StringHolder.h>

// SOP that imports a MikuMikuDance PMX model. Output 0 (Mesh) is produced by the
// cook verb; outputs 1..4 (Skeleton / RigidBodies / Joints / Morphs) are built
// in cookMySopOutput.
class SOP_pmx_reader : public SOP_Node
{
public:
    static PRM_Template *buildTemplates();
    static OP_Node *myConstructor(OP_Network *net, const char *name, OP_Operator *op)
    {
        return new SOP_pmx_reader(net, name, op);
    }

    static const UT_StringHolder theSOPTypeName;

    const SOP_NodeVerb *cookVerb() const override;

    GU_DetailHandle cookMySopOutput(OP_Context &context, int outputidx,
        SOP_Node *interests) override;
    const char *outputLabel(OP_OutputIdx idx) const override;

protected:
    SOP_pmx_reader(OP_Network *net, const char *name, OP_Operator *op)
        : SOP_Node(net, name, op)
    {
        mySopFlags.setManagesDataIDs(true);
    }

    ~SOP_pmx_reader() override {}

    OP_ERROR cookMySop(OP_Context &context) override
    {
        return cookMyselfAsVerb(context);
    }
};

#endif
