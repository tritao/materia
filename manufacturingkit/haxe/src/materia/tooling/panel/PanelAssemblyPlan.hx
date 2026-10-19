package materia.tooling.panel;

import materia.assembly.AssemblyCodec;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.panel.PanelJoints.PanelJoint;
import materia.tooling.panel.PanelJoints.PanelJointPair;

typedef PanelPlacement = {var panel:PanelPlan;var pose:AssemblyFrame;var withdrawal:{x:Float,y:Float,z:Float};}
typedef PanelAssemblyRecord = {var id:String;var members:Array<PanelPlacement>;var joints:Array<PanelJoint>;}
/** A geometric observation supplied by machinekit.tooling, never inferred from datums. */
typedef PanelBlockingRelation = {var movingId:String;var blockerId:String;}

/** Neutral reference checks and graph ordering. All blocking relations come from 3D sweeps. */
class PanelAssemblyPlan {
    public static function pair(pair:PanelJointPair,?pose:AssemblyFrame):PanelAssemblyRecord {
        var base=pose==null?F.identity():F.compose(F.identity(),pose),direction=F.transformVector(base,0,0,1);
        return {id:pair.joint.id,joints:[pair.joint],members:[
            {panel:pair.fixed,pose:base,withdrawal:{x:direction.x,y:direction.y,z:direction.z}},
            {panel:pair.moving,pose:PanelJoints.movingPose(pair.joint,base),withdrawal:{x:direction.x,y:direction.y,z:direction.z}}]};
    }
    public static function validate(plan:PanelAssemblyRecord):Void {
        PanelCodec.identifier(plan.id);
        if(plan.members==null || plan.members.length==0 || plan.members.length>128 || plan.joints==null || plan.joints.length>256)throw "Invalid panel assembly count";
        var members=new Map<String,PanelPlacement>();
        for(m in plan.members) {
            PanelCodec.snapshot(m.panel);frame(m.pose);
            if(members.exists(m.panel.id))throw "Duplicate panel occurrence";
            var v=m.withdrawal;
            if(v==null || !Math.isFinite(v.x) || !Math.isFinite(v.y) || !Math.isFinite(v.z) || Math.abs(v.x*v.x+v.y*v.y+v.z*v.z-1)>1e-8)throw "Invalid withdrawal vector";
            members.set(m.panel.id,m);
        }
        var ids=new Map<String,Bool>();
        for(j in plan.joints) {
            PanelCodec.identifier(j.id);
            var a=members.get(j.fixedId),b=members.get(j.movingId);
            if(a==null || b==null || a==b || ids.exists(j.id))throw "Invalid joint references";
            ids.set(j.id,true);frame(j.fixedFrame);frame(j.movingFrame);
            if(!Math.isFinite(j.clearance) || j.clearance<0)throw "Invalid panel joint clearance";
            var expected=PanelJoints.movingPose(j,a.pose),delta=F.compose(F.inverse(expected),b.pose);
            if(Math.abs(delta.x)+Math.abs(delta.y)+Math.abs(delta.z)>1e-6 ||
                Math.abs(delta.qx)+Math.abs(delta.qy)+Math.abs(delta.qz)>1e-8)throw 'Misaligned or reversed joint ${j.id}';
        }
    }
    /** A -> B means B must be removed before A. Deterministic Kahn ordering. */
    public static function removalOrder(plan:PanelAssemblyRecord,blocking:Array<PanelBlockingRelation>):Array<String> {
        validate(plan);
        return topological([for(m in plan.members)m.panel.id],blocking);
    }
    public static function topological(sourceIds:Array<String>,blocking:Array<PanelBlockingRelation>):Array<String> {
        var ids=sourceIds.copy();ids.sort(Reflect.compare);
        if(blocking==null || blocking.length>ids.length*ids.length)throw "Invalid blocking graph";
        for(e in blocking)if(ids.indexOf(e.movingId)<0 || ids.indexOf(e.blockerId)<0)throw "Unknown blocking graph member";
        var done=new Map<String,Bool>(),order:Array<String> = [];
        while(order.length<ids.length) {
            var progress=false;
            for(id in ids)if(!done.exists(id)) {
                var ready=true;for(e in blocking)if(e.movingId==id && !done.exists(e.blockerId))ready=false;
                if(ready){done.set(id,true);order.push(id);progress=true;}
            }
            if(!progress)throw "Cyclic panel blocking graph: "+[for(id in ids)if(!done.exists(id))id].join(", ");
        }
        return order;
    }
    public static function assemblyOrder(plan:PanelAssemblyRecord,blocking:Array<PanelBlockingRelation>):Array<String> {
        var order=removalOrder(plan,blocking);order.reverse();return order;
    }
    static function frame(p:AssemblyFrame):Void {
        AssemblyCodec.validateFrame(p);
        if(Math.abs(p.qx*p.qx+p.qy*p.qy+p.qz*p.qz+p.qw*p.qw-1)>1e-8)throw "Panel pose needs a unit quaternion";
    }
}
