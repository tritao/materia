package machinekit.tooling;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import machinekit.tooling.SweptRemoval.RemovalReport;
import machinekit.tooling.SweptRemoval.RemovalMember;

class PanelKit {
    public static function assembly(plan:PanelAssemblyRecord,exploded:Bool=false,material:String="birch plywood"):MachineAssembly {
        PanelAssemblyPlan.validate(plan);
        var assembly=new MachineAssembly();
        var separation=0.0;
        if(exploded) {
            var low=new Vector(1e30,1e30,1e30),high=new Vector(-1e30,-1e30,-1e30);
            for(m in plan.members) {
                var b=PanelGeometry.bounds(PanelContours.outside(m.panel));
                for(x in [b.x,b.x+b.width])for(y in [b.y,b.y+b.height])for(z in [0.0,m.panel.stock.measuredThickness]) {
                    var p=F.transformPoint(m.pose,x,y,z);
                    low=new Vector(Math.min(low.x,p.x),Math.min(low.y,p.y),Math.min(low.z,p.z));
                    high=new Vector(Math.max(high.x,p.x),Math.max(high.y,p.y),Math.max(high.z,p.z));
                }
            }
            separation=high.subtract(low).length()+1;
        }
        for(i in 0...plan.members.length) {
            var m=plan.members[i],pose=F.compose(F.identity(),m.pose);
            if(exploded){pose.x+=m.withdrawal.x*(i+1)*separation;pose.y+=m.withdrawal.y*(i+1)*separation;pose.z+=m.withdrawal.z*(i+1)*separation;}
            assembly.addComponent(m.panel.id,ToolingPanel.fromPlan(m.panel,material),pose);
        }
        if(!exploded)for(j in plan.joints) {
            assembly.addMemberConnector(j.fixedId,j.id+"-fixed",j.fixedFrame);
            assembly.addMemberConnector(j.movingId,j.id+"-moving",j.movingFrame);
            assembly.addMate(j.id,"fixed",j.fixedId,j.id+"-fixed",j.movingId,j.id+"-moving");
        }
        return assembly;
    }
    public static function check(plan:PanelAssemblyRecord):RemovalReport {
        PanelAssemblyPlan.validate(plan);
        var parts:Array<Part> = [],members:Array<RemovalMember> = [];
        try {
            for(m in plan.members) {
                var local=ToolingPanel.fromPlan(m.panel).geometry(),world=PosedParts.moved(local,m.pose);local.close();parts.push(world);
                members.push({id:m.panel.id,part:world,direction:new Vector(m.withdrawal.x,m.withdrawal.y,m.withdrawal.z)});
            }
            var report=SweptRemoval.analyze(members);
            for(p in parts)p.close();return report;
        }catch(e:Dynamic){for(p in parts)p.close();throw e;}
    }

    /** Borrowed free-approach tool envelopes; contact/engagement belongs outside them. */
    public static function checkToolAccess(plan:PanelAssemblyRecord,accesses:Array<{id:String,clearance:Part}>,contactVolumeToleranceMm3:Float=1e-5):Void {
        PanelAssemblyPlan.validate(plan);
        if(accesses==null || accesses.length>128 || !Math.isFinite(contactVolumeToleranceMm3) || contactVolumeToleranceMm3<0)throw "Invalid tool access records";
        for(m in plan.members) {
            var local=ToolingPanel.fromPlan(m.panel).geometry(),world=PosedParts.moved(local,m.pose);local.close();
            try {
                for(access in accesses) {
                    var common=world.intersect(access.clearance),volume=common.volume();common.close();
                    if(volume>contactVolumeToleranceMm3)throw 'Blocked tool access ${access.id}/${m.panel.id}';
                }
                world.close();
            }catch(e:Dynamic){world.close();throw e;}
        }
    }
}
