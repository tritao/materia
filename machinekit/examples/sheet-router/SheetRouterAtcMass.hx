import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.PosedParts;
import sys.io.File;
class SheetRouterAtcMass {
    public static function main():Void {
        var payload:Dynamic=haxe.Json.parse(File.getContent("machinekit/examples/sheet-router/atc/tool-table.json"));
        for(bed in [2600.0,3150.0]) {
            var machine=new SheetRouter(new SheetRouterSpec(bed)),tools=SheetRouterAtc.decodeTools(payload.tools);
            var fixture=new SheetRouterAtcAssembly(machine,tools),state=new AssemblyState(fixture.definition);
            state.setJoint("x",800);state.setJoint("y",bed/2);state.setJoint("z",200);state.setJoint("rackCoverLift",600);
            var atc=new SheetRouterAtc(machine,tools,1,json->{});fixture.place(state,atc);
            var rows:Array<Dynamic> = [],parents=machine.mateParents();
            for(member in machine.components()) {
                var id=member.id;
                if(id!="rackSupport"&&id!="setterPad"&&id!="setter"&&id!="chainYFixedBracket"&&id!="toolRack"&&id!="rackCover"&&!StringTools.startsWith(id,"fork")&&!StringTools.startsWith(id,"rackCoverLift")&&!StringTools.startsWith(id,"atc"))continue;
                var component=member.component,pose=state.worldPose(id),mass=component.massProperties(),c=mass.centreOfMass;
                var cache=new PosedParts();var part=cache.posed(component,pose);
                try {
                    var physical=part.massProperties(),inertia=mass.inertia==null?physical.inertiaAtDensity(mass.mass/physical.volume):mass.inertia.rotated(pose.qx,pose.qy,pose.qz,pose.qw);
                    rows.push({id:id,parentId:parents.get(id),materialId:component.materialId,representation:"mass-only",poseMm:pose,massKg:mass.mass,
                        massSource:"CAD/declared",centreOfMassMm:F.transformPoint(pose,c.x,c.y,c.z),volumeMm3:physical.volume,
                        centroidalInertiaKgMm2World:inertia,inertiaSource:"CAD distribution scaled to component mass",boundsMm:PosedParts.boxOf(part),stepFile:null,stepSha256:null});
                    part.close();cache.close();
                }catch(error:Dynamic) {part.close();cache.close();throw error;}
            }
            File.saveContent('build/sheet-router-work/sr7/mass-y${Std.int(bed)}.json',haxe.Json.stringify({workY:bed,parts:rows,guideAllowanceKg:3},null,"  ")+"\n");
            fixture.close();Sys.println('SR7 mass Y=$bed: '+rows.length+' parts');
        }
    }
}
