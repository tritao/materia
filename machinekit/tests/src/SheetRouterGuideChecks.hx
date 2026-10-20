import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;
import machinekit.motion.LinearRail;
import machinekit.transmission.HelicalPinion;
import machinekit.component.ComponentValues;

/** Mechanical guide alignment, moving-rail reach and revised drive sizing. */
class SheetRouterGuideChecks {
    static function check(ok:Bool,label:String):Void {if(!ok)throw "SR6: "+label;}
    public static function main():Void {
        var alternative=new HelicalPinion(new ComponentValues().setNumber("normalModule",2).setInteger("teeth",15));
        check(Math.abs(Math.PI*alternative.pitchDiameter()-100)<1e-7,"module 2 / 15 teeth preserves 100 mm/rev without undercut");
        for(length in [2600.0,3150.0]) {
            var machine=new SheetRouter(new SheetRouterSpec(length)),state=new AssemblyState(machine.definition());
            check(machine.routerSpec.gantry().beamFootWelded,"welded beam feet selected");
            var zPlate:machinekit.gantry.GantryParts.GantryStiffenedPlate=cast machine.component("zCarriage");
            var solid=zPlate.geometry();
            var expected=zPlate.width*zPlate.depth*zPlate.height+2*zPlate.depth*(zPlate.ribWidth*zPlate.ribDepth-(zPlate.ribWidth-2*zPlate.ribWall)*(zPlate.ribDepth-2*zPlate.ribWall));
            check(solid.valid()&&Math.abs(solid.volume()/expected-1)<1e-7,"closed Z side sections have valid geometry and volume");solid.close();
            var braces=0;
            for(member in machine.base.frame.analysisMembers())if(member.name.indexOf("Brace")>=0)braces++;
            check(braces==16,"sixteen frame braces selected");
            var fixed=state.worldPose("blockZ"),rail=state.worldPose("railZ");
            state.setJoint("z",machine.routerSpec.travelZ);state.forwardKinematics();
            check(Math.abs(state.worldPose("blockZ").z-fixed.z)<1e-6,"Z blocks remain on X carriage");
            check(Math.abs(state.worldPose("railZ").z-rail.z+machine.routerSpec.travelZ)<1e-6,"Z rails move with plate");
            for(x in [0.0,800.0,1600.0])for(q in [0.0,machine.routerSpec.travelZ/2,machine.routerSpec.travelZ]) {
                state.setJoint("x",x);state.setJoint("z",q);state.forwardKinematics();
                for(id in ["blockZ","blockZ01","blockZ10","blockZ11"]) {
                    var railId=id=="blockZ"||id=="blockZ01"?"railZ":"railZRight";
                    var railPart:LinearRail=cast machine.component(railId);
                    var at=F.compose(F.inverse(state.worldPose(railId)),state.worldConnector(id,"rail"));
                    check(Math.abs(at.x)+Math.abs(at.y)<1e-6,"fixed Z block remains aligned with moving rail");
                    var reach=railPart.spec.railEndMargin+railPart.spec.blockLength/2;
                    check(at.z>=reach-1e-6&&at.z<=railPart.length-reach+1e-6,"moving Z rail covers each block throughout stroke");
                }
            }
            machine.check().throwIfErrors();
            for(axis in ["x","y","z"]) {
                var row=SheetRouterSizing.report(machine,axis);
                var errors:Array<String> = row.failures;
                check(errors.length==0,axis+" selected drive passes: "+errors.join(","));
                Sys.println("SR6 Y="+length+" "+axis+" mass="+row.movingMassKg+" RMS="+row.demand.rmsNm);
            }
        }
        Sys.println("SR6 guide layout, moving-rail travel and selected drive checks passed");
    }
}
