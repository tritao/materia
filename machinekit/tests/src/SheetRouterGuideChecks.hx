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
            var ram=zPlate.geometry(),posedRam=machinekit.assembly.PosedParts.moved(ram,state.worldPose("zCarriage"));ram.close();
            var gap=posedRam.shape.bounds().get_min().get_z()-machine.boardTop;posedRam.close();
            var tip=state.worldConnector("longestTool","tip"),beam=state.worldPose("beam");
            check(Math.abs(gap-149)<1e-6,"149 mm ram clearance retained after datum recovery");
            check(Math.abs(beam.z-tip.z-367.75)<1e-6,"60 mm spindle drop recovered: 367.75 mm beam-to-tip lever");
            var entryY=machine.zeroPose("frameFront").y-200,exitY=machine.workOrigin.y;
            var blank=cadkit.modeling.Part.box(machine.routerSpec.workX,exitY-entryY,machine.routerSpec.sheetThickness);
            var loading=blank.translated(new cadkit.modeling.Vector(machine.routerSpec.workX/2,(entryY+exitY)/2,machine.boardTop));blank.close();
            for(id in ["base","frameFront","frameLeft","frameRight","railPadYLeft","railPadYRight"]) {
                var body=machine.component(id).geometry(),placed=machinekit.assembly.PosedParts.moved(body,machine.zeroPose(id));body.close();
                var common=placed.intersect(loading),volume=common.volume();common.close();placed.close();
                check(volume<0.05,"full-width 40 mm sheet front-loading clearance: "+id+" / "+volume+" mm3");
            }
            loading.close();
            Sys.println("SR6 Y="+length+" ram clearance="+gap+" mm; beam-to-tip="+(beam.z-tip.z)+" mm; front-loading sweep passed");
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
