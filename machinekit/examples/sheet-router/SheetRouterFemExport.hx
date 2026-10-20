import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;
import machinekit.structural.RectTube;
import machinekit.structural.RectTubeMember;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.structural.fem.AssemblyFemExport;
import materia.project.MaterialLibrary;
import sys.FileSystem;
import sys.io.File;

/** Geometry-backed inputs for the explicitly reduced SR5 structural screen. */
class SheetRouterFemExport {
    static function modulus(value:Null<Float>,spec:String):Float {
        if(value!=null&&value>0)return (cast value:Float)/1000000.0;
        if(spec.toLowerCase().indexOf("steel")>=0)return 210000.0;
        if(spec.toLowerCase().indexOf("aluminium")>=0)return 69000.0;
        throw "Missing FEM modulus: "+spec;
    }
    static function vector(v:Dynamic):Array<Float> return [v.x,v.y,v.z];
    public static function packet(machine:SheetRouter,folder:String):Dynamic {
        if(!FileSystem.exists(folder))FileSystem.createDirectory(folder);
        var path=folder+"/manifest.json";
        if(FileSystem.exists(path))FileSystem.deleteFile(path);
        var state=new AssemblyState(machine.definition());
        var spec=machine.routerSpec;
        state.setJoint("x",spec.workX/2);state.setJoint("y",spec.workY/2);state.setJoint("z",spec.travelZ);state.forwardKinematics();
        var selected=["beam","beamFootLeft","beamFootRight","xCarriage","zColumn","zCarriage","spindleMount"];
        var exported=AssemblyFemExport.bodies(machine,state,folder,id->selected.indexOf(id)>=0);
        var beams:Array<Dynamic> = [],plates:Array<Dynamic> = [];
        for(member in machine.components()) {
            var pose=state.worldPose(member.id),material=MaterialLibrary.require(member.component.materialId);
            if(Std.isOfType(member.component,RectTubeMember)) {
                var tube:RectTubeMember=cast member.component;
                beams.push({id:member.id,startMm:vector(F.transformPoint(pose,0,0,0)),
                    endMm:vector(F.transformPoint(pose,0,0,tube.length)),
                    sectionX:vector(F.transformVector(pose,1,0,0)),widthMm:tube.profile.width,heightMm:tube.profile.height,
                    wallMm:tube.profile.wall,youngsMPa:modulus(material.physical.youngsModulusPa,member.component.materialSpec()),densityKgPerMm3:material.physical.density/1e9});
            }
            if(selected.indexOf(member.id)>=0&&Std.isOfType(member.component,GantryPlate)) {
                var plate:GantryPlate=cast member.component;
                plates.push({id:member.id,widthMm:plate.width,depthMm:plate.depth,thicknessMm:plate.height,
                    poseMm:pose,youngsMPa:modulus(material.physical.youngsModulusPa,member.component.materialSpec())});
            }
        }
        for(member in machine.base.frame.analysisMembers()) {
            if(!Std.isOfType(member.profile,RectTube))throw "Router FEM expects RHS base stock";
            var profile:RectTube=cast member.profile;
            var direction=member.end.subtract(member.start).normalized(),side=member.reference.cross(direction).normalized();
            beams.push({id:"base/"+member.name,startMm:vector(member.start),endMm:vector(member.end),sectionX:vector(side),
                widthMm:profile.width,heightMm:profile.height,wallMm:profile.wall,youngsMPa:210000.0,densityKgPerMm3:7850.0/1e9});
        }
        var poses:Array<Dynamic> = [];
        for(x in [0.0,spec.workX/2,spec.workX])for(y in [0.0,spec.workY/2,spec.workY]) {
            state.setJoint("x",x);state.setJoint("y",y);state.forwardKinematics();
            poses.push({x:x,y:y,z:spec.travelZ,toolTipMm:vector(state.worldConnector("longestTool","tip")),
                blockXMm:vector(state.worldPose("blockX")),blockZMm:vector(state.worldPose("blockZ"))});
        }
        var supports:Array<Dynamic> = [];
        for(side in 0...2)for(end in 0...3) {
            var id='foot${side}${end}';supports.push({id:id,pointMm:vector(state.worldPose(id))});
        }
        var axes:Array<Dynamic> = [];
        for(axis in ["x","y","z"])axes.push(SheetRouterSizing.report(machine,axis));
        var result={schemaVersion:1,study:"sheet-router-sr5",status:"reduced structural screening; assumed interfaces",
            units:{length:"mm",force:"N",mass:"kg",youngsModulus:"MPa"},workY:spec.workY,parts:exported.parts,
            materials:exported.materials,totalMassKg:exported.totalMassKg,selectedBodies:selected,beams:beams,plates:plates,
            poses:poses,supports:supports,axes:axes,loadCases:{cuttingPerAxisN:300,accelerationMPerS2:3,gravityMPerS2:9.80665},
            interfaces:{guideTranslationNPerMm:100000.0,guideRotationNmmPerRad:10000000.0,
                boltTranslationNPerMm:1000000.0,boltRotationNmmPerRad:1000000000.0,footVerticalNPerMm:50000.0,
                note:"Assumed individual block rotation and joint stiffness; not supplier data. Y pitch/yaw also use 140 mm block separation. Rack stiffness from SR4."},
            limitations:["Missing catalogue modulus uses assumed steel 210 GPa or aluminium 69 GPa; all Poisson ratios assumed 0.3",
                "Beam and plate strip idealisations exclude wall distortion, holes, weld/bolt stress and contact separation",
                "Spindle mount, tool, spindle bearings, vacuum bed and workpiece are rigid in the reduced load loop",
                "No damping, servo control, friction or pretension geometric stiffness; first modes are condensed estimates"]};
        File.saveContent(path,haxe.Json.stringify(result,null,"  ")+"\n");return result;
    }
    public static function main():Void {
        var root=Sys.getEnv("SHEET_ROUTER_FEM_OUTPUT");if(root==null)root="build/sheet-router-sr5";
        if(!FileSystem.exists(root))FileSystem.createDirectory(root);
        for(length in [2600.0,3150.0]) {
            var result=packet(new SheetRouter(new SheetRouterSpec(length)),root+"/y"+Std.int(length));
            var parts:Array<Dynamic> = result.parts,beams:Array<Dynamic> = result.beams;
            Sys.println("SR5 packet: Y="+length+"; "+parts.length+" components; "+beams.length+" stock members");
        }
    }
}
