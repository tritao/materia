import machinekit.assembly.MachineAssembly;
import machinekit.assembly.RbkMotorMount;
import machinekit.motion.*;
import machinekit.component.*;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

class RbkSfuAxisChecks {
    static function check(ok:Bool,message:String):Void {if(!ok) throw message;}
    static function pose(part:Part,f:AssemblyFrame):Part {
        var x=F.transformVector(f,1,0,0),z=F.transformVector(f,0,0,1);
        return part.placed(new Location(new Plane(new Vector(f.x,f.y,f.z),new Vector(x.x,x.y,x.z),new Vector(z.x,z.y,z.z))));
    }
    static function overlap(a:Part,b:Part):Float {
        var common=a.shape.common(b.shape),v=common.subshapeCount(CadKit.ShapeKind.Solid)==0 ? 0.0 : common.volume();common.close();return v;
    }
    static function main():Void {
        var root=Sys.getEnv("MATERIA_ROOT");if(root==null) root=Sys.getCwd();var dir=root+"/build/suppliers/vallder/";
        RbkMotorMount.setSupplierDirectory(dir);
        var axis=new RbkSfuAxis(),registry=RbkSfuAxis.registry();
        check(new BallNut().overallLength==45,"Legacy generic nut dimensions retained");
        check(axis.nut.overallLength==46 && axis.nut.flangeOffset==10 && axis.nut.flangeThickness==10,
            "Supplier nut uses selected SKU drawing");
        var holes=axis.nut.boltPattern();
        check(holes.length==6 && Math.abs(holes[0].x-19/Math.sqrt(2))<0.000001 && Math.abs(holes[1].x)<0.000001,
            "Supplier nut uses drawing hole angles");
        axis.validate();
        trace("Axis assembled and validated");
        var saved=axis.encode(),loaded=MachineAssembly.decode(saved,registry);
        var before=axis.billOfMaterials().lines(),after=loaded.billOfMaterials().lines();
        before.sort((a,b)->Reflect.compare(a.partNumber,b.partNumber));after.sort((a,b)->Reflect.compare(a.partNumber,b.partNumber));
        check(haxe.Json.stringify(before)==haxe.Json.stringify(after),"Complete axis BOM survives reload");
        var state=new AssemblyState(loaded.definition());
        var prototypes:Map<String,Part> = [],entries=loaded.components();
        for(entry in entries) {
            var key=entry.component.geometryKey();
            if(!prototypes.exists(key)) {
                var p=entry.component.geometry();check(p.valid(),"Valid component geometry "+entry.id);prototypes.set(key,p);
            }
        }
        trace("All component solids generated");
        var motor=NemaStepper.model("OPENBUILDS-NEMA23-56"),motorPart=motor.geometry();
        for(point in motor.boltPattern()) {
            var driver=Part.cylinderSpan(2.5,-motor.bodyLength-10,-5.6,point.x,point.y);
            check(overlap(motorPart,driver)<0.001,"Rear axial driver clears motor housing");driver.close();
        }
        motorPart.close();
        var comparisons:Array<Array<String>> = [["rbk","screw"],["rbk","coupling"],["rbk","base"],["bf12","screw"],["bf12","base"],
            ["coupling","motor"],["coupling","screw"],["ballNut","screw"],["ballNut","bracket"],["bracket","base"],
            ["bracket","riserLeft"],["bracket","riserRight"],["carriage","rbk"],["carriage","bf12"],["blockLeft","railLeft"],["blockRight","railRight"]];
        for(h in axis.hardware) if(h.interfacePair=="motor/rbk") {
            comparisons.push(["motor",h.id]);comparisons.push(["motor",h.id+"-washer"]);
        }
        var results:Array<Dynamic> = [],failures=0;
        for(travel in [0.0,100,200]) {
            axis.setTravel(state,travel);var poses=loaded.solvedPoses(state),world:Map<String,Part> = [];
            var bracketPose=poses.get("bracket");
            check(bracketPose!=null && Math.abs(bracketPose.z-140-travel)<0.000001,"Carriage follows commanded travel");
            check(Math.abs(state.joint("screwTurn")+travel*2*Math.PI/5)<0.000001,"5 mm lead drives screw rotation");
            for(entry in entries) world.set(entry.id,pose(prototypes.get(entry.component.geometryKey()),poses.get(entry.id)));
            for(pair in comparisons) {
                var v=overlap(world.get(pair[0]),world.get(pair[1]));
                results.push({travel_mm:travel,a:pair[0],b:pair[1],intersection_mm3:v});
                if(v>0.001) failures++;
            }
            if(travel==0 && Sys.args().indexOf("--export-step")>=0) {
                var shapes:Array<Shape> = [];for(p in world) shapes.push(p.shape);
                var compound=Shape.compound(shapes);compound.exportStep(dir+"rbk-sfu1605-axis.step");compound.close();
            }
            for(p in world) p.close();
            trace('Travel checked: $travel mm; accumulated clashes: $failures');
        }
        var engagements:Array<Dynamic> = [];
        for(h in axis.hardware) {
            var clamp=StringTools.startsWith(h.interfacePair,"coupling/");
            var valid=h.through ? h.engagement>=h.nutStack : (clamp ? h.engagement>0 && h.engagement<1 :
                h.engagement>=1.3*Std.parseFloat(h.size.substr(1)) && h.engagement<=h.availableDepth);
            var status=valid ? "nominal_dimensions_pass" : "review_required";
            engagements.push({id:h.id,size:h.size,length_mm:h.length,grip_mm:h.grip,engagement_mm:h.engagement,
                available_depth_mm:h.through ? null : h.availableDepth,nut_stack_mm:h.nutStack,status:status,interfacePair:h.interfacePair});
            check(status=="nominal_dimensions_pass","Nominal screw engagement "+h.id);
        }
        sys.io.File.saveContent(dir+"rbk-sfu1605-axis.assembly.json",saved);
        sys.io.File.saveContent(dir+"rbk-sfu1605-axis.bom.json",haxe.Json.stringify(before,null,"  "));
        sys.io.File.saveContent(dir+"rbk-sfu1605-axis-checks.json",haxe.Json.stringify({schema_version:1,compatibility_basis:"supplier_intended",
            custom_end_machining:false,clearance_checks:results,geometry_clashes:failures,fasteners:engagements,
            reference_discrepancies:["RBK CAD outer sleeve/bearing stack spans 25.5 mm versus standard 25 mm plain journal; screw unchanged",
                "SFU1605 nut SKU1035001 uses 46 mm drawing length; 42 mm calculator value not used"],
            limits:["Nominal CAD clearance checks do not establish bearing preload or shaft fit tolerances",
                "Motor is the bd_warehouse OpenBuilds NEMA23 reference, not the StepperOnline or Leadshine product",
                "M4 rail-block depth 6 mm and riser thread depth 12 mm are explicit reference design assumptions",
                "Small screw end fillets/chamfers and grease fitting are not modeled"]},null,"  "));
        for(p in prototypes) p.close();
        check(failures==0,"Axis model clashes require review; see rbk-sfu1605-axis-checks.json");
        trace("Drawing-based SFU1605 axis geometry, travel, engagement and persistence checks passed");
    }
}
