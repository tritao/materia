import machinekit.assembly.RbkMotorMount;
import machinekit.assembly.MachineAssembly;
import machinekit.motion.NemaStepper;
import machinekit.component.MachineKitComponents;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import materia.assembly.AssemblyFrames;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;

class RbkMountChecks {
    static function check(ok:Bool,message:String):Void {if(!ok) throw message;}
    static function overlap(shape:Part,probe:Part):Float {
        var common=shape.shape.common(probe.shape);
        var volume=common.subshapeCount(CadKit.ShapeKind.Solid)==0 ? 0.0 : common.volume();
        common.close();probe.close();return volume;
    }
    static function vertical(radius:Float,x:Float,z:Float,height:Float):Part {
        var raw=Part.cylinderSpan(radius,-0.1,height);
        var placed=raw.placed(new Location(new Plane(new Vector(x,0,z),Vector.X(),Vector.Y())));raw.close();return placed;
    }
    static function main():Void {
        var root=Sys.getEnv("MATERIA_ROOT");if(root==null) root=Sys.getCwd();
        var dir=root+"/build/suppliers/vallder/";
        RbkMotorMount.setSupplierDirectory(dir);
        var registry=MachineKitComponents.defaultRegistry();
        var persistenceOnly=Sys.args().indexOf("--persistence-only")>=0;
        check(registry.has(RbkMotorMount.recipeType().id),"RBK recipe is registered");
        var report:Array<Dynamic> = [],assembly=new MachineAssembly(),failures=0,index=0;
        for(model in RbkMotorMount.models()) {
            var mount=new RbkMotorMount(model),meta:Dynamic=mount.interfaceMetadata();
            if(persistenceOnly) {
                var normalized=mount.geometry();
                check(shaftAxisMatches(normalized,meta),"Shaft datum matches the CAD bearing bore axis");normalized.close();
                assembly.addComponent("mount-"+model,mount,AssemblyFrames.translation(index*180,0,0));
                assembly.addComponent("motor-"+model,NemaStepper.frame(StringTools.endsWith(model,"57") ? 23 : 34));
                assembly.addMate("motor-mate-"+model,"fixed","mount-"+model,"motor","motor-"+model,"mountFace");
                index++;continue;
            }
            var shape=mount.geometry();
            check(shaftAxisMatches(shape,meta),"Shaft datum matches the CAD bearing bore axis");
            var bounds=shape.shape.bounds();
            check(shape.valid(),"Normalized RBK topology is valid");
            var motorFace=false,baseFace=false;
            var verticalCircles:Array<Dynamic> = [];
            for(i in 0...shape.shape.subshapeCount(CadKit.ShapeKind.Edge)) {
                var edge=shape.shape.subshape(CadKit.ShapeKind.Edge,i);
                if(edge.curveKind()==CadKit.CurveKind.Circle) {
                    var axis=edge.edgeAxis(),origin=axis.get_origin();
                    if(Math.abs(axis.get_direction().get_y())>0.999)
                        verticalCircles.push({x:origin.get_x(),y:origin.get_y(),z:origin.get_z(),radius:axis.get_radius()});
                }
                edge.close();
            }
            for(i in 0...shape.shape.subshapeCount(CadKit.ShapeKind.Face)) {
                var face=shape.shape.subshape(CadKit.ShapeKind.Face,i);
                if(face.surfaceKind()==CadKit.SurfaceKind.Plane && face.faceArea()>500) {
                    var centre=face.center(),normal=face.faceNormal();
                    if(Math.abs(centre.get_z())<0.001 && normal.get_z()< -0.999) motorFace=true;
                    if(Math.abs(centre.get_y())<0.001 && normal.get_y()< -0.999) baseFace=true;
                }
                face.close();
            }
            check(motorFace && baseFace,"Connector datum planes match CAD faces");
            check(Math.abs(bounds.get_min().get_y()+mount.underhang)<0.001,"Flange underhang is explicit");
            var holes:Array<Dynamic> = meta.base_holes;
            for(i in 0...holes.length) {
                var hole=holes[i],volume=overlap(shape,vertical(hole.diameter/2-0.15,hole.x,hole.z,bounds.get_max().get_y()+1));
                check(circleAt(verticalCircles,hole.x,hole.z,hole.diameter/2),"Base datum matches an actual CAD bore edge");
                if(hole.counterbore_diameter!=null)
                    check(circleAt(verticalCircles,hole.x,hole.z,hole.counterbore_diameter/2),"Counterbore diameter matches CAD");
                report.push({model:model,feature:"baseBolt"+(i+1),intersection_mm3:volume,x:hole.x,z:hole.z});
                if(volume>0.0001) failures++;
            }
            var extra:Array<Dynamic> = meta.auxiliary_holes;
            for(i in 0...extra.length) {
                var h=extra[i],volume=overlap(shape,vertical(h.diameter/2-0.15,h.x,h.z,bounds.get_max().get_y()+1));
                check(circleAt(verticalCircles,h.x,h.z,h.diameter/2),"Auxiliary datum matches an actual CAD bore edge");
                report.push({model:model,feature:"baseAux"+(i+1),intersection_mm3:volume,x:h.x,z:h.z});
                if(volume>0.0001) failures++;
            }
            var positions:Array<Array<Float>> = meta.motor_bolts_xy;
            for(i in 0...positions.length) {
                var p=positions[i];
                var bore=Part.cylinderSpan(meta.motor_thread_root_radius_mm-0.1,0.1,Math.min(8,meta.motor_flange_thickness_mm-0.1),p[0],p[1]);
                var volume=overlap(shape,bore);
                report.push({model:model,feature:"motorBolt"+(i+1),intersection_mm3:volume});
                if(volume>0.0001) failures++;
            }
            var pilot=Part.cylinderSpan(mount.pilotDiameter/2-0.1,0.1,1.5,0,mount.shaftHeight);
            var pilotVolume=overlap(shape,pilot);report.push({model:model,feature:"pilot",intersection_mm3:pilotVolume});
            if(pilotVolume>0.0001) failures++;
            var motor=NemaStepper.frame(StringTools.endsWith(model,"57") ? 23 : 34);
            var rawMotor=motor.geometry(),placedMotor=rawMotor.translated(new Vector(0,mount.shaftHeight,0));rawMotor.close();
            var motorVolume=overlap(shape,placedMotor);report.push({model:model,feature:"motor-envelope",intersection_mm3:motorVolume});
            if(motorVolume>0.001) failures++;
            var rawBase=Part.box(120,6,130),base=rawBase.translated(new Vector(0,-3,meta.motor_flange_thickness_mm+1));rawBase.close();
            var baseVolume=overlap(shape,base);report.push({model:model,feature:"relieved-base",intersection_mm3:baseVolume});
            if(baseVolume>0.001) failures++;
            assembly.addComponent("mount-"+model,mount,AssemblyFrames.translation(index*180,0,0));
            assembly.addComponent("motor-"+model,motor);
            assembly.addMate("motor-mate-"+model,"fixed","mount-"+model,"motor","motor-"+model,"mountFace");
            var geometry=shape.volume(),roundtrip=registry.byId(RbkMotorMount.recipeType().id).create(mount.values()).geometry();
            check(roundtrip.valid() && Math.abs(roundtrip.volume()-geometry)<0.001,"Pinned recipe recreates normalized geometry");
            roundtrip.close();shape.close();index++;
            sys.io.File.saveContent(dir+"rbk-interface-checks.json",haxe.Json.stringify({schema_version:1,checks:report,failures:failures},null,"  "));
            trace('Measured interfaces checked: $model; accumulated failures: $failures');
        }
        if(!persistenceOnly) sys.io.File.saveContent(dir+"rbk-interface-checks.json",haxe.Json.stringify({schema_version:1,checks:report,failures:failures},null,"  "));
        check(failures==0,"RBK hole/pilot/motor/base checks need review; see rbk-interface-checks.json");
        var saved=assembly.encode(),loaded=MachineAssembly.decode(saved),poses=loaded.solvedPoses();
        for(model in RbkMotorMount.models()) {
            var mountPose=poses.get("mount-"+model),motorPose=poses.get("motor-"+model);
            check(mountPose!=null && motorPose!=null && Math.abs(motorPose.z-mountPose.z)<0.000001 &&
                Math.abs(motorPose.y-mountPose.y-new RbkMotorMount(model).shaftHeight)<0.000001,
                "Named motor connector solves at the face plane and correct shaft height");
        }
        for(entry in loaded.components()) if(StringTools.startsWith(entry.id,"mount-"))
            check(entry.component.connectors().length==13,"All mounting datums survive assembly reload");
        check(saved.indexOf(dir)<0,"Saved RBK recipe has no absolute cache path");
        var document=new Document(),original=new RbkMotorMount();
        var instance=document.createInstance("RBK",MachineKitDocuments.define(document,RbkMotorMount.recipeType(),original.values()));
        var volume=instance.shape().volume(),reloaded=DocumentCodec.decode(DocumentCodec.encode(document));
        var editable:cadkit.parametric.InstanceElement=cast reloaded.element(instance.id);
        check(Math.abs(editable.shape().volume()-volume)<0.001,"RBK document reload preserves geometry");
        editable.setTypedOverride("model","RBK15-57");
        check(Math.abs(editable.connector("motor").location.plane.origin.y-28)<0.000001 &&
            Math.abs(editable.shape().volume()-volume)>1,"Changing model refreshes CAD and connector datums together");
        MachineKitRecipes.forget(document);MachineKitRecipes.forget(reloaded);document.close();reloaded.close();
        sys.io.File.saveContent(dir+"rbk-mounted-motors.assembly.json",saved);
        trace(persistenceOnly ? "RBK named mating, assembly and editable document persistence checks passed" :
            "RBK interface geometry, named mating and assembly persistence checks passed");
    }
    static function circleAt(edges:Array<Dynamic>,x:Float,z:Float,radius:Float):Bool {
        for(edge in edges) if(Math.abs(edge.x-x)<0.001 && Math.abs(edge.z-z)<0.001 && Math.abs(edge.radius-radius)<0.001) return true;
        return false;
    }
    static function shaftAxisMatches(part:Part,meta:Dynamic):Bool {
        var matched=false;
        for(i in 0...part.shape.subshapeCount(CadKit.ShapeKind.Face)) {
            var face=part.shape.subshape(CadKit.ShapeKind.Face,i);
            if(face.surfaceKind()==CadKit.SurfaceKind.Cylinder) {
                var axis=face.faceAxis(),origin=axis.get_origin();
                if(Math.abs(axis.get_radius()-meta.shaft_diameter_mm/2)<0.001 &&
                    Math.abs(axis.get_direction().get_z())>0.999 && Math.abs(origin.get_x())<0.001 &&
                    Math.abs(origin.get_y()-meta.shaft_height_mm)<0.001) matched=true;
            }
            face.close();
        }
        return matched;
    }
}
