import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.motion.NemaStepper;
import machinekit.motion.StepperOnline23Geometry;
import machinekit.standard.FlatWasher;
import machinekit.component.ComponentValues;


/** Audits the exact public supplier reference before using it in a machine. */
class StepperOnlineMotorChecks {
    static var checks=0;
    static function check(ok:Bool,label:String):Void {checks++;if(!ok)throw label;}
    static function overlap(a:Part,b:Part):Float {
        var common=a.shape.common(b.shape),v=common.subshapeCount(CadKit.ShapeKind.Solid)==0 ? 0.0 : common.volume();common.close();return v;
    }
    static function main():Void {
        var root=Sys.getEnv("MATERIA_ROOT");if(root==null)throw "Set MATERIA_ROOT";
        var path=root+"/build/suppliers/stepperonline/cad/23HS22-2804S.step";
        var bytes=sys.io.File.getBytes(path),sha=haxe.crypto.Sha256.make(bytes),hex=new StringBuf();
        for(i in 0...sha.length)hex.add(StringTools.hex(sha.get(i),2).toLowerCase());
        check(hex.toString()==StepperOnline23Geometry.REFERENCE_SHA256,"Only the pinned reference can qualify geometry");
        var s=Shape.importStep(root+"/build/suppliers/stepperonline/cad/23HS22-2804S.step");
        var b=s.bounds(),lo=b.get_min(),hi=b.get_max();
        var circles:Array<Dynamic> = [],planes:Array<Dynamic> = [],solids:Array<Dynamic> = [];
        for(i in 0...s.subshapeCount(CadKit.ShapeKind.Solid)) {
            var solid=s.subshape(CadKit.ShapeKind.Solid,i),bb=solid.bounds(),mn=bb.get_min(),mx=bb.get_max();
            solids.push({index:i,volume:solid.volume(),min:[mn.get_x(),mn.get_y(),mn.get_z()],max:[mx.get_x(),mx.get_y(),mx.get_z()]});solid.close();
        }
        for(i in 0...s.subshapeCount(CadKit.ShapeKind.Face)) {
            var f=s.subshape(CadKit.ShapeKind.Face,i),bb=f.bounds(),mn=bb.get_min(),mx=bb.get_max();
            if(f.surfaceKind()==CadKit.SurfaceKind.Cylinder) {
                var a=f.faceAxis(),o=a.get_origin(),d=a.get_direction();
                circles.push({r:a.get_radius(),o:[o.get_x(),o.get_y(),o.get_z()],d:[d.get_x(),d.get_y(),d.get_z()],min:[mn.get_x(),mn.get_y(),mn.get_z()],max:[mx.get_x(),mx.get_y(),mx.get_z()]});
            }
            if(f.surfaceKind()==CadKit.SurfaceKind.Plane && f.faceArea()>15) {
                var c=f.center(),n=f.faceNormal();
                planes.push({area:f.faceArea(),center:[c.get_x(),c.get_y(),c.get_z()],normal:[n.get_x(),n.get_y(),n.get_z()],min:[mn.get_x(),mn.get_y(),mn.get_z()],max:[mx.get_x(),mx.get_y(),mx.get_z()]});
            }
            f.close();
        }
        sys.io.File.saveContent(root+"/build/suppliers/stepperonline/raw-audit.json",haxe.Json.stringify({solids:solids,planes:planes,cylinders:circles,volume:s.volume(),min:[lo.get_x(),lo.get_y(),lo.get_z()],max:[hi.get_x(),hi.get_y(),hi.get_z()]},null,"  "));
        var raw=new Part(s),reference=raw.translated(new Vector(0,0,-56));raw.close();
        var motor=NemaStepper.model("23HS22-2804S"),generated=motor.geometry();
        check(generated.valid() && generated.solidCount()==1,"Native motor is one valid solid");
        var bounds=generated.shape.bounds(),mn=bounds.get_min(),mx=bounds.get_max();
        check(Math.abs(mn.get_z()+56)<1e-5 && Math.abs(mx.get_z()-21)<1e-5,"Face-normalized body and shaft length");
        check(Math.abs(mx.get_x()-28.5)<1e-5 && Math.abs(mn.get_y()+28.5)<1e-5,"Measured 57 mm body envelope");
        check(motor.variant.housingProfile==StepperOnline23Geometry.PROFILE,"Named motor retains its own supplier profile");
        var restored=NemaStepper.namedRecipeType().create(new ComponentValues().setToken("model","23HS22-2804S"));
        check(restored.geometryKey()==motor.geometryKey(),"Supplier profile survives recipe reconstruction");
        var points=motor.boltPattern(),headClashes=0.0,washerClashes=0.0,generatedHeadClashes=0.0,generatedWasherClashes=0.0;
        for(p in points) {
            var bore=false,relief=false;
            for(c in circles) {
                var origin:Array<Float> = c.o;
                if(Math.abs(origin[0]-p.x)<1e-6 && Math.abs(origin[1]-p.y)<1e-6) {
                    if(Math.abs(c.r-2.55)<1e-6)bore=true;
                    if(Math.abs(c.r-4)<1e-6)relief=true;
                }
            }
            check(bore && relief,"CAD independently confirms bore and corner relief at each mount");
            var head=Part.cylinderSpan(3.5,-9,-5,p.x,p.y);
            var hole=Part.cylinderSpan(2.55,-5,0,p.x,p.y);
            headClashes+=overlap(reference,head);generatedHeadClashes+=overlap(generated,head);
            check(overlap(reference,hole)<0.001 && overlap(generated,hole)<0.001,"Through flange bore clears in source and native motor");
            var washer=FlatWasher.metric("M4").geometry(),placed=washer.translated(new Vector(p.x,p.y,-5.8));washer.close();
            washerClashes+=overlap(reference,placed);generatedWasherClashes+=overlap(generated,placed);
            head.close();hole.close();placed.close();
        }
        check(headClashes<0.001 && generatedHeadClashes<0.001,"M4 socket heads clear the actual and generated corner relief");
        check(washerClashes>0.001 && generatedWasherClashes>0.001,"ISO7089 M4 washers really exceed the R4 corner clearance");
        // Independent broad agreement: the generator simplifies only documented small details.
        var common=overlap(reference,generated),nativeVolume=generated.volume(),sourceVolume=reference.volume();
        var difference=(nativeVolume+sourceVolume-2*common)/sourceVolume;
        check(difference<0.005,"Native mechanical exterior agrees with pinned CAD within 0.5% symmetric-difference volume including omitted leads/details");
        generated.shape.exportStep(root+"/build/suppliers/stepperonline/23HS22-2804S-generated.step");
        sys.io.File.saveContent(root+"/build/suppliers/stepperonline/geometry-checks.json",haxe.Json.stringify({checks:checks,
            sourceSha256:hex.toString(),sourceVolumeMm3:sourceVolume,nativeVolumeMm3:nativeVolume,symmetricDifferenceFraction:difference,
            sourceHeadClashesMm3:headClashes,nativeHeadClashesMm3:generatedHeadClashes,
            sourceWasherClashesMm3:washerClashes,nativeWasherClashesMm3:generatedWasherClashes},null,"  "));
        generated.close();reference.close();Sys.println("StepperOnline motor checks passed: "+checks);
    }
}
