import machinekit.assembly.NemaMountPlate;
import machinekit.assembly.MachineAssembly;
import machinekit.motion.NemaStepper;
import machinekit.component.ComponentValues;
import machinekit.component.MachineKitComponents;
import cadkit.modeling.Part;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;
import materia.assembly.AssemblyFrames;
import machinekit.assembly.RbkMotorMount;
import machinekit.component.SupplierRecipeLibrary;
import machinekit.component.SourcingPlan;

class MotorMountChecks {
    static function check(ok:Bool,message:String):Void {if(!ok) throw message;}
    static function rejected(fn:Void->Void):Bool {try fn() catch(_:Dynamic) return true;return false;}
    static function main():Void {
        var registry=MachineKitComponents.defaultRegistry(),type=NemaMountPlate.recipeType();
        check(registry.has(type.id),"Motor plate registered by default");
        var assembly=new MachineAssembly();
        for(frame in [17,23,34]) {
            var motor=NemaStepper.frame(frame),plate=new NemaMountPlate(motor,6);
            var part=plate.geometry(),holeRadius=motor.mountScrew(10).clearanceDiameter(Medium)/2;
            var expected=(plate.size*plate.size-Math.PI*(Math.pow((motor.spec.pilotDiameter+0.2)/2,2)+4*holeRadius*holeRadius))*6;
            check(part.valid() && part.solidCount()==1 && Math.abs(part.volume()-expected)<0.001,"Legacy plate keeps exact pilot/bolt geometry");
            part.close();
            assembly.addComponent("frame-"+frame,plate,AssemblyFrames.translation(frame==17 ? 0 : frame==23 ? 160 : 320,0,0));
        }
        var motor=NemaStepper.frame(23);
        for(system in ["TSLOT-30-SLOT8","TSLOT-40-SLOT8"]) {
            var spacing=system=="TSLOT-30-SLOT8" ? 90.0 : 80.0;
            var plate=new NemaMountPlate(motor,8,30,{systemId:system,railSpacing:spacing,alongPitch:40,holeDiameter:6.6});
            var part=plate.geometry(),plain=new NemaMountPlate(motor,8,30).geometry();
            check(part.valid() && part.solidCount()==1,"Profile plate is one valid solid");
            check(Math.abs(plain.volume()-part.volume()-4*Math.PI*3.3*3.3*8)<0.001,"Four profile bores remove the specified volume");
            check(plate.profileHolePositions().length==4 && plate.connector("profileBolt4").frame.z==8,"Four explicit profile-side connector frames");
            var holePositions=plate.profileHolePositions();holePositions[0].x=999;
            check(plate.profileHolePositions()[0].x!=999,"Profile hole positions are defensive copies");
            var probe=Part.cylinderSpan(3.2,-0.1,8.1,-spacing/2,-20),intersection=part.intersect(probe);
            check(intersection.volume()<0.000001,"Profile bolt axis actually passes through a hole");
            intersection.close();probe.close();part.close();plain.close();
            assembly.addComponent(system,plate,AssemblyFrames.translation(0,system=="TSLOT-30-SLOT8" ? 150 : 300,0));
        }
        var saved=assembly.encode(),loaded=MachineAssembly.decode(saved);
        var originalLines=assembly.billOfMaterials().lines(),restoredLines=loaded.billOfMaterials().lines();
        originalLines.sort((a,b) -> Reflect.compare(a.partNumber,b.partNumber));
        restoredLines.sort((a,b) -> Reflect.compare(a.partNumber,b.partNumber));
        var originalBom=haxe.Json.stringify(originalLines),restoredBom=haxe.Json.stringify(restoredLines);
        check(originalBom==restoredBom,"Motor plate BOM survives assembly reload");
        var before=assembly.components(),after=loaded.components();
        before.sort((a,b) -> Reflect.compare(a.id,b.id));after.sort((a,b) -> Reflect.compare(a.id,b.id));
        for(i in 0...before.length) {
            var a=before[i].component,b=after[i].component;
            check(a.geometryKey()==b.geometryKey(),"Mount geometry identity survives reload");
            var sa=a.geometry(),sb=b.geometry();
            check(sb.valid() && Math.abs(sa.volume()-sb.volume())<0.001,"Reload reconstructs matching mount geometry");
            sa.close();sb.close();
        }
        check(rejected(() -> new NemaMountPlate(motor,8,30,{systemId:"TSLOT-40-SLOT8",railSpacing:81,alongPitch:40,holeDiameter:6.6})),"Off-grid rail spacing rejected");
        check(rejected(() -> new NemaMountPlate(motor,8,10,{systemId:"TSLOT-40-SLOT8",railSpacing:80,alongPitch:40,holeDiameter:6.6})),"Insufficient plate edge rejected");
        check(rejected(() -> new NemaMountPlate(motor,8,30,{systemId:"TSLOT-40-SLOT8",railSpacing:40,alongPitch:40,holeDiameter:6.6})),"Obstructed pilot passage rejected");
        check(rejected(() -> type.create(new ComponentValues().setNumber("railSpacing",80))),"Inactive hole parameters cannot silently change identity");
        check(new NemaMountPlate(NemaStepper.frame(17,55),6).componentType()==null,"Custom motor interfaces cannot silently rebuild as named motors");
        check(new machinekit.screwdriving.NemaMotorMount().fastenings().require().length==4,
            "Existing NEMA 17 plate assembly retains four valid screw connections");
        var document=new Document(),plate=new NemaMountPlate(NemaStepper.frame(17),6);
        var instance=document.createInstance("Mount",MachineKitDocuments.define(document,type,plate.values()));
        var volume=instance.shape().volume();
        var restored=DocumentCodec.decode(DocumentCodec.encode(document));
        var editable:cadkit.parametric.InstanceElement=cast restored.element(instance.id);
        editable.setTypedOverride("thickness",9);
        check(Math.abs(editable.shape().volume()/volume-1.5)<0.000001 && editable.connector("bolt1").location.plane.origin.z==9,"Reloaded plate thickness and connectors remain editable");
        MachineKitRecipes.forget(document);MachineKitRecipes.forget(restored);document.close();restored.close();
        var root=Sys.getEnv("MATERIA_ROOT");
        if(root!=null) {
            sys.io.File.saveContent(root+"/build/suppliers/vallder/motor-mounts.assembly.json",saved);
            sys.io.File.saveContent(root+"/build/suppliers/vallder/motor-mounts.bom.json",haxe.Json.stringify(assembly.billOfMaterials().lines(),null,"  "));
            if(Sys.args().indexOf("--supplier-cad")>=0) supplierCad(root+"/build/suppliers/vallder/");
        }
        trace("Motor mount geometry, assembly and editable document checks passed");
    }

    /** Purchased RBK geometry remains a separate engineering requirement from the custom plate. */
    static function supplierCad(dir:String):Void {
        var report:Dynamic=haxe.Json.parse(sys.io.File.getContent(dir+"motor-mounts.json"));
        var catalog:Dynamic=haxe.Json.parse(sys.io.File.getContent(dir+"catalog.json"));
        var bindings:Array<Dynamic> = report.bindings,variants:Array<Dynamic> = catalog.variants;
        var assembly=new MachineAssembly(),plan=new SourcingPlan(),count=0;
        RbkMotorMount.setSupplierDirectory(dir);
        var library=new SupplierRecipeLibrary(sys.io.File.getContent(dir+"recipes.json"),MachineKitComponents.defaultRegistry());
        for(binding in bindings) if(binding.geometry_kind=="supplier_cad") {
            var assets:Array<Dynamic> = binding.cad;
            check(assets.length==1,"RBK binding has exactly one validated STEP asset");
            var asset=assets[0];
            var part=library.create("vallder",binding.sku);
            var shape=part.geometry();check(shape.valid() && shape.volume()>0,"Bound supplier CAD loads correctly");shape.close();
            assembly.addComponent("rbk-"+binding.sku,part,AssemblyFrames.translation(count*180,0,0));
            check(part.connector("motor")!=null && part.connector("base")!=null,"Supplier recipe has named mounting frames");
            var variant:Dynamic=null;
            for(v in variants) if(v.supplier_product_id==binding.supplier_product_id) variant=v;
            check(variant!=null,"Bound RBK offer uses an exact catalogue variant");
            var price:Dynamic=variant.price.raw;
            var minor:Null<Int> = price.price==null || price.price=="" ? null : Std.parseInt(price.price);
            if(minor!=null && Std.string(minor)!=price.price) throw "Invalid price minor units";
            plan.add({id:"vallder:"+binding.sku,partNumber:part.bom.partNumber,supplierId:"vallder",supplierSku:binding.sku,
                productUrl:variant.url,observedAt:variant.source.fetched_at,currency:price.currency_code,
                currencyMinorUnit:price.currency_minor_unit,unitPriceMinor:minor,taxBasis:variant.price.tax_basis,
                inStock:variant.availability.is_in_stock==true,compatibilityVerified:false});
            count++;
        }
        check(count==4,"Four RBK variant bindings checked");
        var lines=plan.lines(assembly.billOfMaterials());
        for(line in lines) check(line.selected==null && line.offers.length==1,"Supplier choice remains optional");
        sys.io.File.saveContent(dir+"supplier-motor-mounts.bom.json",haxe.Json.stringify(lines,null,"  "));
        sys.io.File.saveContent(dir+"supplier-motor-mounts.assembly.json",assembly.encode());
        trace('Supplier motor CAD and optional sourcing checks passed: $count RBK variants');
    }
}
