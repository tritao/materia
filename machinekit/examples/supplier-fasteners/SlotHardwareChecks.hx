import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import machinekit.component.MachineKitComponents;
import machinekit.component.SupplierRecipeLibrary;
import machinekit.structural.SlideInSlotNut;
import machinekit.structural.DrawingExtrusionProfile;
import machinekit.structural.TSlotExtrusion;
import machinekit.assembly.MachineAssembly;
import machinekit.component.SourcingPlan;

/** Exact section/envelope intersections. A collision is inconclusive where chamfers
 * are unspecified; a clear position does not qualify bearing contact or tolerances. */
class SlotHardwareChecks {
    static function check(ok:Bool,message:String):Void {if(!ok) throw message;}
    static function main():Void {
        var root=Sys.getEnv("MATERIA_ROOT");if(root==null) root=Sys.getCwd();
        var dir=root+"/build/suppliers/vallder/";
        var registry=MachineKitComponents.standard();
        var hardware:Dynamic=haxe.Json.parse(sys.io.File.getContent(dir+"hardware.json"));
        var library=new SupplierRecipeLibrary(haxe.Json.stringify(hardware),registry);
        var recipes:Array<Dynamic> = hardware.recipes;
        var assembly=new MachineAssembly();
        var plan=new SourcingPlan();
        var catalog:Dynamic=haxe.Json.parse(sys.io.File.getContent(dir+"catalog.json"));
        var variants:Array<Dynamic> = catalog.variants;
        for(r in recipes) {
            var nut=library.create("vallder",r.sku),shape=nut.geometry();
            check(shape.valid() && shape.solidCount()==1,"Valid threaded nut solid");
            var rebuilt=registry.byId("machinekit.structural.slide-in-slot-nut").create(nut.values());
            check(nut.bom.partNumber==rebuilt.bom.partNumber,"Nut recipe/BOM identity round trip");
            assembly.addComponent("nut-"+r.sku,nut);
            var variant:Dynamic=null;
            for(v in variants) if(v.supplier_product_id==r.supplier_product_id) variant=v;
            check(variant!=null,"Exact sourcing variant exists");
            var price:Dynamic=variant.price.raw;
            var minor:Null<Int> = price.price==null || price.price=="" ? null : Std.parseInt(price.price);
            if(minor!=null && Std.string(minor)!=price.price) throw "Invalid price minor units";
            plan.add({id:"vallder:"+r.sku,partNumber:nut.bom.partNumber,supplierId:"vallder",supplierSku:r.sku,
                productUrl:variant.url,observedAt:variant.source.fetched_at,currency:price.currency_code,
                currencyMinorUnit:price.currency_minor_unit,unitPriceMinor:minor,taxBasis:variant.price.tax_basis,
                inStock:variant.availability.is_in_stock==true,compatibilityVerified:false});
            shape.close();
        }
        var saved=assembly.encode(),reloaded=MachineAssembly.decode(saved);
        check(haxe.Json.stringify(assembly.billOfMaterials().lines())==haxe.Json.stringify(reloaded.billOfMaterials().lines()),
            "Saved nut assembly preserves BOM identities");
        check(saved.indexOf("vallder")<0,"Nut assembly geometry has no supplier dependency");
        sys.io.File.saveContent(dir+"slot-nuts.assembly.json",saved);
        sys.io.File.saveContent(dir+"slot-nuts.bom.json",haxe.Json.stringify(plan.lines(assembly.billOfMaterials()),null,"  "));
        if(Sys.args().indexOf("--recipes-only")>=0) {
            trace('Slot nut geometry and assembly checks passed: ${recipes.length} recipes');
            return;
        }
        var sections:Dynamic=haxe.Json.parse(sys.io.File.getContent(dir+"profiles.json"));
        var profiles:Array<Dynamic> = sections.profiles;
        var results:Array<Dynamic> = [];
        for(section in profiles) {
            if(section.system_id==null) continue;
            var dims:Array<Float> = section.dimensions_mm;
            var sectionProfile=new DrawingExtrusionProfile(section);
            var layout=TSlotExtrusion.forSystem(section.system_id,dims[0],dims[1],sectionProfile);
            var stock=sectionProfile.geometry(40);
            var r:Dynamic=null;
            for(entry in recipes) if(entry.candidate_system_id==section.system_id && entry.values.size=="M6") r=entry;
            check(r!=null,"M6 dimension binding exists");
            var nut:SlideInSlotNut=cast library.create("vallder",r.sku);
            var envelope=nut.fitEnvelope();
            for(face in 0...4) for(offset in layout.slotPositions(face)) {
                var best=1e30,bestDepth=0.0;
                // Coarse search is only for a static clearance candidate, never a seating proof.
                for(i in 0...21) {
                    var depth=i*0.25;
                    var normal=face==0 ? Vector.X() : face==1 ? Vector.Y() :
                        face==2 ? new Vector(-1,0,0) : new Vector(0,-1,0);
                    var tangent=(face==0 || face==2) ? Vector.Y() : Vector.X();
                    var origin=face==0 ? new Vector(dims[0]/2-depth,offset,20) :
                        face==1 ? new Vector(offset,dims[1]/2-depth,20) :
                        face==2 ? new Vector(-dims[0]/2+depth,offset,20) : new Vector(offset,-dims[1]/2+depth,20);
                    var placed=envelope.placed(new Location(new Plane(origin,tangent,normal)));
                    var common=stock.intersect(placed),volume=common.volume();
                    common.close();placed.close();
                    if(volume<best) {best=volume;bestDepth=depth;}
                    if(volume<0.000001) break;
                }
                results.push({family_id:section.family_id,section_sha256:section.source_sha256,
                    nut_sku:r.sku,nut_dimensions:r.values,face:face,offset_mm:offset,ridge_top_depth_mm:bestDepth,
                    intersection_mm3:best,status:best<0.000001 ? "static_clearance_candidate" : "envelope_interference",
                    compatibility_verified:false});
            }
            envelope.close();stock.close();
            trace('Checked measured slots: ${section.family_id}');
        }
        var report={schema_version:1,method:"Exact BRep intersection of unperforated sharp nut envelope and drawing extrusion",
            scope:"Static position only; 0.25 mm depth search from 0 to 5 mm. No seating, insertion motion, tolerances, engagement or strength qualification.",
            results:results};
        sys.io.File.saveContent(dir+"hardware-fit.json",haxe.Json.stringify(report,null,"  "));
        trace('Slot hardware checks passed: ${recipes.length} recipes, ${results.length} measured slots');
    }
}
