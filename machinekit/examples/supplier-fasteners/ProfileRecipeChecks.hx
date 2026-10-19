import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.MachineKitComponents;
import machinekit.structural.DrawingExtrusionProfile;
import machinekit.structural.ProfileMember;
import machinekit.structural.TSlotExtrusion;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;

/** Portable profile recipes: no supplier cache or external CAD needed for these checks. */
class ProfileRecipeChecks {
    static function check(value:Bool,message:String):Void {if(!value) throw message;}
    static function rejected(action:Void->Void):Bool {try action() catch(_:Dynamic) return true;return false;}
    static function main():Void {run();Sys.println("Portable profile assembly/document checks passed");}

    public static function run():Void {
        var registry=MachineKitComponents.defaultRegistry();
        check(registry.has("machinekit.structural.tslot-member") &&
              registry.has("machinekit.structural.drawing-profile-member"),"Portable profile recipes registered by default");
        for(type in [ProfileMember.tslotRecipeType(),ProfileMember.drawingRecipeType()])
            check(type.key(null)==type.key(type.defaults()) && type.bomKey(null)==type.bomKey(type.defaults()),
                  "Profile identity applies defaults consistently");
        var assembly=new MachineAssembly();
        var cases=[{system:"TSLOT-30-SLOT8",w:30.0,h:30.0},{system:"TSLOT-40-SLOT8",w:40.0,h:40.0},
                   {system:"TSLOT-40-SLOT8",w:40.0,h:80.0},{system:"TSLOT-40-SLOT8",w:80.0,h:80.0}];
        for(i in 0...cases.length) {
            var c=cases[i],member=new ProfileMember(TSlotExtrusion.forSystem(c.system,c.w,c.h),125,"aluminium 6061");
            assembly.addComponent("nominal-"+i,member);
        }
        var raw:Dynamic=haxe.Json.parse(DrawingExtrusionProfile.defaultSectionText());
        raw.supplier_id="supplier-must-not-be-saved";raw.source_url="https://example.test/not-needed.dxf";
        raw.sku="supplier-only";
        var drawing=new DrawingExtrusionProfile(raw),plate=new ProfileMember(drawing,175,"unspecified");
        assembly.addComponent("plate",plate);
        assembly.addComponent("plate-copy",plate);
        var saved=assembly.encode(),rebuilt=MachineAssembly.decode(saved);
        check(saved.indexOf("supplier-must-not-be-saved")<0 && saved.indexOf("not-needed.dxf")<0,
              "Saved sections omit supplier provenance and external paths");
        check(haxe.Json.stringify(assembly.billOfMaterials().lines())==haxe.Json.stringify(rebuilt.billOfMaterials().lines()),
              "Profile BOM identities and quantities survive assembly reload");
        var before=assembly.components(),after=rebuilt.components();
        check(before.length==after.length,"Profile occurrences survive reload");
        for(i in 0...before.length) {
            var original=before[i].component,restored=after[i].component;
            check(original.geometryKey()==restored.geometryKey(),"Profile geometry cache identity survives reload");
            check(original.materialSpec()==restored.materialSpec(),"Profile material survives reload");
            check(original.connector("end").frame.z==restored.connector("end").frame.z,"Cut length/end connector survives reload");
            var a=original.geometry(),b=restored.geometry();
            check(a.valid() && b.valid() && Math.abs(a.volume()-b.volume())<0.0001,"Reload rebuilds matching profile geometry");
            a.close();b.close();
            var bom=restored.bom,valuesKey=bom.valuesKey;
            check(bom.partNumber.length<300 && valuesKey!=null && valuesKey.length<600,"Section data stays out of BOM identity text");
        }
        var values=plate.values(),type=plate.componentType();
        check(type!=null,"Drawing member has a recipe");
        check(type.key(values)==type.key(values.copy().setToken("section",haxe.Json.stringify(raw))),
              "Recipe keys canonicalize section data and exclude supplier provenance");
        var changedLength=type.create(values.copy().setNumber("length",200));
        var changedMaterial=type.create(values.copy().setToken("material","steel"));
        var changedSection:Dynamic=haxe.Json.parse(drawing.sectionText());
        changedSection.circles=[{x:0,y:0,radius:2}];changedSection.area_mm2=400-Math.PI*4;
        var otherVariant=type.create(values.copy().setToken("section",haxe.Json.stringify(changedSection)));
        check(changedLength.bom.partNumber!=plate.bom.partNumber && changedMaterial.bom.partNumber!=plate.bom.partNumber,
              "Length and material changes distinguish stock requirements");
        check(otherVariant.bom.partNumber!=plate.bom.partNumber && otherVariant.geometryKey()!=plate.geometryKey(),
              "Different section variants retain distinct BOM and geometry identities");
        check(rejected(() -> ProfileMember.drawingRecipeType().create(new ComponentValues().setToken("section","{}"))),
              "Malformed saved section rejected before CAD execution");
        check(rejected(() -> ProfileMember.tslotRecipeType().create(new ComponentValues().setNumber("height",60))),
              "Saved profile must obey modular dimensions");
        var unsupported=new ProfileMember(TSlotExtrusion.forProfile("HFS5-2020"),100);
        check(unsupported.componentType()==null,"Unsupported legacy/custom profiles do not silently acquire a different recipe");
        var custom=new ProfileMember(new TSlotExtrusion(40,{designation:"CUSTOM-SYSTEM",family:"TSLOT-40-SLOT8",
            systemId:"TSLOT-40-SLOT8",gridPitch:40,width:40,height:40,slotWidth:8,slotDepth:10,
            throatDepth:2,headDepth:8,tWidth:20,boreDiameter:8.5}),100);
        check(custom.componentType()==null,"A custom designation cannot silently rebuild as a different nominal system");
        // Synthetic section isolates optional-text recipe persistence from the supplier cache.
        var emptyCircles:Array<Dynamic> = [];
        var section:Dynamic={dimensions_mm:[40,40],loops:[[[-20,-20,0],[20,-20,0],[20,20,0],[-20,20,0]]],
            circles:emptyCircles,area_mm2:1600};
        var configured=new ProfileMember(TSlotExtrusion.forSystem("TSLOT-40-SLOT8",40,40,
            new DrawingExtrusionProfile(section)),250,"aluminium 6061");
        var document=new Document();
        var instance=document.createInstance("Profile",MachineKitDocuments.define(document,ProfileMember.tslotRecipeType(),configured.values()));
        var partNumber=MachineKitDocuments.partNumber(instance),volume=instance.shape().volume();
        var loaded=DocumentCodec.decode(DocumentCodec.encode(document));
        var loadedInstance:cadkit.parametric.InstanceElement=cast loaded.element(instance.id);
        check(MachineKitDocuments.partNumber(loadedInstance)==partNumber &&
              Math.abs(loadedInstance.shape().volume()-volume)<0.0001,"Detailed T-slot recipe survives document save/reload");
        loadedInstance.setTypedOverride("length",300);
        check(Math.abs(loadedInstance.shape().volume()/volume-1.2)<0.000001 &&
              loadedInstance.connector("end").location.plane.origin.z==300,"Reloaded profile cut length remains editable");
        MachineKitRecipes.forget(loaded);MachineKitRecipes.forget(document);
        loaded.close();document.close();
    }
}
