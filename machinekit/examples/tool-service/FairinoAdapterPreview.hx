import machinekit.robotics.FairinoDualToolAdapter;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;
import materia.assembly.AssemblyFrames;

class FairinoAdapterPreview {
    static function check(ok:Bool,label:String):Void {if(!ok) throw label;}
    public static function main():Void {
        var root=sys.FileSystem.fullPath(Sys.getCwd());
        while(!sys.FileSystem.exists(root+"/machinekit/examples/tool-service/haxeon.fairino-adapters.json")) {
            var parent=haxe.io.Path.directory(root);
            if(parent==root || parent=="") throw "Cannot locate Materia repository";
            root=parent;
        }
        var dir=root+"/machinekit/examples/tool-service/build/fairino-adapters";
        sys.FileSystem.createDirectory(dir);
        var assembly=new MachineAssembly();
        var records:Array<{variant:String,manufacturer:String,manufacturerPartNumber:String,supplier:String,supplierPartNumber:String,sourceUrl:String,externalPower:Bool,geometryStatus:String,pinoutVerified:Bool,currentRatingA:Null<Float>}> = [];
        for(variant in ["Standard","Expert"]) {
            var adapter=new FairinoDualToolAdapter(variant);
            var restored:FairinoDualToolAdapter=cast FairinoDualToolAdapter.recipeType().create(adapter.values());
            check(restored.designation==adapter.designation && restored.externalPower==adapter.externalPower,"Recipe round trip");
            check(!adapter.codeOnly && adapter.bom.typeId!=null,"Purchased registered BOM component");
            check(adapter.hasPort("powerIn") && adapter.bridges().length==0,"Unverified wiring must not become a circuit");
            check(adapter.port("powerIn").connector==(adapter.externalPower ? "externalPower" : "robot"),"Power source distinction");
            var part=adapter.geometry();check(part.valid() && part.volume()>0,"Invalid adapter CAD");
            part.exportStep(dir+"/"+variant.toLowerCase()+".step");part.close();
            var single=new MachineAssembly();single.addComponent("adapter",adapter);
            single.exposePort("robotSignal","adapter","robotSignal");single.exposePort("powerIn","adapter","powerIn");
            sys.io.File.saveBytes(dir+"/"+variant.toLowerCase()+".mtrg",SceneArtifact.encode(AssemblyPreview.scene(single,"Fairino "+variant+" estimated layout")));
            assembly.addComponent(variant,adapter,AssemblyFrames.translation(variant=="Expert" ? 100 : 0,0,0));
            records.push({variant:variant,manufacturer:"Fairino",manufacturerPartNumber:adapter.manufacturerPartNumber,
                supplier:"Welectron",supplierPartNumber:adapter.supplierPartNumber,sourceUrl:adapter.sourceUrl,
                externalPower:adapter.externalPower,geometryStatus:adapter.geometryStatus,pinoutVerified:adapter.pinoutVerified,
                currentRatingA:adapter.currentRatingA});
        }
        // Repeated purchased units aggregate; Standard and Expert remain separate lines.
        assembly.addComponent("Standard-spare",new FairinoDualToolAdapter(),AssemblyFrames.translation(-100,0,0));
        var selected=new MachineAssembly();
        machinekit.robotics.DhDualGripperServices.addPurchases(selected);
        var selectedBom=selected.billOfMaterials();
        check(selectedBom.lines().length==1 && selectedBom.quantity("ICACX1500GSCEXP")==1
            && selectedBom.quantity("ICACX1500GSCSTA")==0,"Dual PGC selection must purchase one Expert");
        sys.io.File.saveContent(dir+"/selected-dual-gripper-bom.json",haxe.Json.stringify(selectedBom.lines(),null,"  "));
        var bom=assembly.billOfMaterials();
        check(bom.lines().length==2 && bom.quantity("ICACX1500GSCSTA")==2 && bom.quantity("ICACX1500GSCEXP")==1,"Assembly BOM aggregation");
        sys.io.File.saveContent(dir+"/bom-check.json",haxe.Json.stringify(bom.lines(),null,"  "));
        sys.io.File.saveContent(dir+"/products.json",haxe.Json.stringify(records,null,"  "));
        Sys.println("Both adapter CAD exports, recipe round trips, power endpoints and assembly BOM aggregation passed: "+dir);
    }
}
