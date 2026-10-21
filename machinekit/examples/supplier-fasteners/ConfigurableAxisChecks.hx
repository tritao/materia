import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.motion.*;
import cadkit.modeling.AssemblyState;
import materia.project.SceneArtifact;

/** Regression, rejection and second-supplier exercise for the reusable axis template. */
class ConfigurableAxisChecks {
    static var checks=0;
    static function check(ok:Bool,label:String):Void {checks++;if(!ok)throw label;}
    static function rejects(action:Void->Void,label:String):Void {
        var failed=false;try action() catch(_:Dynamic)failed=true;
        check(failed,label);
    }
    static function bom(axis:MachineAssembly):String {
        var lines=axis.billOfMaterials().lines();lines.sort((a,b)->Reflect.compare(a.partNumber,b.partNumber));
        return haxe.Json.stringify(lines);
    }
    static function main():Void {
        var root=Sys.getEnv("MATERIA_ROOT");if(root==null)throw "Set MATERIA_ROOT";
        var directory=root+"/build/suppliers/vallder/";RbkMotorMount.setSupplierDirectory(directory);
        var original=MachineAssembly.decode(sys.io.File.getContent(directory+"rbk-sfu1605-axis.assembly.json"),RbkSfuAxis.registry());
        var baseline=new RbkSfuAxis();baseline.validate();
        Sys.println("Baseline assembled and validated");
        check(bom(original)==bom(baseline),"Extraction preserves the original engineering BOM");
        var oldState=new AssemblyState(original.definition()),newState=new AssemblyState(baseline.definition());
        for(travel in [0.0,100,200]) {
            oldState.setJoint("travel",travel);newState.setJoint("travel",travel);
            var before=original.solvedPoses(oldState),after=baseline.solvedPoses(newState);
            for(entry in original.components()) {
                var a=before.get(entry.id),b=after.get(entry.id);
                check(a!=null && b!=null && Math.abs(a.x-b.x)+Math.abs(a.y-b.y)+Math.abs(a.z-b.z)+
                    Math.abs(a.qx-b.qx)+Math.abs(a.qy-b.qy)+Math.abs(a.qz-b.qz)+Math.abs(a.qw-b.qw)<1e-8,
                    "Extraction preserves pose "+entry.id);
            }
        }
        Sys.println("Baseline BOM and poses match saved artifact");
        for(kind in ScrewAxisPart.kinds()) {
            var old=new RbkAxisPart(kind).geometry(),fresh=new ScrewAxisPart(kind).geometry();
            check(fresh.valid() && Math.abs(old.volume()-fresh.volume())<1e-5,"Shared fabrication preserves baseline "+kind);
            old.close();fresh.close();
        }
        Sys.println("Fabrication volumes match baseline");
        var defaults=new ScrewAxisLayout();
        rejects(() -> new ScrewAxisLayout(400),"Reject travel beyond guides");
        rejects(() -> new ScrewAxisLayout(200,300,300),"Reject unsupported rail/support overlap");
        rejects(() -> new ScrewAxisLayout(200,400,300,100),"Reject insufficient rail spacing");
        rejects(() -> new ScrewAxisLayout(Math.NaN),"Reject nonfinite layout");
        rejects(() -> RbkSfuAxis.componentsFor(defaults,"34HS31-5504S").check(defaults),"Reject incompatible motor interface");
        var wrongNut=new ScrewAxisComponents(new RbkMotorMount(),NemaStepper.model("OPENBUILDS-NEMA23-56"),
            new VallderSfu1605Screw(),new BallNut(),new VallderBf12Support(),LinearRail.metric("HGR15",300),new HiwinHgh15ca(),5);
        rejects(() -> wrongNut.check(defaults),"Reject nominal SFU nut with a different flange/length");
        var layout=new ScrewAxisLayout(300,500,400,160);
        var selected=RbkSfuAxis.componentsFor(layout);
        var axis=new TwinRailScrewAxis(layout,selected);axis.validate();
        Sys.println("Second configuration assembled and validated");
        var saved=axis.encode(),loaded=MachineAssembly.decode(saved,MachineKitComponents.standard());
        check(bom(axis)==bom(loaded),"Second supplier configuration preserves saved BOM");
        check(bom(axis)!=bom(baseline),"Second motor and lengths produce distinct engineering requirements");
        // The independent supplier CAD audit proves that bare M4 heads clear this motor;
        // ISO7089 M4 washers do not fit its R4 corner relief.
        var candidate=new TwinRailScrewAxis(layout,RbkSfuAxis.componentsFor(layout,"23HS22-2804S",false));
        candidate.validate();
        var secondSaved=candidate.encode(),secondLoaded=MachineAssembly.decode(secondSaved,MachineKitComponents.standard());
        check(bom(candidate)==bom(secondLoaded),"Second supplier motor and bare-head installation survive reload");
        var supplierClearances=candidate.installationClearances();
        check(supplierClearances.length==168,"Second motor retains 56 required-clearance pairs at three states");
        for(row in supplierClearances)check(row.intersectionMm3<0.001,"Second supplier clash "+row.a+"/"+row.b);
        var secondState=new AssemblyState(secondLoaded.definition());
        for(travel in [0.0,150,300]) {
            candidate.setTravel(secondState,travel);var poses=secondLoaded.solvedPoses(secondState),bracket=poses.get("bracket");
            check(bracket!=null && Math.abs(bracket.z-140-travel)<1e-6,"Second supplier carriage follows travel");
            check(Math.abs(secondState.joint("screwTurn")+travel*2*Math.PI/5)<1e-6,"Second supplier retains screw lead");
        }
        for(h in candidate.hardware)if(h.interfacePair=="motor/rbk")
            check(h.grip==5 && h.engagement==7 && h.engagement<=h.availableDepth,"Bare M4x12 has 7 mm receiving-thread engagement");
        Sys.println("Second supplier axis passed all travel states");
        var state=new AssemblyState(loaded.definition());
        for(h in axis.hardware) {
            var clamp=StringTools.startsWith(h.interfacePair,"coupling/");
            check(clamp ? h.engagement>0 && h.engagement<1 : h.engagement>=h.minimumEngagement && h.engagement<=h.availableDepth,
                "Second configuration screw engagement "+h.id);
        }
        var clearances=axis.installationClearances();
        check(clearances.length==180,"Three states retain all 60 installation clearance pairs");
        for(row in clearances)check(row.intersectionMm3<0.001,"Configured installation clash "+row.a+"/"+row.b);
        for(travel in [0.0,150,300]) {
            axis.setTravel(state,travel);var poses=loaded.solvedPoses(state),bracket=poses.get("bracket");
            if(bracket==null)throw "Missing bracket pose";
            check(Math.abs(bracket.z-140-travel)<1e-6,"Configured carriage travel");
            check(Math.abs(state.joint("screwTurn")+travel*2*Math.PI/5)<1e-6,"Configured screw lead");
            Sys.println("Configured axis checked at "+travel+" mm");
        }
        rejects(() -> axis.setTravel(state,301),"Configured travel bounds enforced");
        // Supplier offers are optional and cannot change the engineering BOM.
        var sourcing=new SourcingPlan(),before=bom(axis),lines=sourcing.lines(axis.billOfMaterials());
        check(lines.length>0 && lines[0].status=="unselected" && before==bom(axis),"Sourcing remains optional and separate");
        var output=root+"/build/suppliers/configurable-axis/";if(!sys.FileSystem.exists(output))sys.FileSystem.createDirectory(output);
        sys.io.File.saveContent(output+"extended-300.assembly.json",saved);
        sys.io.File.saveContent(output+"extended-300.sourcing.json",haxe.Json.stringify(lines,null,"  "));
        sys.io.File.saveContent(output+"extended-300.bom.json",haxe.Json.stringify(axis.billOfMaterials().lines(),null,"  "));
        var scene=AssemblyPreview.scene(loaded,"extended-300-axis"),definition=scene.assemblyDefinition;
        if(definition==null)throw "Missing assembly definition";
        for(joint in definition.joints)if(joint.id=="travel")joint.driven=true;
        scene.presentation=materia.presentation.Presentations.assembled(scene.assemblyDefinition,Math.PI-0.9652516631899266,0.5639426413606289,"supplier-fasteners");
        sys.io.File.saveBytes(output+"extended-300.mtrg",SceneArtifact.encode(scene));
        sys.io.File.saveContent(output+"stepperonline-300.assembly.json",secondSaved);
        sys.io.File.saveContent(output+"stepperonline-300.bom.json",haxe.Json.stringify(candidate.billOfMaterials().lines(),null,"  "));
        sys.io.File.saveContent(output+"stepperonline-300.sourcing.json",haxe.Json.stringify(new SourcingPlan().lines(candidate.billOfMaterials()),null,"  "));
        var secondScene=AssemblyPreview.scene(secondLoaded,"stepperonline-300-axis"),secondDefinition=secondScene.assemblyDefinition;
        if(secondDefinition==null)throw "Missing second assembly definition";
        for(joint in secondDefinition.joints)if(joint.id=="travel")joint.driven=true;
        secondScene.presentation=materia.presentation.Presentations.assembled(secondScene.assemblyDefinition,Math.PI-0.9652516631899266,0.5639426413606289,"supplier-fasteners");
        sys.io.File.saveBytes(output+"stepperonline-300.mtrg",SceneArtifact.encode(secondScene));
        var exportState=new AssemblyState(secondLoaded.definition()),exportPoses=secondLoaded.solvedPoses(exportState),cache=new PosedParts();
        var placed:Array<cadkit.modeling.Part> = [],shapes:Array<cadkit.Shape> = [];
        for(entry in secondLoaded.components()) {var part=cache.posed(entry.component,exportPoses.get(entry.id));placed.push(part);shapes.push(part.shape);}
        var compound=cadkit.Shape.compound(shapes);compound.exportStep(output+"stepperonline-300.step");compound.close();
        for(part in placed)part.close();cache.close();
        sys.io.File.saveContent(output+"checks.json",haxe.Json.stringify({checks:checks,clearances:clearances,
            supplierTrial:{motor:"23HS22-2804S",status:"nominal_cad_checks_pass",clearances:supplierClearances,
                installation:"M4x12 socket heads directly on the 5 mm flange; 7 mm receiving-thread engagement; no ISO7089 M4 washers",
                geometryBasis:"Pinned 2018 distributor STEP and StepperOnline dimension drawing; native analytic profile"},
            motor:selected.motor.designation,travelMm:layout.travel,screwLengthMm:layout.screwLength,railLengthMm:layout.railLength,
            railSpacingMm:layout.railSpacing,basis:"Nominal interface and CAD checks; mixed-supplier configuration, not load qualification"},null,"  "));
        Sys.println("Configurable axis checks passed: "+checks);
    }
}
