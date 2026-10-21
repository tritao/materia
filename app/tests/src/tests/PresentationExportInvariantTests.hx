package tests;

import sys.io.File;
import cadkit.modeling.Part;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.PosedParts;

/** Compare actual CAD/BOM outputs with fixed export metadata, without normalizing output bytes. */
class PresentationExportInvariantTests {
  public static function run(session:app.ProjectDocumentSession,assembly:machinekit.assembly.MachineAssembly,
      viewport:app.EditorPerspectiveViewport,directory:String):Void {
    var controller=viewport.presentation;if(controller==null)throw "Missing export-test presentation";
    var stepPath=directory+"/engineering.step",bomPath=directory+"/engineering-bom.json";
    var checks=0;
    function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
    function exportStep():haxe.io.Bytes {
      var definition=session.projectAssemblyDefinition,state=session.projectAssemblyState;
      if(definition==null||state==null)throw "Missing engineering export state";
      var runtime=new AssemblyState(definition,state),parts:Array<Part> = [];
      try {
        for(row in assembly.components()) {
          var local=row.component.geometry();
          try {parts.push(PosedParts.moved(local,runtime.worldPose(row.id)));local.close();}
          catch(error:Dynamic){local.close();throw error;}
        }
        var shape=cadkit.Shape.compound([for(part in parts)part.shape]);
        try {shape.exportStep(stepPath);shape.close();}
        catch(error:Dynamic){shape.close();throw error;}
        for(part in parts)part.close();
        return File.getBytes(stepPath);
      } catch(error:Dynamic){for(part in parts)part.close();throw error;}
    }
    function exportBom():String {
      File.saveContent(bomPath,haxe.Json.stringify(assembly.billOfMaterials().lines()));return File.getContent(bomPath);
    }
    var oldEpoch=Sys.getEnv("SOURCE_DATE_EPOCH");Sys.putEnv("SOURCE_DATE_EPOCH","0");
    function restore():Void Sys.putEnv("SOURCE_DATE_EPOCH",oldEpoch==null?"":oldEpoch);
    try {
      viewport.resetPresentation();var baseline=exportStep(),bom=exportBom();
      File.saveBytes(directory+"/baseline.step",baseline);
      var imported=cadkit.Shape.importStep(stepPath);
      var volume=imported.volume();imported.close();
      check(Math.abs(volume-2000)<1e-6,"Reproducible STEP retains both engineering solids");
      check(baseline.toString().indexOf("1970-01-01T00:00:00")>=0,"STEP export honors the fixed creation timestamp");
      function unchanged():Void {
        viewport.presentationFrame();
        check(exportStep().compare(baseline)==0,"Complete exported STEP bytes are identical across presentation changes");
        check(exportBom()==bom,"Complete exported BOM bytes are identical across presentation changes");
      }
      for(view in controller.record.views)for(amount in [0.0,1.0]) {
        viewport.selectPresentationView(view.id);viewport.setPresentationExplode(amount);unchanged();
      }
      for(procedure in controller.record.procedures) {
        viewport.selectPresentationProcedure(procedure.id);
        for(_ in 0...procedure.steps.length+1){unchanged();controller.next(false);}
      }
      Sys.putEnv("SOURCE_DATE_EPOCH","invalid");var rejected=false;
      try exportStep() catch(_:Dynamic)rejected=true;
      check(rejected&&File.getBytes(stepPath).compare(baseline)==0,"An invalid timestamp is rejected before changing an existing export");
      restore();viewport.resetPresentation();
      Sys.println('Presentation exports: $checks checks passed');
    } catch(error:Dynamic){restore();throw error;}
  }
}
