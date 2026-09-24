package tests;

import app.CadSceneGeometry;
import app.CadPlateModel;
import app.EditorScene;
import app.EditorScene.EditorSceneObject;
import app.EditorSceneTree;
import app.SceneCodec;
import cadkit.Shape;
import nativekit.scene.Scene;
import nativekit.ui.core.CommandContext;
import nativekit.ui.properties.PropertyBinding;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyEditResult;
import nativekit.ui.properties.PropertyValue;

class CadSceneSmoke {
  static function main():Int {
    var scene=Scene.create();
    try {
      var geometry=scene.createGeometry();
      scene.setGeometryData(geometry,CadSceneGeometry.mountingPlate(0.08,0.05,0.006,0.012));
      var editor=new EditorScene();
      if(!editor.createMountingPlate()||editor.pick(0.0,0.0)!="scene"||
          editor.pick(0.03,0.0)=="scene"||
          editor.pickRay(0.0,0.0,1.0,0.0,0.0,-1.0)!="scene"||
          editor.pickRay(0.03,0.0,1.0,0.0,0.0,-1.0)!=editor.selectedId)
        throw "CAD mesh picking does not match the through-hole";
      var plate=editor.object(editor.selectedId);
      var hole:Null<PropertyDescriptor> = null;
      if(plate==null||plate.cadGraph==null)throw "CAD plate did not retain its feature graph";
      for(property in editor.properties())if(property.label=="Hole diameter")hole=property;
      if(hole==null)throw "CAD plate parameters are missing from the inspector";
      var binding=new PropertyBinding(hole,new CommandContext(editor.document,[editor.selectedId]));
      switch binding.apply(PropertyValue.Float(0.016)) {
        case Applied:
        default: throw "CAD parameter edit was not applied";
      }
      var changedPlate=editor.object(editor.selectedId);
      if(changedPlate==null||changedPlate.cadGraph==null)throw "CAD plate disappeared after recompute";
      var changed=CadPlateModel.decode(changedPlate.cadGraph);
      if(Math.abs(changed.parameters().holeDiameter-0.016) > 0.000000001)throw "CAD recompute lost the edited diameter";
      changed.close();
      var guarded=CadPlateModel.decode(changedPlate.cadGraph), beforeFailure=guarded.encode(), failed=false;
      try guarded.setMetres(CadPlateModel.HOLE_DIAMETER,0.0) catch(_:Dynamic) failed=true;
      if(!failed||guarded.encode()!=beforeFailure)throw "Failed CAD recompute did not retain the last valid graph";
      guarded.close();
      if(!editor.document.undo())throw "CAD parameter edit was not undoable";
      var undonePlate=editor.object(editor.selectedId);
      if(undonePlate==null||undonePlate.cadGraph==null)throw "CAD plate disappeared after undo";
      var undone=CadPlateModel.decode(undonePlate.cadGraph);
      if(Math.abs(undone.parameters().holeDiameter-0.012) > 0.000000001)throw "CAD undo did not restore the graph";
      undone.close();
      if(!editor.document.redo())throw "CAD parameter edit was not redoable";
      var reopened=new EditorScene(SceneCodec.decode(SceneCodec.encode(editor)));
      var restoredPlate:Null<EditorSceneObject> = null;
      for(item in reopened.items())if(item.kind=="cad-plate")restoredPlate=item;
      if(restoredPlate==null||restoredPlate.cadGraph==null)throw "CAD plate disappeared after reopen";
      var restored=CadPlateModel.decode(restoredPlate.cadGraph);
      if(Math.abs(restored.parameters().holeDiameter-0.016) > 0.000000001)throw "CAD graph did not survive save/reopen";
      restored.close();reopened.dispose();
      var plateId=editor.selectedId;
      var dirtyBeforeFacePick=editor.document.isDirty;
      if(editor.selectAtRay(0.025,0.0,1.0,0.0,0.0,-1.0)!=plateId||
          editor.selectedCadFaceIndex<0)throw "CAD top face was not selected by mesh picking";
      if(editor.document.isDirty!=dirtyBeforeFacePick)throw "Face selection changed document history";
      var featureCount=editor.cadFeatureNames(plateId).length;
      var hierarchy=new EditorSceneTree(editor);
      if(hierarchy.childCount(plateId)!=featureCount||
          !editor.selectTreeKey(hierarchy.childKeyAt(plateId,0))||editor.selectedId!=plateId)
        throw "CAD hierarchy did not expose selectable feature nodes";
      editor.selectAtRay(0.025,0.0,1.0,0.0,0.0,-1.0);
      if(!editor.addHoleOnSelectedFace(0.008)||editor.cadFeatureNames(plateId).length!=featureCount+2||
          editor.pick(0.025,0.0)!="scene")throw "Face operation did not cut the selected location";
      if(editor.selectedCadFaceIndex<0)throw "Selected face did not remap after recompute";
      if(!editor.document.undo()||editor.pick(0.025,0.0)!=plateId)
        throw "Undo did not restore the plate before the face operation";
      if(!editor.document.redo()||editor.pick(0.025,0.0)!="scene")
        throw "Redo did not restore the face operation";
      var withHole=new EditorScene(SceneCodec.decode(SceneCodec.encode(editor)));
      if(withHole.pick(0.025,0.0)!="scene")throw "Added face hole did not survive reopen";
      withHole.dispose();
      editor.setDimensions(plateId,0.09,0.05,0.006);
      if(editor.pick(0.025,0.0)!="scene")throw "Added hole was lost when plate dimensions changed";
      if(!editor.duplicateSelected())throw "CAD plate duplication failed";
      var duplicate=editor.object(editor.selectedId);
      if(duplicate==null||duplicate.cadGraph==null||editor.cadFeatureNames(duplicate.id).length!=featureCount+2)
        throw "CAD plate duplicate lost its feature sequence";
      editor.select(plateId);
      var stepPath=Sys.getCwd()+"/cad-plate-smoke.step";
      editor.exportSelectedCad(stepPath);
      var imported=Shape.importStep(stepPath);
      if(imported.volume()<=0)throw "STEP export did not contain a solid";
      imported.close();sys.FileSystem.deleteFile(stepPath);
      editor.dispose();
      Sys.println("CAD SceneKit mesh smoke passed");
      scene.dispose();
      return 0;
    } catch(error:Dynamic) {
      scene.dispose();
      Sys.println("CAD SceneKit mesh smoke failed: "+Std.string(error));
      return 1;
    }
  }
}
