package tests;

import app.CadSceneGeometry;
import app.CadPlateModel;
import app.EditorScene;
import app.EditorScene.EditorSceneObject;
import app.SceneCodec;
import nativekit.scene.Scene;
import nativekit.ui.core.CommandContext;
import nativekit.ui.core.PropertyBinding;
import nativekit.ui.core.PropertyDescriptor;
import nativekit.ui.core.PropertyEditResult;
import nativekit.ui.core.PropertyValue;

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
