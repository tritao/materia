package tests;

import app.CadSceneGeometry;
import app.EditorScene;
import nativekit.scene.Scene;

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
