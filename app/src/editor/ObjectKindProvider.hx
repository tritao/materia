package app.editor;

import app.CadDocumentSession;
import app.EditorScene;
import app.SceneObjectData;
import nativekit.ui.properties.PropertyDescriptor;

/** Object-kind behavior consumed by creation, menus, inspection and picking. */
interface ObjectKindProvider {
  public function id():String;
  public function addSection():String;
  public function addLabel():String;
  public function addCommand():String;
  public function createDefaultRecord(id:String):SceneObjectData;
  public function createCadSession(graph:Null<String>, width:Float, height:Float,
    depth:Float):Null<CadDocumentSession>;
  public function properties(scene:EditorScene, id:String, prefix:String):Array<PropertyDescriptor>;
  public function commands(scene:EditorScene, id:String):Array<String>;
  public function isCad():Bool;
  public function supportsFaceHover():Bool;
  public function supportsEdgeHover():Bool;
}
