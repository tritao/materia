package app;

/** Portable scene data. Native node handles are deliberately not persisted. */
typedef SceneObjectData = {
  var id:String;
  var label:String;
  var type:String;
  var x:Float;
  var y:Float;
  var z:Float;
  var width:Float;
  var height:Float;
  var depth:Float;
  var collisionEnabled:Bool;
  var dynamicBody:Bool;
  var mass:Float;
  var red:Float;
  var green:Float;
  var blue:Float;
  var visible:Bool;
  @:optional var cadGraph:String;
  @:optional var sketchDraft:String;
}
