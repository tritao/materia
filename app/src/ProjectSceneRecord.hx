package app;

/** Authored state layered over a generated Materia project artifact. */
typedef ProjectSceneRecord = {
  var version:Int;
  var reference:String;
  var overrides:Array<Dynamic>;
  var removed:Array<String>;
  var instances:Array<ProjectSceneInstance>;
}

typedef ProjectSceneInstance = {
  var sourceId:String;
  var object:Dynamic;
}
