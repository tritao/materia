package app;

/** Authored state layered over a generated Materia project artifact. */
typedef ProjectSceneRecord = {
  var version:Int;
  var reference:String;
  var overrides:Array<ProjectFieldOverride>;
  var removed:Array<String>;
  var instances:Array<ProjectSceneInstance>;
  /** Encoded AssemblyStateRecord for the generated assembly configuration. */
  @:optional var assemblyState:String;
  /** Tree-joint IDs treated as dependent coordinates in this project. */
  @:optional var assemblyDependentJoints:Array<String>;
}

typedef ProjectSceneInstance = {
  var sourceId:String;
  var id:String;
  var overrides:Array<ProjectFieldOverride>;
}

typedef ProjectFieldOverride = {
  var targetId:String;
  var property:String;
  var kind:String;
  var value:Dynamic;
}
