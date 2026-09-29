package app;

/** Authored worker content stored on a scene object. */
typedef WorkerObjectData = {
  var asset:String;
  var job:String;
  var zones:Array<String>;
  @:optional var migrationNote:String;
}
