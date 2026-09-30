package app;

/**
 * One IK drag of a joint-driven assembly occurrence, created by the document
 * session for the viewport (see `EditorScene.beginAssemblyDrag`). Points are
 * world coordinates in metres. Updates preview the pose without recording
 * history; `commit` records one undoable edit and `cancel` restores the
 * starting pose.
 */
interface SceneAssemblyDrag {
  /** Moves the grabbed point's target and previews the pose that follows it. */
  function update(x:Float, y:Float, z:Float):Void;
  /** Where the grabbed point is in the current preview. */
  function grabbedPoint():Array<Float>;
  /** The current target. */
  function target():Array<Float>;
  /** True while the grabbed point is on its target. */
  function following():Bool;
  /** Why the pose stopped short (or "Following"), for the user. */
  function message():String;
  /** Records the previewed pose as one undoable edit; false when nothing changed. */
  function commit():Bool;
  function cancel():Void;
}
