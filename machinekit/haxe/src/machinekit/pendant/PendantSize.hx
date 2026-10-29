package machinekit.pendant;

/** The case's own dimensions, in millimetres: the design inputs an editor can change. The controls keep their places. */
typedef PendantSize = {
  var width:Float;
  var height:Float;
  /** The face plate's thickness. */
  var thickness:Float;
  /** The case wall. */
  var wall:Float;
  /** Free depth behind the face plate. */
  var depth:Float;
}
