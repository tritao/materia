package machinekit.operator;

/**
 * Where pendant jog distances go on the host. An implementation turns them into jog motion (the
 * jog purpose of a plan) in the host runtime; the pendant driver knows nothing of planners.
 */
interface JogSink {
  /** Jogs `axisId` by `delta` in the axis' user unit (millimetres or degrees), signed. */
  function jog(axisId:String, delta:Float):Void;
}
