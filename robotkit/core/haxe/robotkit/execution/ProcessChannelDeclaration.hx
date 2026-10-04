package robotkit.execution;



/**
 * Deployment output channel with a mandatory safe value. Faults and emergency stops always take it
 * there; a channel that `keepOnStop`, such as a gripper's or vacuum's, keeps its output through a
 * commanded stop or an aborted motion, so a robot that stops does not drop what it holds.
 */
class ProcessChannelDeclaration {
  public final id:String;
  public final safeValue:ProcessEventValue;
  public final keepOnStop:Bool;

  public function new(id:String, safeValue:ProcessEventValue, keepOnStop:Bool = false) {
    ProcessEventCodec.validateId(id);
    ProcessEventCodec.validateValue(safeValue);
    this.id = id;
    this.safeValue = safeValue;
    this.keepOnStop = keepOnStop;
  }
}
