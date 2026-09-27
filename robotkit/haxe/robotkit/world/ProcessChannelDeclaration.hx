package robotkit.world;

/** Deployment output channel with a mandatory safe value. */
class ProcessChannelDeclaration {
  public final id:String;
  public final safeValue:ProcessEventValue;

  public function new(id:String, safeValue:ProcessEventValue) {
    ProcessEventCodec.validateId(id);
    ProcessEventCodec.validateValue(safeValue);
    this.id = id;
    this.safeValue = safeValue;
  }
}
