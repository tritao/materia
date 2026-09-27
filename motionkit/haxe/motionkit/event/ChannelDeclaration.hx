package motionkit.event;

/** Deployment declaration for one process-output channel and its safe value. */
class ChannelDeclaration {
  public final id:String;
  public final kind:ChannelKind;
  public final safeValue:EventValue;

  public function new(id:String, kind:ChannelKind, safeValue:EventValue) {
    if (id == null || StringTools.trim(id).length == 0)
      throw "Process channel needs a non-empty ID";
    if (kind == null) throw 'Process channel "$id" needs a kind';
    EventValueTools.validate(safeValue, 'Process channel "$id" safe value');
    if (!EventValueTools.matchesKind(safeValue, kind))
      throw 'Process channel "$id" safe value does not match its kind';
    this.id = id;
    this.kind = kind;
    this.safeValue = safeValue;
  }
}
