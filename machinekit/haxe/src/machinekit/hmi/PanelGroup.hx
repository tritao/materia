package machinekit.hmi;

/** Panel parts shown together in the assembly's hierarchy: a group id and the ids of the controls in it. */
class PanelGroup {
  public final id:String;
  public final members:Array<String>;

  public function new(id:String, members:Array<String>) {
    this.id = id;
    this.members = members;
  }
}
