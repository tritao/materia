package robotkit.model;

class Link {
  public final id:LinkId;
  public var name:String;

  public function new(name:String, ?id:LinkId) {
    // Legacy callers use the initial name once; imports pass the stored ID.
    this.id = id == null ? name : id;
    this.name = name;
  }
}
