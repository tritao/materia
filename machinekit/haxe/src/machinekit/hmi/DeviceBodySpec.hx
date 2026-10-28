package machinekit.hmi;

import cadkit.modeling.Part;

/** One body of a bought device, as it is made and finished: an id, a label, the material it shows and a builder for its solid. */
class DeviceBodySpec {
  public final id:String;
  public final label:String;
  /** A material specification of the MaterialLibrary, such as "ABS black". */
  public final material:String;
  /** Builds a new solid each time it is called, in the device's frame; the caller closes it. */
  public final build:() -> Part;

  public function new(id:String, label:String, material:String, build:() -> Part) {
    this.id = id;
    this.label = label;
    this.material = material;
    this.build = build;
  }
}
