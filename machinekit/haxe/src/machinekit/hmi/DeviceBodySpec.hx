package machinekit.hmi;

import cadkit.modeling.Part;
import machinekit.component.ComponentPort;

/** One body of a bought device, as it is made and finished: an id, a label, the material it shows and a builder for its solid. */
class DeviceBodySpec {
  public final id:String;
  public final label:String;
  /** A material specification of the MaterialLibrary, such as "ABS black". */
  public final material:String;
  /** Builds a new solid each time it is called, in the device's frame; the caller closes it. */
  public final build:() -> Part;
  /** Ports that belong to this body (the contacts in a switch block), not to the device as a whole. */
  public final ports:Array<ComponentPort>;

  public function new(id:String, label:String, material:String, build:() -> Part, ?ports:Array<ComponentPort>) {
    this.id = id;
    this.label = label;
    this.material = material;
    this.build = build;
    this.ports = ports == null ? [] : ports.copy();
  }
}
