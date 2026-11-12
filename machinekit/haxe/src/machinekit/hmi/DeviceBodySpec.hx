package machinekit.hmi;

import cadkit.modeling.Part;
import machinekit.component.ComponentPort;
import machinekit.hmi.PanelSolid.PanelRecording;

/**
 * One body of a bought device, as it is made and finished: an id, a label, the material it shows and the recorded steps of its
 * solid, in the device's frame (threads, knurling and displays are recorded face details).
 */
class DeviceBodySpec {
  public final id:String;
  public final label:String;
  /** A material specification of the MaterialLibrary, such as "ABS black". */
  public final material:String;
  public final record:PanelRecording;
  /** Ports that belong to this body (the contacts in a switch block), not to the device as a whole. */
  public final ports:Array<ComponentPort>;

  public function new(id:String, label:String, material:String, record:PanelRecording, ?ports:Array<ComponentPort>) {
    this.id = id;
    this.label = label;
    this.material = material;
    this.record = record;
    this.ports = ports == null ? [] : ports.copy();
  }

  /** A new solid of the body; the caller closes it. */
  public function solid():Part {
    var builder = new cadkit.parametric.recording.DocumentBuilder();
    try {
      builder.output(record(builder));
      return machinekit.component.PartRecordings.solid(builder.finish());
    } catch (error:Dynamic) {
      builder.close();
      throw error;
    }
  }
}
