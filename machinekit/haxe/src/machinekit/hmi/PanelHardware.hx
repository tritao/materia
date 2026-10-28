package machinekit.hmi;

import machinekit.component.MachineComponent;

/**
 * A bought fastener, insert or printed fixing placed in a panel: its member id, the component, and where its origin sits in
 * panel coordinates. `flipped` turns it over (about x) so its top faces -z, for a screw entering from behind the panel.
 */
class PanelHardware {
  public final id:String;
  public final component:MachineComponent;
  public final x:Float;
  public final y:Float;
  public final z:Float;
  public final flipped:Bool;

  public function new(id:String, component:MachineComponent, x:Float, y:Float, z:Float, flipped:Bool = false) {
    this.id = id;
    this.component = component;
    this.x = x;
    this.y = y;
    this.z = z;
    this.flipped = flipped;
  }

  /** The same hardware moved by (dx, dy, dz) and renamed, as a part's hardware lands in its panel. */
  public function placed(newId:String, dx:Float, dy:Float, dz:Float):PanelHardware return new PanelHardware(newId, component, x + dx, y + dy, z + dz, flipped);
}
