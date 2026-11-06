package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/** Generic robot-side changer half with bridged air and signal channels. */
class ToolChangerMaster extends MachineComponent {
  public final airChannels:Int;
  public final diameter:Float;
  public final thickness:Float;


  public function new(airChannels:Int, diameter:Float = 60, thickness:Float = 15) {
    if (airChannels < 1 || !Math.isFinite(diameter) || diameter <= 0 || !Math.isFinite(thickness)
      || thickness <= 0) throw "Tool changer master needs positive channels and dimensions";
    super(
      'CHANGER-MASTER-$airChannels-${Dimension.format(diameter)}',
      'Generic $airChannels-channel robot-side tool changer',
      "aluminium 6061",
      true
    );
    this.airChannels = airChannels;
    this.diameter = diameter;
    this.thickness = thickness;
    addConnector("robot", Mount, Solids.axial(0, 0, 0));
    addConnector("tool", Mount, Solids.axial(0, 0, thickness));
    for (i in 1...airChannels + 1) {
      addPort({name: 'airIn$i', kind: Pneumatic, role: Consumer, iface: PushIn(6), required: false});
      addPort({name: 'airOut$i', kind: Pneumatic, role: Supply, iface: PushIn(6), required: false});
      addBridge('airIn$i', 'airOut$i');
    }
    addPort({name: "signalIn", kind: Signal, role: Consumer, iface: Plug("generic", 4), required: false});
    addPort({name: "signalOut", kind: Signal, role: Supply, iface: Plug("generic", 4), required: false});
    addBridge("signalIn", "signalOut");
    addPort({name: "lock", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
    addFacet(new ChangerLockFacet("lock"));
    addFacet(new machinekit.component.CouplingFacet('generic:$airChannels:${Dimension.format(diameter)}', "tool"));
  }

  static final recipeTypeCache = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipeTypeCache.get(() -> new ComponentType(
      "machinekit.robotics.tool-changer-master",
      [
        ComponentRecipeSupport.count("airChannels", 2),
        ComponentRecipeSupport.length("diameter", 60),
        ComponentRecipeSupport.length("thickness", 15)
      ],
      v -> new ToolChangerMaster(v.integer("airChannels"), v.number("diameter"), v.number("thickness")),
      true
    ));
  }

  /** Subclasses must declare their own recipe and saved values. */
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    ToolChangerMaster) ? recipeType() : null;

  override public function values():machinekit
    .component.ComponentValues return new machinekit.component.ComponentValues().setInteger(
      "airChannels",
      airChannels
    ).setNumber(
      "diameter",
      diameter
    ).setNumber(
      "thickness",
      thickness
    ).setToken(
      "material",
      materialSpec()
    );

  override public function hasGeometry():Bool return true;

  /** The body with its coupling boss on the tool side and an air port on its rim for each channel. Envelope detail is the cylinder, which also sets the mass. */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (detail == Envelope || thickness < 6) return Part.cylinderSpan(diameter / 2, 0, thickness);
    var r = diameter / 2, rim = Math.max(1.0, thickness - 3);
    var shape = Solids.union([
      Solids.named(Part.cylinderSpan(r, 0, rim), "body"),
      Solids.named(Part.cylinderSpan(0.6 * r, rim - 0.01, thickness), "coupling-boss")
    ]);
    return Solids.cut(shape, [for (i in 0...airChannels) {
      var angle = 2 * Math.PI * i / airChannels, dir = new cadkit.modeling.Vector(Math.cos(angle), Math.sin(angle), 0);
      Solids.named(
        Part.cylinderAlong(1.5,
        new cadkit.modeling.Vector(dir.x * (r - 6), dir.y * (r - 6), 0.4 * thickness), dir, 6.1),
        "air-port" + (i + 1)
      );
    }
    ]);
  }

  override public function massGeometry():Null<Part> return Part.cylinderSpan(diameter / 2, 0, thickness);
}
