package machinekit.control;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/**
 * The controller's BiSS-C master: a rail module with `channels` encoder inputs, one `ch<n>` port each, for the absolute scales and
 * encoders that speak BiSS-C (`EncoderPart.interfaceName`). Each channel is a point-to-point link; the clock rate and the number of
 * channels read in one control cycle are what the module is bought for, and both are assumed here (no product behind them yet).
 */
class BissMaster extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final channels:Int;
  /** Highest clock the master drives a channel at, MHz. */
  public final clockMHz:Float;
  public final width:Float;
  public final height:Float;
  public final depth:Float;

  public function new(channels:Int = 4, clockMHz:Float = 10, width:Float = 100, height:Float = 90, depth:Float = 60) {
    if (channels < 1 || channels > 16) throw "A BiSS master needs 1 to 16 channels";
    for (value in [clockMHz, width, height, depth]) if (!(value > 0) || !Math.isFinite(value)) throw "A BiSS master needs finite positive ratings and dimensions";
    super('BISS-MASTER-$channels', "Controller BiSS-C master module (assumed ratings and envelope)", "device housing");
    addFacet(machinekit.component.EngineeringMetadata.assumed(""));
    this.channels = channels;
    this.clockMHz = clockMHz;
    this.width = width;
    this.height = height;
    this.depth = depth;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    for (slot in 0...channels) addPort({name: channelName(slot), kind: Signal, role: Consumer, iface: Unspecified, required: false});
  }

  /** The port of channel `slot` (0-based). */
  public function channelName(slot:Int):String {
    if (slot < 0 || slot >= channels) throw "BiSS channel is outside the master";
    return 'ch${slot + 1}';
  }

  public function interfaceName():String return "BiSS-C";

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.control.biss-master",
      [
        ComponentRecipeSupport.count("channels", 4),
        ComponentRecipeSupport.scalar("clockMHz", 10, 0.001),
        ComponentRecipeSupport.length("width", 100),
        ComponentRecipeSupport.length("height", 90),
        ComponentRecipeSupport.length("depth", 60)
      ],
      v -> new BissMaster(v.integer("channels"), v.number("clockMHz"), v.number("width"), v.number("height"), v.number("depth"))
    ).numbered(v -> 'BISS-MASTER-ASSUMED-${v.integer("channels")}CH-${Dimension.format(v.number("clockMHz"))}MHZ')
      .procured(_ -> Buy(null, null, null)).requiringInputs(["clockMHz"], ["channels"]));
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function values():ComponentValues return new ComponentValues().setInteger("channels", channels)
    .setNumber("clockMHz", clockMHz).setNumber("width", width).setNumber("height", height).setNumber("depth", depth)
    .setToken("material", materialSpec());

  override public function hasGeometry():Bool return true;

  /** A rail module with the encoder connectors along its front (`DeviceHousing.panelModule`). */
  override public function geometry(detail:ComponentDetail = Preview):Part return detail == Envelope ? envelope()
    : machinekit.power.DeviceHousing.panelModule(width, height, depth, {vents : true, terminals : Std.int(Math.min(16, channels))});

  override public function massGeometry():Null<Part> return envelope();

  function envelope():Part return Solids.named(Part.box(width, height, depth), "biss-master");

  override public function presentationView():Null<machinekit.component.ComponentView
    > return machinekit.power.DeviceHousing.panelView();
}
