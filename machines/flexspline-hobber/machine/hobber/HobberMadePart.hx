package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Part;
import machinekit.motion.ErobActuatorPart;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ManufacturingRoute.MachinedFace;
import machinekit.structural.GroundStock;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Shared catalogue identity and manufacturing lifecycle for the machine's made parts.
 * Geometry, fastener interfaces and routes belong to the concrete part classes.
 * The catalogue family ID and complete input record preserve existing BOM identity. */
class HobberMadePart extends MachineComponent {
  public final designId:String;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return [
    P.choice(
      "design",
      [
        "GE-X-carriage",
        "GE-Z-saddle",
        "GE-X-fixed-drive-bar",
        "GE-X-floating-drive-bar",
        "GE-Z-fixed-drive-bar",
        "GE-Z-floating-drive-bar",
        "GE-X-scale-seat",
        "GE-Z-scale-seat",
        "GE-Z-scale-arm",
        "GE-chip-tray",
        "GE-coolant",
        "GE-spindle-seat",
        "GE-motor-cradle",
        "GE-outboard-carrier",
        "electrical-panel"
      ],
      "GE-Z-saddle"
    ),
    P.quantity("tableFaceDiameter", 200, "mm"),
    P.quantity("tableHousingDiameter", 260, "mm"),
    P.quantity("tableFaceHeight", 228, "mm"),
    P.quantity("tableSeatHeight", 18, "mm")
  ];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.hobbing.saddle",
      parameters(),
      v -> create(v.token("design"), v)
    ).versioned(7).numbered(v -> 'HOBBER-${v.token("design").toUpperCase()}')
      .procured(_ -> Make("steel fabrication and machining"))
      .requiringInputs(["design", "tableFaceDiameter", "tableHousingDiameter", "tableFaceHeight", "tableSeatHeight"]));
  }
  public static function create(design:String, ?values:ComponentValues):HobberMadePart return switch design {
    case "GE-X-carriage": new HobberXCarriage(values);
    case "GE-Z-saddle": new HobberZSaddle(values);
    case "GE-X-fixed-drive-bar": new HobberDriveBar("X", "fixed", values);
    case "GE-X-scale-seat": new HobberScaleSeat("X", values);
    case "GE-Z-scale-arm": new HobberZScaleArm(values);
    case "GE-chip-tray":
      var table = new ComponentValues();
      if (values != null) for (name in ["tableFaceDiameter", "tableHousingDiameter", "tableFaceHeight", "tableSeatHeight"])
        if (values.get(name) != null) table.setNumber(name, values.number(name));
      new HobberChipTray(table);
    case "GE-coolant": new HobberCoolantPort();
    case "GE-spindle-seat": new HobberSpindleSeat(values);
    case "GE-motor-cradle": new HobberMotorCradle(values);
    case "GE-outboard-carrier": new HobberOutboardCarrier(values);
    case "electrical-panel": new HobberElectricalPanel(values);
    case "GE-X-floating-drive-bar": new HobberDriveBar("X", "floating", values);
    case "GE-Z-scale-seat": new HobberScaleSeat("Z", values);
    case "GE-Z-fixed-drive-bar": new HobberDriveBar("Z", "fixed", values);
    case "GE-Z-floating-drive-bar": new HobberDriveBar("Z", "floating", values);
    default: throw "Unknown hobber made part: "+design;
  };
  /**
   * `ownValues`: the part has a recipe of its own (its `componentType`) and these are that recipe's values, so it carries none
   * of the shared made-part recipe's.
   */
  public function new(design:String, ?values:ComponentValues, ?ownValues:ComponentValues) {
    var supplied = values == null ? new ComponentValues() : values.copy();
    supplied.setToken("design", design);
    var v = ownValues != null ? ownValues.copy() : P.resolve(parameters(), supplied);
    super(
      "HOBBER-" + design,
      "Designed steel " + design + "; machined seats, assumed structural adequacy",
      "steel",
      false,
      v
    );
    addFacet(new EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    designId = design;
    addConnector("datum", Mount, F.identity());
  }
  /** Called after the concrete part has initialized its dimensions. */
  public function finish(collision:Bool):Void {
    var route=manufacturingRoute();
    if(route!=null) { addFacet(route); addFacet(route.intent()); }
    if(collision) {
      var parts=pieces();
      try addFacet(ExtrusionCollision.pieces(parts)) catch(e:Dynamic) {
        for(p in parts)p.close();
        throw e;
      }
      for(p in parts)p.close();
    }
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function pinnedFaces():Array<PinnedFace> return [];
  override public function fasteningFaces():Array<BoltedFace> return [];
  public function manufacturingRoute():Null<ManufacturingRoute> return null;
  public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return [];
  /** The stock pieces (no derived holes), for the collision pieces: a recorded part's recorded body. */
  public function pieces():Array<Part> {
    if (!isRecorded()) throw "Concrete made part must declare its stock pieces";
    var self = this;
    var document = cadkit.parametric.recording.DocumentBuilder.build(b -> b.output(self.record(b, Preview)));
    return [PartRecordings.solid(document)];
  }
  /** A part that records its geometry (`record`) takes it from the recording; the others still build from `pieces()`. */
  override public function geometry(detail:ComponentDetail = Preview):Part
    return isRecorded() ? super.geometry(detail) : machined(Solids.union(pieces()));
}
