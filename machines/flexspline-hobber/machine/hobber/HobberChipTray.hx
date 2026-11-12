package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Part;
import machinekit.motion.ErobActuatorPart;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ManufacturingRoute.MachinedFace;
import machinekit.structural.GroundStock;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** GE-chip-tray; owns its stock, interfaces and manufacturing route. */
/**
 * The removable chip skirt and tray around the C table: a revolved skirt (5 mm radial face clearance, 15 mm housing clearance,
 * 2 mm sheet; the lower skirt covers the full table height) and a flat tray with side walls running forward to the work well.
 * Recorded in its own frame, the table axis at the table seat (`frameFor`). Its inputs are the table it fits.
 */
class HobberChipTray extends HobberMadePart {
  static final trayRecipe = new machinekit.cache.Once<ComponentType>();

  static function inputs():Array<ComponentParameter> return [P.length("tableFaceDiameter", 200), P.length("tableHousingDiameter", 260),
    P.length("tableFaceHeight", 228), P.length("tableSeatHeight", 18)];

  public static function trayType():ComponentType return trayRecipe.get(() -> new ComponentType("machinekit.hobbing.chip-tray", inputs(),
    v -> new HobberChipTray(v), false, false, null, false).versioned(1).numbered(_ -> "HOBBER-GE-CHIP-TRAY")
    .procured(_ -> Make("steel fabrication and machining")).requiringInputs(["tableFaceDiameter", "tableHousingDiameter", "tableFaceHeight", "tableSeatHeight"]));

  /** The tray's own frame in the part datum frame: on the table axis at the table seat. */
  public static function frameFor(tableSeatHeight:Float):AssemblyFrame return F.translation(0, 0, tableSeatHeight);

  public function new(?values:ComponentValues) {
    super("GE-chip-tray", P.resolve(inputs(), values));
    finish(false);
  }
  override public function componentType():Null<ComponentType> return trayType();
  override public function machinable():Bool return false;

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var faceD = b.length("tableFaceDiameter", values().number("tableFaceDiameter")).value;
    var housingD = b.length("tableHousingDiameter", values().number("tableHousingDiameter")).value;
    var faceH = b.length("tableFaceHeight", values().number("tableFaceHeight")).value;
    var seatH = b.length("tableSeatHeight", values().number("tableSeatHeight")).value;
    // Heights from the seat (this frame's z 0).
    var lip = faceD / 2 + 5, skirt = Math.max(housingD / 2 + 15, lip + 5);
    var top = faceH - 3 - seatH, bottom = 0.0, shoulder = Math.max(bottom + 5, top - 61);
    var section = [new Vector(lip + 2, top, 0), new Vector(skirt + 2, shoulder, 0), new Vector(skirt + 2, bottom, 0), new Vector(skirt, bottom, 0),
      new Vector(skirt, shoulder, 0), new Vector(lip, top - 2, 0)];
    var ring = b.label(b.revolve(b.polygon(section, Plane.XZ()), new Vector(0, 0, 0), Vector.Z(), Math.PI * 2), "Skirt");
    var front = MineralCastBed.WORK_WELL_CENTRE_X - MineralCastBed.WORK_WELL_WIDTH / 2 + MineralCastBed.CHIP_TRAY_WELL_CLEARANCE_MM;
    var z = (native:Float) -> native - seatH;
    var floor = b.boxBetween(front, 90, -170, 170, z(135), z(139));
    var left = b.boxBetween(front, 90, -174, -170, z(135), z(164)), right = b.boxBetween(front, 90, 170, 174, z(135), z(164));
    var opening = b.boxBetween(-skirt, skirt, -skirt, skirt, z(134), z(165));
    var tray = b.label(b.cutAll(b.unionAll([floor, left, right]), [opening], ["skirt opening"]), "Tray");
    return b.label(b.unionAll([ring, tray]), "Chip tray");
  }
}
