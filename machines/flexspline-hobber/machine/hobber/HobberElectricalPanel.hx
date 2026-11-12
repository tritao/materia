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

/**
 * The welded electrical panel standing on the floor beside the bed: an upright plate on a floor base, tied by two triangular
 * webs. Recorded in its own frame (the layout places it there): the upright's foot, centred along it.
 */
class HobberElectricalPanel extends HobberMadePart {
  public static inline final ELECTRICAL_HEIGHT = MineralCastBed.PLINTH_HEIGHT - machinekit.enclosure.TraySeat.DEPTH - 6;
  static final panelRecipe = new machinekit.cache.Once<ComponentType>();

  public static function panelType():ComponentType return panelRecipe.get(() -> new ComponentType("machinekit.hobbing.electrical-panel", [],
    _ -> new HobberElectricalPanel(), false, false, null, false).versioned(1).numbered(_ -> "HOBBER-ELECTRICAL-PANEL")
    .procured(_ -> Make("cut plate, welded")).requiring(_ -> ManufacturingRoute.cutPlate().requirements()));

  public function new() {
    super("electrical-panel", new ComponentValues());
    finish(false);
  }
  override public function componentType():Null<ComponentType> return panelType();
  override public function machinable():Bool return false;
  /** Cut plate, welded: the upright, the base and the webs. */
  override public function manufacturingRoute():Null<ManufacturingRoute> return ManufacturingRoute.cutPlate();

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var height = b.length("panel.height", ELECTRICAL_HEIGHT).derivedBy("bed plinth height less the tray seat depth and 6 mm");
    var upright = b.label(b.boxBetween(-210, 210, -10, 0, 0, height), "Upright");
    var base = b.label(b.boxBetween(-230, 230, -150, 50, 0, 20), "Floor base");
    // Two welded triangular webs tie the upright to its floor base.
    var parts:Array<cadkit.parametric.Feature> = [upright, base];
    for (x in [-190.0, 190.0]) parts.push(b.label(b.extrude(b.polygon([new Vector(0, 20, 0), new Vector(-130, 20, 0), new Vector(0, 200, 0)],
      new Plane(new Vector(x, 0, 0), new Vector(0, 1, 0), new Vector(1, 0, 0))), 6, null, false, true), "Web"));
    return b.label(b.unionAll(parts), "Welded panel");
  }
}
