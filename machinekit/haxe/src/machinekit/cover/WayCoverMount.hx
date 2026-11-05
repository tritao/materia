package machinekit.cover;

import cadkit.modeling.Part;
import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.ThreadMaterial;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A round hole through a cover mount, in the mount's frame: `x` across, `y` up from the surface the cover stands on, `d` its diameter, mm. */
typedef MountBore = {x:Float, y:Float, d:Float};

/** A rectangular cut-out through a cover mount (`y0` of 0 or less cuts to the surface), mm. */
typedef MountCut = {x0:Float, x1:Float, y0:Float, y1:Float};

/**
 * The plate a way cover's flange is bolted to where the machine has no face of its own: an end wall behind the folded cover or a carrier
 * on the moving member. Frame: the mounting face at z = 0 with the plate behind it (z from `-thickness` to 0), `x` across, `y` up from the
 * surface the cover stands on, the same frame a cover's flange has (`WayCoverSection`), so a mount and the flange it holds are placed with
 * one pose. The face is a tapped, drilled-to-suit receiving face over the whole plate: the flange's holes decide where it is tapped.
 * Bores let a screw journal or a shaft through; cuts clear the rails and blocks a wall stands over.
 */
class WayCoverMount extends MachineComponent {
  public final halfWidth:Float;
  public final height:Float;
  /** How far the plate reaches below the surface the cover stands on (a wall on a bridge lower than the surface), mm. */
  public final below:Float;
  public final thickness:Float;
  public final bores:Array<MountBore>;
  public final cuts:Array<MountCut>;

  public function new(drawing:String, halfWidth:Float, height:Float, thickness:Float, ?bores:Array<MountBore>, ?cuts:Array<MountCut>, below:Float = 0) {
    super('WAYCOVER-MOUNT-${Dimension.format(2 * halfWidth)}x${Dimension.format(height)}x${Dimension.format(thickness)}-${cutKey(bores, cuts)}' + (below > 0 ? '-D${Dimension.format(below)}' : ''),
      "Way cover mounting plate", "steel", true);
    if (!(halfWidth > 0) || !(height > 0) || !(thickness >= 8)) throw "A cover mount needs a width, a height and at least 8 mm of steel to tap";
    declareDrawingNumber(drawing);
    addFacet(new machinekit.component.ProcurementIntent(machinekit.component.Procurement.Make("laser cutting and tapping")));
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.halfWidth = halfWidth;
    this.height = height;
    this.below = below;
    this.thickness = thickness;
    this.bores = bores == null ? [] : bores.copy();
    this.cuts = cuts == null ? [] : cuts.copy();
  }

  static function cutKey(bores:Null<Array<MountBore>>, cuts:Null<Array<MountCut>>):String {
    var parts:Array<String> = [];
    if (bores != null) for (b in bores) parts.push('B${Dimension.format(b.x)},${Dimension.format(b.y)},${Dimension.format(b.d)}');
    if (cuts != null) for (c in cuts) parts.push('C${Dimension.format(c.x0)},${Dimension.format(c.x1)},${Dimension.format(c.y0)},${Dimension.format(c.y1)}');
    return parts.length == 0 ? "plain" : machinekit.component.BomRevision.fingerprint(parts.join("|"), 1).substr(0, 8);
  }

  override public function revisionFingerprint():Null<String> {
    var inputs = [drawingNumber, materialSpec(), Dimension.format(halfWidth), Dimension.format(height), Dimension.format(below), Dimension.format(thickness)];
    for (b in bores) inputs.push('B${Dimension.format(b.x)},${Dimension.format(b.y)},${Dimension.format(b.d)}');
    for (c in cuts) inputs.push('C${Dimension.format(c.x0)},${Dimension.format(c.x1)},${Dimension.format(c.y0)},${Dimension.format(c.y1)}');
    return machinekit.component.BomRevision.fingerprint(inputs.join("|"), 1);
  }

  /** The face the cover's flange is bolted to: the whole front of the plate, tapped to suit the flange's holes. */
  override public function fasteningFaces():Array<BoltedFace> return [BoltedFace.drilledToSuit(
    "cover",
    BoltedFace.frameOf({x: 0, y: (height - below) / 2, z: 0}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
    2 * halfWidth,
    height + below,
    thickness - 2,
    thickness,
    ThreadMaterial.Steel
  )];

  override public function machinable():Bool return true;

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var body = Part.box(2 * halfWidth, height + below, thickness).translated(new cadkit.modeling.Vector(0, (height - below) / 2, -thickness));
    var tools:Array<Part> = [];
    for (b in bores) tools.push(Part.cylinderSpan(b.d / 2, -thickness - 1, 1, b.x, b.y));
    for (c in cuts) {
      var tool = Part.box(c.x1 - c.x0, c.y1 - c.y0, thickness + 2).translated(new cadkit.modeling.Vector((c.x0 + c.x1) / 2, (c.y0 + c.y1) / 2, -thickness - 1));
      tools.push(tool);
    }
    return machined(Solids.cut(Solids.named(body, "plate"), tools));
  }
}
