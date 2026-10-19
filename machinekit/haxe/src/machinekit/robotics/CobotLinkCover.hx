package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;

/** Separate service lid or its nominal uncompressed NBR gasket. No ingress rating is implied. */
class CobotLinkCover extends MachineComponent {
  public static inline var GAP:Float = 0.25;
  public static inline var THICKNESS:Float = 2;
  public final length:Float;
  public final width:Float;
  public final gasket:Bool;
  public function new(length:Float, width:Float, gasket:Bool = false) {
    if (!Math.isFinite(length) || !Math.isFinite(width) || length < 40 || width < 30)
      throw "Link service cover has no room for its opening and four fasteners";
    super('COBOT-LINK-COVER-$length-$width-$gasket', gasket ? "Arm service-cover gasket" : "Removable arm service cover",
      gasket ? "nitrile rubber NBR" : "aluminium 6061", true);
    this.length = length; this.width = width; this.gasket = gasket;
    addConnector("back", Mount, Solids.axial(0, 0, 0));
    var index = 1;
    for (p in boltPoints(length, width)) addConnector('bolt${index++}', Mount, Solids.axial(p.x, p.y, GAP + THICKNESS));
  }
  public static function boltPoints(length:Float, width:Float):Array<Vector>
    return [for (x in [-1, 1]) for (y in [-1, 1]) new Vector(x * (length / 2 - 4), y * (width / 2 - 4), 0)];
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var body = CobotArmLink.roundedPlate(length, width, gasket ? GAP : THICKNESS, 4, gasket ? 0 : GAP);
    var cuts:Array<Part> = [];
    if (gasket) cuts.push(CobotArmLink.roundedPlate(length - 16 + 0.4, width - 16 + 0.4, GAP + 0.2, 3, -0.1));
    if (detail != Envelope) for (p in boltPoints(length, width)) cuts.push(Part.cylinderSpan(1.7, -0.1, GAP + THICKNESS + 0.1, p.x, p.y));
    return Solids.cut(body, cuts);
  }
}
