package machinekit.hmi;

import cadkit.modeling.Footprint;
import machinekit.component.MachineComponent;
import machinekit.component.PanelMountFacet;

/**
 * A panel-mounted part at a position on a panel. The layout asks the part's PanelMountFacet for everything
 * else (footprints, heights, provenance), so there is one source for each number. Coordinates are
 * millimetres from the panel centre, x to the right, y up.
 */
class PlacedPart {
  public final id:String;
  public final part:MachineComponent;
  public final mount:PanelMountFacet;
  public final x:Float;
  public final y:Float;

  public function new(id:String, part:MachineComponent, x:Float, y:Float) {
    if (id == null || id == "") throw "A placed part needs an id";
    if (part == null) throw 'Placed part $id has no part';
    var facet = PanelMountFacet.of(part);
    if (facet == null) throw 'Placed part $id (${part.designation}) does not state a panel mount';
    if (!Math.isFinite(x) || !Math.isFinite(y)) throw 'Placed part $id has a non-finite position';
    this.id = id; this.part = part; this.mount = facet; this.x = x; this.y = y;
  }

  /** The same part at another position. */
  public function movedTo(newX:Float, newY:Float):PlacedPart return new PlacedPart(id, part, newX, newY);

  public function cutout():Footprint return mount.cutout;
  public function keepOut():Footprint return mount.keepOut;
  public function holes():Array<{x:Float, y:Float, diameter:Float}> return mount.holes;
  public function body():Footprint return mount.body;
  public function above():Float return mount.above;
  public function behind():Float return mount.behind;
  public function maxPanelThickness():Float return mount.maxPanelThickness;
  public function source():String return mount.source;
  public function massGrams():Float return part.massProperties().mass * 1000.0;
}
