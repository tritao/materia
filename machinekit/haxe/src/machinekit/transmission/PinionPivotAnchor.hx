package machinekit.transmission;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.assembly.AxisBuilder;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Machined C45 fixed clevis and closed-section reach arm. Manufacturing stock
 * and machining allowances are separate from this finished load-bearing shape.
 * X/Y/Z match RadialPinionCarrier. Host face is explicitly top or axial. */
class PinionPivotAnchor extends MachineComponent {
  public final hostZ:Float;
  public final topMount:Bool;
  public final bores:Array<{id:String, frame:AssemblyFrame, diameter:Float, depth:Float}>;
  public function new(hostZ:Float, topMount:Bool) {
    if (!Math.isFinite(hostZ) || hostZ < 55) throw "Pivot anchor needs a declared host beyond its clevis";
    super('PINION-PIVOT-ANCHOR-' + hostZ + '-' + topMount, "Machined pinion pivot anchor", "steel C45");
    addFacet(new ProcurementIntent(Make("machining")));
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("errorsBasis", "Assumed").setToken("source", "").setToken("ratingsBasis", "Stated")));
    lockManufacturingMaterial("steel C45");
    this.hostZ = hostZ;
    this.topMount = topMount;
    bores = [];
    for (z in [-9.0, 16.0]) bores.push({
      id: 'hinge-' + z,
      frame: AxisBuilder.orient(0, 0, z, [0.0, 1, 0], [0.0, 0, 1]),
      diameter: 10,
      depth: 8
    });
    bores.push({
      id: "spring-seat-M10",
      frame: AxisBuilder.orient(-56, -29, 7.5, [1.0, 0, 0], [0.0, -1, 0]),
      diameter: 10,
      depth: 14
    });
    bores.push({
      id: "upper-stop-M6",
      frame: AxisBuilder.orient(-56, 34, -16, [1.0, 0, 0], [0.0, -1, 0]),
      diameter: 6,
      depth: 14
    });
    bores.push({
      id: "lower-stop-M6",
      frame: AxisBuilder.orient(-56, -35, -16, [1.0, 0, 0], [0.0, 1, 0]),
      diameter: 6,
      depth: 15
    });
    // X root screws sit outside the reach-arm side walls, with room for
    // the washer and socket. Bolting on a wall centre traps the head.
    for (x in (topMount ? [-6.0, 16.0] : [-28.0, 38.0])) for (row in 0...2) bores.push(topMount ? {
      id:'host-$x-$row', frame:AxisBuilder.orient(x,
        30, hostZ + (row == 0 ? 14 : 40), [1.0, 0, 0], [0.0, -1, 0]), diameter : 8.6, depth : 6.0
    }
    : {
      id: 'host-$x-$row',
      frame: AxisBuilder.orient(x, row == 0 ? -12.0 : 48.0, hostZ - 8, [1.0, 0, 0], [0.0, 0, 1]),
      diameter : 8.6,
      depth : 8.0
    });
    for (bore in bores) addConnector(bore.id, Mount, bore.frame);
  }
  override public function hasGeometry():Bool return true;
  /** Canonical box bounds x0,x1,y0,y1,z0,z1, shared by CAD and local FE. */
  public function stockRegions():Array<Array<Float>> {
    var end = hostZ - (topMount ? 16 : 8);
    var regions = [
      [-17.0, 27, 24, 30, -9, hostZ + (topMount ? 56 : 0)],
      [-17.0, -13, 30, 90, 26, end],
      [23.0, 27, 30, 90, 26, end],
      [-17.0, 27, 90, 94, 26, end],
      [-14.0, 14, -16, 24, -9, -1],
      [-14.0, 14, -16, 24, 16, 24],
      [-80.0, -17, 24, 30, -24, 22.5],
      [-80.0, -72, -43, 30, -12, 22.5],
      [-80.0, -42, -43, -29, -7.5, 22.5],
      [-64.0, -48, 20, 34, -24, -8],
      [-80.0, -48, -35, -20, -24, -8]
    ];
    if (!topMount) regions.push([-42, 52, -30, 96, hostZ - 8, hostZ]);
    return regions;
  }
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var parts = [for (r in stockRegions()) RadialPinionCarrier.block(r[0], r[1], r[2], r[3], r[4], r[5])];
    var stock = Solids.union(parts), tools:Array<Part> = [];
    for (bore in bores) {
      var raw = Part.cylinder(bore.diameter / 2, bore.depth + 0.2);
      var frame = materia.assembly.AssemblyFrames.compose(
        bore.frame,
        materia.assembly.AssemblyFrames.translation(0, 0, -0.1)
      );
      tools.push(PosedParts.moved(raw, frame));
      raw.close();
    }
    // Threaded bores use the major-diameter collision envelope, as do the
    // standard nuts. Tap-drill/engagement checks are separate manufacturing data.
    return Solids.cut(stock, tools);
  }
}
