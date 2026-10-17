package stockkit;

import StockKitNative;
import gearkit.GearSpec;

/** Circumferential stock for gear cutting. Millimetres, radians; unlike Cartesian Stock's metres.
    Exact angular interval endpoints at cell-centred (r,z) samples. No angle raster. */
class RingStock {
  public final grid:RingGrid;
  final owner:Ownedsk_ring_stock_handle;
  var disposed = false;
  function new(grid:RingGrid, owner:Ownedsk_ring_stock_handle) { this.grid = grid; this.owner = owner; }

  public static function annulus(grid:RingGrid):RingStock {
    var g = new sk_ring_grid();
    g.set_struct_size(sk_ring_grid.size());
    g.set_radial_count(grid.radialCount); g.set_axial_count(grid.axialCount);
    g.set_radius_min(grid.radiusMin); g.set_radius_max(grid.radiusMax);
    g.set_z_min(grid.zMin); g.set_z_max(grid.zMax);
    var result = StockKitNative.sk_ring_stock_create(g);
    check(result.status, "annulus");
    return new RingStock(grid, result.out_stock);
  }

  /** Cylindrical tooth band plus a continuous wall. wallThickness is measured BELOW
      the external tooth roots (outside internal roots), not from the blank's tip circle. */
  public static function fromGearBlank(spec:GearSpec, wallThickness:Float,
      radialSpacing:Float = 0.005, axialSpacing:Float = 0.1):RingStock {
    if (!Math.isFinite(wallThickness) || wallThickness <= 0) throw "gear blank wall must be finite and positive";
    var inner = spec.internal ? spec.tipRadius() : spec.rootRadius() - wallThickness;
    var outer = spec.internal ? spec.rootRadius() + wallThickness : spec.tipRadius();
    return annulus(new RingGrid(inner, outer, 0, spec.faceWidth, radialSpacing, axialSpacing));
  }

  public function setThreads(threads:Int):Void {
    alive();
    if (threads < 0 || threads > 128) throw "ring thread count must be in [0,128]";
    check(StockKitNative.sk_ring_stock_set_threads(owner.borrow(), threads), "setThreads");
  }
  public function circle(radialIndex:Int, axialIndex:Int):Array<RingInterval> {
    alive(); grid.radius(radialIndex); grid.z(axialIndex);
    var result = StockKitNative.sk_ring_stock_read_circle(owner.borrow(), radialIndex, axialIndex);
    check(result.status, "circle");
    return [for (v in result.out_intervals) new RingInterval(v.get_lo(), v.get_hi(),
      v.get_lo_gash(), v.get_lo_pass(), v.get_hi_gash(), v.get_hi_pass())];
  }
  /** Entire batch is validated before native mutation; array order determines overlapping sources. */
  public function cutRegion(cuts:Array<RingCut>):Void {
    alive();
    var native:Array<sk_ring_cut> = [];
    for (c in cuts) {
      grid.radius(c.radialIndex);
      if (c.axialFirst >= grid.axialCount || c.axialCount > grid.axialCount - c.axialFirst)
        throw "ring cut axial range is outside grid";
      var v = new sk_ring_cut();
      v.set_struct_size(sk_ring_cut.size());
      v.set_radial_index(c.radialIndex); v.set_axial_first(c.axialFirst); v.set_axial_count(c.axialCount);
      v.set_start(c.start); v.set_sweep(c.sweep); v.set_gash(c.gash); v.set_pass(c.pass);
      native.push(v);
    }
    check(StockKitNative.sk_ring_stock_cut_region(owner.borrow(), native), "cutRegion");
  }
  /** Exact analytic intersection with a transverse planar slab, through the full Z range. */
  public function cutSlab(normalAngle:Float, offsetMin:Float, offsetMax:Float, gash:Int, pass:Int):Void {
    alive();
    if (gash < 0 || pass < 0) throw "slab sources must be non-negative";
    check(StockKitNative.sk_ring_stock_cut_slab(owner.borrow(), normalAngle, offsetMin, offsetMax, gash, pass), "cutSlab");
  }
  public function snapshot():RingSnapshot {
    alive();
    var result = StockKitNative.sk_ring_stock_snapshot(owner.borrow());
    check(result.status, "snapshot");
    return new RingSnapshot(result.out_snapshot);
  }
  public function restore(snapshot:RingSnapshot):Void {
    alive();
    check(StockKitNative.sk_ring_stock_restore(owner.borrow(), snapshot.borrow()), "restore");
  }
  public function bytes():Float {
    alive(); var result = StockKitNative.sk_ring_stock_get_info(owner.borrow());
    check(result.status, "info"); return haxe.Int64.toFloat(result.out_info.get_bytes());
  }
  public function intervalCount():Float {
    alive(); var result = StockKitNative.sk_ring_stock_get_info(owner.borrow());
    check(result.status, "info"); return haxe.Int64.toFloat(result.out_info.get_interval_count());
  }
  public function compareStock(target:RingStock):RingComparison {
    alive(); target.alive();
    var result = StockKitNative.sk_ring_stock_compare(owner.borrow(), target.owner.borrow(), grid.radialCount * grid.axialCount);
    check(result.status, "compareStock");
    return new RingComparison(grid, [for (v in result.out_comparisons) v.get_leftover()],
      [for (v in result.out_comparisons) v.get_gouge()]);
  }
  /** Geometric comparison against the complete GearKit tooth, root and tip fillets included. */
  public function compare(target:GearSpec):RingComparison {
    alive();
    var reference = RingGearTarget.create(grid, target);
    try {
      var result = compareStock(reference);
      reference.dispose(); return result;
    } catch (error:Dynamic) { reference.dispose(); throw error; }
  }
  /** Boundary of the piecewise-cell solid. Angular chords only affect display.
      Streams use mm; convert to metres when passing to the existing app scene. */
  public function contour(maxAngle:Float = 0.05):StockMesh {
    alive();
    var created = StockKitNative.sk_ring_stock_contour(owner.borrow(), maxAngle);
    check(created.status, "contour");
    var mesh = created.out_mesh;
    try {
      var handle = mesh.borrow();
      var info = StockKitNative.sk_mesh_get_info(handle);
      var positions = StockKitNative.sk_mesh_copy_positions(handle);
      var normals = StockKitNative.sk_mesh_copy_normals(handle);
      var indices = StockKitNative.sk_mesh_copy_indices(handle);
      var colors = StockKitNative.sk_mesh_copy_colors(handle);
      var sources = StockKitNative.sk_mesh_copy_triangle_sources(handle);
      for (status in [info.status, positions.status, normals.status, indices.status, colors.status, sources.status])
        check(status, "contour streams");
      var result = new StockMesh(info.out_info.get_vertex_count(), info.out_info.get_triangle_count(),
        positions.output, normals.output, indices.output, colors.output, sources.output);
      mesh.close(); return result;
    } catch (error:Dynamic) { mesh.close(); throw error; }
  }
  @:allow(stockkit.hobbing.HobbingSweep)
  function borrowForSweep():sk_ring_stock_handle { alive(); return owner.borrow(); }
  public function dispose():Void { if (!disposed) { disposed = true; owner.close(); } }
  function alive():Void { if (disposed) throw "ring stock has been disposed"; }
  static function check(status:Int, operation:String):Void {
    if (status != StockKitNativeConstants.SK_OK) throw 'ring.$operation failed ($status)';
  }
}
