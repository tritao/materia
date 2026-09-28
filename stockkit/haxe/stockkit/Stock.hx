package stockkit;

import StockKitNative;
import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.tool.CutterProfile;
import haxe.io.Bytes;

/**
  In-process stock on a Z dexel grid, held by StockKit core. Each ray keeps
  sorted material intervals whose ends record the outward normal and the move
  that made them.

  Only the cutting zone of each tool removes material; its shank and holder
  are swept too, and their overlap with the stock is reported per move. Moves
  cut in order; every cut move is kept in `history` and its index there is
  the source recorded on the surfaces it made.
**/
class Stock {
  /** Source of surfaces no move has touched. */
  public static inline final ORIGINAL = -1;

  public final grid:StockGrid;
  public final history:Array<CutMove> = [];
  final owner:Ownedsk_stock_handle;
  final tools:Array<NativeTool> = [];
  var disposed = false;

  function new(grid:StockGrid, owner:Ownedsk_stock_handle) {
    this.grid = grid;
    this.owner = owner;
  }

  /** A solid box; rays on its sides count as inside. */
  public static function box(grid:StockGrid, minX:Float, minY:Float, minZ:Float,
      maxX:Float, maxY:Float, maxZ:Float):Stock {
    var box = new sk_box();
    box.set_struct_size(sk_box.size());
    box.set_min(0, minX);
    box.set_min(1, minY);
    box.set_min(2, minZ);
    box.set_max(0, maxX);
    box.set_max(1, maxY);
    box.set_max(2, maxZ);
    var created = StockKitNative.sk_stock_create_box(nativeGrid(grid), box);
    check(created.status, "stock.box");
    return new Stock(grid, created.out_stock);
  }

  /**
    Stock from a closed triangle mesh with outward, counter-clockwise faces:
    `positions` holds xyz triples and `indices` vertex triples. A ray through
    a shared edge or vertex is counted exactly once.
  **/
  public static function fromTriangles(grid:StockGrid, positions:Array<Float>,
      indices:Array<Int>):Stock {
    var created = StockKitNative.sk_stock_create_mesh(nativeGrid(grid), positions,
      indices);
    check(created.status, "stock.fromTriangles");
    return new Stock(grid, created.out_stock);
  }

  /** Stock from a CadKit tessellation (vertex and index buffers in native layout). */
  public static function fromMesh(grid:StockGrid, mesh:cadkit.Mesh):Stock {
    var positions = [for (k in 0...mesh.vertexCount * 3) mesh.vertices.getDouble(8 * k)];
    var indices = [for (k in 0...mesh.indexCount) mesh.indices.getInt32(4 * k)];
    return fromTriangles(grid, positions, indices);
  }

  /**
    Threads the core cuts with: 0 (the default) means one per hardware
    thread. The result is bit-identical for any count.
  **/
  public function setThreads(threads:Int):Void {
    alive();
    if (threads < 0) throw "stock thread count cannot be negative";
    check(StockKitNative.sk_stock_set_threads(owner.borrow(), threads), "stock.setThreads");
  }

  /**
    Removes the material each move's flutes sweep, in order, and measures
    the stock each move's shank and holder overlap after its cut.
  **/
  public function cut(moves:Array<CutMove>):CutReport {
    alive();
    var outcomes:Array<MoveOutcome> = [];
    var start = 0;
    while (start < moves.length) {
      // One native call per run of moves with the same tool.
      var end = start + 1;
      while (end < moves.length && moves[end].tool == moves[start].tool) end++;
      var first = history.length;
      var native = [for (index in start...end) nativeMove(moves[index], first + index - start)];
      var result = StockKitNative.sk_stock_cut(owner.borrow(), toolFor(moves[start].tool).borrow(),
        native, native.length);
      check(result.status, "stock.cut");
      for (index in start...end) {
        var outcome = result.out_results[index - start];
        history.push(moves[index]);
        outcomes.push(new MoveOutcome(first + index - start, moves[index], outcome.get_removed(),
          outcome.get_contact(StockKitNativeConstants.SK_ZONE_SHANK),
          outcome.get_contact(StockKitNativeConstants.SK_ZONE_HOLDER)));
      }
      start = end;
    }
    return new CutReport(outcomes);
  }

  /**
    Compares this stock with `target` ray by ray; `target` must use the same
    grid, for example `Stock.fromMesh(stock.grid, part.shape.tessellate(...))`.
  **/
  public function compare(target:Stock):StockComparison {
    alive();
    target.alive();
    var result = StockKitNative.sk_stock_compare(owner.borrow(), target.owner.borrow(), 0, 0,
      grid.countX, grid.countY, grid.countX * grid.countY);
    check(result.status, "stock.compare");
    var rays = result.out_comparisons;
    return new StockComparison(grid, [for (ray in rays) ray.get_leftover()],
      [for (ray in rays) ray.get_gouge()], [for (ray in rays) ray.get_largest_leftover()],
      [for (ray in rays) ray.get_largest_gouge()],
      [for (ray in rays) {
        var source = ray.get_gouge_source();
        source == StockKitNativeConstants.SK_SOURCE_NONE || source == StockKitNativeConstants.SK_SOURCE_STOCK
          ? -1 : source;
      }]);
  }

  /** Captures the stock and its history; see `StockSnapshot`. */
  public function snapshot():StockSnapshot {
    alive();
    var created = StockKitNative.sk_stock_snapshot(owner.borrow());
    check(created.status, "stock.snapshot");
    return new StockSnapshot(grid, history.copy(), created.out_snapshot);
  }

  /** Returns the stock and its history to `snapshot`, which must share its grid. */
  public function restore(snapshot:StockSnapshot):Void {
    alive();
    check(StockKitNative.sk_stock_restore(owner.borrow(), snapshot.borrow()), "stock.restore");
    history.resize(0);
    for (move in snapshot.history) history.push(move);
  }

  /** Tiles across (x) and down (y); tile (tx, ty) holds rays from (tx, ty) times the tile size. */
  public function tilesX():Int
    return info().get_tiles(0);

  public function tilesY():Int
    return info().get_tiles(1);

  /** Each tile's revision, row by row; it changes whenever the tile's rays change. */
  public function tileRevisions():Array<haxe.Int64> {
    alive();
    var count = tilesX() * tilesY();
    var result = StockKitNative.sk_stock_read_revisions(owner.borrow(), count);
    check(result.status, "stock.revisions");
    return result.out_revisions;
  }

  /**
    A display mesh of `tilesWide` by `tilesHigh` tiles from tile (tileX,
    tileY). With `merge`, equal faces along rows are joined (not with
    `rayColors`). Colours come from `rayColors` (one 0xRRGGBBAA per ray of the
    grid) if given, else from `palette` indexed by source with `original` for
    untouched stock.
  **/
  public function mesh(tileX:Int, tileY:Int, tilesWide:Int, tilesHigh:Int, merge:Bool,
      bottoms:Bool, ?palette:Array<Int>, original:Int = -1, ?rayColors:Array<Int>):StockMesh {
    alive();
    var flags = (merge ? StockKitNativeConstants.SK_MESH_MERGE : 0)
      | (bottoms ? StockKitNativeConstants.SK_MESH_BOTTOMS : 0);
    var created = StockKitNative.sk_stock_mesh(owner.borrow(), tileX, tileY, tilesWide, tilesHigh, flags);
    check(created.status, "stock.mesh");
    var native = created.out_mesh;
    try {
      var handle = native.borrow();
      if (rayColors != null)
        check(StockKitNative.sk_mesh_color_by_ray(handle, rayColors), "mesh.colorByRay");
      else if (palette != null)
        check(StockKitNative.sk_mesh_color_by_source(handle, palette, original, original), "mesh.colorBySource");
      var info = StockKitNative.sk_mesh_get_info(handle);
      check(info.status, "mesh.info");
      var positions = StockKitNative.sk_mesh_copy_positions(handle);
      var normals = StockKitNative.sk_mesh_copy_normals(handle);
      var indices = StockKitNative.sk_mesh_copy_indices(handle);
      var colors = StockKitNative.sk_mesh_copy_colors(handle);
      var sources = StockKitNative.sk_mesh_copy_triangle_sources(handle);
      for (status in [positions.status, normals.status, indices.status, colors.status, sources.status])
        check(status, "mesh.copy");
      var result = new StockMesh(info.out_info.get_vertex_count(), info.out_info.get_triangle_count(),
        positions.output, normals.output, indices.output, colors.output, sources.output);
      native.close();
      return result;
    } catch (error:Dynamic) {
      native.close();
      throw error;
    }
  }

  /** The intervals along ray (i, j), bottom to top. */
  public function ray(i:Int, j:Int):Array<StockInterval>
    return rays(i, j, 1, 1)[0];

  /** Intervals of every ray in a block, i fastest. */
  public function rays(i0:Int, j0:Int, ni:Int, nj:Int):Array<Array<StockInterval>> {
    alive();
    if (i0 < 0 || j0 < 0 || ni < 0 || nj < 0 || i0 + ni > grid.countX || j0 + nj > grid.countY)
      throw "stock ray block is outside the grid";
    var counts = StockKitNative.sk_stock_read_counts(owner.borrow(), i0, j0, ni, nj, ni * nj);
    check(counts.status, "stock.readCounts");
    var total = 0;
    for (count in counts.out_counts) total += count;
    var read = StockKitNative.sk_stock_read_intervals(owner.borrow(), i0, j0, ni, nj, total);
    check(read.status, "stock.readIntervals");
    var result:Array<Array<StockInterval>> = [];
    var at = 0;
    for (count in counts.out_counts) {
      result.push([for (k in 0...count) interval(read.out_intervals[at + k])]);
      at += count;
    }
    return result;
  }

  /** Material volume: interval lengths times spacing squared. */
  public function volume():Float
    return info().get_volume();

  public function intervalCount():Float
    return haxe.Int64.toFloat(info().get_interval_count());

  /** Rays whose sweep the core evaluated over every cut so far. */
  public function raysTested():Float
    return haxe.Int64.toFloat(info().get_rays_tested());

  /** Bytes the core holds for this stock. */
  public function bytes():Float
    return haxe.Int64.toFloat(info().get_bytes());

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    for (tool in tools) tool.dispose();
    owner.close();
  }

  /**
    The material one move of `tool` would sweep along the +Z ray through
    (x, y), for checking the core against references. Uses the tool's
    cutting zone, as `cut` does.
  **/
  public static function sweep(tool:Tool, geometry:PathGeometry, x:Float,
      y:Float):Array<StockInterval> {
    var cutting = new NativeTool(tool);
    try {
      var move = nativeMove(new CutMove(tool, Path(geometry), Cut, 0,
        new toolpathkit.path.Provenance(1, 1, 0)), 0);
      var counted = StockKitNative.sk_sweep_count_ray(cutting.borrow(), move,
        StockKitNativeConstants.SK_AXIS_Z, x, y);
      check(counted.status, "sweep.count");
      var read = StockKitNative.sk_sweep_read_ray(cutting.borrow(), move,
        StockKitNativeConstants.SK_AXIS_Z, x, y, counted.out_count);
      check(read.status, "sweep.read");
      var result = [for (native in read.out_intervals) interval(native)];
      cutting.dispose();
      return result;
    } catch (error:Dynamic) {
      cutting.dispose();
      throw error;
    }
  }

  function toolFor(tool:Tool):NativeTool {
    for (known in tools) if (known.tool == tool) return known;
    var created = new NativeTool(tool);
    tools.push(created);
    return created;
  }

  function info():sk_stock_info {
    alive();
    var result = StockKitNative.sk_stock_get_info(owner.borrow());
    check(result.status, "stock.info");
    return result.out_info;
  }

  function alive():Void {
    if (disposed) throw "stock has been disposed";
  }

  static function nativeGrid(grid:StockGrid):sk_grid {
    var native = new sk_grid();
    native.set_struct_size(sk_grid.size());
    native.set_axis(StockKitNativeConstants.SK_AXIS_Z);
    native.set_origin(0, grid.originX);
    native.set_origin(1, grid.originY);
    native.set_spacing(grid.spacing);
    native.set_count(0, grid.countX);
    native.set_count(1, grid.countY);
    native.set_tile_size(0);
    return native;
  }

  static function nativeMove(move:CutMove, source:Int):sk_move {
    var native = new sk_move();
    native.set_struct_size(sk_move.size());
    native.set_source(source);
    native.set_flags(move.rapid ? StockKitNativeConstants.SK_MOVE_RAPID : 0);
    switch move.motion {
      case Path(Line(start, end)):
        native.set_kind(StockKitNativeConstants.SK_MOVE_LINE);
        native.set_start(0, start.x);
        native.set_start(1, start.y);
        native.set_start(2, start.z);
        native.set_end(0, end.x);
        native.set_end(1, end.y);
        native.set_end(2, end.z);
      case Path(Arc(center, radius, startAngle, sweepAngle)):
        arc(native, center, radius, startAngle, sweepAngle, 0.0);
      case Path(Circular(center, radius, startAngle, sweepAngle, XY, rise)):
        arc(native, center, radius, startAngle, sweepAngle, rise);
      case Path(Circular(_, _, _, _, _, _)):
        throw "stock cuts support XY arcs and helices only; XZ and YZ arcs need tilted sweeps";
    }
    return native;
  }

  static function arc(native:sk_move, center:toolpathkit.path.Point3, radius:Float,
      startAngle:Float, sweepAngle:Float, rise:Float):Void {
    native.set_kind(StockKitNativeConstants.SK_MOVE_ARC);
    native.set_center(0, center.x);
    native.set_center(1, center.y);
    native.set_center(2, center.z);
    native.set_radius(radius);
    native.set_start_angle(startAngle);
    native.set_sweep(sweepAngle);
    native.set_rise(rise);
  }

  static function interval(native:sk_interval):StockInterval {
    return new StockInterval(native.get_lo(), native.get_hi(),
      source(native.get_lo_source()), source(native.get_hi_source()),
      [for (k in 0...3) native.get_lo_normal(k)],
      [for (k in 0...3) native.get_hi_normal(k)]);
  }

  static function source(native:Int):Int
    return native == StockKitNativeConstants.SK_SOURCE_STOCK ? ORIGINAL : native;

  static function check(status:Int, operation:String):Void {
    if (status != StockKitNativeConstants.SK_OK)
      throw '$operation failed with StockKit error $status';
  }
}

/** A tool registered with the core: its flutes cut, its shank and holder are checked for contact. */
private class NativeTool {
  public final tool:Tool;
  final owner:Ownedsk_tool_handle;
  var disposed = false;

  public function new(tool:Tool) {
    this.tool = tool;
    var segments:Array<sk_profile_segment> = [];
    for (segment in tool.profile().segments) {
      var native = new sk_profile_segment();
      native.set_struct_size(sk_profile_segment.size());
      native.set_zone(switch CutterProfile.zoneOf(segment) {
        case Cutting: StockKitNativeConstants.SK_ZONE_CUTTING;
        case Shank: StockKitNativeConstants.SK_ZONE_SHANK;
        case Holder: StockKitNativeConstants.SK_ZONE_HOLDER;
      });
      switch segment {
        case Line(r0, z0, r1, z1, _):
          native.set_kind(StockKitNativeConstants.SK_SEGMENT_LINE);
          native.set_r0(r0);
          native.set_z0(z0);
          native.set_r1(r1);
          native.set_z1(z1);
        case Arc(cr, cz, r0, z0, r1, z1, _):
          native.set_kind(StockKitNativeConstants.SK_SEGMENT_ARC);
          native.set_r0(r0);
          native.set_z0(z0);
          native.set_r1(r1);
          native.set_z1(z1);
          native.set_center_r(cr);
          native.set_center_z(cz);
      }
      segments.push(native);
    }
    var created = StockKitNative.sk_tool_create(segments);
    if (created.status != StockKitNativeConstants.SK_OK)
      throw 'tool ${tool.number} profile rejected by StockKit core (error ${created.status})';
    owner = created.out_tool;
  }

  public function borrow():sk_tool_handle
    return owner.borrow();

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }
}
