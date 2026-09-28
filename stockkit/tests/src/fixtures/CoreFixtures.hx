package fixtures;

import camkit.CamContour;
import camkit.CamJob;
import cadkit.modeling.Part;
import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.tool.CutterProfile;
import oracle.ExactOracle;
import stockkit.CutMove;
import stockkit.CutMoves;
import stockkit.Stock;
import stockkit.StockAxis;
import stockkit.StockLattice;
import stockkit.StockPreview;
import stockkit.StockTimeline;

/**
  StockKit core through its Haxe API: moves the exact oracle refuses (ramps,
  helices, CamKit's ramped pocket entry) against the sampled reference, stock
  from CadKit meshes, and provenance. Oracle-backed comparisons run inside
  `OracleFixtures` and `ChainFixtures`, which already hold exact stock.
**/
class CoreFixtures {
  static inline final STOCK_X = 0.04;
  static inline final STOCK_Y = 0.02;
  static inline final STOCK_Z = 0.01;
  static inline final SAMPLES = 1500;

  public static function run():Void {
    ramps();
    helices();
    horizontalLevel();
    rampedPocket();
    meshStock();
    provenance();
    collisions();
    targetComparison();
    timelineAndPreview();
  }

  /** Scrubbing a CamKit pocket matches cutting afresh, and the preview follows it. */
  static function timelineAndPreview():Void {
    var contour = new CamContour([
      new Point3(0.01, 0.005, 0), new Point3(0.03, 0.005, 0),
      new Point3(0.03, 0.015, 0), new Point3(0.01, 0.015, 0)
    ]);
    var tool = Tool.shaped(2, 0.0, CutterProfile.flat(0.002, 0.02));
    var program = new CamJob(0.005, 12000, new Point3(0, 0, 0.005))
      .pocket(contour, tool, -0.004, 0.01, 0.0015, 0.002)
      .finish();
    var moves = CutMoves.fromProgram(program.toolpath());
    Assert.check(moves.length > 20, "pocket has enough moves to scrub");
    var lattice = new StockLattice(0, 0, -STOCK_Z + 0.0002, 0.0004, 101, 51, 25);
    function fresh(count:Int):Stock {
      var stock = Stock.box(lattice, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0);
      stock.cut(moves.slice(0, count));
      return stock;
    }
    // Every grid must scrub back, not only the Z grid the preview reads.
    function same(a:Stock, b:Stock):Bool {
      for (axis in [StockAxis.X, StockAxis.Z]) {
        var grid = lattice.grid(axis);
        var x = a.rays(0, 0, grid.countU, grid.countV, axis), y = b.rays(0, 0, grid.countU, grid.countV, axis);
        for (k in 0...x.length) {
          if (x[k].length != y[k].length) return false;
          for (n in 0...x[k].length)
            if (x[k][n].lo != y[k][n].lo || x[k][n].hi != y[k][n].hi || x[k][n].hiSource != y[k][n].hiSource)
              return false;
        }
      }
      return true;
    }
    var timeline = new StockTimeline(Stock.box(lattice, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0), moves, 7);
    var stock = timeline.stock;
    Assert.check(timeline.position == moves.length && stock.history.length == moves.length,
      "timeline starts at the program's end");
    var opColor = (move:CutMove) -> 0x10000000 * (move.opIndex % 7) + 0x80FF;
    var preview = new StockPreview(stock, BySource(opColor, 0xC0C0C0FF), 2);
    var all = preview.update();
    Assert.check(all.length == preview.chunksX * preview.chunksY, "first update meshes every chunk");
    Assert.check(preview.update().length == 0, "an unchanged stock rebuilds nothing");
    for (target in [moves.length - 3, 5, 0, 13, moves.length - 1, 13, 14]) {
      timeline.seek(target);
      var reference = fresh(target);
      Assert.check(same(stock, reference), 'seeking to move $target matches cutting afresh');
      Assert.check(stock.history.length == target, 'history is truncated to move $target');
      reference.dispose();
    }
    var before = preview.update().length;
    timeline.seek(15);
    var changed = preview.update();
    Assert.check(changed.length > 0 && changed.length < preview.chunksX * preview.chunksY,
      'one move rebuilds only the chunks it touched (${changed.length} of ${preview.chunksX * preview.chunksY}, $before before)');
    // Picking a cut surface leads back to its move and source line.
    var picked:Null<CutMove> = null;
    for (chunk in 0...preview.meshes.length) {
      var mesh = preview.meshes[chunk];
      if (mesh == null) continue;
      for (triangle in 0...mesh.triangleCount)
        if (mesh.sourceAt(triangle) >= 0) {
          picked = preview.pick(chunk, triangle);
          break;
        }
      if (picked != null) break;
    }
    Assert.check(picked != null && moves.indexOf(picked) >= 0 && picked.provenance != null,
      "a picked cut surface names its move and source span");
    // Surfaces are coloured by their move's operation.
    var mesh = preview.meshes[changed[0]];
    if (mesh == null) throw "rebuilt chunk has a mesh";
    var coloured = true;
    for (triangle in 0...mesh.triangleCount) {
      var vertex = mesh.indices.getInt32(12 * triangle);
      var rgba = (mesh.colors.get(4 * vertex) << 24) | (mesh.colors.get(4 * vertex + 1) << 16)
        | (mesh.colors.get(4 * vertex + 2) << 8) | mesh.colors.get(4 * vertex + 3);
      var source = mesh.sourceAt(triangle);
      var expected = source < 0 ? 0xC0C0C0FF : opColor(stock.history[source]);
      if (rgba != expected) coloured = false;
    }
    Assert.check(coloured, "preview colours surfaces by operation");
    timeline.dispose();
    stock.dispose();
  }

  /** A pocket deeper than the flutes: the shank rubs, and the holder too once it is low enough. */
  static function collisions():Void {
    var contour = new CamContour([
      new Point3(0.01, 0.005, 0), new Point3(0.03, 0.005, 0),
      new Point3(0.03, 0.015, 0), new Point3(0.01, 0.015, 0)
    ]);
    // 3 mm of flutes on a 6 mm shank, then a 20 mm holder 10 mm above the tip.
    var profile = CutterProfile.flat(0.004, 0.003).withShank(0.004, 0.007).withHolder(0.02, 0.03);
    var tool = Tool.shaped(3, 0.0, profile);
    var program = new CamJob(0.02, 12000, new Point3(0, 0, 0.02))
      .pocket(contour, tool, -0.008, 0.01, 0.003, 0.004)
      .finish();
    var moves = CutMoves.fromProgram(program.toolpath());
    var lattice = StockLattice.covering(0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0, 0.0005);
    var stock = Stock.box(lattice, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0);
    var report = stock.cut(moves);
    // Steps of 4 mm with 3 mm flutes leave 1 mm above the flutes at each
    // level for the shank to rub; the holder, 10 mm up, stays above the
    // stock even at 8 mm down.
    var rubbing = report.collisions();
    Assert.check(rubbing.length > 0, "a pocket deeper than the flutes rubs the shank");
    for (outcome in rubbing) {
      Assert.check(outcome.holderContact == 0, "the holder stays clear of an 8 mm pocket");
      Assert.check(lowest(outcome.move) < -0.003 - 1e-9, "only moves deeper than the flutes rub");
    }
    var totals = report.byOperation();
    var removedTotal = 0.0, rubbed = 0.0;
    for (op in totals) {
      removedTotal += op.removed;
      rubbed += op.shankContact;
    }
    Assert.near(removedTotal, report.removedVolume(), "operation totals add up to the cut", 1e-15);
    Assert.check(rubbed > 0, "operation totals carry the shank contact");
    // Cut 2 mm deeper with a short holder reach: now the holder hits.
    var short = Tool.shaped(4, 0.0, CutterProfile.flat(0.004, 0.003).withHolder(0.02, 0.03));
    var deeper = [for (move in CutMoves.fromProgram(new CamJob(0.02, 12000, new Point3(0, 0, 0.02))
      .pocket(contour, short, -0.006, 0.01, 0.003, 0.002).finish().toolpath())) move];
    var fresh = Stock.box(lattice, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0);
    var hits = [for (outcome in fresh.cut(deeper).moves) if (outcome.holderContact > 0) outcome];
    Assert.check(hits.length > 0, "a holder 3 mm above the tip hits a 6 mm pocket");
    fresh.dispose();
    stock.dispose();
  }

  /** A CamKit pocket compared with the finished part, then with a deliberate dip below the floor. */
  static function targetComparison():Void {
    var contour = new CamContour([
      new Point3(0.01, 0.005, 0), new Point3(0.03, 0.005, 0),
      new Point3(0.03, 0.015, 0), new Point3(0.01, 0.015, 0)
    ]);
    var tool = Tool.shaped(2, 0.0, CutterProfile.flat(0.002, 0.02));
    var program = new CamJob(0.005, 12000, new Point3(0, 0, 0.005))
      .pocket(contour, tool, -0.002, 0.01, 0.0015, 0.002)
      .finish();
    var moves = CutMoves.fromProgram(program.toolpath());
    var blank = Part.box(STOCK_X, STOCK_Y, STOCK_Z, Min, Min, Max);
    var pocketTool = Part.box(0.02, 0.01, 0.003, Min, Min, Min);
    var pocket = pocketTool.translated(new cadkit.modeling.Vector(0.01, 0.005, -0.002));
    var part = blank.subtract(pocket);
    var mesh = part.shape.tessellate(1e-6, 0.1);
    for (p in [blank, pocketTool, pocket, part]) p.close();
    // Rays off the pocket's round coordinates.
    var lattice = new StockLattice(0.00013, 0.00017, -STOCK_Z + 0.00019, 0.0005, 80, 40, 20);
    var target = Stock.fromMesh(lattice, mesh);
    var stock = Stock.box(lattice, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0);
    stock.cut(moves);
    var comparison = stock.compare(target);
    Assert.near(comparison.gougeVolume(), 0, "CamKit's pocket does not gouge its part", 1e-15);
    // A round tool leaves a fillet of its radius in each inside corner.
    var r = 0.001;
    Assert.near(comparison.leftoverVolume(), (4 - Math.PI) * r * r * 0.002,
      "leftover is the corner fillets", 0.25 * (4 - Math.PI) * r * r * 0.002);
    Assert.near(comparison.thickestLeftover(), 0.002, "corner leftover is the pocket's full depth", 1e-12);
    // A stray move 0.1 mm below the floor gouges, and is named.
    var span = new Provenance(99, 1, 0);
    var stray = new CutMove(tool, Path(Line(new Point3(0.015, 0.008, -0.0021),
      new Point3(0.025, 0.008, -0.0021))), Cut, 99, span);
    stock.cut([stray]);
    var gouged = stock.compare(target);
    Assert.near(gouged.deepestGouge(), 0.0001, "the dip is 0.1 mm deep", 1e-12);
    var culprits = gouged.gougingMoves(1e-6);
    Assert.check(culprits.length == 1 && stock.history[culprits[0]] == stray,
      "the gouge is attributed to the stray move");
    target.dispose();
    stock.dispose();
  }

  static function lowest(move:CutMove):Float
    return switch move.motion {
      case Path(Line(start, end)): Math.min(start.z, end.z);
      case Path(Arc(center, _, _, _)): center.z;
      case Path(Circular(center, _, _, _, _, rise)): center.z + Math.min(0, rise);
    };

  static function ramps():Void {
    var r = 0.003;
    var ramp:PathGeometry = Line(new Point3(0.008, 0.007, 0.001),
      new Point3(0.032, 0.013, -0.004));
    for (tool in [
      {name: "flat", profile: CutterProfile.flat(2 * r, 0.02)},
      {name: "ball", profile: CutterProfile.ball(2 * r, 0.02)},
      {name: "bull-nose", profile: CutterProfile.bullNose(2 * r, 0.001, 0.02)},
      {name: "V-bit", profile: CutterProfile.vee(2 * r, Math.PI / 2, 0.02)},
      {name: "tapered ball", profile: CutterProfile.taperedBall(0.002, 10 * Math.PI / 180, 2 * r, 0.02)}
    ])
      sampled(tool.profile, [ramp], '${tool.name} ramp');
    // A ramp whose floor rises: the tool climbs out of the stock.
    sampled(CutterProfile.ball(2 * r, 0.02), [Line(new Point3(0.03, 0.012, -0.005),
      new Point3(0.012, 0.008, 0.002))], "ball climbing ramp");
  }

  static function helices():Void {
    var r = 0.003;
    function helix(radius:Float, turns:Float):PathGeometry
      return Circular(new Point3(0.02, 0.01, 0.001), radius, 0.3, -2 * Math.PI * turns, XY, -0.004);
    sampled(CutterProfile.flat(2 * r, 0.02), [helix(0.002, 2)], "flat helix inside its radius");
    sampled(CutterProfile.ball(2 * r, 0.02), [helix(0.004, 1.5)], "ball helix wider than its radius");
    sampled(CutterProfile.bullNose(2 * r, 0.001, 0.02), [helix(0.0025, 3)], "bull-nose helix");
  }

  /**
    CamKit's pocket at a depth its ramp fits: the entry moves in XY and Z
    together, which the OCCT oracle cannot build.
  **/
  static function rampedPocket():Void {
    var contour = new CamContour([
      new Point3(0.01, 0.005, 0), new Point3(0.03, 0.005, 0),
      new Point3(0.03, 0.015, 0), new Point3(0.01, 0.015, 0)
    ]);
    var tool = Tool.shaped(2, 0.0, CutterProfile.flat(0.002, 0.02));
    var program = new CamJob(0.005, 12000, new Point3(0, 0, 0.005))
      .pocket(contour, tool, -0.001, 0.01, 0.0015, 0.002)
      .finish();
    var moves = CutMoves.fromProgram(program.toolpath());
    var ramped = false;
    for (move in moves) switch move.motion {
      case Path(Line(start, end)):
        if (start.z != end.z && (start.x != end.x || start.y != end.y)) ramped = true;
      case _:
    }
    Assert.check(ramped, "CamKit ramps into this pocket");
    CoreComparison.againstSampled(moves, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0, SAMPLES,
      "ramped CAM pocket");
  }

  /** Stock cast from CadKit tessellations matches the analytic solids. */
  static function meshStock():Void {
    var lattice = new StockLattice(-0.0198, -0.0098, -0.00018, 0.0004, 100, 50, 32);
    var grid = lattice.grid(Z);
    var part = Part.box(0.03, 0.015, 0.01, Center, Center, Min);
    var mesh = part.shape.tessellate(1e-5, 0.1);
    part.close();
    var fromMesh = Stock.fromMesh(lattice, mesh);
    var box = Stock.box(lattice, -0.015, -0.0075, 0, 0.015, 0.0075, 0.01);
    Assert.near(fromMesh.intervalCount(), box.intervalCount(), "box mesh covers the box's rays", 0);
    Assert.near(fromMesh.volume(), box.volume(), "box mesh volume matches the box", 1e-15);
    var inside = fromMesh.ray(40, 20);
    Assert.check(inside.length == 1 && inside[0].lo == 0 && inside[0].hi == 0.01
      && inside[0].hiNormal[2] == 1 && inside[0].loSource == Stock.ORIGINAL,
      "box mesh ray keeps exact faces, normals and source");
    fromMesh.dispose();
    box.dispose();

    // A cylinder: rays inside the tessellated circle span its full height.
    var radius = 0.008;
    var cylinder = Part.cylinder(radius, 0.012);
    var cylinderMesh = cylinder.shape.tessellate(1e-6, 0.05);
    cylinder.close();
    var stock = Stock.fromMesh(lattice, cylinderMesh);
    var rays = stock.rays(0, 0, grid.countU, grid.countV);
    var wrong = 0;
    for (j in 0...grid.countV)
      for (i in 0...grid.countU) {
        var d = Math.sqrt(grid.u(i) * grid.u(i) + grid.v(j) * grid.v(j));
        var ray = rays[j * grid.countU + i];
        // Chords of the tessellation lie at most the deflection inside the circle.
        if (d < radius - 2e-6) {
          if (ray.length != 1 || ray[0].lo != 0 || ray[0].hi != 0.012) wrong++;
        } else if (d > radius) {
          if (ray.length != 0) wrong++;
        }
      }
    Assert.check(wrong == 0, 'cylinder mesh rays ($wrong wrong)');
    stock.dispose();
  }

  /** Surfaces remember the move that made them, and rapids through stock are reported. */
  static function provenance():Void {
    var tool = Tool.shaped(1, 0.0, CutterProfile.flat(0.006, 0.02));
    var span = new Provenance(1, 1, 0);
    var moves = [
      new CutMove(tool, Path(Line(new Point3(0.01, 0.01, -0.002), new Point3(0.03, 0.01, -0.002))),
        Cut, 0, span),
      new CutMove(tool, Path(Line(new Point3(0.02, 0.002, -0.001), new Point3(0.02, 0.018, -0.001))),
        Rapid, 1, span)
    ];
    var lattice = new StockLattice(0, 0, -STOCK_Z + 0.00025, 0.0005, 81, 41, 20);
    var stock = Stock.box(lattice, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0);
    var report = stock.cut(moves);
    var removed = report.removed();
    Assert.check(removed.length == 2 && removed[0] > 0 && removed[1] > 0, "both moves remove material");
    var rapids = report.rapidContacts();
    Assert.check(rapids.length == 1 && rapids[0].source == 1 && rapids[0].move == moves[1],
      "the rapid through stock is reported");
    var floor = stock.ray(40, 20); // (0.02, 0.01): the slot, then the rapid above it
    Assert.check(floor.length == 1 && floor[0].hi == -0.002 && floor[0].hiSource == 0
      && stock.history[floor[0].hiSource] == moves[0], "slot floor points back at its move");
    var crossing = stock.ray(40, 8); // (0.02, 0.004): only the rapid
    Assert.check(crossing.length == 1 && crossing[0].hi == -0.001 && crossing[0].hiSource == 1,
      "rapid's cut points back at the rapid");
    var untouched = stock.ray(2, 2);
    Assert.check(untouched[0].hiSource == Stock.ORIGINAL, "untouched stock keeps its original surface");
    // The same moves on one thread give the same stock.
    var single = Stock.box(lattice, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0);
    single.setThreads(1);
    var again = single.cut(moves).removed();
    Assert.check(again[0] == removed[0] && again[1] == removed[1], "one thread removes the same volumes");
    Assert.check(single.volume() == stock.volume(), "one thread leaves the same stock");
    single.dispose();
    // Cutting again appends to the history.
    stock.cut([moves[0]]);
    Assert.check(stock.history.length == 3, "history grows with every cut");
    stock.dispose();
  }

  /** Cuts `geometry` and checks the stock, then sweeps each move along X and Y rays. */
  static function sampled(profile:CutterProfile, geometry:Array<PathGeometry>, label:String):Void {
    CoreComparison.againstSampled(ExactOracle.pathMoves(profile, geometry), 0, 0, -STOCK_Z,
      STOCK_X, STOCK_Y, 0, SAMPLES, label);
    horizontal(profile, geometry, label);
  }

  static function horizontal(profile:CutterProfile, geometry:Array<PathGeometry>, label:String):Void
    for (g in geometry)
      CoreComparison.sweepAgainstSampled(profile, g, 0, 0, -STOCK_Z, STOCK_X, STOCK_Y, 0.004,
        SAMPLES, '$label (horizontal rays)');

  /**
    Level lines, arcs and plunges along X and Y rays. Once X and Y grids exist
    these are checked exactly against the OCCT oracle too.
  **/
  static function horizontalLevel():Void {
    var r = 0.003;
    var moves:Array<PathGeometry> = [
      Line(new Point3(0.008, 0.006, -0.004), new Point3(0.033, 0.013, -0.004)),
      Circular(new Point3(0.02, 0.01, -0.003), 0.006, 0.4, 3.9, XY, 0),
      Circular(new Point3(0.02, 0.01, -0.003), 0.002, -1.0, -2 * Math.PI, XY, 0),
      Line(new Point3(0.02, 0.011, 0.001), new Point3(0.02, 0.011, -0.006))
    ];
    for (tool in [
      {name: "flat", profile: CutterProfile.flat(2 * r, 0.02)},
      {name: "ball", profile: CutterProfile.ball(2 * r, 0.02)},
      {name: "bull-nose", profile: CutterProfile.bullNose(2 * r, 0.001, 0.02)},
      {name: "V-bit", profile: CutterProfile.vee(2 * r, 60 * Math.PI / 180, 0.02)}
    ])
      horizontal(tool.profile, moves, '${tool.name} level moves');
  }
}
