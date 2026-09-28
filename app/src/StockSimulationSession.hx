package app;

import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import camkit.CamJob;
import cnckit.CncCompiler;
import cnckit.CncMachine;
import cnckit.CncWriter;
import toolpathkit.path.Point3;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.setup.Setup;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.Tool;
import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import stockkit.CutMove;
import stockkit.CutMoves;
import stockkit.MoveOutcome;
import stockkit.Stock;
import stockkit.StockColoring;
import stockkit.StockLattice;
import stockkit.StockTimeline;

/**
  A machining program cut from a block of stock by StockKit, for the editor's
  stock-simulation objects. The app has no CAM workspace yet, so the program
  is a built-in demo sized to the block: an island pocket and four drilled
  holes, written to G-code by CamKit and compiled back by CncKit, so every
  simulated surface leads to a real G-code line.

  The block spans the object's width, height and depth, centred on its
  origin; the program's work origin is the block's top corner at minimum x
  and y.
**/
class StockSimulationSession {
  public static inline final KIND = "stock-simulation";
  /** Ray spacing of the simulation: 0.25 mm. */
  public static inline final SPACING = 0.00025;
  static final OPERATION_COLOURS = [0x3A86C8FF, 0xE0A030FF, 0x5DB36AFF, 0xC0504DFF, 0x8E6CC0FF];
  static inline final UNTOUCHED = 0xC8C8C8FF;
  static inline final ON_TARGET = 0x6FB66FFF;
  static inline final LEFTOVER = 0xE0C040FF;
  static inline final GOUGE = 0xD04040FF;
  /** Deviation below this counts as on target, in metres. */
  static inline final TOLERANCE = 0.00001;

  public final width:Float;
  public final height:Float;
  public final depth:Float;
  /** The program as G-code, one entry per line. */
  public final gcode:Array<String>;
  public final moves:Array<CutMove>;
  public final timeline:StockTimeline;
  /** "operation" or "deviation". */
  public var colorBy(default, null):String = "operation";
  public var picked(default, null):Null<CutMove> = null;
  final operations:Array<Int>;
  final target:Stock;
  var disposed = false;

  public function new(width:Float, height:Float, depth:Float) {
    if (!(width > 0.01) || !(height > 0.01) || !(depth > 0.003) || width > 0.5 || height > 0.5 || depth > 0.2)
      throw "Stock simulation needs a block between 10 × 10 × 3 mm and 500 × 500 × 200 mm";
    this.width = width;
    this.height = height;
    this.depth = depth;
    var program = demoProgram();
    var machine = new CncMachine("work", "x", "y", "z", 0.2);
    for (tool in program.program.tools) machine.toolLibrary.set(tool);
    var setup = new Setup(0, width, 0, height, 0, -depth, program.safeZ);
    var text = CncWriter.write(program.program.ops, setup, machine);
    gcode = text.split("\n");
    var compiled = new CncCompiler(machine).compileDetailed(text);
    if (compiled.diagnostics.length > 0)
      throw "Stock simulation program does not compile: " + compiled.diagnostics[0];
    if (compiled.ops.length != program.program.ops.length)
      throw "Stock simulation G-code does not match its CAM operations";
    // Compiled ops follow the CAM ops one for one; each CAM op records its operation number.
    operations = [for (op in program.program.ops) {
      var index = provenanceOf(op).operationIndex;
      index == null ? 0 : index;
    }];
    // Work coordinates put the block's top corner at the origin; the object centres the
    // block, so object = work - (width / 2, height / 2, -depth / 2).
    moves = CutMoves.fromProgram(new ToolpathProgram(compiled.ops, machine.toolLibrary),
      new Point3(width / 2, height / 2, -depth / 2));
    // Rays at cell centres, so none lies exactly on the program's millimetre-round walls,
    // where cut stock and the finished part's mesh could disagree about which side it is on.
    var lattice = StockLattice.covering(-width / 2, -height / 2, -depth / 2, width / 2, height / 2, depth / 2,
      SPACING);
    timeline = new StockTimeline(Stock.box(lattice, -width / 2, -height / 2, -depth / 2, width / 2,
      height / 2, depth / 2), moves, 250);
    var part = finishedPart(program);
    try {
      target = Stock.fromMesh(lattice, part.shape.tessellate(1e-6, 0.1));
    } catch (error:Dynamic) {
      part.close();
      timeline.stock.dispose();
      throw error;
    }
    part.close();
  }

  public function moveCount():Int return moves.length;

  public function position():Int return timeline.position;

  public function seek(position:Int):Void {
    timeline.seek(Std.int(Math.max(0, Math.min(moves.length, position))));
    if (picked != null && timeline.stock.history.indexOf(picked) < 0) picked = null;
  }

  public function setColorBy(mode:String):Void {
    if (mode != "operation" && mode != "deviation") throw 'Unknown stock colouring "$mode"';
    colorBy = mode;
  }

  /** The stock as it is now, in the object's frame, coloured by the current mode. */
  public function geometry():GeometryData {
    var stock = timeline.stock;
    var mesh = colorBy == "operation"
      ? stock.contour(0, 0, stock.tilesX(), stock.tilesY(), BySource(operationColour, UNTOUCHED))
      : stock.contour(0, 0, stock.tilesX(), stock.tilesY(), ByDeviation(target, TOLERANCE, ON_TARGET, LEFTOVER, GOUGE));
    var geometry = new GeometryData();
    geometry.addStream(1, 2, mesh.positions, mesh.vertexCount, 12);
    geometry.addStream(2, 2, mesh.normals, mesh.vertexCount, 12);
    geometry.addStream(6, 4, mesh.colors, mesh.vertexCount, 4);
    geometry.setIndexBuffer(mesh.indices, mesh.triangleCount * 3);
    geometry.setBounds(-width / 2, -height / 2, -depth / 2, width / 2, height / 2, depth / 2);
    lastMesh = mesh;
    return geometry;
  }
  var lastMesh:Null<stockkit.StockMesh> = null;

  /**
    Picks the surface under triangle `triangle` of the last geometry: the move
    that made it becomes `picked`, or null for untouched stock.
  **/
  public function pick(triangle:Int):Null<CutMove> {
    var mesh = lastMesh;
    if (mesh == null || triangle < 0 || triangle >= mesh.triangleCount) return picked = null;
    var source = mesh.sourceAt(triangle);
    return picked = source < 0 ? null : timeline.stock.history[source];
  }

  /** "Operation 1 · line 42: G1 X12 Y8" for a move. */
  public function describe(move:CutMove):String {
    var line = move.provenance.line;
    var text = line >= 1 && line <= gcode.length ? StringTools.trim(gcode[line - 1]) : "";
    return 'Operation ${operationOf(move)} · line $line: $text';
  }

  public function operationOf(move:CutMove):Int return operations[move.opIndex];

  /** Moves that removed stock at rapid feed, over the whole program. */
  public function rapidContacts():Int {
    var count = 0;
    for (outcome in timeline.outcomes) if (outcome.move.rapid && outcome.removed > 0.0) count++;
    return count;
  }

  /** Moves whose shank or holder touched stock, over the whole program. */
  public function collisions():Int {
    var count = 0;
    for (outcome in timeline.outcomes)
      if (outcome.shankContact > 0.0 || outcome.holderContact > 0.0) count++;
    return count;
  }

  /** Deepest gouge into the finished part at the current position, in metres. */
  public function deepestGouge():Float {
    var deepest = 0.0;
    for (comparison in timeline.stock.compareAll(target)) deepest = Math.max(deepest, comparison.deepestGouge());
    return deepest;
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    timeline.dispose();
    timeline.stock.dispose();
    target.dispose();
  }

  static function provenanceOf(op:ToolpathOp):toolpathkit.path.Provenance
    return switch op {
      case SetSetup(_, provenance), Move(_, _, _, _, provenance), MachineMove(_, _, _, _, provenance),
          Dwell(_, provenance), Spindle(_, _, provenance), Coolant(_, _, provenance),
          ToolChange(_, provenance), ToolLengthOffset(_, _, provenance), OptionalStop(provenance),
          ProgramStop(provenance), End(provenance): provenance;
    };

  function operationColour(move:CutMove):Int
    return OPERATION_COLOURS[(operationOf(move) - 1) % OPERATION_COLOURS.length];

  /** Work-coordinate geometry of the demo, in metres. */
  function demoProgram():{program:camkit.CamProgram, safeZ:Float, outer:Array<Float>,
      boss:Array<Float>, pocketDepth:Float, holes:Array<Point3>, holeDiameter:Float, holeDepth:Float} {
    var size = Math.min(width, height);
    var margin = 0.15 * size;
    var outer = [margin, margin, width - margin, height - margin];
    var bossHalfX = 0.15 * (width - 2 * margin), bossHalfY = 0.15 * (height - 2 * margin);
    var boss = [width / 2 - bossHalfX, height / 2 - bossHalfY, width / 2 + bossHalfX, height / 2 + bossHalfY];
    var diameter = Math.max(0.002, Math.min(0.006, size / 15));
    var pocketDepth = Math.min(0.5 * depth, 0.004);
    var mill = Tool.shaped(1, 0.0, CutterProfile.flat(diameter, 0.02).withShank(diameter, 0.02));
    var drillDiameter = Math.max(0.0015, diameter / 2);
    var drill = Tool.shaped(2, 0.0, CutterProfile.vee(drillDiameter, 118 * Math.PI / 180, 0.02));
    var holeDepth = Math.min(0.8 * depth, 0.008);
    var inset = margin / 2;
    var holes = [new Point3(inset, inset, 0), new Point3(width - inset, inset, 0),
      new Point3(width - inset, height - inset, 0), new Point3(inset, height - inset, 0)];
    var mm = 1000.0;
    function rectangle(r:Array<Float>):Curve
      return Curve.polyline([new Vector(r[0] * mm, r[1] * mm), new Vector(r[2] * mm, r[1] * mm),
        new Vector(r[2] * mm, r[3] * mm), new Vector(r[0] * mm, r[3] * mm)], true);
    var outerCurve = rectangle(outer), bossCurve = rectangle(boss);
    var sketch = Sketch.face(outerCurve, [bossCurve]);
    var face = sketch.shape.faces().at(0);
    var safeZ = 0.005;
    var program = new CamJob(safeZ, 10000)
      .pocketFace(face, mill, -pocketDepth, 0.005, 0.45 * diameter, 0.001, "mm", 0.00005, 0.001)
      .drill(holes, drill, -holeDepth, 0.002, 0.002)
      .finish();
    face.close();
    sketch.close();
    outerCurve.close();
    bossCurve.close();
    return {program: program, safeZ: safeZ, outer: outer, boss: boss, pocketDepth: pocketDepth,
      holes: holes, holeDiameter: drillDiameter, holeDepth: holeDepth};
  }

  /**
    The part the demo program should leave, in the object's frame: the block
    minus the pocket around the boss (square corners: a round cutter leaves
    fillets there, which show as leftover) and minus the drilled holes as
    cylinders (the drill's cone tip shows as leftover below them).
  **/
  function finishedPart(program:{program:camkit.CamProgram, safeZ:Float, outer:Array<Float>,
      boss:Array<Float>, pocketDepth:Float, holes:Array<Point3>, holeDiameter:Float, holeDepth:Float}):Part {
    var parts:Array<Part> = [];
    function place(part:Part, x:Float, y:Float, z:Float):Part {
      var placed = part.translated(new Vector(x, y, z));
      parts.push(part);
      parts.push(placed);
      return placed;
    }
    var ox = -width / 2, oy = -height / 2, top = depth / 2;
    var outer = program.outer, boss = program.boss;
    var result = place(Part.box(width, height, depth, Min, Min, Min), ox, oy, -depth / 2);
    var cavity = place(Part.box(outer[2] - outer[0], outer[3] - outer[1], program.pocketDepth + 0.001, Min, Min, Min),
      ox + outer[0], oy + outer[1], top - program.pocketDepth);
    var island = place(Part.box(boss[2] - boss[0], boss[3] - boss[1], program.pocketDepth + 0.002, Min, Min, Min),
      ox + boss[0], oy + boss[1], top - program.pocketDepth - 0.001);
    var pocket = cavity.subtract(island);
    parts.push(pocket);
    var next = result.subtract(pocket);
    for (hole in program.holes) {
      var cylinder = Part.cylinderSpan(program.holeDiameter / 2, top - program.holeDepth, top + 0.001,
        ox + hole.x, oy + hole.y);
      parts.push(cylinder);
      parts.push(next);
      next = next.subtract(cylinder);
    }
    for (part in parts) if (part != next) part.close();
    return next;
  }
}
