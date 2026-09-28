package cnckit.interp;

import cnckit.CncChannels;
import cnckit.CncDiagnostic;
import cnckit.CncDiagnostic.CncSeverity;
import cnckit.CncMachine;
import cnckit.ir.CncGeometry;
import cnckit.ir.CncOp;
import cnckit.ir.CncPoint;
import cnckit.parse.CncBlock;
import cnckit.parse.CncSpan;
import cnckit.parse.CncWord;

/** Interprets LinuxCNC blocks into plain metre geometry and ordered CNC ops. */
class CncInterpreter {
  public final machine:CncMachine;
  public var state(default, null):CncState;
  public var ops(default, null):Array<CncOp> = [];
  public var diagnostics(default, null):Array<CncDiagnostic> = [];

  public function new(machine:CncMachine) {
    this.machine = machine;
    state = new CncState(machine);
  }

  public function interpret(blocks:Array<CncBlock>):Array<CncOp> {
    state = new CncState(machine);
    ops = [];
    diagnostics = [];
    for (block in blocks) {
      var previous = state.copy(), count = ops.length;
      try {
        if (state.ended) fail(block.span.line, block.words[0].column,
          "code after M2/M30 program end");
        compileLine(block);
      } catch (error:CncDiagnostic) {
        state = previous;
        while (ops.length > count) ops.pop();
        diagnostics.push(error);
      }
    }
    return ops.copy();
  }

  function compileLine(block:CncBlock):Void {
    var words = block.words, line = block.span.line;
    var values:Map<String, CncWord> = new Map();
    var gWords:Array<CncWord> = [], mWords:Array<CncWord> = [];
    for (word in words) {
      switch word.letter {
        case "G": gWords.push(word);
        case "M": mWords.push(word);
        case "N", "O": integer(word, line);
        case "X", "Y", "Z", "I", "J", "R", "F", "S", "P", "H", "T", "Q", "L":
          if (values.exists(word.letter)) fail(line, word.column,
            'duplicate ${word.letter} word');
          values.set(word.letter, word);
        case _: fail(line, word.column, 'unsupported ${word.letter} word');
      }
    }
    var modalMotion = -1, dwell = false, setToolOffset = false;
    var clearToolOffset = false, setBlend = false, useBlend = false;
    var cycleChange = -1, homeCode = 0, machineCoordinates = false;
    var retractChange = -1;
    var lineBlend = state.blendTolerance;
    var unitChange = -1, distanceChange = -1, nextWcs = -1;
    for (word in gWords) {
      var code = gCode(word, line);
      switch code {
        case 0, 1, 2, 3:
          if (modalMotion >= 0) fail(line, word.column, "multiple motion G codes");
          modalMotion = code;
        case 4:
          if (dwell) fail(line, word.column, "duplicate G4");
          dwell = true;
        case 17: // The only declared plane.
        case 28, 30:
          if (homeCode != 0) fail(line, word.column, "multiple home G codes");
          homeCode = code;
        case 40: // Cutter compensation is currently always off.
        case 53: machineCoordinates = true;
        case 73, 81, 82, 83:
          if (cycleChange >= 0) fail(line, word.column, "multiple drilling cycles");
          cycleChange = code;
        case 80: cycleChange = 80;
        case 20, 21:
          if (unitChange >= 0) fail(line, word.column, "multiple unit modes");
          unitChange = code;
        case 90, 91:
          if (distanceChange >= 0) fail(line, word.column, "multiple distance modes");
          distanceChange = code;
        case 54, 55, 56, 57, 58, 59:
          if (nextWcs >= 0) fail(line, word.column, "multiple work offsets");
          nextWcs = code;
        case 43: setToolOffset = true;
        case 49: clearToolOffset = true;
        case 94: // Units per minute is the only supported feed mode.
        case 93: fail(line, word.column, "G93 inverse-time feed is unsupported");
        case 95: fail(line, word.column, "G95 units-per-revolution feed is unsupported");
        case 98, 99: retractChange = code;
        case 911: // LinuxCNC incremental I/J centres are already the default.
        case 901: fail(line, word.column, "G90.1 absolute arc centres are unsupported");
        case 92: fail(line, word.column, "G92 persistent offsets are unsupported");
        case 61:
          if (setBlend) fail(line, word.column, "multiple path-control G codes");
          setBlend = true; lineBlend = 0.0;
        case 64:
          if (setBlend) fail(line, word.column, "multiple path-control G codes");
          setBlend = true; useBlend = true;
          var p = values.get("P");
          if (p == null) fail(line, word.column, "G64 requires P tolerance");
        case 18, 19: fail(line, word.column, "only G17 XY arcs are supported");
        case _: fail(line, word.column, 'unsupported G$code');
      }
    }
    if (useBlend) {
      var blendP:CncWord = cast values.get("P");
      lineBlend = blendP.value * (unitChange == 20 ? 0.0254 :
        unitChange == 21 ? 0.001 : unitScale());
      if (!Math.isFinite(lineBlend) || lineBlend <= 0.0)
        fail(line, blendP.column, "G64 P tolerance must be positive");
    }
    if (modalMotion >= 0 && dwell) fail(line, gWords[0].column,
      "G4 cannot share a block with motion");
    if (cycleChange > 0 && cycleChange != 80 && modalMotion >= 0)
      fail(line, gWords[0].column, "drilling cycle conflicts with G0-G3");
    if (homeCode != 0 && (modalMotion >= 0 || cycleChange > 0 ||
        machineCoordinates)) fail(line, gWords[0].column,
      "G28/G30 cannot share a block with motion or G53");
    if (machineCoordinates && (homeCode != 0 || cycleChange > 0 || dwell))
      fail(line, gWords[0].column, "G53 requires G0/G1 motion");
    if (setToolOffset && clearToolOffset) fail(line, gWords[0].column,
      "G43 and G49 conflict");
    var h = values.get("H");
    if (setToolOffset && h == null) fail(line, gWords[0].column,
      "G43 requires H tool length");
    if (!setToolOffset && h != null) fail(line, h.column,
      "H requires G43 in the same block");
    var p = values.get("P");
    var cycleForBlock = cycleChange > 0 && cycleChange != 80 ? cycleChange :
      cycleChange == 80 ? 0 : state.cycleCode;
    if (p != null && !dwell && !(setBlend && lineBlend > 0.0) &&
        cycleForBlock != 82) fail(line, p.column, "P requires G4, G64 or G82");
    if (dwell && p == null) fail(line, gWords[0].column,
      "G4 requires P seconds");
    var r = values.get("R"), q = values.get("Q"), l = values.get("L");
    var effectiveMotion = modalMotion >= 0 ? modalMotion : state.motionMode;
    if (r != null && cycleForBlock == 0 && effectiveMotion != 2 &&
        effectiveMotion != 3)
      fail(line, r.column, "R requires an arc or drilling cycle");
    if (q != null && cycleForBlock != 73 && cycleForBlock != 83)
      fail(line, q.column, "Q requires G73 or G83");
    if (l != null && cycleForBlock == 0)
      fail(line, l.column, "L requires a drilling cycle");
    // LinuxCNC block order: F/S, T, M6, spindle, coolant, dwell,
    // modal state, motion, then operator and program stops.
    var f = values.get("F");
    if (f != null) {
      if (f.value <= 0.0) fail(line, f.column, "F feed must be positive");
      state.feedCommand = f.value;
    }
    var s = values.get("S");
    if (s != null) {
      if (s.value < 0.0) fail(line, s.column, "S spindle speed must be non-negative");
      state.spindleSpeed = s.value;
    }
    var t = values.get("T");
    if (t != null) state.selectedTool = integer(t, line);

    var spindleCode = -1, endCode = -1;
    for (word in mWords) {
      var code = integer(word, line);
      switch code {
        case 3, 4, 5:
          if (spindleCode >= 0) fail(line, word.column, "multiple spindle M codes");
          spindleCode = code;
        case 0, 1, 2, 6, 7, 8, 9, 30:
          if (code == 2 || code == 30) endCode = code;
        case _: fail(line, word.column, 'unsupported M$code');
      }
    }
    if (s != null && state.spindleDirection != 0 && spindleCode < 0)
      spindle(CncChannels.SpindleSpeed, state.spindleSpeed, block.span);
    for (word in mWords) if (integer(word, line) == 6) {
      if (state.selectedTool < 0) fail(line, word.column, "M6 requires selected T tool");
      ops.push(CncOp.ToolChange(state.selectedTool, block.span));
    }
    if (spindleCode == 3 || spindleCode == 4) {
      state.spindleDirection = spindleCode == 3 ? 1 : -1;
      spindle(CncChannels.SpindleDirection, state.spindleDirection, block.span);
      spindle(CncChannels.SpindleSpeed, state.spindleSpeed, block.span);
    } else if (spindleCode == 5) {
      state.spindleDirection = 0;
      spindle(CncChannels.SpindleSpeed, 0.0, block.span);
      spindle(CncChannels.SpindleDirection, 0.0, block.span);
    }
    for (word in mWords) switch integer(word, line) {
      case 7: coolant(CncChannels.CoolantMist, true, block.span);
      case 8: coolant(CncChannels.CoolantFlood, true, block.span);
      case 9:
        coolant(CncChannels.CoolantMist, false, block.span);
        coolant(CncChannels.CoolantFlood, false, block.span);
      case _:
    }

    if (dwell) {
      var dwellP:CncWord = cast p;
      if (dwellP.value <= 0.0) fail(line, dwellP.column,
        "G4 P seconds must be positive");
      ops.push(CncOp.Dwell(dwellP.value, block.span));
    }
    if (unitChange >= 0) state.metric = unitChange == 21;
    if (distanceChange >= 0) state.absolute = distanceChange == 90;
    if (nextWcs >= 0) state.wcs = nextWcs;
    if (retractChange >= 0) state.retractToInitial = retractChange == 98;
    if (setToolOffset) {
      var hWord:CncWord = cast h;
      try state.toolLength = machine.toolLength(integer(hWord, line))
      catch (error:Dynamic) fail(line, hWord.column, Std.string(error));
    }
    if (clearToolOffset) state.toolLength = 0.0;
    if (setBlend && lineBlend != state.blendTolerance) {
      state.blendTolerance = lineBlend;
    }

    if (cycleChange == 80) {
      state.cycleCode = 0; state.motionMode = -1; state.cycleSpan = null;
    }
    if (modalMotion >= 0) {
      state.motionMode = modalMotion; state.cycleCode = 0;
      state.cycleSpan = null;
    }
    if (cycleChange > 0 && cycleChange != 80) {
      if (state.cycleCode == 0) state.cycleInitialZ = state.position[2];
      state.cycleCode = cycleChange; state.motionMode = -1;
      state.cycleSpan = block.span;
    }
    var x = values.get("X"), y = values.get("Y"), z = values.get("Z");
    var i = values.get("I"), j = values.get("J");
    var hasCoordinates = x != null || y != null || z != null || i != null ||
      j != null || r != null;
    if (dwell && hasCoordinates) fail(line, gWords[0].column,
      "G4 cannot include axis or arc words");
    if (homeCode != 0) {
      if (i != null || j != null || r != null || q != null || l != null)
        fail(line, words[0].column, "G28/G30 accept only XYZ axes");
      home(homeCode, x, y, z, block.span);
    } else if (state.cycleCode != 0 && (cycleChange > 0 ||
        x != null || y != null || z != null || r != null || q != null ||
        p != null || l != null)) {
      if (i != null || j != null) fail(line, column(i, column(j, 1)),
        "I/J are unsupported in drilling cycles");
      drill(state.cycleCode, x, y, z, r, q, p, l,
        state.cycleSpan == null ? block.span : state.cycleSpan, line);
    } else if (hasCoordinates) {
      if (state.motionMode < 0) fail(line, words[0].column,
        "axis words need G0-G3 motion mode");
      move(line, state.motionMode, x, y, z, i, j, r,
        machineCoordinates, block.span);
    } else if (machineCoordinates) {
      fail(line, gWords[0].column, "G53 requires axis words");
    }
    for (word in mWords) switch integer(word, line) {
      case 0, 1:
        ops.push(integer(word, line) == 1 ? CncOp.OptionalStop(block.span) :
          CncOp.ProgramStop(block.span));
      case _:
    }
    if (endCode >= 0) { ops.push(CncOp.End(block.span)); state.ended = true; }
  }

  function move(line:Int, mode:Int, x:Null<CncWord>, y:Null<CncWord>,
      z:Null<CncWord>, i:Null<CncWord>, j:Null<CncWord>,
      r:Null<CncWord>, machineCoordinates:Bool, span:CncSpan):Void {
    var start = new CncPoint(state.position[0], state.position[1], state.position[2]);
    var offset = machine.workOffset(state.wcs);
    var words = [x, y, z];
    var target = state.position.copy();
    for (axis in 0...3) {
      var word = words[axis];
      if (word == null) continue;
      var value = word.value * unitScale();
      target[axis] = machineCoordinates ? value : state.absolute ?
        value + offset[axis] + (axis == 2 ? state.toolLength : 0.0) :
        target[axis] + value;
    }
    var end = new CncPoint(target[0], target[1], target[2]);
    if (mode < 2) {
      if (machineCoordinates && !state.absolute)
        fail(line, span.column, "G53 requires G90 absolute mode");
      if (i != null || j != null || r != null)
        fail(line, column(i, column(j, column(r, 1))),
          "I/J/R require G2 or G3");
      if (start.distanceTo(end) <= 1e-12) return;
      if (mode == 0) ops.push(CncOp.Rapid(CncGeometry.Line(start, end), span));
      else ops.push(CncOp.Feed(CncGeometry.Line(start, end), feed(line), state.blendTolerance, span));
    } else {
      if (machineCoordinates) fail(line, span.column, "G53 requires G0/G1 motion");
      if (r != null && (i != null || j != null))
        fail(line, r.column, "R and I/J arc centres conflict");
      if (r == null && i == null && j == null)
        fail(line, 1, "G2/G3 require I/J or R arc centre");
      if (Math.abs(start.z - end.z) > 1e-10)
        fail(line, column(z, 1),
          "helical Z arcs are outside G17 v1");
      var center = r == null ?
        new CncPoint(start.x + (i == null ? 0.0 : i.value * unitScale()),
          start.y + (j == null ? 0.0 : j.value * unitScale()), start.z) :
        radiusCenter(start, end, r.value * unitScale(), mode, line, r.column);
      var radius = center.distanceTo(start);
      if (radius <= 1e-12 || Math.abs(center.distanceTo(end) - radius) >
          Math.max(1e-8, radius * 1e-5))
        fail(line, column(i, column(j, 1)),
          "arc endpoint is not on its I/J circle");
      var begin = Math.atan2(start.y - center.y, start.x - center.x);
      var finish = Math.atan2(end.y - center.y, end.x - center.x);
      var sweep = finish - begin;
      if (mode == 2) {
        while (sweep >= -1e-12) sweep -= 2.0 * Math.PI;
      } else {
        while (sweep <= 1e-12) sweep += 2.0 * Math.PI;
      }
      ops.push(CncOp.Feed(CncGeometry.Arc(center, radius, begin, sweep),
        feed(line), state.blendTolerance, span));
    }
    state.position = target;
  }

  function home(code:Int, x:Null<CncWord>, y:Null<CncWord>,
      z:Null<CncWord>, span:CncSpan):Void {
    var axes = [x, y, z], intermediate = state.position.copy();
    var offset = machine.workOffset(state.wcs);
    var anyAxis = false;
    for (axis in 0...3) if (axes[axis] != null) {
      anyAxis = true;
      var axisWord:CncWord = cast axes[axis];
      var value = axisWord.value * unitScale();
      intermediate[axis] = state.absolute ? value + offset[axis] +
        (axis == 2 ? state.toolLength : 0.0) : intermediate[axis] + value;
    }
    if (anyAxis) rapidTo(intermediate, span);
    var stored = machine.homePosition(code), destination = state.position.copy();
    for (axis in 0...3) if (!anyAxis || axes[axis] != null)
      destination[axis] = stored[axis];
    rapidTo(destination, span);
  }

  function drill(code:Int, x:Null<CncWord>, y:Null<CncWord>,
      z:Null<CncWord>, r:Null<CncWord>, q:Null<CncWord>,
      p:Null<CncWord>, l:Null<CncWord>, span:CncSpan, line:Int):Void {
    var offset = machine.workOffset(state.wcs), scale = unitScale();
    if (r != null) state.cycleR = state.absolute ?
      r.value * scale + offset[2] + state.toolLength :
      state.cycleInitialZ + r.value * scale;
    if (!Math.isFinite(state.cycleR))
      fail(line, span.column, 'G$code requires R retract plane');
    if (z != null) state.cycleDepth = state.absolute ?
      z.value * scale + offset[2] + state.toolLength :
      state.cycleR + z.value * scale;
    if (!Math.isFinite(state.cycleDepth))
      fail(line, span.column, 'G$code requires Z depth');
    if (state.cycleDepth >= state.cycleR - 1e-12)
      fail(line, z == null ? span.column : z.column,
        'G$code Z depth must be below R plane');
    if (q != null) state.cycleQ = q.value * scale;
    if ((code == 73 || code == 83) &&
        (!Math.isFinite(state.cycleQ) || state.cycleQ <= 0.0))
      fail(line, q == null ? span.column : q.column,
        'G$code requires positive Q peck distance');
    if (p != null) state.cycleP = p.value;
    if (code == 82 && (!Math.isFinite(state.cycleP) || state.cycleP <= 0.0))
      fail(line, p == null ? span.column : p.column,
        "G82 requires positive P dwell seconds");
    var repeats = l == null ? 1 : integer(l, line);
    if (repeats < 1 || repeats > 10000)
      fail(line, l == null ? span.column : l.column,
        "L repeat count must be 1..10000");
    var speed = feed(line);
    for (_ in 0...repeats) {
      var plane = state.cycleR;
      if (state.position[2] < plane - 1e-12)
        rapidTo([state.position[0], state.position[1], plane], span);
      var target = state.position.copy();
      if (x != null) target[0] = state.absolute ?
        x.value * scale + offset[0] : target[0] + x.value * scale;
      if (y != null) target[1] = state.absolute ?
        y.value * scale + offset[1] : target[1] + y.value * scale;
      rapidTo(target, span);
      rapidTo([state.position[0], state.position[1], plane], span);
      var depth = state.cycleDepth;
      if (code == 81 || code == 82) {
        feedTo([state.position[0], state.position[1], depth], speed, span);
        if (code == 82) ops.push(CncOp.Dwell(state.cycleP, span));
      } else {
        var lastCut = plane, guard = 0;
        while (lastCut > depth + 1e-12) {
          if (++guard > 10000) fail(line, span.column,
            "drilling cycle exceeds 10000 pecks");
          var nextCut = Math.max(depth, lastCut - state.cycleQ);
          if (code == 83 && lastCut < plane - 1e-12) {
            var approach = Math.min(plane, lastCut + 0.000254);
            rapidTo([state.position[0], state.position[1], approach], span);
          }
          feedTo([state.position[0], state.position[1], nextCut], speed, span);
          if (nextCut > depth + 1e-12) {
            if (code == 83) rapidTo([state.position[0], state.position[1], plane], span);
            else rapidTo([state.position[0], state.position[1],
              Math.min(plane, nextCut + 0.000254)], span);
          }
          lastCut = nextCut;
        }
      }
      var retract = state.retractToInitial ?
        Math.max(state.cycleInitialZ, plane) : plane;
      rapidTo([state.position[0], state.position[1], retract], span);
    }
  }

  function rapidTo(target:Array<Float>, span:CncSpan):Void {
    var start = new CncPoint(state.position[0], state.position[1], state.position[2]);
    var end = new CncPoint(target[0], target[1], target[2]);
    if (start.distanceTo(end) > 1e-12)
      ops.push(CncOp.Rapid(CncGeometry.Line(start, end), span));
    state.position = target.copy();
  }

  function feedTo(target:Array<Float>, speed:Float, span:CncSpan):Void {
    var start = new CncPoint(state.position[0], state.position[1], state.position[2]);
    var end = new CncPoint(target[0], target[1], target[2]);
    if (start.distanceTo(end) > 1e-12)
      ops.push(CncOp.Feed(CncGeometry.Line(start, end), speed, 0.0, span));
    state.position = target.copy();
  }

  static function radiusCenter(start:CncPoint, end:CncPoint, radius:Float,
      mode:Int, line:Int, column:Int):CncPoint {
    var dx = end.x - start.x, dy = end.y - start.y;
    var chord = Math.sqrt(dx * dx + dy * dy), magnitude = Math.abs(radius);
    if (!Math.isFinite(radius) || magnitude <= 1e-12 || chord <= 1e-12 ||
        chord > 2.0 * magnitude + 1e-10)
      fail(line, column, "R arc radius cannot reach its endpoint");
    var height = Math.sqrt(Math.max(0.0,
      magnitude * magnitude - chord * chord * 0.25));
    var mx = (start.x + end.x) * 0.5, my = (start.y + end.y) * 0.5;
    var nx = -dy / chord, ny = dx / chord;
    var first = new CncPoint(mx + nx * height, my + ny * height, start.z);
    var second = new CncPoint(mx - nx * height, my - ny * height, start.z);
    var firstSweep = Math.abs(arcSweep(start, end, first, mode));
    var secondSweep = Math.abs(arcSweep(start, end, second, mode));
    return radius >= 0.0 ?
      (firstSweep <= secondSweep ? first : second) :
      (firstSweep >= secondSweep ? first : second);
  }

  static function arcSweep(start:CncPoint, end:CncPoint,
      center:CncPoint, mode:Int):Float {
    var begin = Math.atan2(start.y - center.y, start.x - center.x);
    var finish = Math.atan2(end.y - center.y, end.x - center.x);
    var sweep = finish - begin;
    if (mode == 2) while (sweep >= -1e-12) sweep -= 2.0 * Math.PI;
    else while (sweep <= 1e-12) sweep += 2.0 * Math.PI;
    return sweep;
  }

  function spindle(channel:String, value:Float, span:CncSpan):Void
    ops.push(CncOp.Spindle(channel, value, span));

  function coolant(channel:String, enabled:Bool, span:CncSpan):Void
    ops.push(CncOp.Coolant(channel, enabled, span));

  function feed(line:Int):Float {
    if (!Math.isFinite(state.feedCommand)) fail(line, 1, "G1/G2/G3 require F feed");
    return state.feedCommand * unitScale() / 60.0;
  }

  function unitScale():Float return state.metric ? 0.001 : 0.0254;

  static function integer(word:CncWord, line:Int):Int {
    if (word == null || word.value < 0.0 || word.value != Math.floor(word.value))
      fail(line, word == null ? 1 : word.column, "code must be a non-negative integer");
    return Std.int(word.value);
  }

  static function gCode(word:CncWord, line:Int):Int {
    if (Math.abs(word.value - 91.1) < 1e-8) return 911;
    if (Math.abs(word.value - 90.1) < 1e-8) return 901;
    return integer(word, line);
  }

  static function column(word:Null<CncWord>, fallback:Int):Int
    return word == null ? fallback : word.column;

  static function fail(line:Int, column:Int, message:String):Void
    throw new CncDiagnostic(Error, "CNC_INTERPRET", new CncSpan(line, column, 1), message);
}
