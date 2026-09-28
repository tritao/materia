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
        case "X", "Y", "Z", "I", "J", "R", "F", "S", "P", "H", "T":
          if (values.exists(word.letter)) fail(line, word.column,
            'duplicate ${word.letter} word');
          values.set(word.letter, word);
        case _: fail(line, word.column, 'unsupported ${word.letter} word');
      }
    }
    var r = values.get("R");
    if (r != null) fail(line, r.column, "R-form arcs are unsupported; use I/J");
    var modalMotion = -1, dwell = false, setToolOffset = false;
    var clearToolOffset = false, setBlend = false, useBlend = false;
    var lineBlend = state.blendTolerance;
    var unitChange = -1, distanceChange = -1, nextWcs = -1;
    for (word in gWords) {
      var code = integer(word, line);
      switch code {
        case 0, 1, 2, 3:
          if (modalMotion >= 0) fail(line, word.column, "multiple motion G codes");
          modalMotion = code;
        case 4:
          if (dwell) fail(line, word.column, "duplicate G4");
          dwell = true;
        case 17: // The only declared plane.
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
    if (setToolOffset && clearToolOffset) fail(line, gWords[0].column,
      "G43 and G49 conflict");
    var h = values.get("H");
    if (setToolOffset && h == null) fail(line, gWords[0].column,
      "G43 requires H tool length");
    if (!setToolOffset && h != null) fail(line, h.column,
      "H requires G43 in the same block");
    var p = values.get("P");
    if (p != null && !dwell && !(setBlend && lineBlend > 0.0))
      fail(line, p.column, "P requires G4 or G64");
    if (dwell && p == null) fail(line, gWords[0].column,
      "G4 requires P seconds");
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
    if (setToolOffset) {
      var hWord:CncWord = cast h;
      try state.toolLength = machine.toolLength(integer(hWord, line))
      catch (error:Dynamic) fail(line, hWord.column, Std.string(error));
    }
    if (clearToolOffset) state.toolLength = 0.0;
    if (setBlend && lineBlend != state.blendTolerance) {
      state.blendTolerance = lineBlend;
    }

    if (modalMotion >= 0) state.motionMode = modalMotion;
    var x = values.get("X"), y = values.get("Y"), z = values.get("Z");
    var i = values.get("I"), j = values.get("J");
    var hasCoordinates = x != null || y != null || z != null || i != null || j != null;
    if (dwell && hasCoordinates) fail(line, gWords[0].column,
      "G4 cannot include axis or arc words");
    if (hasCoordinates) {
      if (state.motionMode < 0) fail(line, words[0].column,
        "axis words need G0-G3 motion mode");
      move(line, state.motionMode, x, y, z, i, j, block.span);
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
      z:Null<CncWord>, i:Null<CncWord>, j:Null<CncWord>, span:CncSpan):Void {
    var start = new CncPoint(state.position[0], state.position[1], state.position[2]);
    var offset = machine.workOffset(state.wcs);
    var words = [x, y, z];
    var target = state.position.copy();
    for (axis in 0...3) {
      var word = words[axis];
      if (word == null) continue;
      var value = word.value * unitScale();
      target[axis] = state.absolute ? value + offset[axis] +
        (axis == 2 ? state.toolLength : 0.0) : target[axis] + value;
    }
    var end = new CncPoint(target[0], target[1], target[2]);
    if (mode < 2) {
      if (i != null || j != null) fail(line, column(i, column(j, 1)),
        "I/J require G2 or G3");
      if (start.distanceTo(end) <= 1e-12) return;
      if (mode == 0) ops.push(CncOp.Rapid(CncGeometry.Line(start, end), span));
      else ops.push(CncOp.Feed(CncGeometry.Line(start, end), feed(line), state.blendTolerance, span));
    } else {
      if (i == null && j == null) fail(line, 1, "G2/G3 require I/J arc centre");
      if (Math.abs(start.z - end.z) > 1e-10)
        fail(line, column(z, 1),
          "helical Z arcs are outside G17 v1");
      var center = new CncPoint(start.x + (i == null ? 0.0 : i.value * unitScale()),
        start.y + (j == null ? 0.0 : j.value * unitScale()), start.z);
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

  static function column(word:Null<CncWord>, fallback:Int):Int
    return word == null ? fallback : word.column;

  static function fail(line:Int, column:Int, message:String):Void
    throw new CncDiagnostic(Error, "CNC_INTERPRET", new CncSpan(line, column, 1), message);
}
