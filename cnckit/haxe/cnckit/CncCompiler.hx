package cnckit;

import motionkit.event.EventValue;
import motionkit.path.ArcSegment;
import motionkit.path.CornerBlender;
import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.path.PathPrimitive;
import motionkit.path.PosePrimitive;
import motionkit.path.PosePath;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;

private typedef GWord = {letter:String, value:Float, column:Int};

/** Strict G17/IJ G-code subset. Coordinates enter in mm or inches; paths leave in metres. */
class CncCompiler {
  public final machine:CncMachine;
  var ops:Array<MotionOp>;
  var pending:Array<PathPrimitive>;
  var pendingFeed:Float;
  var pendingBlend:Float;
  var metric:Bool;
  var absolute:Bool;
  var wcs:Int;
  var toolLength:Float;
  var feedCommand:Float;
  var spindleSpeed:Float;
  var spindleDirection:Int;
  var selectedTool:Int;
  var motionMode:Int;
  var blendTolerance:Float;
  var position:Array<Float>;
  var ended:Bool;

  public function new(machine:CncMachine) {
    if (machine == null) throw "CNC compiler needs a machine";
    this.machine = machine;
  }

  public function compile(source:String):MotionProgram {
    if (source == null) throw "CNC source must not be null";
    ops = []; pending = []; pendingFeed = 0.0; pendingBlend = 0.0;
    metric = true; absolute = true; wcs = 54; toolLength = 0.0;
    feedCommand = Math.NaN; spindleSpeed = 0.0; spindleDirection = 0;
    selectedTool = -1; motionMode = -1; blendTolerance = 0.0;
    position = machine.initialPosition.copy(); ended = false;
    var lines = source.split("\n");
    for (index in 0...lines.length) {
      var words = lex(lines[index], index + 1);
      if (words.length == 0) continue;
      if (ended) fail(index + 1, words[0].column,
        "code after M2/M30 program end");
      compileLine(words, index + 1);
    }
    flush();
    if (ops.length == 0) throw "G-code contains no executable motion or barrier";
    return new MotionProgram(ops);
  }

  function compileLine(words:Array<GWord>, line:Int):Void {
    var values:Map<String, GWord> = new Map();
    var gWords:Array<GWord> = [], mWords:Array<GWord> = [];
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
    var lineBlend = blendTolerance;
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
      var blendP:GWord = cast values.get("P");
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
    if (unitChange >= 0) metric = unitChange == 21;
    if (distanceChange >= 0) absolute = distanceChange == 90;
    if (nextWcs >= 0) wcs = nextWcs;
    if (setToolOffset) {
      var hWord:GWord = cast h;
      try toolLength = machine.toolLength(integer(hWord, line))
      catch (error:Dynamic) fail(line, hWord.column, Std.string(error));
    }
    if (clearToolOffset) toolLength = 0.0;
    if (setBlend && lineBlend != blendTolerance) {
      flush(); blendTolerance = lineBlend;
    }
    var f = values.get("F");
    if (f != null) {
      if (f.value <= 0.0) fail(line, f.column, "F feed must be positive");
      feedCommand = f.value;
    }
    var s = values.get("S");
    if (s != null) {
      if (s.value < 0.0) fail(line, s.column, "S spindle speed must be non-negative");
      spindleSpeed = s.value;
    }
    var t = values.get("T");
    if (t != null) selectedTool = integer(t, line);

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
    if (s != null && spindleDirection != 0 && spindleCode < 0)
      output("spindle.speed", EventValue.Analog(spindleSpeed));
    if (spindleCode == 3 || spindleCode == 4) {
      spindleDirection = spindleCode == 3 ? 1 : -1;
      output("spindle.direction", EventValue.Analog(spindleDirection));
      output("spindle.speed", EventValue.Analog(spindleSpeed));
    }
    for (word in mWords) switch integer(word, line) {
      case 7: output("coolant.mist", EventValue.Digital(true));
      case 8: output("coolant.flood", EventValue.Digital(true));
      case _:
    }

    if (modalMotion >= 0) motionMode = modalMotion;
    var x = values.get("X"), y = values.get("Y"), z = values.get("Z");
    var i = values.get("I"), j = values.get("J");
    var hasCoordinates = x != null || y != null || z != null || i != null || j != null;
    if (dwell && hasCoordinates) fail(line, gWords[0].column,
      "G4 cannot include axis or arc words");
    if (hasCoordinates) {
      if (motionMode < 0) fail(line, words[0].column,
        "axis words need G0-G3 motion mode");
      move(line, motionMode, x, y, z, i, j);
    }
    if (dwell) {
      var dwellP:GWord = cast p;
      if (dwellP.value <= 0.0) fail(line, dwellP.column,
        "G4 P seconds must be positive");
      flush(); ops.push(MotionOp.Dwell(dwellP.value));
    }
    if (spindleCode == 5) {
      spindleDirection = 0;
      output("spindle.speed", EventValue.Analog(0.0));
      output("spindle.direction", EventValue.Analog(0.0));
    }
    for (word in mWords) switch integer(word, line) {
      case 9:
        output("coolant.mist", EventValue.Digital(false));
        output("coolant.flood", EventValue.Digital(false));
      case 0, 1:
        flush(); ops.push(MotionOp.WaitInput("cnc.operator.resume",
          InputPredicate.Equals(EventValue.Digital(true)), 1e9));
      case 6:
        if (selectedTool < 0) fail(line, word.column, "M6 requires selected T tool");
        flush(); ops.push(MotionOp.WaitInput('cnc.tool_change.$selectedTool',
          InputPredicate.Equals(EventValue.Digital(true)), 1e9));
      case _:
    }
    if (endCode >= 0) { flush(); ended = true; }
  }

  function move(line:Int, mode:Int, x:Null<GWord>, y:Null<GWord>,
      z:Null<GWord>, i:Null<GWord>, j:Null<GWord>):Void {
    var start = new PathPoint(position[0], position[1], position[2]);
    var offset = machine.workOffset(wcs);
    var words = [x, y, z];
    var target = position.copy();
    for (axis in 0...3) {
      var word = words[axis];
      if (word == null) continue;
      var value = word.value * unitScale();
      target[axis] = absolute ? value + offset[axis] +
        (axis == 2 ? toolLength : 0.0) : target[axis] + value;
    }
    var end = new PathPoint(target[0], target[1], target[2]);
    if (mode < 2) {
      if (i != null || j != null) fail(line, column(i, column(j, 1)),
        "I/J require G2 or G3");
      if (start.distanceTo(end) <= 1e-12) return;
      addMove(new LineSegment(start, end), mode == 0 ? machine.rapidSpeed : feed(line),
        mode == 0);
    } else {
      if (i == null && j == null) fail(line, 1, "G2/G3 require I/J arc centre");
      if (Math.abs(start.z - end.z) > 1e-10)
        fail(line, column(z, 1),
          "helical Z arcs are outside G17 v1");
      var center = new PathPoint(start.x + (i == null ? 0.0 : i.value * unitScale()),
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
      addMove(new ArcSegment(center, radius, begin, sweep), feed(line), false);
    }
    position = target;
  }

  function addMove(geometry:PathPrimitive, speed:Float, rapid:Bool):Void {
    var blend = rapid ? 0.0 : blendTolerance;
    if (pending.length > 0 && (rapid || pendingBlend == 0.0 ||
        blend == 0.0 || Math.abs(speed - pendingFeed) > 1e-12 ||
        blend != pendingBlend)) flush();
    pending.push(geometry); pendingFeed = speed; pendingBlend = blend;
    if (rapid || blend == 0.0) flush();
  }

  function flush():Void {
    if (pending.length == 0) return;
    var geometry = pending.copy();
    var combined = false;
    if (geometry.length > 1 && pendingBlend > 0.0) {
      var blended = CornerBlender.blend(new GeometricPath(geometry),
        pendingBlend, Math.PI * 5.0 / 6.0);
      if (blended.diagnostics.length == 0) {
        geometry = blended.path.primitives;
        combined = true;
      }
    }
    if (combined) emitPath(geometry, pendingFeed,
      new GeometricPath(pending), pendingBlend);
    else for (primitive in pending) emitPath([primitive], pendingFeed);
    pending = [];
  }

  function emitPath(geometry:Array<PathPrimitive>, speed:Float,
      ?authored:GeometricPath, ?blend:Float = 0.0):Void {
    var primitives:Array<PosePrimitive> = [for (primitive in geometry)
      new CncPosePrimitive(primitive, speed, machine.positionTolerance,
        machine.orientationTolerance)];
    var path = new PosePath(machine.frameId, primitives);
    if (authored != null) path.withAuthoredGeometry(authored, blend);
    ops.push(MotionOp.FollowPath(path, machine.frameId, speed, []));
  }

  function output(channel:String, value:EventValue):Void {
    flush(); ops.push(MotionOp.SetOutput(channel, value));
  }

  function feed(line:Int):Float {
    if (!Math.isFinite(feedCommand)) fail(line, 1, "G1/G2/G3 require F feed");
    return feedCommand * unitScale() / 60.0;
  }
  function unitScale():Float return metric ? 0.001 : 0.0254;

  static function integer(word:GWord, line:Int):Int {
    if (word == null || word.value < 0.0 || word.value != Math.floor(word.value))
      fail(line, word == null ? 1 : word.column, "code must be a non-negative integer");
    return Std.int(word.value);
  }
  static function column(word:Null<GWord>, fallback:Int):Int {
    if (word == null) return fallback;
    var present:GWord = cast word;
    return present.column;
  }
  static function fail(line:Int, column:Int, message:String):Void
    throw 'G-code line $line column $column: $message';

  static function lex(line:String, lineNumber:Int):Array<GWord> {
    var words:Array<GWord> = [];
    var index = 0;
    while (index < line.length) {
      var ch = line.charAt(index);
      if (ch == ";") break;
      if (ch == "(" ) {
        var close = line.indexOf(")", index + 1);
        if (close < 0) fail(lineNumber, index + 1, "unclosed comment");
        index = close + 1; continue;
      }
      if (ch == " " || ch == "\t" || ch == "\r") { index++; continue; }
      var code = line.charCodeAt(index);
      if (!((code >= 65 && code <= 90) || (code >= 97 && code <= 122)))
        fail(lineNumber, index + 1, 'unexpected character "$ch"');
      var letter = ch.toUpperCase(), column = index + 1;
      index++;
      var start = index, digits = 0, dots = 0;
      if (index < line.length && (line.charAt(index) == "+" ||
          line.charAt(index) == "-")) index++;
      while (index < line.length) {
        var current = line.charAt(index), currentCode = line.charCodeAt(index);
        if (currentCode >= 48 && currentCode <= 57) { digits++; index++; }
        else if (current == ".") { dots++; index++; }
        else break;
      }
      if (digits == 0 || dots > 1) fail(lineNumber, column,
        'invalid $letter number');
      var number = Std.parseFloat(line.substring(start, index));
      if (!Math.isFinite(number)) fail(lineNumber, column,
        'non-finite $letter number');
      words.push({letter:letter, value:number, column:column});
    }
    return words;
  }
}
