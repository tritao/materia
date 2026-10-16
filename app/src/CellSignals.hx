package app;

import haxe.Int64;
import materia.project.SceneArtifact.SceneArtifactCellSignal;
import motionkit.event.EventValue;

/** Carries cabinet outputs to the other controller on the next simulation tick. */
class CellSignals implements SessionMember {
  final wires:Array<SceneArtifactCellSignal>;
  final readOutput:String->String->Bool;
  final publishInput:String->String->Bool->Int64->Int64->Void;
  final writeOutput:Null<String->String->Bool->Void>;
  final simulationTime:Void->Float;
  var values = new Map<String, Bool>();
  var sequence = 0;

  public function new(wires:Array<SceneArtifactCellSignal>,
      simulationTime:Void->Float, readOutput:String->String->Bool,
      publishInput:String->String->Bool->Int64->Int64->Void,
      ?writeOutput:String->String->Bool->Void) {
    this.wires = wires.copy();
    this.simulationTime = simulationTime; this.readOutput = readOutput;
    this.publishInput = publishInput;
    this.writeOutput = writeOutput;
    for (wire in wires) values.set(wire.name, false);
  }

  public function input(controller:String, channel:String):Null<EventValue> {
    for (wire in wires)
      if (wire.to.controller == controller && (wire.name == channel || wire.to.port == channel))
        return EventValue.Digital(values.get(wire.name) == true);
    return null;
  }

  public function inputFor(controller:String):String->Null<EventValue>
    return channel -> input(controller, channel);

  /** Writes a declared digital wire at its source controller's process-output boundary. */
  public function writeSignal(name:String, value:Bool):Void {
    var writer = writeOutput;
    if (writer == null) throw "This cell signal bus has no controller output writer";
    for (wire in wires) if (wire.name == name) {
      writer(wire.from.controller, wire.name, value);
      return;
    }
    throw 'Unknown cell signal "$name"';
  }

  /** Output callback scoped to one cabinet; a robot mission cannot write a CNC-owned output. */
  public function outputFor(controller:String):String->Bool->Void
    return (name, value) -> writeControllerSignal(controller, name, value);

  public function writeControllerSignal(controller:String, name:String, value:Bool):Void {
    var writer = writeOutput;
    if (writer == null) throw "This cell signal bus has no controller output writer";
    for (wire in wires) if (wire.name == name) {
      if (wire.from.controller != controller)
        throw 'Controller "$controller" does not own output "$name"';
      writer(controller, wire.name, value);
      return;
    }
    throw 'Unknown cell signal "$name"';
  }

  public function feed():Void {
    sequence++;
    var stamp = Int64.fromFloat(simulationTime() * 1e9);
    var number = Int64.ofInt(sequence);
    for (wire in wires) {
      var on = readOutput(wire.from.controller, wire.name);
      values.set(wire.name, on);
      publishInput(wire.to.controller, wire.name, on, number, stamp);
    }
  }

  public function beforeReset():Void {}
  public function reset():Void {
    // Runtime adapters retain external sensor frames across a physics reset.
    // Keep publication sequences monotonic for this bus's lifetime.
    for (wire in wires) values.set(wire.name, false);
  }
  public function present():Void {}
}
