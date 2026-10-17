package toolpathkit.path;

import toolpathkit.tool.ToolLibrary;
import toolpathkit.setup.Setup;

/** Controller-independent operations and the tools referenced by changes. */
class ToolpathProgram {
  public final ops:Array<ToolpathOp>;
  public final tools:ToolLibrary;
  public final setups:Array<Setup>;

  public function new(ops:Array<ToolpathOp>, tools:ToolLibrary, setups:Array<Setup>) {
    if (ops == null || tools == null || setups == null || setups.length == 0)
      throw "toolpath program needs operations, tools and setups";
    var ids = new Map<String, Bool>();
    for (setup in setups) {
      if (setup == null || ids.exists(setup.id))
        throw "toolpath program has a duplicate or missing setup";
      ids.set(setup.id, true);
    }
    for (index in 0...ops.length) {
      var op = ops[index];
      switch op {
        case ElectronicGear(follower, terms, _):
          if (follower == null || follower.length == 0 || terms == null || terms.length == 0 || terms.length > 8)
            throw 'toolpath op $index: electronic gear needs a follower and one to eight masters';
          var masters:Array<String> = [];
          for (term in terms) {
            if (term == null || term.master == follower || masters.indexOf(term.master) >= 0)
              throw 'toolpath op $index: invalid electronic gear master';
            masters.push(term.master);
          }
        case ReleaseGear(follower, _):
          if (follower == null || follower.length == 0) throw 'toolpath op $index: release gear needs a follower';
        case SetSetup(id, _):
          if (!ids.exists(id)) throw 'toolpath program names unknown setup $id';
        case _: PrinterOps.validate(op, index);
      }
    }
    this.ops = ops.copy();
    this.tools = tools;
    this.setups = setups.copy();
  }

  /**
    The volume, in m^3, of the beads the program lays: each `Cut` move's length times the bead
    area of the `Deposit` in force.
  **/
  public function depositedVolume():Float {
    var total = 0.0;
    walkDeposition((volume, _, _) -> { total += volume; });
    return total;
  }

  /** Deposited volume per layer, in the order the layers first appear. Beads before any `Layer` marker are layer -1. */
  public function layerVolumes():Array<LayerVolume> {
    var layers:Array<LayerVolume> = [];
    walkDeposition((volume, layer, _) -> {
      for (entry in layers) if (entry.index == layer.index) { entry.volume += volume; return; }
      layers.push({index: layer.index, z: layer.z, volume: volume});
    });
    return layers;
  }

  /** Deposited volume per `PrintObject`, in the order the objects first appear. Beads outside any object are not listed. */
  public function objectVolumes():Array<ObjectVolume> {
    var objects:Array<ObjectVolume> = [];
    walkDeposition((volume, _, name) -> {
      if (name == null) return;
      for (entry in objects) if (entry.name == name) { entry.volume += volume; return; }
      objects.push({name: name, volume: volume});
    });
    return objects;
  }

  function walkDeposition(visit:(Float, {index:Int, z:Float}, Null<String>) -> Void):Void {
    var area = 0.0, layer = {index: -1, z: 0.0}, object:Null<String> = null;
    for (op in ops) switch op {
      case Deposit(bead, _, _, _): area = bead;
      case DepositOff(_): area = 0.0;
      case Layer(index, z, _): layer = {index: index, z: z};
      case PrintObject(name, start, _): object = start ? name : null;
      case Move(Cut, geometry, _, _, _), MachineMove(Cut, geometry, _, _, _):
        if (area > 0.0) visit(area * GeometryTools.length(geometry), layer, object);
      case _:
    }
  }
}

/** Material laid on one layer, in m^3. */
typedef LayerVolume = {var index:Int; var z:Float; var volume:Float;}

/** Material laid for one printed object, in m^3. */
typedef ObjectVolume = {var name:String; var volume:Float;}
