package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.structural.MineralFoundation.CastInsertPlacement;

/**
 * The placed source steel a candidate insert is checked against (`InsertAnchorGeometry.checkSourceClearance`): each
 * neighbouring insert's blank fused with its anchor hardware, at its pose, with its bounds. A sizing loop tries many
 * candidates against the same neighbours, and most are settled by the bounds alone: each neighbour's bounds are taken
 * when it is added (its blank's and its screws' boxes carried to the pose, which contain the fused solid), and its solid
 * is built the first time a candidate comes near it, then kept. The solids are native and belong to this object: close
 * it when the loop is done.
 */
class SourceSteel {
  public final rows:Array<CastInsertPlacement> = [];
  final solids:Array<Null<Part>> = [];
  final bounds:Array<PosedParts.PartBox> = [];
  /** Anchor screws by recipe: neighbours repeat a few screw sizes, so each solid is built once. */
  final screws = new PosedParts();

  public function new(rows:Array<CastInsertPlacement>) {
    try {
      for (row in rows) add(row);
    } catch (error:Dynamic) {
      close();
      throw error;
    }
  }

  /** A neighbour placed after construction, such as a lifting insert just chosen. */
  public function add(row:CastInsertPlacement):Void {
    var blank = machinekit.structural.InsertBodies.part(row.part.bodySpec()), own:PosedParts.PartBox;
    try own = PosedParts.carried(PosedParts.boxOf(blank), row.pose) catch (error:Dynamic) {
      blank.close();
      throw error;
    }
    blank.close();
    for (hardware in InsertAnchorGeometry.components(row.part.insertSpec())) {
      var screw = screws.box(hardware.part, materia.assembly.AssemblyFrames.compose(row.pose, hardware.pose));
      own = {
        minX: Math.min(own.minX, screw.minX),
        minY: Math.min(own.minY, screw.minY),
        minZ: Math.min(own.minZ, screw.minZ),
        maxX: Math.max(own.maxX, screw.maxX),
        maxY: Math.max(own.maxY, screw.maxY),
        maxZ: Math.max(own.maxZ, screw.maxZ)
      };
    }
    rows.push(row);
    solids.push(null);
    bounds.push(own);
  }

  function build(row:CastInsertPlacement):Part {
    var owned:Array<Part> = [];
    function own(part:Part):Part {
      owned.push(part);
      return part;
    }
    var placed:Part;
    try {
      var blanks = [own(machinekit.structural.InsertBodies.part(row.part.bodySpec()))];
      for (hardware in InsertAnchorGeometry.components(row.part.insertSpec())) {
        var screw = own(screws.posed(hardware.part, hardware.pose));
        screw.shape.setThreadedFaces([]);
        blanks.push(screw);
      }
      var blank = own(Part.fuseAll(blanks));
      placed = PosedParts.moved(blank, row.pose);
    } catch (error:Dynamic) {
      for (part in owned) part.close();
      throw error;
    }
    for (part in owned) part.close();
    return placed;
  }

  /** Neighbour `index` (as in `rows`), placed: kept by this object, do not close it. */
  public function solid(index:Int):Part {
    var known = solids[index];
    if (known == null) {
      known = build(rows[index]);
      solids[index] = known;
    }
    return known;
  }

  /** Bounds containing `solid(index)`, including its anchor hardware. */
  public function box(index:Int):PosedParts.PartBox return bounds[index];

  public function close():Void {
    for (part in solids) if (part != null) part.close();
    solids.resize(0);
    screws.close();
  }
}
