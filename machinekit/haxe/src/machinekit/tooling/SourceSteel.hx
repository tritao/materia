package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.structural.MineralFoundation.CastInsertPlacement;

/**
 * The placed source steel a candidate insert is checked against (`InsertAnchorGeometry.checkSourceClearance`): each
 * neighbouring insert's blank fused with its anchor hardware, at its pose, with its bounds. A sizing loop tries many
 * candidates against the same neighbours, so each neighbour's solid is built once, when it is added, not per
 * candidate. The solids are native and belong to this object: close it when the loop is done.
 */
class SourceSteel {
  public final rows:Array<CastInsertPlacement> = [];
  final solids:Array<Part> = [];
  final bounds:Array<PosedParts.PartBox> = [];

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
    var owned:Array<Part> = [];
    function own(part:Part):Part {
      owned.push(part);
      return part;
    }
    var placed:Part;
    try {
      var blanks = [own(machinekit.structural.InsertBodies.part(row.part.bodySpec()))];
      for (hardware in InsertAnchorGeometry.components(row.part.insertSpec())) {
        var local = own(hardware.part.geometry());
        local.shape.setThreadedFaces([]);
        blanks.push(own(PosedParts.moved(local, hardware.pose)));
      }
      var blank = own(Part.fuseAll(blanks));
      placed = PosedParts.moved(blank, row.pose);
    } catch (error:Dynamic) {
      for (part in owned) part.close();
      throw error;
    }
    for (part in owned) part.close();
    rows.push(row);
    solids.push(placed);
    bounds.push(PosedParts.boxOf(placed));
  }

  /** Neighbour `index` (as in `rows`), placed: kept by this object, do not close it. */
  public function solid(index:Int):Part return solids[index];

  /** The bounds of `solid(index)`, including its anchor hardware. */
  public function box(index:Int):PosedParts.PartBox return bounds[index];

  public function close():Void {
    for (part in solids) part.close();
    solids.resize(0);
  }
}
