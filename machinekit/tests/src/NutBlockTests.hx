import cadkit.modeling.Part;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.AxisBuilder;
import machinekit.component.ManufacturingRoute;
import machinekit.component.ManufacturingRoute.RouteKind;
import machinekit.component.Procurement.ProcurementCodec;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.motion.BallNut;
import machinekit.motion.NutBlock;
import machinekit.motion.SfuNut;
import materia.assembly.AssemblyFrames;

/** The nut mounting cut-out and nut block (L3 of the hobber frame plan). */
class NutBlockTests {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw 'Nut block: $message';
  }

  static function rejects(action:Void -> Void, label:String):Void {
    var rejected = false;
    try action() catch (_:Dynamic) rejected = true;
    check(rejected, '$label should be rejected');
  }

  /** The nut seated on the block, the block placed at the origin. */
  static function seated(nut:machinekit.motion.LeadScrewNut, host:machinekit.component.MachineComponent, nutPose:materia.assembly.AssemblyRecord.AssemblyFrame):AxisBuilder {
    var builder = new AxisBuilder();
    builder.place("block", host, AssemblyFrames.translation(0, 0, 0));
    builder.attach("nut", nut, nutPose, "block");
    return builder;
  }

  static function cutouts():Void {
    for (nut in [(new SfuNut("SFU1605", 13) : machinekit.motion.LeadScrewNut), new BallNut()]) {
      var face = nut.mountingFace();
      check(face != null, "a flanged nut has a mounting face");
      if (face == null) return;
      var tool = face.tool(10);
      check(tool.volume() > 0, "the nut's cut-out has volume");
      tool.close();
      var block = new NutBlock("NB-" + nut.designation.substr(0, 20), nut, 25);
      check(block.centreHeight == 25 && block.width >= nut.flangeDiameter + 2 * NutBlock.EDGE, "block sized from the nut");
      var sound = AssemblyIntegrity.check(seated(nut, block, block.nutPose()), {strict: true});
      check(sound.errors().length == 0, 'a nut on its block should be sound, found: ${[for (item in sound.errors()) item.message].join("; ")}');
      // A plain bar with no bore and no holes is not cut for the nut.
      var plain = new GantryPlate("NB-PLAIN", "Plain bar", block.width, block.length, block.thickness);
      var wrong = AssemblyIntegrity.check(seated(nut, plain, AssemblyFrames.compose(AssemblyFrames.translation(0, block.length / 2 - 25, 0), block.nutPose())), {flangeCutouts: true});
      check([for (item in wrong.errors()) item.code].indexOf("integrity.mount.cutout") >= 0, "an uncut bar lacks the nut's cut-out");
      rejects(() -> new NutBlock("NB-LOW", nut, nut.flangeDiameter / 2 - 2), "a centre height under the flange rim");
    }
    rejects(() -> new NutBlock("NB-BARREL", new machinekit.motion.LeadScrewNut(new machinekit.motion.LeadScrewThread(MetricTrapezoidal, 8, 2, 4), 4, false), 30), "an unflanged nut");
  }

  /** The hole list of the drawing package: bore, flange holes through, base holes blind, all on the bar. */
  static function holeList():Void {
    var nut = new BallNut(), block = new NutBlock("NB-HOLES", nut, 28);
    var holes = block.holes(), body = block.geometry(), box = body.shape.bounds(), lo = box.get_min(), hi = box.get_max();
    check(holes.length == 1 + nut.boltCount + 2, "a bore, the flange holes and two base holes: " + holes.length);
    check(holes[0].kind == "bore" && holes[0].diameter > nut.bodyDiameter, "the bore clears the barrel");
    for (h in holes) {
      check(h.position[0] >= lo.get_x() - 1e-6 && h.position[0] <= hi.get_x() + 1e-6 && h.position[1] >= lo.get_y() - 1e-6
        && h.position[1] <= hi.get_y() + 1e-6 && h.position[2] >= lo.get_z() - 1e-6 && h.position[2] <= hi.get_z() + 1e-6, 'hole ${h.id} mouth on the bar');
      check(h.kind != "tapped" || (h.thread != null && h.diameter < h.nominal), 'tapped hole ${h.id} is cut at the tap drill');
    }
    var base = holes.filter(h -> h.thread == NutBlock.BASE_THREAD);
    check(base.length == 2 && base[0].depth == NutBlock.BASE_DEPTH, "two blind base holes");
    body.close();
  }

  static function routes():Void {
    var nut = new BallNut();
    var block = new NutBlock("NB-ROUTE", nut, 25);
    var route = ManufacturingRoute.of(block);
    check(route != null && route.kind == RouteKind.GroundStock && route.machinedFaces().length == 2, "the nut block is cut from ground stock with two machined faces");
    if (route == null) return;
    var bom = block.bom;
    check(bom.procurement != null && ProcurementCodec.document(bom.procurement).process == route.process(), "the BOM's Make process is the route's");
    var builder = seated(nut, block, block.nutPose());
    var listed = builder.manufacturingRoutes();
    check(listed.length == 1 && listed[0].partNumber == bom.partNumber && listed[0].route == route, "the assembly lists its parts' routes by part number");
  }

  public static function run():Void {
    cutouts();
    holeList();
    routes();
    Sys.println("nut block: cut-out, block, ground stock rows and routes pass");
  }
}

/** Standalone run. */
function main():Void NutBlockTests.run();
