import gearkit.GearSpec;
import gearkit.ToothProfile;
import stockkit.RingStock;
import stockkit.RingGrid;
import stockkit.RingCut;
import stockkit.RingGearTarget;
import stockkit.RingInterval;

class RingStockTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void { assertions++; if (!value) throw message; }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void
    check(Math.isFinite(a) && Math.abs(a - b) <= tolerance, '$message: $a versus $b');
  static function rejects(f:Void->Void, message:String):Void {
    var failed = false;
    try f() catch (_:Dynamic) failed = true;
    check(failed, message);
  }
  static function identical(a:Array<RingInterval>, b:Array<RingInterval>):Void {
    check(a.length == b.length, "same number of intervals");
    for (i in 0...a.length) {
      check(a[i].lo == b[i].lo && a[i].hi == b[i].hi, "identical angular endpoints");
      check(a[i].loGash == b[i].loGash && a[i].hiGash == b[i].hiGash
        && a[i].loPass == b[i].loPass && a[i].hiPass == b[i].hiPass, "identical provenance");
    }
  }
  public static function run():Void {
    var grid = new RingGrid(1, 3, 0, 2, 0.25, 0.5);
    var stock = RingStock.annulus(grid);
    check(grid.radialCount == 8 && grid.axialCount == 4, "grid counts");
    near(grid.radius(0), 1.125, 0, "cell-centred radius");
    stock.cutSlab(0, 0.3, 1.2, 3, 7);
    for (r in 0...grid.radialCount) for (z in 0...grid.axialCount) {
      var circle = stock.circle(r, z), radius = grid.radius(r);
      var a = Math.acos(Math.min(1, 1.2 / radius)), b = Math.acos(0.3 / radius);
      near(circle[a == 0 ? 0 : 1].lo, b, 1e-13, "analytic slab endpoint");
      check(circle[a == 0 ? 0 : 1].loGash == 3 && circle[a == 0 ? 0 : 1].loPass == 7, "slab provenance");
    }
    var before = stock.circle(2, 0), shot = stock.snapshot();
    stock.cutRegion([new RingCut(2, 0, 1, 0, 2 * Math.PI, 10, 11)]);
    check(stock.circle(2, 0).length == 0, "full circle removed");
    stock.restore(shot); identical(before, stock.circle(2, 0));
    var peer = RingStock.annulus(grid);
    peer.setThreads(4); peer.cutSlab(0, 0.3, 1.2, 3, 7);
    for (r in 0...grid.radialCount) for (z in 0...grid.axialCount) identical(stock.circle(r,z), peer.circle(r,z));
    var comparison = peer.compareStock(stock);
    near(comparison.leftoverVolume(), 0, 0, "equal stock leftover");
    near(comparison.gougeVolume(), 0, 0, "equal stock gouge");
    var mesh = stock.contour(0.1);
    check(mesh.vertexCount > 0 && mesh.triangleCount > 0, "ring contour");
    check(mesh.positions.length == mesh.vertexCount * 12, "position stream");
    check(mesh.indices.length == mesh.triangleCount * 12, "index stream");
    check(mesh.normals.length == mesh.positions.length && mesh.colors.length == mesh.vertexCount * 4, "normal/color streams");
    stock.dispose(); peer.restore(shot); identical(before, peer.circle(2,0));
    shot.dispose(); shot.dispose();
    rejects(() -> { peer.restore(shot); }, "closed snapshot rejected");
    var saved = peer.circle(0,0);
    rejects(() -> { peer.cutRegion([new RingCut(0,0,1,0,1,0,0), new RingCut(0,3,2,0,1,0,0)]); }, "invalid batch rejected");
    identical(saved, peer.circle(0,0));
    rejects(() -> { peer.cutRegion([new RingCut(-1,0,1,0,1,0,0)]); }, "negative indices rejected");
    peer.dispose(); peer.dispose();
    rejects(() -> { peer.circle(0,0); }, "disposed stock rejected");
    rejects(() -> { new RingGrid(0,1,0,1,0.1,0.1); }, "zero inner radius rejected");
    gearChecks();
    Sys.println('RingStockTests: $assertions assertions passed');
  }
  static function gearChecks():Void {
    var alpha = 20 * Math.PI / 180;
    for (internal in [false, true]) for (shift in [-0.2, 0.0, 0.2]) {
      var spec = new GearSpec(0.3, internal ? 202 : 200, 8, alpha, internal, shift);
      var profile = new ToothProfile(spec);
      var blank = RingStock.fromGearBlank(spec, 0.6, 0.06, 4);
      var ideal = RingGearTarget.create(blank.grid, spec);
      var checkIdeal = ideal.compare(spec);
      near(checkIdeal.leftoverVolume(), 0, 0, "ideal gear agrees with GearSpec");
      near(checkIdeal.gougeVolume(), 0, 0, "ideal gear has no gouge");
      // At a pitch-circle sample, compare independent involute thickness (not the target builder).
      var g = new RingGrid(spec.pitchRadius() - 0.001, spec.pitchRadius() + 0.001, 0, 8, 1, 4);
      var atPitch = RingGearTarget.create(g, spec);
      var intervals = atPitch.circle(0,0), angle = 0.0;
      for (span in intervals) angle += span.hi - span.lo;
      near(angle * spec.pitchRadius(), spec.teeth * spec.pitchToothThickness(), 1e-9, "pitch-circle tooth thickness oracle");
      var difference = blank.compare(spec);
      check(difference.leftoverVolume() > 0, "blank has leftover between teeth");
      near(difference.gougeVolume(), 0, 1e-12, "blank contains ideal gear");
      ideal.cutRegion([new RingCut(ideal.grid.radialCount >> 1, 0, 1, 0, 2 * Math.PI, 1, 2)]);
      check(ideal.compare(spec).gougeVolume() > 0, "overcut appears as gouge");
      atPitch.dispose(); ideal.dispose(); blank.dispose();
    }
    // Exercise circles that cross both root and tip fillets, not only the active involute.
    var spec = new GearSpec(1,40,10,alpha);
    var p = new ToothProfile(spec);
    for (root in [false,true]) {
      var sample = root ? p.rootFillet(0.45) : p.tipFillet(0.45), radius = sample.radius();
      var grid = new RingGrid(radius-0.00001,radius+0.00001,0,10,1,10);
      var target = RingGearTarget.create(grid,spec), angular = 0.0;
      for (span in target.circle(0,0)) angular += span.hi-span.lo;
      near(angular, 2*spec.teeth*Math.atan2(sample.x,sample.y), 1e-10, "fillet crossings match GearKit curve");
      target.dispose();
    }
    var faceGrid = new RingGrid(19.99,20.01,-1,3,1,1);
    var shortGear = RingGearTarget.create(faceGrid,new GearSpec(1,40,2,alpha));
    check(shortGear.circle(0,0).length == 0 && shortGear.circle(0,3).length == 0, "target excludes circles outside face");
    check(shortGear.circle(0,1).length > 0 && shortGear.circle(0,2).length > 0, "target includes circles within face");
    shortGear.dispose();
    var grid = new RingGrid(2,3,0,2,0.5,1);
    var full = RingStock.annulus(grid), empty = RingStock.annulus(grid);
    empty.cutRegion([for (r in 0...grid.radialCount) new RingCut(r,0,grid.axialCount,0,2*Math.PI,0,0)]);
    near(full.compareStock(empty).leftoverVolume(), Math.PI*(9-4)*2, 1e-12, "annular cell-volume integral");
    near(empty.compareStock(full).gougeArc(0,0), 2*Math.PI*grid.radius(0), 1e-12, "gouge arc in mm");
    full.dispose();empty.dispose();
  }
  public static function main():Void run();
}
