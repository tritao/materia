import machinekit.component.ManufacturingRoute;
import machinekit.structural.GroundStock;

/** Ground stock rows and manufacturing routes (L4 of the hobber frame plan). */
class ManufacturingRouteTests {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw 'Manufacturing route: $message';
  }

  static function rejects(action:Void -> Void, label:String):Void {
    var rejected = false;
    try action() catch (_:Dynamic) rejected = true;
    check(rejected, '$label should be rejected');
  }

  static function stock():Void {
    check(GroundStock.plate(12).thickness == 12 && GroundStock.plate(12).thicknessTolerance > 0, "a standard ground plate row");
    rejects(() -> GroundStock.plate(13), "a 13 mm ground plate");
    check(GroundStock.bar(60, 16).material == "steel" && GroundStock.bar(60, 16).kind == "flat bar", "a standard ground bar row");
    rejects(() -> GroundStock.bar(7, 16), "a 7 mm wide bar");
    rejects(() -> GroundStock.bar(12, 16), "a bar thicker than it is wide");
    check(GroundStock.atLeast("flat bar", 14) == 15 && GroundStock.atLeast("plate", 10) == 10 && GroundStock.atLeast("plate", 17) == 20, "stock is picked at the next standard thickness");
    check(GroundStock.plate(6).thicknessTolerance < GroundStock.plate(40).thicknessTolerance, "thin stock is ground tighter");
  }

  static function routes():Void {
    var face = {name: "seat", tolerance: 0.02, roughnessRa: (1.6 : Null<Float>)};
    rejects(() -> ManufacturingRoute.milled(3, "general", [face], 200), "three setups");
    rejects(() -> ManufacturingRoute.milled(2, "general", [face], 700), "a 700 mm milled part");
    rejects(() -> ManufacturingRoute.milled(1, "tight", [], 100), "a tight class naming no face");
    rejects(() -> ManufacturingRoute.milled(1, "exact", [face], 100), "an unknown tolerance class");
    rejects(() -> ManufacturingRoute.groundStock(GroundStock.plate(10), [{name: "bad", tolerance: 0, roughnessRa: null}]), "a zero tolerance");
    var milled = ManufacturingRoute.milled(2, "fine", [face], 300);
    check(milled.process() == "CNC milling, 2 setups, fine tolerance" && milled.supplier() == "machine shop", "a milled route says what and where");
    check(ManufacturingRoute.cutPlate().supplier() == "plate cutter" && ManufacturingRoute.turned(1, "general", []).supplier() == "lathe shop", "cut and turned suppliers");
    var ground = ManufacturingRoute.groundStock(GroundStock.bar(60, 16), [face]);
    check(ground.supplier() == "stock supplier and machine shop" && ground.machinedFaces().length == 1, "a ground route with a machined face");
    check(ManufacturingRoute.groundStock(GroundStock.plate(10)).supplier() == "stock supplier", "ground stock as ground goes to the stock supplier only");

  }

  public static function run():Void {
    stock();
    routes();
    Sys.println("manufacturing routes: ground stock rows and routes pass");
  }
}

/** Standalone run. */
function main():Void ManufacturingRouteTests.run();
