import machinekit.printing.PrintMaterialSpec;

class PrintMaterialTests {
	static function check(ok:Bool,message:String):Void { if(!ok)throw message; }
	public static function main():Void run();
	public static function run():Void {
		var material=PrintMaterialSpec.assumedPla175();
		check(material.filamentDiameter==0.00175 && material.minExtrusionTemperature==170 && material.meltLag==0.04,
			"Default material changed the example's assumptions");
		var instant=new PrintMaterialSpec("Explicit cold/instant process",0.00173,0,0);
		check(instant.minExtrusionTemperature==0 && instant.meltLag==0,"Explicit zero threshold/lag was lost");
		for(bad in [0.0,-1.0,Math.NaN,Math.POSITIVE_INFINITY]) {
			var rejected=false;try new PrintMaterialSpec("invalid",bad,170,0.04) catch(_:Dynamic)rejected=true;
			check(rejected,"Invalid filament diameter accepted");
		}
		for(bad in [-1.0,Math.NaN,Math.POSITIVE_INFINITY]) {
			var rejected=false;try new PrintMaterialSpec("invalid",0.00175,bad,0.04) catch(_:Dynamic)rejected=true;
			check(rejected,"Invalid material temperature accepted");
			rejected=false;try new PrintMaterialSpec("invalid",0.00175,170,bad) catch(_:Dynamic)rejected=true;
			check(rejected,"Invalid material lag accepted");
		}
		var rejected=false;try new PrintMaterialSpec(" ",0.00175,170,0.04) catch(_:Dynamic)rejected=true;
		check(rejected,"Unnamed feedstock accepted");
		Sys.println("Print material specification checks passed");
	}
}
