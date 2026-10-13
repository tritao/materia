import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.ProcessBendRule;
import machinekit.pressbrake.ToolLayout;
import machinekit.pressbrake.ToolRow;
import machinekit.pressbrake.ToolSegment;
import machinekit.pressbrake.ToolStation;
import machinekit.pressbrake.ToolingCatalogue;
import machinekit.pressbrake.planning.BendSequencer;
import machinekit.pressbrake.planning.BendTimeModel;
import machinekit.pressbrake.planning.GeometricGripCheck;
import machinekit.pressbrake.planning.GripperBox;
import machinekit.pressbrake.planning.PressGeometry;
import machinekit.pressbrake.planning.CoalPrismQuery;

/** A narrow channel must name the physical punch interference after exhausting both orders. */
class BendSequenceFailureChecks {
	public static function main():Void run();
	public static function run():Void {
		var station = new ToolStation("straight-raised-V12",
			new ToolRow([new ToolSegment(Punch(ToolingCatalogue.straight()),100),
				new ToolSegment(Punch(ToolingCatalogue.straight()),50)],[0]),
			new ToolRow([new ToolSegment(SingleV(ToolingCatalogue.raisedV12()),200)],[]),700);
		var press = new PressBrake(true,new ToolLayout(1600,[station]),null,30);
		var stock = new SheetMaterial("sequence-steel",1.5,
			BendRule.Process(new ProcessBendRule(station.processTooling())),{uts:400,yield:250,youngs:205000});
		var sheet = new Sheet(stock), root = sheet.addWall(WallOutline.rectangle(15,120));
		var radius = stock.defaultRadius(Math.PI/2);
		var left = SheetDemoParts.flange(sheet,root,1,40,Math.PI/2,radius);
		var right = SheetDemoParts.flange(sheet,root,3,40,Math.PI/2,radius);
		var geometry = PressGeometry.of(press), prisms = new CoalPrismQuery();
		var grips = new GeometricGripCheck(geometry,new GripperBox(10,10,15),prisms);
		var timing = new BendTimeModel(press.phaseSpeeds,1,3,1,4,2,200,100,0.3);
		var search = new BendSequencer(geometry,new AirBending(),timing,grips,prisms);
		switch search.plan(sheet,grips.candidates(sheet),20) {
			case NoSequence(failures):
				var firstOrder = false, secondOrder = false;
				for (failure in failures) {
					trace(failure.completed().join(",") + ": " + failure.diagnosis.toString());
					var completed = failure.completed();
					if (completed.length == 1 && failure.diagnosis.code == "press.collision")
						for (element in failure.diagnosis.elements) if (StringTools.startsWith(element,"punch:")) {
							if (completed[0] == 1) firstOrder = true;
							if (completed[0] == 2) secondOrder = true;
						}
				}
				if (!firstOrder || !secondOrder) throw "Exhausted channel search must retain the proved punch collision in both orders";
			case Planned(_): throw "Narrow channel must refuse every order that hits the straight punch";
			case SearchLimit(_,reason): throw "Collision refusal must exhaust the domain: " + reason.toString();
		}
		trace("Bend sequence collision refusal passed");
	}
}
