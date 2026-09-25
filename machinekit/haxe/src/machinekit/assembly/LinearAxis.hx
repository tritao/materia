package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.NemaStepper;
import machinekit.motion.SteppedShaft;
import machinekit.standard.DeepGrooveBearing;
import machinekit.structural.FrameAssembly;
import machinekit.structural.RectTube;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Block riding the lead screw. CAD frame: bore centred, spanning z=0..length. */
class Carriage extends MachineComponent {
	public final boreDiameter:Float;
	public final width:Float;
	public final length:Float;

	public function new(boreDiameter:Float, width:Float, length:Float) {
		super('CARRIAGE-D${boreDiameter}-${width}x${length}', 'Lead-screw carriage ${width}x${width}x${length}',
			"aluminium 6061");
		this.boreDiameter = boreDiameter;
		this.width = width;
		this.length = length;
		addConnector("bore", Axis, Solids.axial(0, 0, length / 2));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.box(width, width, length);
		if (detail == Envelope) return body;
		return Solids.cut(body, [Solids.cylinder(boreDiameter / 2, -0.1, length + 0.1)]);
	}
}

/** Parametric linear axis: a NEMA motor drives a lead screw (its thread is semantic, not
 * modelled) through a continuous coupling; the screw carries a sliding carriage on a prismatic
 * joint along its axis. Two flange bearing housings (`PillowBlock`) and a rect-tube rail
 * represent the fixed machine frame -- they are independently placed, fixed instances in the
 * same `AssemblyModel`, not mated to the screw: a real frame constrains the screw at both ends
 * (making the assembly statically indeterminate), which this simplified kinematic model does not
 * attempt to capture.
 */
class LinearAxis {
	public final motor:NemaStepper;
	public final screw:SteppedShaft;
	public final bearing:DeepGrooveBearing;
	public final carriage:Carriage;
	public final pillowBlockA:PillowBlock;
	public final pillowBlockB:PillowBlock;
	public final rail:RectTube;
	public final frame:FrameAssembly;
	public final stroke:Float;
	public final margin:Float;
	public final length:Float;

	public function new(motorFrame:Int = 23, screwDiameter:Float = 10, stroke:Float = 200,
			bearingDesignation:String = "6001", margin:Float = 20) {
		if (!(screwDiameter > 0)) throw "Linear axis needs a positive screw diameter";
		if (!(stroke > 0)) throw "Linear axis needs a positive stroke";
		if (!(margin > 0)) throw "Linear axis needs a positive end margin";
		this.stroke = stroke;
		this.margin = margin;
		length = stroke + 2 * margin;
		motor = NemaStepper.frame(motorFrame);
		bearing = DeepGrooveBearing.metric(bearingDesignation);
		if (!(bearing.bore >= screwDiameter)) throw 'Bearing "$bearingDesignation" bore is smaller than the screw diameter';
		screw = new SteppedShaft([{diameter: screwDiameter, length: length}]);
		carriage = new Carriage(screwDiameter, screwDiameter * 4, screwDiameter * 6);
		pillowBlockA = new PillowBlock(bearing);
		pillowBlockB = new PillowBlock(bearing);
		rail = new RectTube(Math.max(20, screwDiameter * 2), Math.max(15, screwDiameter * 1.5), 2);
		frame = new FrameAssembly();
		frame.point("railStart", 0, 0, 0);
		frame.point("railEnd", 0, 0, length);
		frame.member("rail", "railStart", "railEnd", rail);
	}

	public function assembly():AssemblyModel {
		var model = new AssemblyModel();
		motor.addTo(model, "motor");
		screw.addTo(model, "screw");
		model.mate("coupling", "continuous", "motor", "shaftTip", "screw", "input");
		carriage.addTo(model, "carriage");
		// The prismatic coordinate places the carriage's bore centre relative to the screw's
		// `input` connector (its z=0 end), so travel is clamped to keep the carriage on the screw.
		var travelMin = carriage.length / 2, travelMax = length - carriage.length / 2;
		model.mateOnAxis("carriage-slide", "prismatic", "screw", "input", "carriage", "bore",
			{x: 0, y: 1, z: 0}, margin + carriage.length / 2,
			{lower: travelMin, upper: travelMax, velocity: null, effort: null});
		pillowBlockA.addTo(model, "pillowA", housingPose(margin / 2));
		pillowBlockB.addTo(model, "pillowB", housingPose(length - margin / 2));
		return model;
	}

	function housingPose(boreZ:Float):AssemblyFrame
		return AssemblyFrames.translation(0, 0, boreZ - pillowBlockA.housing.depth / 2);

	public function bom():Bom {
		var result = new Bom();
		result.addComponent(motor);
		result.addComponent(screw);
		result.addComponent(carriage);
		for (line in pillowBlockA.bom().lines()) result.add(line);
		for (line in pillowBlockB.bom().lines()) result.add(line);
		return result;
	}

	/** Instance id to component, for geometry generation by a preview or exporter. Excludes the
	 * frame rail, which is generated through `frame.geometry("rail")` instead.
	 */
	public function components():Array<{id:String, component:MachineComponent}> {
		var result:Array<{id:String, component:MachineComponent}> = [
			{id: "motor", component: motor}, {id: "screw", component: screw}, {id: "carriage", component: carriage}];
		for (entry in pillowBlockA.components()) result.push({id: 'pillowA-${entry.id}', component: entry.component});
		for (entry in pillowBlockB.components()) result.push({id: 'pillowB-${entry.id}', component: entry.component});
		return result;
	}
}
