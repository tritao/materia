package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import machinekit.component.Bom;
import machinekit.component.BomItem;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.NemaStepper;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.SteppedShaft;
import machinekit.standard.DeepGrooveBearing;
import machinekit.structural.FrameAssembly;
import machinekit.structural.RectTube;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Block riding the lead screw. CAD frame: bore centred, spanning z=0..length. */
class Carriage extends MachineComponent {
	public final boreDiameter:Float;
	public final width:Float;
	public final length:Float;

	public function new(boreDiameter:Float, width:Float, length:Float) {
		var bore = Dimension.format(boreDiameter), widthText = Dimension.format(width), lengthText = Dimension.format(length);
		super('CARRIAGE-D$bore-${widthText}x$lengthText', 'Lead-screw carriage ${widthText}x${widthText}x$lengthText',
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
 * modelled) through a `ShaftCoupling` on a continuous joint; the screw carries a sliding carriage
 * on a prismatic joint along its axis.
 *
 * Layout along the screw, measured from its input end (`screwStart` in world, the motor shaft
 * tip): the coupling's screw half, a small gap, pillow block A, `margin`, the carriage's `stroke`
 * of travel, `margin`, then pillow block B flush with the screw's far end. The carriage joint's
 * limits (`travelMin`..`travelMax`, bore centre from the screw input) keep it `margin` clear of
 * both pillow blocks. Both housings mount on their outboard face (A toward the motor, B turned
 * over toward the far end), so their screw shanks point away from the carriage; only the screw
 * heads face it, and `margin` must exceed their height.
 *
 * Two flange bearing housings (`PillowBlock`) and a rect-tube rail represent the fixed machine
 * frame -- they are independently placed, fixed instances in the same `AssemblyModel`, not mated
 * to the screw: a real frame constrains the screw at both ends (making the assembly statically
 * indeterminate), which this simplified kinematic model does not attempt to capture. The rail
 * runs the screw's length beside it (offset along world -Y, past the housings and carriage) so
 * it intersects nothing.
 */
class LinearAxis {
	/** Axial gap between the coupling's end and pillow block A's housing. */
	public static inline var COUPLING_GAP:Float = 2;
	/** Gap between the rail and the widest part around the screw. */
	public static inline var RAIL_GAP:Float = 5;

	public final motor:NemaStepper;
	public final coupling:ShaftCoupling;
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
	/** World z of the screw's input end, with the motor at the identity pose. */
	public final screwStart:Float;
	/** Pillow block bearing centres, measured from the screw's input end. */
	public final bearingAPosition:Float;
	public final bearingBPosition:Float;
	/** Carriage joint limits: bore centre measured from the screw's input end. */
	public final travelMin:Float;
	public final travelMax:Float;

	/** `bearingDesignation` defaults to the first catalog deep groove bearing whose bore equals
	 * `screwDiameter` (6000 for 10 mm); a given bearing must match the screw diameter. `margin` is
	 * the clearance between the carriage at either travel limit and the adjacent pillow block.
	 */
	public function new(motorFrame:Int = 23, screwDiameter:Float = 10, stroke:Float = 200,
			?bearingDesignation:String, margin:Float = 20) {
		if (!(screwDiameter > 0)) throw "Linear axis needs a positive screw diameter";
		if (!(stroke > 0)) throw "Linear axis needs a positive stroke";
		if (!(margin > 0)) throw "Linear axis needs a positive end margin";
		this.stroke = stroke;
		this.margin = margin;
		motor = NemaStepper.frame(motorFrame);
		bearing = bearingDesignation == null ? matchingBearing(screwDiameter) : DeepGrooveBearing.metric(bearingDesignation);
		if (!(Math.abs(bearing.bore - screwDiameter) < 1e-9))
			throw 'Bearing "${bearing.spec.designation}" bore does not match the screw diameter';
		coupling = new ShaftCoupling(motor.spec.shaftDiameter, screwDiameter);
		carriage = new Carriage(screwDiameter, screwDiameter * 4, screwDiameter * 6);
		pillowBlockA = new PillowBlock(bearing);
		pillowBlockB = new PillowBlock(bearing);
		if (!(margin > pillowBlockA.screw.spec.headHeight))
			throw "Linear axis end margin must clear the pillow block screw heads";
		var depth = pillowBlockA.housing.depth;
		bearingAPosition = coupling.length / 2 + COUPLING_GAP + depth / 2;
		travelMin = bearingAPosition + depth / 2 + margin + carriage.length / 2;
		travelMax = travelMin + stroke;
		bearingBPosition = travelMax + carriage.length / 2 + margin + depth / 2;
		length = bearingBPosition + depth / 2;
		screwStart = motor.connector("shaftTip").frame.z;
		screw = new SteppedShaft([{diameter: screwDiameter, length: length}]);
		rail = new RectTube(Math.max(20, screwDiameter * 2), Math.max(15, screwDiameter * 1.5), 2);
		var housing = pillowBlockA.housing;
		var screwHeadReach = housing.boltSpacing / 2 + pillowBlockA.screw.spec.headDiameter / 2;
		var reach = Math.max(Math.max(housing.face / 2, screwHeadReach), Math.max(carriage.width, coupling.outerDiameter) / 2);
		var railY = -(reach + RAIL_GAP + rail.height / 2);
		frame = new FrameAssembly();
		frame.point("railStart", 0, railY, screwStart);
		frame.point("railEnd", 0, railY, screwStart + length);
		frame.member("rail", "railStart", "railEnd", rail);
	}

	static function matchingBearing(bore:Float):DeepGrooveBearing {
		for (designation in DeepGrooveBearing.catalog().designations())
			if (Math.abs(DeepGrooveBearing.catalog().get(designation).bore - bore) < 1e-9)
				return DeepGrooveBearing.metric(designation);
		throw 'No catalog deep groove bearing has a ${Dimension.format(bore)} mm bore to match the screw';
	}

	public function assembly():AssemblyModel {
		var model = new AssemblyModel();
		motor.addTo(model, "motor");
		coupling.addTo(model, "coupling");
		model.mate("coupling", "continuous", "motor", "shaftTip", "coupling", "axis");
		screw.addTo(model, "screw");
		model.mate("coupling-screw", "fixed", "coupling", "axis", "screw", "input");
		carriage.addTo(model, "carriage");
		model.mateOnAxis("carriage-slide", "prismatic", "screw", "input", "carriage", "bore",
			{x: 0, y: 1, z: 0}, travelMin, {lower: travelMin, upper: travelMax, velocity: null, effort: null});
		var depth = pillowBlockA.housing.depth;
		// Housing A: mounting face (local z=0) toward the motor.
		var poseA:AssemblyFrame = {x: 0, y: 0, z: screwStart + bearingAPosition - depth / 2, qx: 0, qy: 0, qz: 0, qw: 1};
		pillowBlockA.addTo(model, "pillowA", poseA);
		// Housing B: turned half a turn about X so its mounting face points at the far end.
		var poseB:AssemblyFrame = {x: 0, y: 0, z: screwStart + bearingBPosition + depth / 2, qx: 1, qy: 0, qz: 0, qw: 0};
		pillowBlockB.addTo(model, "pillowB", poseB);
		return model;
	}

	/** The rail as a BOM line: its profile cut to the screw length. */
	public function railBomItem():BomItem {
		var lengthText = Dimension.format(length);
		return {partNumber: '${rail.designation}-L$lengthText', description: '${rail.description}, $lengthText mm long',
			quantity: 1, material: "steel"};
	}

	public function bom():Bom {
		var result = new Bom();
		result.addComponent(motor);
		result.addComponent(coupling);
		result.addComponent(screw);
		result.addComponent(carriage);
		result.add(railBomItem());
		for (line in pillowBlockA.bom().lines()) result.add(line);
		for (line in pillowBlockB.bom().lines()) result.add(line);
		return result;
	}

	/** Instance id to component, for geometry generation by a preview or exporter. Excludes the
	 * frame rail, which is generated through `frame.geometry("rail")` instead.
	 */
	public function components():Array<{id:String, component:MachineComponent}> {
		var result:Array<{id:String, component:MachineComponent}> = [
			{id: "motor", component: motor}, {id: "coupling", component: coupling}, {id: "screw", component: screw},
			{id: "carriage", component: carriage}];
		for (entry in pillowBlockA.components()) result.push({id: 'pillowA-${entry.id}', component: entry.component});
		for (entry in pillowBlockB.components()) result.push({id: 'pillowB-${entry.id}', component: entry.component});
		return result;
	}
}
