package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import machinekit.component.Bom;
import machinekit.component.BomItem;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.LeadScrewTransmission;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.motion.LeadScrewThread.LeadScrewHand;
import machinekit.motion.LinearBearing;
import machinekit.motion.NemaStepper;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.SteppedShaft;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.ClearanceFit;
import machinekit.structural.FrameAssembly;
import machinekit.structural.RectTube;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Block riding the lead screw. CAD frame: bore centred, spanning z=0..length. */
class Carriage extends MachineComponent {
	public final boreDiameter:Float;
	public final width:Float;
	public final length:Float;
	public final guideSpacing:Float;
	public final guideSeatDiameter:Float;
	public final nut:LeadScrewNut;

	public function new(boreDiameter:Float, width:Float, length:Float, guideSpacing:Float, guideSeatDiameter:Float, nut:LeadScrewNut) {
		var bore = Dimension.format(boreDiameter), widthText = Dimension.format(width), lengthText = Dimension.format(length);
		super('CARRIAGE-D$bore-${widthText}x$lengthText-G${Dimension.format(guideSpacing)}-${nut.designation}', 'Lead-screw carriage ${widthText}x${widthText}x$lengthText',
			"aluminium 6061");
		this.boreDiameter = boreDiameter;
		this.width = width;
		this.length = length;
		this.guideSpacing = guideSpacing;
		this.guideSeatDiameter = guideSeatDiameter;
		this.nut = nut;
		addConnector("bore", Axis, Solids.axial(0, 0, length / 2));
		addConnector("nutMount", Face, Solids.axial(0, 0, 0));
		addConnector("guideA", Axis, Solids.axial(-guideSpacing, 0, length / 2));
		addConnector("guideB", Axis, Solids.axial(guideSpacing, 0, length / 2));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.box(width, width, length);
		if (detail == Envelope) return body;
		var tools = [
			Solids.cylinder(boreDiameter / 2, -0.1, length + 0.1),
			Solids.cylinder(guideSeatDiameter / 2, -0.1, length + 0.1, -guideSpacing),
			Solids.cylinder(guideSeatDiameter / 2, -0.1, length + 0.1, guideSpacing),
		];
		var mountHoleRadius = nut.mountScrewPart(10).clearanceDiameter(Medium) / 2;
		for (point in nut.boltPattern())
			tools.push(Solids.cylinder(mountHoleRadius, -0.1, length + 0.1, point.x, point.y));
		return Solids.cut(body, tools);
	}
}

/** Parametric lead-screw axis. The screw rotates through a coupling; a separate prismatic
 * joint keeps the carriage oriented to the fixed frame. `setTravel` applies the nut's lead to
 * both joint coordinates. Two fixed round guide rods pass through linear bearings carried by
 * the carriage. The nut's flange mounts on the carriage's motor-facing side.
 *
 * Layout along the screw: coupling, pillow block A, end margin, carriage travel, end margin,
 * then pillow block B. The margin clears the nut protruding from the carriage. Pillow block
 * housings, guide rods and the frame rail are fixed roots in the assembly. Their real mounting
 * structure and screw bearing closure remain outside this kinematic preview.
 */
class LinearAxis {
	/** Axial gap between the coupling's end and pillow block A's housing. */
	public static inline var COUPLING_GAP:Float = 2;
	/** Gap between the rail and the widest part around the screw. */
	public static inline var RAIL_GAP:Float = 5;

	public final motor:NemaStepper;
	public final coupling:ShaftCoupling;
	public final screw:LeadScrew;
	public final nut:LeadScrewNut;
	public final transmission:LeadScrewTransmission;
	public final guideRodA:SteppedShaft;
	public final guideRodB:SteppedShaft;
	public final guideBearingA:LinearBearing;
	public final guideBearingB:LinearBearing;
	public final guideSpacing:Float;
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
	 * The default screw is right-hand Tr10 × 2, single-start. Other diameters require `thread`.
	 */
	public function new(motorFrame:Int = 23, screwDiameter:Float = 10, stroke:Float = 200,
			?bearingDesignation:String, margin:Float = 30, ?thread:LeadScrewThread) {
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
		if (thread == null && screwDiameter != 10)
			throw "Linear axis needs an explicit thread for a nondefault screw diameter";
		var threadSpec = thread == null ? new LeadScrewThread(MetricTrapezoidal, 10, 2) : thread;
		if (Math.abs(threadSpec.screwDiameter - screwDiameter) > 1e-9)
			throw "Linear axis thread diameter must match the screw diameter";
		nut = new LeadScrewNut(threadSpec);
		guideBearingA = LinearBearing.metric("LM8UU");
		guideBearingB = LinearBearing.metric("LM8UU");
		guideSpacing = Math.max(25, screwDiameter * 2.5);
		var carriageWidth = 2 * (guideSpacing + guideBearingA.outerDiameter / 2 + 5);
		carriage = new Carriage(screwDiameter, carriageWidth, Math.max(screwDiameter * 6, guideBearingA.length + 4),
			guideSpacing, guideBearingA.outerDiameter, nut);
		pillowBlockA = new PillowBlock(bearing);
		pillowBlockB = new PillowBlock(bearing);
		if (!(margin > Math.max(pillowBlockA.screw.spec.headHeight, nut.bodyLength + nut.flangeThickness)))
			throw "Linear axis end margin must clear the pillow block screw heads and lead nut";
		var depth = pillowBlockA.housing.depth;
		bearingAPosition = coupling.length / 2 + COUPLING_GAP + depth / 2;
		travelMin = bearingAPosition + depth / 2 + margin + carriage.length / 2;
		travelMax = travelMin + stroke;
		transmission = new LeadScrewTransmission("coupling", "carriage-slide", nut.lead, travelMin, stroke, threadSpec.hand == RightHand ? 1 : -1);
		bearingBPosition = travelMax + carriage.length / 2 + margin + depth / 2;
		length = bearingBPosition + depth / 2;
		screwStart = motor.connector("shaftTip").frame.z;
		screw = new LeadScrew(threadSpec, length);
		guideRodA = new SteppedShaft([{diameter: guideBearingA.boreDiameter, length: length}]);
		guideRodB = new SteppedShaft([{diameter: guideBearingB.boreDiameter, length: length}]);
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
		model.mateOnAxis("carriage-slide", "prismatic", "motor", "shaftTip", "carriage", "bore",
			{x: 0, y: 1, z: 0}, travelMin, {lower: travelMin, upper: travelMax, velocity: null, effort: null});
		nut.addTo(model, "leadNut");
		model.mate("nut-carriage", "fixed", "carriage", "nutMount", "leadNut", "mountFace");
		guideBearingA.addTo(model, "guideBearingA");
		guideBearingB.addTo(model, "guideBearingB");
		model.mate("guide-bearing-a", "fixed", "carriage", "guideA", "guideBearingA", "axis");
		model.mate("guide-bearing-b", "fixed", "carriage", "guideB", "guideBearingB", "axis");
		var poseGuideA:AssemblyFrame = {x: -guideSpacing, y: 0, z: screwStart, qx: 0, qy: 0, qz: 0, qw: 1};
		var poseGuideB:AssemblyFrame = {x: guideSpacing, y: 0, z: screwStart, qx: 0, qy: 0, qz: 0, qw: 1};
		guideRodA.addTo(model, "guideRodA", poseGuideA);
		guideRodB.addTo(model, "guideRodB", poseGuideB);
		var depth = pillowBlockA.housing.depth;
		// Housing A: mounting face (local z=0) toward the motor.
		var poseA:AssemblyFrame = {x: 0, y: 0, z: screwStart + bearingAPosition - depth / 2, qx: 0, qy: 0, qz: 0, qw: 1};
		pillowBlockA.addTo(model, "pillowA", poseA);
		// Housing B: turned half a turn about X so its mounting face points at the far end.
		var poseB:AssemblyFrame = {x: 0, y: 0, z: screwStart + bearingBPosition + depth / 2, qx: 1, qy: 0, qz: 0, qw: 0};
		pillowBlockB.addTo(model, "pillowB", poseB);
		return model;
	}

	/** Apply the screw-to-nut transmission to both assembly coordinates. Travel is measured
	 * from the carriage's lower limit, and one screw turn advances it by `nut.lead`.
	 */
	public function setTravel(state:AssemblyState, travel:Float):Void
		transmission.setTravel(state, travel);

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
		result.addComponent(nut);
		result.addComponent(guideRodA);
		result.addComponent(guideRodB);
		result.addComponent(guideBearingA);
		result.addComponent(guideBearingB);
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
			{id: "carriage", component: carriage}, {id: "leadNut", component: nut},
			{id: "guideRodA", component: guideRodA}, {id: "guideRodB", component: guideRodB},
			{id: "guideBearingA", component: guideBearingA}, {id: "guideBearingB", component: guideBearingB}];
		for (entry in pillowBlockA.components()) result.push({id: 'pillowA-${entry.id}', component: entry.component});
		for (entry in pillowBlockB.components()) result.push({id: 'pillowB-${entry.id}', component: entry.component});
		return result;
	}
}
