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
import machinekit.motion.LinearGuideSystem;
import machinekit.motion.LinearRailSystem;
import machinekit.motion.NemaStepper;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.SteppedShaft;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.BearingFit.BearingHousingFit;
import machinekit.standard.BearingFit.BearingShaftFit;
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
	public final guideSeatFit:BearingHousingFit;
	/** Carriage-side mounting frame for a profile rail block, when this carriage uses one. */
	public final railMountY:Float;
	public final hasRailMount:Bool;
	public final nut:LeadScrewNut;

	public function new(boreDiameter:Float, width:Float, length:Float, guideSpacing:Float, guideSeatDiameter:Float,
			nut:LeadScrewNut, guideSeatFit:BearingHousingFit = Slip, ?railMountY:Float) {
		var bore = Dimension.format(boreDiameter), widthText = Dimension.format(width), lengthText = Dimension.format(length);
		super('CARRIAGE-D$bore-${widthText}x$lengthText-G${Dimension.format(guideSpacing)}-${nut.designation}', 'Lead-screw carriage ${widthText}x${widthText}x$lengthText',
			"aluminium 6061");
		this.boreDiameter = boreDiameter;
		this.width = width;
		this.length = length;
		this.guideSpacing = guideSpacing;
		this.guideSeatDiameter = guideSeatDiameter;
		this.guideSeatFit = guideSeatFit;
		this.railMountY = railMountY == null ? 0 : railMountY;
		this.hasRailMount = railMountY != null;
		this.nut = nut;
		addConnector("bore", Axis, Solids.axial(0, 0, length / 2));
		addConnector("nutMount", Face, Solids.axial(0, 0, 0));
		addConnector("guideA", Axis, Solids.axial(-guideSpacing, 0, length / 2));
		addConnector("guideB", Axis, Solids.axial(guideSpacing, 0, length / 2));
		if (hasRailMount)
			addConnector("railMount", Mount, Solids.axial(0, this.railMountY, length / 2));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.box(width, width, length);
		if (detail == Envelope) return body;
		var tools = [Solids.cylinder(boreDiameter / 2, -0.1, length + 0.1)];
		if (!hasRailMount) {
			tools.push(Solids.cylinder(guideSeatDiameter / 2, -0.1, length + 0.1, -guideSpacing));
			tools.push(Solids.cylinder(guideSeatDiameter / 2, -0.1, length + 0.1, guideSpacing));
		}
		var mountHoleRadius = nut.mountScrewPart(10).clearanceDiameter(Medium) / 2;
		for (point in nut.boltPattern())
			tools.push(Solids.cylinder(mountHoleRadius, -0.1, length + 0.1, point.x, point.y));
		return Solids.cut(body, tools);
	}
}

/** Parametric lead-screw axis. The screw rotates through a coupling; a separate prismatic
 * joint keeps the carriage oriented to the fixed frame. `setTravel` applies the nut's lead to
 * both joint coordinates. The default uses two fixed round guide rods passing through linear
 * bearings carried by the carriage; `forRailProfile()` selects a catalog-backed profile rail
 * and block closure instead. The nut's flange mounts on the carriage's motor-facing side.
 *
 * Layout along the screw: coupling, flange bearing A, end margin, carriage travel, end margin,
 * then flange bearing B. The margin clears the nut protruding from the carriage. Flange bearing
 * housings, guide rods and the frame rail are fixed roots in the assembly. The `LinearGuideSystem`
 * keeps the guide bearing catalog row and shaft/seat fit intent with those parts. Their real
 * mounting structure and screw bearing closure remain outside this kinematic preview.
 */
class LinearAxis {
	/** Axial gap between the coupling's end and flange bearing A's housing. */
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
	/** Round-rod guide retained for the default axis and for source compatibility. */
	public final guideSystem:LinearGuideSystem;
	/** Profile-rail guide selected by `forRailProfile`; null for the round-rod default. */
	public final railGuide:Null<LinearRailSystem>;
	/** World-z offset of the profile rail's axis connector from the screw input. */
	public final railGuideOffset:Float;
	public final guideSpacing:Float;
	public final bearing:DeepGrooveBearing;
	public final carriage:Carriage;
	public final flangeBearingA:FlangeBearingAssembly;
	public final flangeBearingB:FlangeBearingAssembly;
	public final rail:RectTube;
	public final frame:FrameAssembly;
	public final stroke:Float;
	public final margin:Float;
	public final length:Float;
	/** World z of the screw's input end, with the motor at the identity pose. */
	public final screwStart:Float;
	/** Flange bearing centres, measured from the screw's input end. */
	public final bearingAPosition:Float;
	public final bearingBPosition:Float;
	/** Carriage joint limits: bore centre measured from the screw's input end. */
	public final travelMin:Float;
	public final travelMax:Float;

	/** `bearingDesignation` defaults to the first catalog deep groove bearing whose bore equals
	 * `screwDiameter` (6000 for 10 mm); a given bearing must match the screw diameter. `margin` is
	 * the clearance between the carriage at either travel limit and the adjacent flange bearing.
	 * The default screw is right-hand Tr10 × 2, single-start. Other diameters require `thread`.
	 */
	public function new(motorFrame:Int = 23, screwDiameter:Float = 10, stroke:Float = 200,
			?bearingDesignation:String, margin:Float = 30, ?thread:LeadScrewThread, ?railProfile:String) {
		if (!(screwDiameter > 0)) throw "Linear axis needs a positive screw diameter";
		if (!(stroke > 0)) throw "Linear axis needs a positive stroke";
		if (!(margin > 0)) throw "Linear axis needs a positive end margin";
		this.stroke = stroke;
		this.margin = margin;
		motor = NemaStepper.frame(motorFrame);
		bearing = bearingDesignation == null ? matchingBearing(screwDiameter) : DeepGrooveBearing.metric(bearingDesignation);
		if (!(Math.abs(bearing.bore - screwDiameter) < 1e-9))
			throw 'Bearing "${bearing.spec.designation}" bore does not match the screw diameter';
		coupling = new ShaftCoupling(motor.variant.shaftDiameter, screwDiameter);
		if (thread == null && screwDiameter != 10)
			throw "Linear axis needs an explicit thread for a nondefault screw diameter";
		var threadSpec = thread == null ? new LeadScrewThread(MetricTrapezoidal, 10, 2) : thread;
		if (Math.abs(threadSpec.screwDiameter - screwDiameter) > 1e-9)
			throw "Linear axis thread diameter must match the screw diameter";
		nut = new LeadScrewNut(threadSpec);
		var profileSpec = railProfile == null ? null : LinearRailSystem.catalog().get(railProfile);
		var guideBearingSpec = LinearBearing.metric("LM8UU");
		guideSpacing = Math.max(25, screwDiameter * 2.5);
		var guideSeatFit = BearingHousingFit.Slip;
		var guideSeatDiameter = guideBearingSpec.housingSeatDiameter(guideSeatFit);
		var carriageWidth = 2 * (guideSpacing + guideSeatDiameter / 2 + 5);
		if (profileSpec != null)
			carriageWidth = Math.max(carriageWidth, profileSpec.blockWidth + 10);
		var carriageLength = Math.max(screwDiameter * 6, guideBearingSpec.length + 4);
		if (profileSpec != null)
			carriageLength = Math.max(carriageLength, profileSpec.blockLength + 4);
		carriage = new Carriage(screwDiameter, carriageWidth, carriageLength,
			guideSpacing, guideSeatDiameter, nut, guideSeatFit,
			profileSpec == null ? null : -(carriageWidth / 2 + profileSpec.blockHeight));
		flangeBearingA = new FlangeBearingAssembly(bearing);
		flangeBearingB = new FlangeBearingAssembly(bearing);
		if (!(margin > Math.max(flangeBearingA.screw.spec.headHeight, nut.bodyLength + nut.flangeThickness)))
			throw "Linear axis end margin must clear the flange housing screw heads and lead nut";
		var depth = flangeBearingA.housing.depth;
		bearingAPosition = coupling.length / 2 + COUPLING_GAP + depth / 2;
		travelMin = bearingAPosition + depth / 2 + margin + carriage.length / 2;
		travelMax = travelMin + stroke;
		transmission = new LeadScrewTransmission("coupling", "carriage-slide", nut.lead, travelMin, stroke, threadSpec.hand == RightHand ? 1 : -1);
		bearingBPosition = travelMax + carriage.length / 2 + margin + depth / 2;
		length = bearingBPosition + depth / 2;
		screwStart = motor.connector("shaftTip").frame.z;
		screw = new LeadScrew(threadSpec, length);
		if (profileSpec == null) {
			guideSystem = new LinearGuideSystem("LM8UU", guideSpacing, length, BearingShaftFit.Slip, guideSeatFit);
			guideSystem.validateCarriage(carriage.width, carriage.length);
			guideRodA = guideSystem.rodA;
			guideRodB = guideSystem.rodB;
			guideBearingA = guideSystem.bearingA;
			guideBearingB = guideSystem.bearingB;
			railGuide = null;
			railGuideOffset = 0;
		} else {
			// Keep the existing round-guide fields populated for source compatibility. The selected
			// rail guide is the assembly/BOM path whenever `railGuide` is non-null.
			guideSystem = new LinearGuideSystem("LM8UU", guideSpacing, length, BearingShaftFit.Slip, guideSeatFit);
			guideRodA = guideSystem.rodA;
			guideRodB = guideSystem.rodB;
			guideBearingA = guideSystem.bearingA;
			guideBearingB = guideSystem.bearingB;
			railGuide = LinearRailSystem.forProfile(railProfile,
				stroke + 2 * profileSpec.railEndMargin + profileSpec.blockLength);
			railGuideOffset = travelMin - railGuide.travelMin;
		}
		rail = new RectTube(Math.max(20, screwDiameter * 2), Math.max(15, screwDiameter * 1.5), 2);
		var housing = flangeBearingA.housing;
		var screwHeadReach = housing.boltSpacing / 2 + flangeBearingA.screw.spec.headDiameter / 2;
		var reach = Math.max(Math.max(housing.face / 2, screwHeadReach), Math.max(carriage.width, coupling.outerDiameter) / 2);
		var railY = -(reach + RAIL_GAP + rail.height / 2);
		frame = new FrameAssembly();
		frame.point("railStart", 0, railY, screwStart);
		frame.point("railEnd", 0, railY, screwStart + length);
		frame.member("rail", "railStart", "railEnd", rail);
	}

	/** Construct an axis whose carriage is supported by a catalog-backed profile rail. The
	 * ordinary constructor keeps the LM8UU round-rod guide for compatibility. */
	public static function forRailProfile(designation:String, motorFrame:Int = 23, screwDiameter:Float = 10,
			stroke:Float = 200, ?bearingDesignation:String, margin:Float = 30, ?thread:LeadScrewThread):LinearAxis
		return new LinearAxis(motorFrame, screwDiameter, stroke, bearingDesignation, margin, thread, designation);

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
		if (railGuide == null) {
			guideBearingA.addTo(model, "guideBearingA");
			guideBearingB.addTo(model, "guideBearingB");
			model.mate("guide-bearing-a", "fixed", "carriage", "guideA", "guideBearingA", "axis");
			model.mate("guide-bearing-b", "fixed", "carriage", "guideB", "guideBearingB", "axis");
			var poseGuideA:AssemblyFrame = {x: -guideSpacing, y: 0, z: screwStart, qx: 0, qy: 0, qz: 0, qw: 1};
			var poseGuideB:AssemblyFrame = {x: guideSpacing, y: 0, z: screwStart, qx: 0, qy: 0, qz: 0, qw: 1};
			guideRodA.addTo(model, "guideRodA", poseGuideA);
			guideRodB.addTo(model, "guideRodB", poseGuideB);
		} else {
			var profileRailPose:AssemblyFrame = {x: 0, y: carriage.railMountY, z: screwStart + railGuideOffset,
				qx: 0, qy: 0, qz: 0, qw: 1};
			railGuide.rail.addTo(model, "profileRail", profileRailPose);
			for (i in 0...railGuide.blocks.length) {
				var blockId = 'profileBlock${i + 1}';
				railGuide.blocks[i].addTo(model, blockId);
				model.mate('profile-block-carriage-$i', "fixed", "carriage", "railMount", blockId, "rail");
				model.constrainOnAxis('profile-block-rail-$i', "prismatic", "profileRail", "axis", blockId, "rail",
					{x: 0, y: 1, z: 0});
			}
		}
		var depth = flangeBearingA.housing.depth;
		// Housing A: mounting face (local z=0) toward the motor.
		var poseA:AssemblyFrame = {x: 0, y: 0, z: screwStart + bearingAPosition - depth / 2, qx: 0, qy: 0, qz: 0, qw: 1};
		flangeBearingA.addTo(model, "flangeA", poseA);
		// Housing B: turned half a turn about X so its mounting face points at the far end.
		var poseB:AssemblyFrame = {x: 0, y: 0, z: screwStart + bearingBPosition + depth / 2, qx: 1, qy: 0, qz: 0, qw: 0};
		flangeBearingB.addTo(model, "flangeB", poseB);
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
		if (railGuide == null) {
			result.addComponent(guideRodA);
			result.addComponent(guideRodB);
			result.addComponent(guideBearingA);
			result.addComponent(guideBearingB);
		} else {
			result.addComponent(railGuide.rail);
			for (block in railGuide.blocks) result.addComponent(block);
		}
		result.add(railBomItem());
		for (line in flangeBearingA.bom().lines()) result.add(line);
		for (line in flangeBearingB.bom().lines()) result.add(line);
		return result;
	}

	/** Instance id to component, for geometry generation by a preview or exporter. Excludes the
	 * frame rail, which is generated through `frame.geometry("rail")` instead.
	 */
	public function components():Array<{id:String, component:MachineComponent}> {
		var result:Array<{id:String, component:MachineComponent}> = [
			{id: "motor", component: motor}, {id: "coupling", component: coupling}, {id: "screw", component: screw},
			{id: "carriage", component: carriage}, {id: "leadNut", component: nut}];
		if (railGuide == null) {
			result.push({id: "guideRodA", component: guideRodA});
			result.push({id: "guideRodB", component: guideRodB});
			result.push({id: "guideBearingA", component: guideBearingA});
			result.push({id: "guideBearingB", component: guideBearingB});
		} else {
			result.push({id: "profileRail", component: railGuide.rail});
			for (i in 0...railGuide.blocks.length)
				result.push({id: 'profileBlock${i + 1}', component: railGuide.blocks[i]});
		}
		for (entry in flangeBearingA.components()) result.push({id: 'flangeA-${entry.id}', component: entry.component});
		for (entry in flangeBearingB.components()) result.push({id: 'flangeB-${entry.id}', component: entry.component});
		return result;
	}
}
