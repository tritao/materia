package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Solids;
import materia.assembly.AssemblyDefinition.AssemblyActuator;

/** 50–7500 W ELM1/ELM2, 220/380 VAC. Face at z=0, housing -Z, separate rotating shaft +Z.
 * Interfaces follow datasheet pp. 3–16; extrusion, rear cover and connector shapes are approximations.
 * E = brake, F = no brake. ELM1 uses magnetic feedback, ELM2 optical; both are 23 bit.
 */
class LeadshineServo extends ServoMotor {
	public static inline final SOURCE = "https://www.leadshine.com/upfiles/downloads/006a40c553d0c6fc60a1221d6a764222_1675822261873.pdf";
	public static inline final MS1AE_SOURCE = "https://www.leadshine.com/product-detail/ELM2L-1500LA100FT-H%20MS1AE.html";
	static var table:Null<Catalog<LeadshineServoSpec>>;
	static var recipe:Null<ComponentType>;
	public final spec:LeadshineServoSpec;
	var massDeclared:Bool = false;

	public static function catalog():Catalog<LeadshineServoSpec> {
		if (table == null) {
			var rows:Array<LeadshineServoSpec> = [];
			var powers = [50, 100, 200, 400, 750, 1000];
			var lengths = [56.7, 67.7, 71.6, 88.8, 90.9, 103.9];
			var brakedLengths = [84.0, 95, 100.9, 118.1, 121.9, 134.9];
			var torques = [0.16, 0.32, 0.64, 1.27, 2.39, 3.18];
			var peaks = [0.48, 0.95, 1.92, 3.81, 7.17, 9.54];
			var currents = [0.93, 0.92, 1.5, 2.1, 4.1, 5.7];
			var peakCurrents = [2.88, 2.85, 4.5, 6.5, 13.4, 17.7];
			var inertias = [0.036, 0.062, 0.28, 0.56, 1.5, 2.0];
			var brakedInertias = [0.046, 0.072, 0.30, 0.58, 1.65, 2.15];
			var masses = [0.28, 0.38, 1.0, 1.3, 2.12, 2.8];
			var brakedMasses = [0.44, 0.54, 1.3, 1.55, 2.7, 3.4];
			for (series in ["ELM1", "ELM2"]) for (i in 0...powers.length) for (brake in [false, true]) {
				var frame = i < 2 ? 40 : i < 4 ? 60 : 80;
				var winding = series == "ELM1" ? "MA" : "LA";
				rows.push({designation: '${series}H-${StringTools.lpad(Std.string(powers[i]), "0", 4)}${winding}${frame}${brake ? "E" : "F"}',
					series: series, brake: brake, watts: powers[i], voltage: 220, ratedRpm: 3000, maxRpm: 5000, face: frame,
					bodyLength: brake ? brakedLengths[i] : lengths[i],
					shaftDiameter: frame == 40 ? 8 : frame == 60 ? 14 : 19,
					shaftLength: frame == 40 ? 25 : frame == 60 ? 30 : 35,
					boltCircle: frame == 40 ? 46 : frame == 60 ? 70 : 90,
					holeDiameter: frame == 40 ? 4.5 : frame == 60 ? 5.5 : 6.5,
					flangeThickness: frame == 40 ? 5 : frame == 60 ? 6.6 : 8.1,
					pilotDiameter: frame == 40 ? 30 : frame == 60 ? 50 : 70, pilotHeight: 3,
					connectorHeight: frame == 40 ? 35 : frame == 60 ? 45 : 55,
					keyLength: frame == 40 ? 14 : frame == 60 ? 22.5 : 25,
					keyWidth: frame == 40 ? 3 : frame == 60 ? 5 : 6,
					keyHeight: frame == 40 ? 3 : frame == 60 ? 5 : 6,
					keyFloor: frame == 40 ? 6.2 : frame == 60 ? 11 : 15.5,
					ratedTorque: torques[i], peakTorque: peaks[i], ratedCurrent: currents[i], peakCurrent: peakCurrents[i],
					rotorInertia: (brake ? brakedInertias[i] : inertias[i]) * 1e-4,
					// ELM2 50 W brake mass is truncated in the PDF; ELM1's identical frame is used explicitly.
					mass: brake ? brakedMasses[i] : masses[i]});
			}
			for (row in LeadshineServoLargeCatalog.rows()) rows.push(row);
			// Exact page variant; keep the older PDF row and saved designation intact.
			var pageVariant:LeadshineServoSpec = cast Reflect.copy(rows.filter(r -> r.designation == "ELM2L-1500LA100FT-H")[0]);
			pageVariant.designation = "ELM2L-1500LA100FT-H MS1AE";
			pageVariant.voltage = 400; pageVariant.ratedCurrent = 4.5; pageVariant.peakCurrent = 13.5;
			rows.push(pageVariant);
			table = new Catalog("Leadshine servo", s -> s.designation, rows, s -> ({source: s.designation == pageVariant.designation ? MS1AE_SOURCE : SOURCE,
				sources: s.designation == pageVariant.designation ? [SOURCE, MS1AE_SOURCE] : [SOURCE],
				standard: null, standardEdition: null, dimensionKind: Mixed, conformance: NominalEnvelope,
				verifiedFields: ["face", "bodyLength", "shaftDiameter", "shaftLength", "boltCircle", "holeDiameter",
					"flangeThickness", "pilotDiameter", "pilotHeight", "connectorHeight", "keyLength", "keyWidth", "keyFloor",
					"keyHeight", "voltage", "ratedRpm", "maxRpm", "ratedTorque", "peakTorque", "ratedCurrent", "peakCurrent", "rotorInertia"]}));
		}
		return table;
	}

	public static function model(designation:String):LeadshineServo return new LeadshineServo(catalog().get(designation));

	private function new(spec:LeadshineServoSpec) {
		var rpm = 2 * Math.PI / 60;
		super({designation: spec.designation, ratedTorque: spec.ratedTorque, peakTorque: spec.peakTorque,
			ratedSpeed: spec.ratedRpm * rpm, maxSpeed: spec.maxRpm * rpm, rotorInertia: spec.rotorInertia, encoderCounts: 8388608,
			bodyDiameter: spec.face, bodyLength: spec.bodyLength, shaftDiameter: spec.shaftDiameter,
			shaftLength: spec.shaftLength, electrical: spec.voltage > 220
				? ServoMotor.highVoltageWinding(spec.ratedTorque, spec.ratedRpm * rpm)
				: ServoMotor.acWinding(spec.ratedTorque, spec.ratedRpm * rpm)}, Catalog);
		this.spec = spec;
		var i = 1;
		for (p in boltPattern()) addConnector('bolt${i++}', Mount, Solids.axial(p.x, p.y, 0));
		addConnector("pilot", Mount, Solids.axial(0, 0, spec.pilotHeight));
	}

	/** Split published total mass between housing and exposed shaft, avoiding double counting.
	 * Total centre is estimated at the body midpoint; housing inertia is unknown.
	 */
	override public function massProperties():machinekit.component.MassProperties {
		if (!massDeclared) {
			var rotating = shaft().massProperties(), housingMass = spec.mass - rotating.mass;
			declareMass(housingMass, new Vector(-rotating.mass*rotating.centreOfMass.x/housingMass,
				-rotating.mass*rotating.centreOfMass.y/housingMass,
				(-spec.mass*spec.bodyLength/2-rotating.mass*rotating.centreOfMass.z)/housingMass));
			massDeclared = true;
		}
		return super.massProperties();
	}

	/** No verified winding constants: prevent simulation from treating estimates as vendor data. */
	override public function electricalRating():Null<ServoElectricalRating> return null;
	override public function ratedPhaseCurrent():Float return spec.ratedCurrent;
	public function torqueCurveEvidence():Array<LeadshineTorqueCurves.LeadshineTorqueCurve>
		return LeadshineTorqueCurves.evidenceFor(spec.designation);
	/** Explicit study-only ordinate; null for unmatched variants and speeds outside the graph. */
	public function studyContinuousTorque(rpm:Float):Null<Float> {
		var curve = LeadshineTorqueCurves.studyCurve(spec.designation);
		return curve == null ? null : LeadshineTorqueCurves.studyTorque(curve,rpm);
	}
	override public function actuator(id:String, joint:String, volts:Float, margin:Float, ?current:Float):AssemblyActuator {
		var result = super.actuator(id, joint, volts, margin, current);
		result.assumed = ["servo torque-speed interpolation"];
		result.assumptions = [{quantity: "motor curve", label: "servo torque-speed interpolation"}];
		return result;
	}

	override public function mounting():ServoMotor.ServoMounting return {
		face:spec.face,flangeThickness:spec.flangeThickness,pilotDiameter:spec.pilotDiameter,
		pilotHeight:spec.pilotHeight,holeDiameter:spec.holeDiameter,holes:boltPattern()};

	/** LA is the pitch-circle diameter, not the spacing of adjacent holes. */
	public function boltPattern():Array<{x:Float, y:Float}> {
		var h = spec.boltCircle / (2 * Math.sqrt(2));
		return [{x:h, y:h}, {x:-h, y:h}, {x:-h, y:-h}, {x:h, y:-h}];
	}

	/** Shaft stays separate so assembly joints can rotate it independently of the housing. */
	override public function shaft():SteppedShaft {
		var depth = spec.shaftDiameter - spec.keyFloor;
		var key = machinekit.standard.ParallelKey.custom({minShaft: spec.shaftDiameter, maxShaft: spec.shaftDiameter,
			width: spec.keyWidth, height: spec.keyHeight, shaftDepth: depth, hubDepth: spec.keyHeight - depth}, spec.keyLength);
		return new SteppedShaft([{diameter: spec.shaftDiameter, length: spec.shaftLength}], null,
			[{name: "key", z0: spec.shaftLength - 3 - spec.keyLength, key: key}]);
	}

	static function box(w:Float, d:Float, z0:Float, z1:Float, x:Float = 0, y:Float = 0):Part
		return Part.prism([new Vector(x-w/2, y-d/2), new Vector(x+w/2, y-d/2),
			new Vector(x+w/2, y+d/2), new Vector(x-w/2, y+d/2)], z0, z1);

	/** Continuous local-Z sweep of the envelope, retaining the narrow pilot/shaft
	 * and each connector's axial region rather than filling a single large box.
	 */
	override public function axialEnvelope(minOffset:Float,maxOffset:Float,includeShaft:Bool=false):Part {
		if(!Math.isFinite(minOffset)||!Math.isFinite(maxOffset)||minOffset>maxOffset)throw "Invalid servo axial sweep";
		var owned:Array<Part> = [];
		return Solids.building(owned,tracked->{
			var f=spec.face,l=spec.bodyLength,t=spec.flangeThickness;
			tracked.push(box(f,f,-l+minOffset,maxOffset));
			tracked.push(Part.cylinderSpan(spec.pilotDiameter/2,minOffset,spec.pilotHeight+maxOffset));
			tracked.push(box(0.28*f,2*(spec.connectorHeight-f/2),-t-0.30*f+minOffset,-t-0.08*f+maxOffset,0,f/2));
			tracked.push(box(0.24*f,2*(spec.connectorHeight-f/2),-l+0.08*f+minOffset,-l+0.27*f+maxOffset,0,f/2));
			if(includeShaft)tracked.push(Part.cylinderSpan(spec.shaftDiameter/2,minOffset,spec.shaftLength+maxOffset));
			return Solids.union(tracked.copy());
		});
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var owned:Array<Part> = [];
		return Solids.building(owned, tracked -> {
			function keep(p:Part, name:String):Part {
				var named = Solids.named(p, name); tracked.push(named); return named;
			}
			var f = spec.face, l = spec.bodyLength, t = spec.flangeThickness;
			var parts:Array<Part> = [];
			if (detail == Envelope) parts.push(keep(box(f, f, -l, 0), "body"));
			else {
				var flange = keep(box(f, f, -t, 0), "flange");
				var holes:Array<Part> = [], i = 1;
				for (p in boltPattern()) holes.push(keep(Part.cylinderSpan(spec.holeDiameter/2, -t-0.1, 0.1, p.x, p.y), 'bolt${i++}'));
				flange = Solids.cut(flange, holes); tracked.push(flange); parts.push(flange);
				// Undimensioned cover proportions and longitudinal ribs are visual approximations.
				// Recessed extrusion corners leave the flange through-holes accessible.
				var rear = Math.min(0.22*l, 0.3*f), half = f/2-1, c = 0.30*f;
				parts.push(keep(Part.prism([new Vector(-half+c,-half),new Vector(half-c,-half),
					new Vector(half,-half+c),new Vector(half,half-c),new Vector(half-c,half),
					new Vector(-half+c,half),new Vector(-half,half-c),new Vector(-half,-half+c)], -l+rear, -t), "housing"));
				parts.push(keep(box(0.88*f, 0.88*f, -l, -l+rear), "encoderCover"));
				for (side in [-1,1]) for (i in 0...5) {
					var offset = (i-2)*f*0.09;
					parts.push(keep(box(f*0.035, f*0.055, -l+rear, -t, side*(f/2-f*0.0175), offset), 'ribX${side}_${i}'));
					parts.push(keep(box(f*0.055, f*0.035, -l+rear, -t, offset, side*(f/2-f*0.0175)), 'ribY${side}_${i}'));
				}
			}
			var pilot=keep(Part.cylinderSpan(spec.pilotDiameter/2, 0, spec.pilotHeight), "pilot");
			if(detail==Preview) {
				var bore=keep(Part.cylinderSpan(spec.shaftDiameter/2+0.05,-0.1,spec.pilotHeight+0.1),"shaftBore");
				pilot=Solids.cut(pilot,[bore]);tracked.push(pilot);
			}
			parts.push(pilot);
			// Two connector envelopes remain within published maximum LH; their exact positions are estimated.
			parts.push(keep(box(0.28*f, 2*(spec.connectorHeight-f/2), -t-0.30*f, -t-0.08*f, 0, f/2), "powerConnector"));
			parts.push(keep(box(0.24*f, 2*(spec.connectorHeight-f/2), -l+0.08*f, -l+0.27*f, 0, f/2), "encoderConnector"));
			var result = Solids.union(parts); tracked.push(result); return result;
		});
	}

	/** Tool for a plate ahead of z=0. Hole diameters follow the motor drawing. */
	override public function mountingCutout(thickness:Float, pilotClearance:Float = 0.2):Part {
		if (!Math.isFinite(thickness) || thickness <= 0 || !Math.isFinite(pilotClearance) || pilotClearance < 0)
			throw "Servo mounting cutout needs positive thickness and nonnegative pilot clearance";
		var owned:Array<Part> = [];
		return Solids.building(owned, tracked -> {
			tracked.push(Solids.named(Part.cylinderSpan((spec.pilotDiameter+pilotClearance)/2, 0, thickness), "pilot"));
			var i = 1;
			for (p in boltPattern()) tracked.push(Solids.named(Part.cylinderSpan(spec.holeDiameter/2, 0, thickness, p.x, p.y), 'bolt${i++}'));
			return Solids.union(tracked.copy());
		});
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.motion.leadshine-servo",
			[ComponentRecipeSupport.choice("designation", catalog().designations(), "ELM1H-0400MA60F")],
			v -> model(v.token("designation")), true);
		return recipe;
	}
	override public function toolSpecs():Array<machinekit.component.ToolSpec> return [new machinekit.component.ToolSpec("mountingCutout", [
		ComponentRecipeSupport.toolDepth(10), ComponentRecipeSupport.length("pilotClearance", 0.2)])];
	override function buildTool(name:String, values:ComponentValues):Part {
		if (name == "mountingCutout") return mountingCutout(values.number("depth"), values.number("pilotClearance"));
		return super.buildTool(name, values);
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("designation", spec.designation).setToken("material", materialSpec());
}
