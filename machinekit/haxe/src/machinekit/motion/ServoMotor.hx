package machinekit.motion;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyDefinition.AssemblyActuator;

/** What a servo motor can deliver, in SI units: torques in N m, speeds in rad/s, inertia in kg m². */
typedef ServoRating = {
	var designation:String;
	var ratedTorque:Float;
	var peakTorque:Float;
	var ratedSpeed:Float;
	var maxSpeed:Float;
	var rotorInertia:Float;
	/** Encoder counts per revolution. */
	var encoderCounts:Float;
	/** Round body, mm. */
	var bodyDiameter:Float;
	var bodyLength:Float;
	/** Exposed steel shaft dimensions, mm, measured from the mounting face. */
	var shaftDiameter:Float;
	var shaftLength:Float;
	var electrical:ServoElectricalRating;
}

/** Mounting face at Z=0; hole coordinates are in that face, dimensions in mm.
 * Null mounting data means the interface has not been specified.
 */
typedef ServoMounting = {
	var face:Float;
	var flangeThickness:Float;
	var pilotDiameter:Float;
	var pilotHeight:Float;
	var holeDiameter:Float;
	var holes:Array<{x:Float,y:Float}>;
}

/**
 * A servo motor: rated and peak torque, rated and maximum speed, rotor inertia and the encoder counts
 * of its own feedback. Its actuator is a servo drive, so a planner relies on its peak torque and
 * maximum speed and its rated torque bounds the average (see `PlanCheck`). Usually it drives a joint
 * through a `Gearbox`.
 *
 * The named ratings are generic: round numbers in the range of the 3000 rpm AC servo families made at 50,
 * 100, 200, 400 and 750 W (rated torque is power over 3000 rpm, peak three times that, 5000 rpm at most, a little
 * under 20 bits' worth of encoder). They are assumptions, not a vendor's datasheet.
 * CAD frame: mounting face at z=0, body toward -Z, shaft along +Z. Connectors: `mountFace`, `shaftAxis`.
 */
class ServoMotor extends MachineComponent implements MotorDrive {
	/** Assumed DC bus for the low-voltage servo winding family. */
	public static final LOW_VOLTAGE_BUS:machinekit.power.DcVoltage = new machinekit.power.DcVoltage(48);
	/** Assumed 48 V winding: back EMF uses at most eighty percent of the bus
	 * at maximum speed, capped at 0.06 V/(rad/s) line RMS. Rated copper loss
	 * is five percent of mechanical output. Resistance scales with the motor. */
	public static function lowVoltageWinding(ratedTorque:Float, ratedSpeed:Float, maxSpeed:Float):ServoElectricalRating {
		var backEmf = Math.min(0.06, 0.8 * LOW_VOLTAGE_BUS.volts() / (Math.sqrt(2.0) * maxSpeed));
		var constant = backEmf * Math.sqrt(3.0);
		var current = ratedTorque / constant;
		return {torqueConstant: constant, backEmfConstant: backEmf,
			phaseResistance: 0.05 * ratedTorque * ratedSpeed / (3 * current * current), mechanicalEfficiency: 0.95};
	}
	/** Reference 200 V class AC winding, with the same assumed rated loss budget. */
	public static function acWinding(ratedTorque:Float, ratedSpeed:Float):ServoElectricalRating {
		return winding(ratedTorque, ratedSpeed, 0.3);
	}
	/** Assumed 400 V class winding, selected independently of the installed supply. */
	public static function highVoltageWinding(ratedTorque:Float, ratedSpeed:Float):ServoElectricalRating {
		return winding(ratedTorque, ratedSpeed, 0.6);
	}
	static function winding(ratedTorque:Float, ratedSpeed:Float, backEmf:Float):ServoElectricalRating {
		var constant = backEmf * Math.sqrt(3.0);
		var current = ratedTorque / constant;
		return {torqueConstant: constant, backEmfConstant: backEmf,
			phaseResistance: 0.05 * ratedTorque * ratedSpeed / (3 * current * current), mechanicalEfficiency: 0.95};
	}
	static var table:Null<Array<ServoRating>>;

	public final rating:ServoRating;
	public final ratingsBasis:machinekit.transmission.ValueBasis;

	public static function ratings():Array<ServoRating> {
		if (table == null) {
			var rpm = 2.0 * Math.PI / 60.0;
			function generic(name:String, watts:Float, inertia:Float, face:Float, length:Float, shaftDiameter:Float, shaftLength:Float):ServoRating {
				var rated = watts / (3000.0 * rpm);
				return {designation: name, ratedTorque: rated, peakTorque: 3.0 * rated, ratedSpeed: 3000.0 * rpm, maxSpeed: 5000.0 * rpm,
					rotorInertia: inertia, encoderCounts: 131072, bodyDiameter: face, bodyLength: length, shaftDiameter: shaftDiameter, shaftLength: shaftLength, electrical: acWinding(rated, 3000.0 * rpm)};
			}
			table = [
				generic("GENERIC-SERVO-50W", 50, 3.0e-6, 40, 70, 8, 20),
				generic("GENERIC-SERVO-100W", 100, 5.0e-6, 40, 85, 8, 20),
				generic("GENERIC-SERVO-200W", 200, 2.5e-5, 60, 100, 14, 30),
				generic("GENERIC-SERVO-400W", 400, 5.0e-5, 60, 130, 14, 30),
				generic("GENERIC-SERVO-750W", 750, 1.2e-4, 80, 145, 19, 35)
			];
			// Distinct saved designations preserve the selected winding on reconstruction.
			var baseRows = table.copy();
			for (row in baseRows) table.push({designation: StringTools.replace(row.designation, "GENERIC-SERVO-", "GENERIC-LV-SERVO-"),
				ratedTorque: row.ratedTorque, peakTorque: row.peakTorque, ratedSpeed: row.ratedSpeed, maxSpeed: row.maxSpeed,
				rotorInertia: row.rotorInertia, encoderCounts: row.encoderCounts, bodyDiameter: row.bodyDiameter,
				bodyLength: row.bodyLength, shaftDiameter: row.shaftDiameter, shaftLength: row.shaftLength,
				electrical: lowVoltageWinding(row.ratedTorque, row.ratedSpeed, row.maxSpeed)});
			for (row in baseRows) table.push({designation: StringTools.replace(row.designation, "GENERIC-SERVO-", "GENERIC-HV-SERVO-"),
				ratedTorque: row.ratedTorque, peakTorque: row.peakTorque, ratedSpeed: row.ratedSpeed, maxSpeed: row.maxSpeed,
				rotorInertia: row.rotorInertia, encoderCounts: row.encoderCounts, bodyDiameter: row.bodyDiameter,
				bodyLength: row.bodyLength, shaftDiameter: row.shaftDiameter, shaftLength: row.shaftLength,
				electrical: highVoltageWinding(row.ratedTorque, row.ratedSpeed)});
			for (name in LeadshineServo.catalog().designations()) table.push(LeadshineServo.model(name).rating);
		}
		return table;
	}

	static var namedRecipe:Null<machinekit.component.ComponentType>;
	public static function namedRecipeType():machinekit.component.ComponentType {
		if (namedRecipe == null) namedRecipe = new machinekit.component.ComponentType("machinekit.motion.servo-motor",
			[machinekit.component.ComponentRecipeSupport.choice("designation", [for (row in ratings()) row.designation],
				"GENERIC-SERVO-200W")], v -> model(v.token("designation")));
		return namedRecipe;
	}

	/** Named generic or Leadshine servo. Vendor rows retain their catalog provenance. */
	public static function model(designation:String, basis:machinekit.transmission.ValueBasis = Assumed):ServoMotor {
		if (LeadshineServo.catalog().exists(designation)) return LeadshineServo.model(designation);
		for (rating in ratings()) if (rating.designation == designation) return basis == Assumed ? new CatalogServoMotor(rating) : new ServoMotor(rating, basis);
		throw 'Unknown servo motor "$designation"; known: ${[for (rating in ratings()) rating.designation].join(", ")}';
	}

	public static function basisFromToken(token:String):machinekit.transmission.ValueBasis return switch token {
		case "derived": Derived;
		case "catalog": Catalog;
		case "stated": Stated;
		case "assumed": Assumed;
		case "measured": Measured;
		case _: throw 'Unknown servo ratings basis "$token"';
	};

	public function basisToken():String return switch ratingsBasis {
		case Derived: "derived";
		case Catalog: "catalog";
		case Stated: "stated";
		case Assumed: "assumed";
		case Measured: "measured";
	};

	public function new(rating:ServoRating, ratingsBasis:machinekit.transmission.ValueBasis = Stated) {
		if (!(rating.ratedTorque > 0) || !(rating.peakTorque >= rating.ratedTorque) || !(rating.ratedSpeed > 0) ||
			!(rating.maxSpeed >= rating.ratedSpeed) || !(rating.rotorInertia >= 0) || !(rating.encoderCounts >= 0))
			throw "A servo needs 0 < rated torque <= peak torque, 0 < rated speed <= maximum speed and finite ratings";
		for (dimension in [rating.bodyDiameter, rating.bodyLength, rating.shaftDiameter, rating.shaftLength])
			if (!(dimension > 0) || !Math.isFinite(dimension)) throw "A servo needs finite positive body and shaft dimensions";
		var electrical = rating.electrical;
		for (value in [electrical.torqueConstant, electrical.backEmfConstant, electrical.phaseResistance, electrical.mechanicalEfficiency])
			if (!(value > 0) || !Math.isFinite(value)) throw "A servo needs finite positive winding ratings";
		if (electrical.mechanicalEfficiency > 1) throw "Motor efficiency exceeds one";
		super(rating.designation, 'Servo motor, ${Dimension.format(rating.peakTorque)} N m peak', "aluminium 6061", true);
		this.rating = rating;
		this.ratingsBasis = ratingsBasis;
		addConnector("mountFace", Mount, Solids.axial(0, 0, 0));
		addConnector("shaftAxis", Axis, Solids.axial(0, 0, 0));
		addConnector("shaftEnd", Axis, Solids.axial(0, 0, rating.shaftLength));
	}

	override public function motorDrive():Null<MotorDrive> return this;
	public function electricalRating():Null<ServoElectricalRating> return rating.electrical;
	public function ratedPhaseCurrent():Float return rating.ratedTorque / rating.electrical.torqueConstant;

	/** The actuator: a servo drive with its peak torque and maximum speed as the limits a planner may rely on. */
	public function actuator(id:String, joint:String, volts:Float, margin:Float, ?current:Float):AssemblyActuator
		return {id: id, joint: joint, maxEffort: rating.peakTorque, maxRate: rating.maxSpeed, rotorInertia: rating.rotorInertia, drive: "servo",
			ratedTorque: rating.ratedTorque, peakTorque: rating.peakTorque, ratedSpeed: rating.ratedSpeed, maxSpeed: rating.maxSpeed,
			encoderCounts: rating.encoderCounts,
			torqueSpeed: [0, rating.peakTorque, rating.ratedSpeed, rating.peakTorque, rating.maxSpeed, rating.ratedTorque],
			assumed: ratingsBasis == Assumed ? ["servo ratings"] : null,
			assumptions: ratingsBasis == Assumed ? [{quantity: "motor curve", label: "servo ratings"},
				{quantity: "inertia", label: "servo ratings"}, {quantity: "speed limit", label: "servo ratings"}] : null};

	/** Supplier or explicitly assumed mounting dimensions; never infer them from body size. */
	public function mounting():Null<ServoMounting> return null;

    override public function axisMotor():Null<AxisMotor> {
        var face=mounting();
        if(face==null)return null;
        return new AxisMotor(this,face.face,rating.shaftDiameter,rating.shaftLength,depth->mountingCutout(depth));
    }

    /** Pilot and fastener clearances from the declared interface, in the mounting face. */
    public function mountingCutout(depth:Float,pilotClearance:Float=0.2):Part {
        var face=mounting();
        if(face==null || !Math.isFinite(depth)||depth<=0)throw "Servo needs declared mounting data and positive cutout depth";
        var tools=[Solids.named(Part.cylinderSpan((face.pilotDiameter+pilotClearance)/2,0,depth),"pilot")];
        return Solids.building(tools,parts->{
            var i=0;
            for(hole in face.holes)parts.push(Solids.named(Part.cylinderSpan(face.holeDiameter/2,0,depth,hole.x,hole.y),"bolt"+(++i)));
            return Solids.union(parts);
        });
    }

    /** Explicit, unverified flange rows for the generic motor families. */
    public static function assumedMounting(designation:String):ServoMounting {
        var face=0.0,pilot=0.0,pitch=0.0,hole=0.0;
        if(StringTools.endsWith(designation,"50W")&&!StringTools.endsWith(designation,"750W") || StringTools.endsWith(designation,"100W")) {face=40;pilot=30;pitch=31;hole=3.5;}
        else if(StringTools.endsWith(designation,"200W")||StringTools.endsWith(designation,"400W")) {face=60;pilot=50;pitch=50;hole=5.5;}
        else if(StringTools.endsWith(designation,"750W")) {face=80;pilot=70;pitch=70;hole=6.6;}
        else throw "No assumed flange row for "+designation;
        return {face:face,flangeThickness:6,pilotDiameter:pilot,pilotHeight:2,holeDiameter:hole,
            holes:[for(x in [-1,1])for(y in [-1,1]){x:x*pitch/2,y:y*pitch/2}]};
    }

	/** Continuous local-Z clearance sweep. The default bounds the actual envelope;
	 * families may override this with a tighter union of swept features.
	 */
	public function axialEnvelope(minOffset:Float,maxOffset:Float,includeShaft:Bool=false):Part {
		if(!Math.isFinite(minOffset)||!Math.isFinite(maxOffset)||minOffset>maxOffset)throw "Invalid motor axial envelope";
		var envelope=geometry(Envelope);
		try {
			var bounds=envelope.shape.bounds(),low=bounds.get_min(),high=bounds.get_max();
			var top=includeShaft?Math.max(high.get_z(),rating.shaftLength):high.get_z();
			var result=Part.box(high.get_x()-low.get_x(),high.get_y()-low.get_y(),top-low.get_z()+maxOffset-minOffset)
				.translated(new cadkit.modeling.Vector((low.get_x()+high.get_x())/2,(low.get_y()+high.get_y())/2,low.get_z()+minOffset));
			envelope.close();return result;
		} catch(e:Dynamic) {envelope.close();throw e;}
	}

	/** The exposed rotating shaft is a separate part attached to the driven joint. */
	public function shaft():SteppedShaft return new SteppedShaft([{diameter: rating.shaftDiameter, length: rating.shaftLength}]);

	override public function hasGeometry():Bool return true;

	/**
	 * The envelope is the box behind the mounting face, square to the flange; the preview the motor body
	 * (`MotorBody`), with bolt holes only where the mounting interface is declared.
	 */
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var face = rating.bodyDiameter, length = rating.bodyLength;
		if (detail == Envelope)
			return Solids.named(Part.box(face, face, length, Center, Center, Max), "body");
		var declared = mounting();
		return Solids.named(MotorBody.part(face, length, declared == null ? Math.max(6, face * 0.1) : declared.flangeThickness,
			declared == null ? 0 : declared.holeDiameter, declared == null ? [] : declared.holes), "body");
	}

	/** The motor's mass is that of its body as a solid cylinder of the flange's width (the drawn body does not set it). */
	override public function massGeometry():Null<Part>
		return Part.cylinderSpan(rating.bodyDiameter / 2, -rating.bodyLength, 0);
}

/** A catalogue servo keeps its source designation in saved assemblies. Explicit rating objects
 * remain code-only, so reconstruction never silently replaces their stated or edited ratings.
 */
private class CatalogServoMotor extends ServoMotor {
	public function new(rating:ServoRating) super(rating, Assumed);
    override public function mounting():Null<ServoMounting> return ServoMotor.assumedMounting(rating.designation);
	override public function componentType():Null<machinekit.component.ComponentType> return ServoMotor.namedRecipeType();
	override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues()
		.setToken("designation", rating.designation).setToken("material", materialSpec());
}
