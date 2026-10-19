import haxe.Json;
import machinekit.motion.LeadshineServo;
import machinekit.milling.Bt30SpindleMotor;
import cadkit.modeling.Part;

/** Export analysis inputs from executable MachineKit models, without a second motor table. */
class ExportLocalMotors {
	static function bounds(part:Part):Dynamic {
		try {
			var box = part.shape.bounds(), lo = box.get_min(), hi = box.get_max();
			var result = {width_mm:hi.get_x()-lo.get_x(), depth_mm:hi.get_y()-lo.get_y(),
				height_mm:hi.get_z()-lo.get_z()};
			part.close(); return result;
		} catch (error:Dynamic) { part.close(); throw error; }
	}
	public static function main():Void {
		var path = Sys.getEnv("LOCAL_MOTOR_CATALOG_OUTPUT");
		if (path == null) throw "Set LOCAL_MOTOR_CATALOG_OUTPUT to the snapshot JSON path";
		var baseline = new Bt30SpindleMotor();
		var rows:Array<Dynamic> = [];
		for (name in LeadshineServo.catalog().designations()) {
			var motor = LeadshineServo.model(name);
			rows.push({spec:motor.spec, envelope:bounds(motor.geometry(Envelope)),
				encoder_counts_per_rev:motor.rating.encoderCounts,
				rated_speed_rad_s:motor.rating.ratedSpeed, maximum_speed_rad_s:motor.rating.maxSpeed,
				winding_verified:motor.electricalRating()!=null,
				curve_evidence:motor.torqueCurveEvidence(),
				study_continuous_curve:machinekit.motion.LeadshineTorqueCurves.studyCurve(name),
				continuous_torque_curve_verified:false});
		}
		var text = Json.stringify({schema_version:1,
			source_models:["machinekit.motion.LeadshineServo", "machinekit.milling.Bt30SpindleMotor"],
			baseline:{designation:baseline.designation, rating:baseline.rating,
				envelope:bounds(baseline.geometry(Envelope)), basis:baseline.basisToken()},
			motors:rows}, null, "  ")+"\n";
		if (Sys.getEnv("LOCAL_MOTOR_CATALOG_CHECK") == "1") {
			// Compare values rather than object-field serialization order.
			if (!sys.FileSystem.exists(path) || Json.stringify(Json.parse(sys.io.File.getContent(path))) != Json.stringify(Json.parse(text)))
				throw "Local motor snapshot is stale; regenerate it from MachineKit";
		} else sys.io.File.saveContent(path,text);
		Sys.println('Exported ${rows.length} local motor models and BT30 reference');
	}
}
