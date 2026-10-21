package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertManufacturing.InsertHoleRoute;
import materia.casting.HoleClearanceBudget.HoleClearanceBudgets;

/** A source mounting-hole interface: permissions and accuracy allocations are
 * independent of the precision face's finishing route. The hole owns its entry
 * frame (+Z into steel); the placed bought part's connector supplies its
 * nominal +Y axis lineage, following the assembly connector convention.
 */
typedef InsertMountHoleRequirement = {
	var holeId:String;
	var componentPath:String;
	var componentFingerprint:String;
	var connectorId:String;
	var connectorFrame:AssemblyFrame;
	var clearanceReference:Null<String>;
	var permittedRoutes:Array<InsertHoleRoute>;
	var budget:HoleClearanceBudget;
}

class InsertMountHoleRequirements {
	public static function snapshot(raw:Dynamic):Array<InsertMountHoleRequirement> {
		if (!Std.isOfType(raw, Array)) throw "Invalid mounting-hole requirements";
		var rows:Array<Dynamic> = cast raw;
		if (rows.length > 256) throw "Too many mounting-hole requirements";
		var result:Array<InsertMountHoleRequirement> = [], seen:Map<String,Bool> = [];
		for (row in rows) {
			CastingFeatureCodec.fields(row, ["holeId", "componentPath", "componentFingerprint", "connectorId",
				"connectorFrame", "clearanceReference", "permittedRoutes", "budget"]);
			var id = CastingRecordCodec.string(row, "holeId");
			CastingFeatureCodec.identifier(id);
			if (seen.exists(id)) throw "Duplicate mounting-hole requirement: " + id;
			seen.set(id, true);
			var component = CastingRecordCodec.string(row, "componentPath");
			CastingRecordCodec.referencePath(component);
			if (component.length > 512) throw "Mounting component path is too long: " + id;
			var fingerprint = CastingRecordCodec.string(row, "componentFingerprint");
			if (!~/^[a-f0-9]{64}$/.match(fingerprint)) throw "Invalid mounting component fingerprint: " + id;
			var connector = CastingRecordCodec.string(row, "connectorId");
			CastingFeatureCodec.identifier(connector);
			var reference = CastingRecordCodec.optionalString(row, "clearanceReference");
			if (reference != null) {
				if (StringTools.trim(reference).length == 0 || reference.length > 2048)
					throw "Invalid mounting clearance reference: " + id;
				for (index in 0...reference.length)
					if (reference.charCodeAt(index) == 0) throw "Invalid mounting clearance reference: " + id;
			}
			var routes:Array<InsertHoleRoute> = [];
			for (value in CastingRecordCodec.array(row, "permittedRoutes")) {
				if (!Std.isOfType(value, String)) throw "Invalid permitted mounting-hole route: " + id;
				var route:InsertHoleRoute = cast value;
				if (route != InsertHoleRoute.BeforeCasting && route != InsertHoleRoute.TransferAfterCasting)
					throw "Unknown permitted mounting-hole route: " + id;
				if (routes.indexOf(route) >= 0) throw "Duplicate permitted mounting-hole route: " + id;
				routes.push(route);
			}
			routes.sort(Reflect.compare);
			result.push({holeId: id, componentPath: component, componentFingerprint: fingerprint,
				connectorId: connector, connectorFrame: CastingRecordCodec.frame(Reflect.field(row, "connectorFrame")),
				clearanceReference: reference, permittedRoutes: routes,
				budget: HoleClearanceBudgets.snapshot(Reflect.field(row, "budget"))});
		}
		result.sort((a, b) -> Reflect.compare(a.holeId, b.holeId));
		return result;
	}

	/** Validate lineage, without changing the one stored hole frame. These are
 * numerical checks of nominal source geometry, not physical accuracy evidence.
 */
	public static function references(requirements:Array<InsertMountHoleRequirement>, holes:Array<InsertHole>):Void {
		for (requirement in requirements) {
			var matches = holes.filter(hole -> hole.id == requirement.holeId);
			if (matches.length != 1) throw "Mounting requirement names an unknown source hole: " + requirement.holeId;
			var hole = matches[0], thread = hole.thread;
			var lever=Math.sqrt(hole.frame.x*hole.frame.x+hole.frame.y*hole.frame.y);
			if (Math.abs(lever-requirement.budget.distanceFromDatumMm)>1e-8)
				throw "Mounting budget datum distance differs from source hole: "+requirement.holeId;
			if (thread == null || requirement.budget.boltDiameterMm != thread.majorDiameter)
				throw "Mounting bolt differs from the source thread: " + requirement.holeId;
			var axis = F.transformVector(requirement.connectorFrame, 0, 1, 0);
			var drill = F.transformVector(hole.frame, 0, 0, 1);
			var ax = axis.y * drill.z - axis.z * drill.y;
			var ay = axis.z * drill.x - axis.x * drill.z;
			var az = axis.x * drill.y - axis.y * drill.x;
			if (ax * ax + ay * ay + az * az > 1e-18)
				throw "Mounting connector axis differs from source hole: " + requirement.holeId;
			var dx = hole.frame.x - requirement.connectorFrame.x;
			var dy = hole.frame.y - requirement.connectorFrame.y;
			var dz = hole.frame.z - requirement.connectorFrame.z;
			var x = dy * axis.z - dz * axis.y;
			var y = dz * axis.x - dx * axis.z;
			var z = dx * axis.y - dy * axis.x;
			if (x * x + y * y + z * z > 1e-12)
				throw "Mounting connector is not coaxial with source hole: " + requirement.holeId;
		}
	}
}
