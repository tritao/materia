package machinekit.structural;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;
import machinekit.assembly.Diagnostics;

enum abstract CastingCollisionStatus(String) to String {
	var Supported = "supported";
	var Unsupported = "unsupported";
}

/** Explicit qualification of the exact collision model, separate from CAD availability. */
class CastingCollisionQualification implements ComponentFacet {
	public final status:CastingCollisionStatus;
	public final reason:String;

	public function new(status:CastingCollisionStatus, reason:String) {
		this.status = status;
		this.reason = reason;
	}

	public function check(component:MachineComponent):Void {
		if (!component.hasGeometry() || reason.length == 0 || (status != Supported && status != Unsupported))
			throw "Invalid casting collision qualification";
		if ((machinekit.component.CollisionHullFacet.of(component) != null) != (status == Supported))
			throw "Casting collision qualification disagrees with its collision model";
	}

	public function diagnostics(subject:String):Diagnostics {
		var result = new Diagnostics();
		if (status == Unsupported)
			result.error("casting.collision-hull.unsupported", subject, describe());
		return result;
	}

	public function describe():String
		return "Casting collision hull " + status + ": " + reason;
}
