package machinekit.screwdriving;

import machinekit.assembly.MachineAssembly;
import machinekit.screwdriving.Fastenings.FasteningsResult;
import machinekit.screwdriving.Fastening.TighteningAngleWindow;

/** Authored joint limits; thread faces, engagement and identities are always derived again. */
typedef FasteningPolicy = {
	var minimumDiameters:Null<Float>;
	var angleWindow:Null<TighteningAngleWindow>;
}

/** A CAD assembly with optional joint-specific limits, like a weldment's declared weld process. */
class FastenedAssembly extends MachineAssembly {
	final policies:Map<String, FasteningPolicy> = [];

	public function new() super();

	/** Declare an engineering minimum or qualified angle limits for a seated screw occurrence. */
	public function join(screw:String, minimumDiameters:Null<Float> = null,
		angleWindow:Null<TighteningAngleWindow> = null):FastenedAssembly {
		if (policies.exists(screw)) throw 'Duplicate fastening policy for $screw';
		if (minimumDiameters != null && (!Math.isFinite(minimumDiameters) || minimumDiameters <= 0))
			throw 'Invalid engagement minimum for $screw';
		if (angleWindow != null && (!Math.isFinite(angleWindow.minimum) || !Math.isFinite(angleWindow.maximum)
			|| angleWindow.minimum < 0 || angleWindow.maximum <= angleWindow.minimum))
			throw 'Invalid tightening angle window for $screw';
		policies.set(screw, {minimumDiameters: minimumDiameters, angleWindow: angleWindow == null ? null : {minimum: angleWindow.minimum, maximum: angleWindow.maximum}});
		return this;
	}

	/** Rebuild from current recipes, geometry and solved poses; no fastening cache survives an edit. */
	public function fastenings():FasteningsResult return Fastenings.find(this, policies);
}
