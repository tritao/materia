import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;

/** Check the installed press's joint tree, coupled ram and physical tool datums. */
class PressBrakeChecks {
	public static function main():Void run();
	public static function run():Void {
		for (withR in [false, true]) {
			var press = new PressBrake(withR);
			press.validate();
			for (entry in press.components()) {
				var part = entry.component.geometry();
				var valid = CadKit.shapeValidChecked(part.shape.borrowHandle()) != 0 && part.volume() > 0;
				part.close();
				if (!valid) throw 'Press component "${entry.id}" has invalid or empty native geometry';
			}
			var bomQuantity = 0;
			for (line in press.billOfMaterials().lines()) bomQuantity += line.quantity;
			if (bomQuantity < press.components().length) throw "Press BOM loses components";
			var model = new AssemblyModel("mm");
			press.addTo(model, "");
			var state = new AssemblyState(model.definition("press-brake"));
			for (travel in [0.0, 37, 100]) {
				state.setJoint("Y1", travel);
				state.setJoint("X", travel * 5);
				state.setJoint("depth", travel);
				if (withR) state.setJoint("R", travel * 1.2);
				state.forwardKinematics();
				near(state.worldPose("guideLeft").z, press.bedHeight + press.openHeight - travel, "Left ram guide");
				near(state.worldPose("guideRight").z, state.worldPose("guideLeft").z, "Torsion-bar equality");
				near(state.worldConnector("ram", "tool-mount").z, press.bedHeight + press.openHeight - travel, "Ram mount");
				near(state.worldConnector("fingerLeft", "contact").y, travel * 5, "Backgauge distance");
				near(state.worldConnector("fingerRight", "contact").z, press.bedHeight + 30 + (withR ? travel * 1.2 : 0), "Backgauge lift");
				near(state.worldPose("depthStop").z, press.bedHeight + press.openHeight - travel, "Independent depth stop");
			}
		}
		trace("Press brake assembly checks passed");
	}
	public static function near(actual:Float, expected:Float, label:String):Void {
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > 1e-6) throw '$label: $actual instead of $expected';
	}
}
