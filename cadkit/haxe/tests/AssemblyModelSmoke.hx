import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;

/** Checks that recorded mates solve independently of insertion order. */
class AssemblyModelSmoke {
	public static function run():Void {
		var ordered = chain(false), reversed = chain(true);
		near(ordered.pose("middle").x, 110, "parent-first middle pose");
		near(ordered.pose("leaf").x, 115, "parent-first leaf pose");
		near(reversed.pose("middle").x, ordered.pose("middle").x, "child-first middle pose");
		near(reversed.pose("leaf").x, ordered.pose("leaf").x, "child-first leaf pose");
		near(reversed.worldPoint("leaf", "base").x, 115, "world connector uses solved pose");
		if (reversed.definition() != reversed.definition()) throw "Definition is not the recorded object";
		near(reversed.definition().occurrences[2].initialPose.x, 0, "definition retains recorded placement");
		near(reversed.record().instances[2].pose.x, 115, "legacy record retains solved pose");
		throws(() -> reversed.place("leaf", AssemblyFrames.identity()), "cannot be placed independently");
		reversed.place("root", at(200));
		near(reversed.pose("leaf").x, 215, "root placement invalidates cached solve");
		reversed.constrain("closed", "fixed", "root", "tip", "middle", "base");
		near(reversed.pose("leaf").x, 215, "closure checks solved frames");

		var open = chain(true);
		open.constrain("separated", "fixed", "root", "tip", "leaf", "base");
		throws(() -> open.pose("leaf"), "separated connectors");
	}

	static function chain(childFirst:Bool):AssemblyModel {
		var model = new AssemblyModel();
		model.add("root", at(100));
		model.add("middle");
		model.add("leaf");
		model.connector("root", "tip", at(10));
		model.connector("middle", "base", AssemblyFrames.identity());
		model.connector("middle", "tip", at(5));
		model.connector("leaf", "base", AssemblyFrames.identity());
		if (childFirst) {
			model.mate("second", "fixed", "middle", "tip", "leaf", "base");
			near(model.pose("leaf").x, 5, "pose after child-first mate");
			model.mate("first", "fixed", "root", "tip", "middle", "base");
		} else {
			model.mate("first", "fixed", "root", "tip", "middle", "base");
			model.mate("second", "fixed", "middle", "tip", "leaf", "base");
		}
		return model;
	}

	static function at(x:Float):materia.assembly.AssemblyRecord.AssemblyFrame
		return {x: x, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1};

	static function near(actual:Float, expected:Float, label:String):Void {
		if (Math.abs(actual - expected) > 1e-9) throw '$label: expected $expected, got $actual';
	}

	static function throws(action:Void->Void, fragment:String):Void {
		try {
			action();
		} catch (error:Dynamic) {
			if (Std.string(error).indexOf(fragment) >= 0) return;
			throw error;
		}
		throw 'Expected error containing "$fragment"';
	}
}
