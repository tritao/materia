import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;

/** Checks that recorded mates solve independently of insertion order. */
class AssemblyModelSmoke {
	public static function run():Void {
		checkAutomaticClosure();
		var ordered = chain(false), reversed = chain(true);
		near(ordered.pose("middle").x, 110, "parent-first middle pose");
		near(ordered.pose("leaf").x, 115, "parent-first leaf pose");
		near(reversed.pose("middle").x, ordered.pose("middle").x, "child-first middle pose");
		near(reversed.pose("leaf").x, ordered.pose("leaf").x, "child-first leaf pose");
		near(reversed.worldPoint("leaf", "base").x, 115, "world connector uses solved pose");
		if (reversed.definition() == reversed.definition()) throw "Definition exposes mutable model data";
		var namedA = reversed.definition("a"), namedB = reversed.definition("b");
		if (namedA.id != "a" || namedB.id != "b" || reversed.definition().id != "assembly")
			throw "Definition export changed the model id";
		near(reversed.definition().occurrences[2].initialPose.x, 0, "definition retains recorded placement");
		near(reversed.record().instances[2].pose.x, 115, "legacy record retains solved pose");
		throws(() -> reversed.place("leaf", AssemblyFrames.identity()), "cannot be placed independently");
		reversed.place("root", at(200));
		near(reversed.pose("leaf").x, 215, "root placement invalidates cached solve");
		var earlier = reversed.initialState();
		reversed.place("root", at(300));
		near(earlier.worldPose("root").x, 200, "initial state retains its original placement");
		near(reversed.pose("root").x, 300, "model placement changes independently");
		var returned = reversed.solve();
		returned.setRootPose("root", at(400));
		near(reversed.pose("root").x, 300, "returned state cannot mutate model pose");
		reversed.place("root", at(200));
		reversed.constrain("closed", "fixed", "root", "tip", "middle", "base");
		near(reversed.pose("leaf").x, 215, "closure checks solved frames");

		var open = chain(true);
		open.constrain("separated", "fixed", "root", "tip", "leaf", "base");
		throws(() -> open.pose("leaf"), "separated connectors");
		var moving = new AssemblyModel();
		moving.add("base"); moving.add("slide");
		moving.connector("base", "axis", AssemblyFrames.identity());
		moving.connector("slide", "axis", AssemblyFrames.identity());
		moving.mate("travel", "prismatic", "base", "axis", "slide", "axis");
		var detached = moving.solve();
		detached.setJoint("travel", 12);
		near(moving.pose("slide").y, 0, "returned joint state cannot mutate model pose");
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

	/** Mating a fourth bar onto an already jointed one closes the loop: it becomes a closure, not an error. */
	static function checkAutomaticClosure():Void {
		var model = new AssemblyModel();
		for (name in ["ground", "crank", "coupler", "rocker"]) model.add(name);
		for (name in ["ground", "crank", "coupler", "rocker"]) {
			model.connector(name, "a", AssemblyFrames.identity());
			model.connector(name, "b", AssemblyFrames.translation(1000, 0, 0));
		}
		model.mateOnAxis("crank", "revolute", "ground", "a", "crank", "a", {x: 0, y: 0, z: 1}, 0.6);
		model.mateOnAxis("coupler", "revolute", "crank", "b", "coupler", "a", {x: 0, y: 0, z: 1}, 0.2);
		model.mateOnAxis("rocker", "revolute", "ground", "b", "rocker", "a", {x: 0, y: 0, z: 1}, 1.4);
		model.mateOnAxis("pin", "revolute", "coupler", "b", "rocker", "b", {x: 0, y: 0, z: 1});
		var roles = [for (joint in model.definition().joints) joint.id + ":" + joint.role].join(",");
		if (roles != "crank:tree,coupler:tree,rocker:tree,pin:closure") throw 'the loop-closing joint becomes a closure: $roles';
		// Mating back up the tree (the parent hangs below the child) closes a loop too.
		model.mateOnAxis("back", "revolute", "coupler", "a", "ground", "a", {x: 0, y: 0, z: 1});
		if (model.definition().joints[4].role != "closure") throw "a joint whose parent hangs below its child is a closure";
		throws(() -> model.mateOnAxis("valued", "revolute", "rocker", "b", "coupler", "b", {x: 0, y: 0, z: 1}, 0.3),
			"no coordinate of its own");
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
