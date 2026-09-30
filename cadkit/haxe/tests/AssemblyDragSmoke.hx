import cadkit.modeling.AssemblyDrag;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Dragging assembly frames the way the editor's IK gizmo does, in millimetres. */
class AssemblyDragSmoke {
	public static function run():Void {
		dragsASerialArm();
		reportsWhyItCannotFollow();
		keepsALinkageClosed();
	}

	/** Three links (400, 300, 200 mm) on vertical-axis hinges, with a tool connector at the tip. */
	static function planarArm(?limit:Float):AssemblyDefinition {
		var tip:AssemblyComponentDefinition = {id: "tip-link", connectors: [{name: "a", frame: AssemblyLoopSmoke.frame(0, 0, 0)},
			{name: "tool", frame: AssemblyLoopSmoke.frame(200, 0, 0)}]};
		return AssemblyLoopSmoke.definition("planar-arm", [AssemblyLoopSmoke.bar("base", 0), AssemblyLoopSmoke.bar("upper", 400),
			AssemblyLoopSmoke.bar("fore", 300), tip], [
			AssemblyLoopSmoke.joint("shoulder", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "base", "a", "upper", "a", 0.3),
			AssemblyLoopSmoke.joint("elbow", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "upper", "b", "fore", "a", 0.6,
				limit == null ? null : -limit, limit),
			AssemblyLoopSmoke.joint("wrist", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "fore", "b", "tip-link", "a", -0.4)]);
	}

	static function dragsASerialArm():Void {
		var state = new AssemblyState(planarArm());
		var before = state.worldConnector("tip-link", "tool");
		var drag = new AssemblyDrag(state, "tip-link", "tool", null, false);
		check(drag.movableJoints.join(",") == "shoulder,elbow,wrist", 'the grabbed tip moves the joints above it (${drag.movableJoints})');
		// Drag the tool 60 mm along a straight line in 30 small steps, as a mouse would.
		var previous = [state.joint("shoulder"), state.joint("elbow"), state.joint("wrist")];
		var largestStep = 0.0;
		var result:Null<cadkit.modeling.AssemblyDrag.AssemblyDragResult> = null;
		for (step in 1...31) {
			var target = at(before.x - 2.0 * step, before.y + 1.0 * step, before.z);
			result = drag.drag(target);
			check(result.status == Following, 'every small step is followed (${result.message})');
			for (i in 0...3) largestStep = Math.max(largestStep, Math.abs(result.coordinates[i] - previous[i]));
			previous = result.coordinates;
		}
		check(largestStep < 0.02, 'joints move smoothly along the drag ($largestStep rad per step)');
		var tool = drag.grabbedPose();
		near(tool.x, before.x - 60, 1e-3, "the tool ends on the target (x)");
		near(tool.y, before.y + 30, 1e-3, "the tool ends on the target (y)");
		near(state.worldConnector("tip-link", "tool").x, before.x, 1e-9, "dragging never modifies the state");
		var committed = new AssemblyState(state.definition, drag.commit());
		near(committed.worldConnector("tip-link", "tool").x, before.x - 60, 1e-3, "the committed record holds the dragged pose");
		near(drag.previewPose("tip-link").x, committed.worldPose("tip-link").x, 1e-9, "the preview is what gets committed");
	}

	static function reportsWhyItCannotFollow():Void {
		var state = new AssemblyState(planarArm());
		var drag = new AssemblyDrag(state, "tip-link", "tool", null, false);
		var far = drag.drag(at(2000, 0, 0));
		check(far.status == OutOfReach && StringTools.startsWith(far.message, "Out of reach"),
			'a target beyond 900 mm is out of reach (${far.message})');
		check(far.positionError > 1000 && far.positionError < 1200, 'the reported miss is the real distance (${far.positionError})');

		// With the elbow held within ±0.2 rad the arm cannot fold enough to reach close to its base.
		var limitedState = new AssemblyState(planarArm(0.8));
		var limitedDrag = new AssemblyDrag(limitedState, "tip-link", "tool", null, false);
		var close = limitedDrag.drag(at(150, 200, 0), 200);
		check(close.status == Limited && close.limitedJoints.indexOf("elbow") >= 0,
			'a limit in the way names the joint (${close.message})');
	}

	static function keepsALinkageClosed():Void {
		// Four-bar: grab the rocker's free end; the crank and coupler are the dependent joints that keep the pin closed.
		var state = new AssemblyState(AssemblyLoopSmoke.fourBarDefinition(2200, null, null, 1.4));
		check(state.solveClosures(["coupler", "rocker"]).converged, "the four-bar starts closed");
		var drag = new AssemblyDrag(state, "rocker", "b", ["crank", "coupler"], false);
		check(drag.movableJoints.join(",") == "rocker,crank,coupler", 'the rocker path plus the dependent joints move (${drag.movableJoints})');
		var start = drag.grabbedPose();
		// Swing the rocker end 10 degrees about its pivot (2000, 0) in small steps.
		var angle0 = Math.atan2(start.y, start.x - 2000);
		var result:Null<cadkit.modeling.AssemblyDrag.AssemblyDragResult> = null;
		for (step in 1...21) {
			var angle = angle0 + 10 * Math.PI / 180 * step / 20;
			result = drag.drag(at(2000 + 1500 * Math.cos(angle), 1500 * Math.sin(angle), 0));
			check(result.status == Following, 'the linkage follows while staying closed (${result.message})');
		}
		var committed = new AssemblyState(state.definition, drag.commit());
		committed.checkClosures();
		check(Math.abs(committed.joint("crank") - state.joint("crank")) > 0.01, "the crank turned to keep the pin closed");

		// Pull the rocker end straight away from its pivot: the rocker cannot stretch.
		var stretch = drag.drag(at(2000 + 1700 * Math.cos(angle0), 1700 * Math.sin(angle0), 0));
		check(stretch.status != Following, 'a target the rocker cannot reach is reported (${stretch.message})');
	}

	static function at(x:Float, y:Float, z:Float):AssemblyFrame return {x: x, y: y, z: z, qx: 0, qy: 0, qz: 0, qw: 1};

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}

	static function near(actual:Float, expected:Float, tolerance:Float, label:String):Void {
		if (Math.abs(actual - expected) > tolerance) throw '$label: expected $expected, got $actual';
	}
}
