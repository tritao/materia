package cadkit.modeling;

import materia.project.AssemblyRecord;
import materia.project.AssemblyCodec;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyFrame;
import materia.project.AssemblyRecord.AssemblyConnector;
import materia.project.AssemblyRecord.AssemblyInstance;
import materia.project.AssemblyRecord.AssemblyJoint;

/** Builds a posed assembly from part instances, connector frames, and tree joints. */
class AssemblyModel {
	final instances:Array<AssemblyInstance> = [];
	final joints:Array<AssemblyJoint> = [];
	final byId:Map<String, AssemblyInstance> = [];
	final attached:Map<String, Bool> = [];

	public function new() {}

	public function add(id:String, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || byId.exists(id)) throw 'Duplicate assembly instance "$id"';
		var instance:AssemblyInstance = {id: id, pose: pose == null ? AssemblyFrames.identity() : pose,
			connectors: []};
		AssemblyCodec.validateFrame(instance.pose);
		instances.push(instance);
		byId.set(id, instance);
	}

	public function connector(id:String, name:String, frame:AssemblyFrame):Void {
		var instance = require(id);
		if (name == null || name.length == 0) throw "Assembly connector needs a name";
		for (existing in instance.connectors) if (existing.name == name)
			throw 'Duplicate connector "$id/$name"';
		AssemblyCodec.validateFrame(frame);
		instance.connectors.push({name: name, frame: frame});
	}

	public function pose(id:String):AssemblyFrame return require(id).pose;

	public function worldPoint(id:String, connectorName:String):{x:Float, y:Float, z:Float} {
		var frame = requireConnector(id, connectorName);
		return AssemblyFrames.transformPoint(require(id).pose, frame.x, frame.y, frame.z);
	}

	public function place(id:String, pose:AssemblyFrame):Void {
		if (attached.exists(id)) throw 'Joint-owned instance "$id" cannot be placed independently';
		AssemblyCodec.validateFrame(pose);
		require(id).pose = pose;
	}

	/** The child pose makes its named connector coincide with the parent joint frame. */
	public function mate(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float = 0):Void {
		if (attached.exists(child)) throw 'Instance "$child" already has a parent joint';
		if (kind != "fixed" && kind != "revolute" && kind != "prismatic")
			throw 'Unsupported assembly joint "$kind"';
		if (!Math.isFinite(value)) throw "Assembly joint value must be finite";
		var parentPose = require(parent).pose;
		var parentFrame = requireConnector(parent, parentConnector);
		var childFrame = requireConnector(child, childConnector);
		var motion = kind == "revolute" ? AssemblyFrames.turnY(value)
			: kind == "prismatic" ? AssemblyFrames.translation(0, value, 0)
			: AssemblyFrames.identity();
		var worldJoint = AssemblyFrames.compose(AssemblyFrames.compose(parentPose, parentFrame), motion);
		var childPose = AssemblyFrames.compose(worldJoint, AssemblyFrames.inverse(childFrame));
		AssemblyCodec.validateFrame(childPose);
		require(child).pose = childPose;
		attached.set(child, true);
		joints.push({id: id, kind: kind, parent: parent, parentConnector: parentConnector,
			child: child, childConnector: childConnector, value: value});
	}

	/** Records a closure between already placed instances after checking its pin centers. */
	public function constrain(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, tolerance:Float = 0.001):Void {
		if (kind != "fixed" && kind != "revolute" && kind != "prismatic")
			throw 'Unsupported assembly joint "$kind"';
		if (!Math.isFinite(tolerance) || tolerance < 0) throw "Assembly tolerance must be finite and nonnegative";
		var first = worldPoint(parent, parentConnector), second = worldPoint(child, childConnector);
		var dx = first.x - second.x, dy = first.y - second.y, dz = first.z - second.z;
		if (Math.sqrt(dx * dx + dy * dy + dz * dz) > tolerance)
			throw 'Assembly joint "$id" has separated connectors';
		var firstFrame = AssemblyFrames.compose(require(parent).pose, requireConnector(parent, parentConnector));
		var secondFrame = AssemblyFrames.compose(require(child).pose, requireConnector(child, childConnector));
		if (kind == "fixed") {
			var dot = firstFrame.qx * secondFrame.qx + firstFrame.qy * secondFrame.qy +
				firstFrame.qz * secondFrame.qz + firstFrame.qw * secondFrame.qw;
			if (1 - Math.abs(dot) > 1e-5) throw 'Assembly fixed joint "$id" has misaligned frames';
		} else {
			var firstAxis = AssemblyFrames.transformPoint(firstFrame, 0, 1, 0);
			var secondAxis = AssemblyFrames.transformPoint(secondFrame, 0, 1, 0);
			var ax = firstAxis.x - firstFrame.x, ay = firstAxis.y - firstFrame.y, az = firstAxis.z - firstFrame.z;
			var bx = secondAxis.x - secondFrame.x, by = secondAxis.y - secondFrame.y, bz = secondAxis.z - secondFrame.z;
			if (1 - Math.abs(ax * bx + ay * by + az * bz) > 1e-5)
				throw 'Assembly joint "$id" has misaligned axes';
		}
		joints.push({id: id, kind: kind, parent: parent, parentConnector: parentConnector,
			child: child, childConnector: childConnector, value: 0});
	}

	public function record():AssemblyRecord {
		var result:AssemblyRecord = {instances: instances, joints: joints};
		AssemblyCodec.validate(result);
		return result;
	}

	function require(id:String):AssemblyInstance {
		var instance = byId.get(id);
		if (instance == null) throw 'Missing assembly instance "$id"';
		return instance;
	}
	function requireConnector(id:String, name:String):AssemblyFrame {
		for (connector in require(id).connectors) if (connector.name == name) return connector.frame;
		throw 'Missing assembly connector "$id/$name"';
	}
}
