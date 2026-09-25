package materia.project;

import haxe.Json;
import materia.project.AssemblyRecord.AssemblyFrame;
import materia.project.AssemblyRecord.AssemblyConnector;
import materia.project.AssemblyRecord.AssemblyInstance;
import materia.project.AssemblyRecord.AssemblyJoint;

/** Bounded, validated metadata for rigid instances and their mating frames. */
class AssemblyCodec {
	public static function encode(record:AssemblyRecord):String {
		validate(record);
		return Json.stringify(record);
	}

	public static function decode(text:String):AssemblyRecord {
		var raw:Dynamic = Json.parse(text);
		var rawInstances = arrayField(raw, "instances"), rawJoints = arrayField(raw, "joints");
		var instances:Array<AssemblyInstance> = [], joints:Array<AssemblyJoint> = [];
		for (item in rawInstances) {
			var connectors:Array<AssemblyConnector> = [];
			for (connector in arrayField(item, "connectors"))
				connectors.push({name: textField(connector, "name"), frame: readFrame(field(connector, "frame"))});
			instances.push({id: textField(item, "id"), pose: readFrame(field(item, "pose")), connectors: connectors});
		}
		for (item in rawJoints)
			joints.push({id: textField(item, "id"), kind: textField(item, "kind"),
				parent: textField(item, "parent"), parentConnector: textField(item, "parentConnector"),
				child: textField(item, "child"), childConnector: textField(item, "childConnector"),
				value: numberField(item, "value")});
		var record:AssemblyRecord = {instances: instances, joints: joints};
		validate(record);
		return record;
	}

	public static function validate(record:AssemblyRecord):Void {
		if (record == null || record.instances == null || record.joints == null ||
			record.instances.length == 0 || record.instances.length > 1000 || record.joints.length > 4000)
			throw "Assembly has invalid instance or joint counts";
		var instances = new Map<String, AssemblyInstance>(), joints = new Map<String, Bool>();
		for (instance in record.instances) {
			if (instance == null || !validText(instance.id) || instances.exists(instance.id) ||
				instance.connectors == null || instance.connectors.length > 100)
				throw "Assembly has an invalid or duplicate instance";
			validateFrame(instance.pose);
			var names = new Map<String, Bool>();
			for (connector in instance.connectors) {
				if (connector == null || !validText(connector.name) || names.exists(connector.name))
					throw 'Assembly instance "${instance.id}" has an invalid connector';
				names.set(connector.name, true);
				validateFrame(connector.frame);
			}
			instances.set(instance.id, instance);
		}
		for (joint in record.joints) {
			if (joint == null || !validText(joint.id) || joints.exists(joint.id) ||
				(joint.kind != "fixed" && joint.kind != "revolute" && joint.kind != "continuous" &&
					joint.kind != "prismatic") ||
				joint.parent == joint.child || !Math.isFinite(joint.value))
				throw "Assembly has an invalid joint";
			joints.set(joint.id, true);
			if (!hasConnector(instances.get(joint.parent), joint.parentConnector) ||
				!hasConnector(instances.get(joint.child), joint.childConnector))
				throw 'Assembly joint "${joint.id}" has a missing endpoint';
		}
	}

	public static function validateFrame(frame:AssemblyFrame):Void {
		if (frame == null || !Math.isFinite(frame.x) || !Math.isFinite(frame.y) ||
			!Math.isFinite(frame.z) || !Math.isFinite(frame.qx) || !Math.isFinite(frame.qy) ||
			!Math.isFinite(frame.qz) || !Math.isFinite(frame.qw) ||
			Math.abs(frame.x) > 1e9 || Math.abs(frame.y) > 1e9 || Math.abs(frame.z) > 1e9 ||
			Math.abs(frame.qx * frame.qx + frame.qy * frame.qy + frame.qz * frame.qz + frame.qw * frame.qw - 1) > 1e-4)
			throw "Assembly frame must have finite position and unit rotation";
	}

	static function hasConnector(instance:Null<AssemblyInstance>, name:String):Bool {
		if (instance == null) return false;
		for (connector in instance.connectors) if (connector.name == name) return true;
		return false;
	}

	static function readFrame(value:Dynamic):AssemblyFrame
		return {x: numberField(value, "x"), y: numberField(value, "y"), z: numberField(value, "z"),
			qx: numberField(value, "qx"), qy: numberField(value, "qy"),
			qz: numberField(value, "qz"), qw: numberField(value, "qw")};

	static function field(value:Dynamic, name:String):Dynamic {
		if (value == null || !Reflect.hasField(value, name)) throw 'Assembly field "$name" is missing';
		return Reflect.field(value, name);
	}
	static function arrayField(value:Dynamic, name:String):Array<Dynamic> {
		var result = field(value, name);
		if (!Std.isOfType(result, Array)) throw 'Assembly field "$name" must be an array';
		return cast result;
	}
	static function textField(value:Dynamic, name:String):String {
		var result = field(value, name);
		if (!Std.isOfType(result, String) || !validText(cast result))
			throw 'Assembly field "$name" must be nonempty text';
		return cast result;
	}
	static function numberField(value:Dynamic, name:String):Float {
		var result = field(value, name);
		if ((!Std.isOfType(result, Float) && !Std.isOfType(result, Int)) || !Math.isFinite(cast result))
			throw 'Assembly field "$name" must be finite';
		return cast result;
	}
	static function validText(value:Null<String>):Bool
		return value != null && value.length > 0 && value.length <= 4096 &&
			StringTools.trim(value).length > 0 && value.indexOf("\x00") < 0;
}
