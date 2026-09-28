package humankit;

/**
 * Names of an asset rig's joints for each standard bone. Joint names are
 * compared after dropping any namespace prefix ending in ':', so Mixamo's
 * "mixamorig:Hips" and "mixamorig1:Hips" both match "Hips".
 */
class RigMapping {
	public final name:String;
	final joints:Map<String, String>;

	public function new(name:String, joints:Map<String, String>) {
		this.name = name;
		this.joints = joints;
	}

	/** The asset joint name for a standard bone, or null when the rig lacks it. */
	public function jointName(bone:HumanBone):Null<String>
		return joints.get(bone);

	/** Quaternius characters, including the Ultimate Modular and Universal Animation rigs. */
	public static function quaternius():RigMapping {
		var joints:Map<String, String> = [
			HumanBone.Pelvis => "Hips", HumanBone.Spine => "Abdomen", HumanBone.Spine2 => "Torso",
			HumanBone.Chest => "Chest", HumanBone.Neck => "Neck", HumanBone.Head => "Head"
		];
		for (side in ["L", "R"]) {
			joints.set('shoulder.$side', 'Shoulder.$side');
			joints.set('upper_arm.$side', 'UpperArm.$side');
			joints.set('forearm.$side', 'LowerArm.$side');
			joints.set('hand.$side', 'Wrist.$side');
			// Quaternius's *1 finger joints share one pivot inside the palm; the
			// knuckles the standard bones describe are the *2 joints.
			joints.set('index1.$side', 'Index2.$side');
			joints.set('middle1.$side', 'Middle2.$side');
			joints.set('pinky1.$side', 'Pinky2.$side');
			joints.set('thigh.$side', 'UpperLeg.$side');
			joints.set('shin.$side', 'LowerLeg.$side');
			joints.set('foot.$side', 'Foot.$side');
			joints.set('toe.$side', 'Toes.$side');
		}
		return new RigMapping("quaternius", joints);
	}

	/** Mixamo exports, with or without the "mixamorig:" namespace. */
	public static function mixamo():RigMapping {
		var joints:Map<String, String> = [
			HumanBone.Pelvis => "Hips", HumanBone.Spine => "Spine", HumanBone.Spine2 => "Spine1",
			HumanBone.Chest => "Spine2", HumanBone.Neck => "Neck", HumanBone.Head => "Head"
		];
		for (pair in [["L", "Left"], ["R", "Right"]]) {
			var side = pair[0], prefix = pair[1];
			joints.set('shoulder.$side', '${prefix}Shoulder');
			joints.set('upper_arm.$side', '${prefix}Arm');
			joints.set('forearm.$side', '${prefix}ForeArm');
			joints.set('hand.$side', '${prefix}Hand');
			joints.set('index1.$side', '${prefix}HandIndex1');
			joints.set('middle1.$side', '${prefix}HandMiddle1');
			joints.set('pinky1.$side', '${prefix}HandPinky1');
			joints.set('thigh.$side', '${prefix}UpLeg');
			joints.set('shin.$side', '${prefix}Leg');
			joints.set('foot.$side', '${prefix}Foot');
			joints.set('toe.$side', '${prefix}ToeBase');
		}
		return new RigMapping("mixamo", joints);
	}

	public static function presets():Array<RigMapping>
		return [quaternius(), mixamo()];

	/** Drops a namespace prefix such as "mixamorig:". */
	public static function localName(joint:String):String
		return joint.substr(joint.lastIndexOf(":") + 1);
}
