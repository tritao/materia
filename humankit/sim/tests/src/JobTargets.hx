import humankit.HumanJobTargets;
import humankit.HumanTargetBox;

class JobTargets implements HumanJobTargets {
	public final boxes:Map<String, HumanTargetBox> = new Map();
	public function new() {}
	public function box(objectId:String):Null<HumanTargetBox>
		return boxes.get(objectId);
}
