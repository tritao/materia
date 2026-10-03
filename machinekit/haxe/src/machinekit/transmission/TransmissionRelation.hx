package machinekit.transmission;

import haxe.ds.ReadOnlyArray;

/** Resolved motion and engineering allowances, in the coupling's units. */
class TransmissionRelation {
	public var ratio:Float;
	public var efficiency:Float;
	public var stiffness:Null<Float>;
	public var backlash:Null<Float>;
	public var drag:Null<Float>;
	public var followerSpeedCap:Null<Float>;
	final bases:Map<String, ValueBasis> = [];
	final labels:Map<String, String> = [];
	/** Assumptions are a projection of the field bases, so a stated override removes its label. */
	public var assumed(get, never):ReadOnlyArray<String>;

	public function setBasis(field:String, basis:ValueBasis, ?label:String):Void {
		if (!switch field { case "ratio", "efficiency", "stiffness", "backlash", "drag", "followerSpeedCap": true; default: false; })
			throw 'Unknown relation field "$field"';
		bases.set(field, basis);
		if (basis == Assumed) labels.set(field, label == null ? field : label);
		else labels.remove(field);
	}

	public function basis(field:String):ValueBasis {
		var value = bases.get(field);
		return value == null ? Derived : value;
	}

	function get_assumed():ReadOnlyArray<String> {
		var result:Array<String> = [];
		for (field in labels.keys()) {
			var label = labels.get(field);
			if (label != null && result.indexOf(label) < 0) result.push(label);
		}
		result.sort(Reflect.compare);
		return result;
	}

	public function new(ratio:Float, efficiency:Float, ?stiffness:Float, ?backlash:Float,
			?drag:Float, ?followerSpeedCap:Float) {
		this.ratio = ratio;
		this.efficiency = efficiency;
		this.stiffness = stiffness;
		this.backlash = backlash;
		this.drag = drag;
		this.followerSpeedCap = followerSpeedCap;
	}
}
