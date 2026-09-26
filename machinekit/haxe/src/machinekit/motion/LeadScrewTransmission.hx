package machinekit.motion;

import cadkit.modeling.AssemblyState;

/** Couples a rotary joint to a prismatic joint using a screw's axial lead.
 * `travel` is measured from the lower end of the allowed stroke.
 */
class LeadScrewTransmission {
	public final rotaryJoint:String;
	public final linearJoint:String;
	public final lead:Float;
	public final linearOffset:Float;
	public final stroke:Float;

	public function new(rotaryJoint:String, linearJoint:String, lead:Float, linearOffset:Float, stroke:Float) {
		if (!(lead > 0) || !Math.isFinite(lead)) throw "Lead-screw transmission needs a positive lead";
		if (!(stroke > 0) || !Math.isFinite(stroke)) throw "Lead-screw transmission needs a positive stroke";
		this.rotaryJoint = rotaryJoint;
		this.linearJoint = linearJoint;
		this.lead = lead;
		this.linearOffset = linearOffset;
		this.stroke = stroke;
	}

	public function rotationFor(travel:Float):Float
		return travel * 2 * Math.PI / lead;

	public function travelFor(rotation:Float):Float
		return rotation * lead / (2 * Math.PI);

	public function setTravel(state:AssemblyState, travel:Float):Void {
		if (!Math.isFinite(travel) || travel < 0 || travel > stroke)
			throw "Linear axis travel is outside its stroke";
		state.setJoint(rotaryJoint, rotationFor(travel));
		state.setJoint(linearJoint, linearOffset + travel);
		state.forwardKinematics();
	}
}
