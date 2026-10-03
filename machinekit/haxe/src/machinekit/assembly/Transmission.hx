package machinekit.assembly;

/** Parts that define a coupling. The leader is the coordinate motion is planned in. */
@:wire enum Transmission {
	/** The nut rides the leader's slide; the screw turns on the follower. Same follows the screw's
	 * thread handedness: positive slide turns a right-hand screw negatively. */
	@:id(1) LeadScrew(screw:String, nut:String);
	/** The driver turns on the leader and meshes with the follower's gear. Same is an external mesh,
	 * so the gears turn in opposite directions. */
	@:id(2) GearMesh(driver:String, driven:String);
	/** The follower's pinion rolls along the leader's rack. Same maps positive rack travel to
	 * positive pinion rotation. */
	@:id(3) RackAndPinion(pinion:String, rack:Null<String>);
	/** The pulley turns with the belt clamped to the leader on `strand`. Same maps positive belt
	 * travel to positive pulley rotation. */
	@:id(4) TimingBelt(belt:String, pulley:String, strand:Int);
	/** A roller chain is a separate family from a timing belt. Same maps positive chain travel
	 * to positive sprocket rotation. */
	@:id(5) RollerChain(chain:Null<String>, sprocket:String);
}
