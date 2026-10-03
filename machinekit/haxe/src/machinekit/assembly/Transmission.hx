package machinekit.assembly;

/** Parts that define a coupling. The leader is the coordinate motion is planned in. */
@:wire enum Transmission {
	/** The nut rides the leader's slide; the screw turns on the follower. */
	@:id(1) LeadScrew(screw:String, nut:Null<String>);
	/** The driver turns on the leader and meshes with the follower's gear. */
	@:id(2) GearMesh(driver:String, driven:String);
	/** The follower's pinion rolls along the leader's rack. */
	@:id(3) RackAndPinion(pinion:String, rack:Null<String>);
	/** The pulley turns with the belt clamped to the leader on `strand`. */
	@:id(4) TimingBelt(belt:String, pulley:String, strand:Int);
	/** A roller chain is a separate family from a timing belt. */
	@:id(5) RollerChain(chain:Null<String>, sprocket:String);
}
