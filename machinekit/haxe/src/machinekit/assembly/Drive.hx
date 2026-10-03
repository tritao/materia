package machinekit.assembly;

/**
 * What makes one joint move with another: the parts that set a coupling's ratio. A drive names
 * its parts by member id, so the ratio follows them when a part's values change and the
 * assembly is rebuilt.
 *
 * The leader is the joint motion is planned in, such as a machine axis's carriage, and the
 * follower turns with it. `alignment` is +1 when the leader moves along the follower's turning
 * axis (right-hand rule) as the follower turns positively, and -1 when it moves against it.
 */
enum Drive {
	/** Lead screw member `screw` (a `LeadScrew`) turns on the follower and its nut rides the
	 * leader's slide: one turn advances the nut by the thread's signed lead. */
	LeadScrew(screw:String, alignment:Float);
	/** Gear member `driver` turns on the leader and meshes with `driven` on the follower, which
	 * turns the other way by the ratio of their teeth. Both are `SpurGear`s; `alignment` is +1
	 * when the two joints' axes point the same way. */
	GearMesh(driver:String, driven:String, alignment:Float);
	/** Pinion member (a `SpurGear`) turns on the follower and rolls along a rack fixed beside the
	 * leader's slide: one radian of the pinion moves the slide its pitch radius. */
	RackAndPinion(pinion:String, alignment:Float);
	/** Pulley or sprocket member (a `TimingPulley` or `Sprocket`) turns on the follower, its belt
	 * or chain clamped to the leader's slide: one radian moves the slide its pitch radius. */
	Belt(pulley:String, alignment:Float);
}
