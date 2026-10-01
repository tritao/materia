package humankit;

/** The frame a limb's reach target is given in. */
enum abstract ReachSpace(Int) {
	/** The world: the target stays put while the body moves. */
	var World = 0;
	/** The body's root frame: the target moves with the worker's feet but not with its torso. */
	var Model = 1;
	/** Relative to the chest: the target moves with the torso, so it stays in the same place against the shoulder as the body leans or straightens. */
	var Torso = 2;
}
