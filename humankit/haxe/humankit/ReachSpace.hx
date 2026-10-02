package humankit;

/** The frame a limb's reach target is given in. */
enum abstract ReachSpace(Int) {
	/** The world: the target stays put while the body moves. */
	var World = 0;
	/** The body's root frame: the target moves with the worker's feet but not with its torso. */
	var Model = 1;
	/** Relative to the limb's shoulder, in the model's axes: the target moves with the shoulder, so the arm keeps its shape as the body walks. */
	var Shoulder = 2;
}
