package humankit;

/** What a limb is being asked to do; Hang is never asked for, it is what a free arm does while the body leans. */
enum abstract LimbMode(Int) {
	/** Follows its animation. */
	var Free = 0;
	/** Reaches for a point. */
	var Reach = 1;
	/** Holds the carry pose at the chest. */
	var Carry = 2;
	/** An arm not otherwise in use, held hanging under its shoulder. */
	var Hang = 3;
}
