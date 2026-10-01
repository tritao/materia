package humankit;

/** Where a worker stands and how it holds its body to reach a point: what ApproachFor weighs between. */
typedef Stance = {
	/** Why this depth cannot reach the point, or null when it can. */
	var failure:Null<String>;
	/** How deep a crouch, 0 upright to 1 the full crouch. */
	var crouch:Float;
	/** How deep a kneel, 0 upright to 1 the lowest; a stance is a crouch or a kneel, not both. */
	var kneel:Float;
	var lean:Float;
	/** How far back from the point, along the line to it, the shoulder's root stands. */
	var standDistance:Float;
	/** How far short of clearing the surface's edge the belly falls once the lean and the stretch are spent. */
	var shortfall:Float;
	/** The unit direction from the worker to the point on the floor. */
	var ux:Float;
	var uy:Float;
	var lateral:Float;
	/** Whether the point is no further below the shoulder than the arm comfortably reaches. */
	var comfortable:Bool;
}
