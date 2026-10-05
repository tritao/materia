package machinekit.fluid;

/** Authored hydraulic flow setup. Stroke and pressure are rebuilt from installed parts. */
@:wire typedef CylinderPhaseRecord = {
	@:id(1) var approach:Float;
	@:id(2) var working:Float;
	@:id(3) var returning:Float;
	@:id(4) var changePoint:Float;
}
