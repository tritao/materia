package materia.project;

/** Explicit simulation contact settings; zero numerical fields keep backend defaults. */
@:wire typedef SceneContactSurface = {
	@:id(1) var friction:Array<Float>;
	@:id(2) var frictionDimensions:Int;
	@:id(3) var contactTimeConstant:Float;
	@:id(4) var contactDampingRatio:Float;
	@:id(5) var filter:SceneContactFilter;
}
