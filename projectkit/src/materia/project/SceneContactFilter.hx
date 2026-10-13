package materia.project;

/** Which contacts an authored assembly hull participates in. */
@:wire enum SceneContactFilter {
	@:id(1) Layers;
	@:id(2) PairsOnly;
	@:id(3) PairsAndEnvironment;
}
