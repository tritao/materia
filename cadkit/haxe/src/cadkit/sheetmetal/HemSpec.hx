package cadkit.sheetmetal;

/** Authored hem dimensions; a missing teardrop radius selects the material's explicit default. */
@:wire enum HemSpec {
	@:id(1) Closed;
	@:id(2) Open(gap:Float);
	@:id(3) Teardrop(gap:Float, radius:Null<Float>);
}
