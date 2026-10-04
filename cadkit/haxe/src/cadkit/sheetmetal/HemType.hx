package cadkit.sheetmetal;

/** Return flange geometry: contacting layers, parallel layers with a gap, or a rounded bulb and sloping return. */
@:wire enum HemType {
	@:id(1) Closed;
	@:id(2) Open;
	@:id(3) Teardrop;
}
