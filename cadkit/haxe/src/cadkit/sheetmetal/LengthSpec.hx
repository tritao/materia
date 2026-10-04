package cadkit.sheetmetal;

/** Which drawing dimension measures a flange. */
@:wire enum LengthSpec {
	@:id(1) Leg;
	@:id(2) OuterSharp;
	@:id(3) InnerSharp;
	@:id(4) Tangential;
}
