package materia.assembly;

/** An assembly-scoped component and one of its authored feature IDs. The using
 * contract determines whether the feature must be a hole, datum or interface.
 */
@:wire typedef AssemblyFeatureReference = {
	@:id(1) var member:String;
	@:id(2) var feature:String;
}
