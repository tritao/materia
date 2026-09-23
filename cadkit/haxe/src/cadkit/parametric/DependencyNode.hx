package cadkit.parametric;

/** Typed source or target in a document dependency graph. */
enum DependencyNode {
	FeatureNode(id:Int);
	ElementNode(id:String);
	DefinitionNode(id:String);
	ParameterNode(name:String);
}
