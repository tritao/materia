package machinekit.component;

/**
 * A typed fact a part states about itself, such as a suction contact or a welding supply. Each kind
 * is its own class in the package of its domain, so a new domain adds a class rather than changing
 * the component core; look one up with that class's `of`.
 */
interface ComponentFacet {
	/** Throws when `component` lacks a port or connector this facet names, or a value is out of range. */
	function check(component:MachineComponent):Void;

	/** The facet and its values in words, for diagnostics and comparisons. */
	function describe():String;
}
