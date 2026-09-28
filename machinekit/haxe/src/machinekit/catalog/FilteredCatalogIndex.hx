package machinekit.catalog;

/** Designation subset of an existing catalog, retaining its provenance metadata. */
class FilteredCatalogIndex implements CatalogIndex {
	final source:CatalogIndex;
	final allowed:Array<String>;

	public function new(source:CatalogIndex, designations:Array<String>) {
		this.source = source;
		allowed = designations.copy();
	}

	public function designations():Array<String> return allowed.copy();

	public function metadata(designation:String):CatalogMetadata {
		if (allowed.indexOf(designation) < 0) throw 'Unknown catalog designation "$designation"';
		return source.metadata(designation);
	}
}
