package machinekit.catalog;

/** Read-only designation and provenance index shared by editable catalog inputs. */
interface CatalogIndex {
	public function designations():Array<String>;
	public function metadata(designation:String):CatalogMetadata;
}
