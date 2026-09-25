package machinekit.catalog;

/** Designation-keyed table of standard sizes, kept separate from geometry code. */
class Catalog<T> {
	public final kind:String;
	final entries:Map<String, T> = [];
	final order:Array<String> = [];

	public function new(kind:String, designation:T->String, rows:Array<T>) {
		this.kind = kind;
		for (row in rows) {
			var key = designation(row);
			if (entries.exists(key)) throw 'Duplicate $kind "$key"';
			entries.set(key, row);
			order.push(key);
		}
	}

	public function get(designation:String):T {
		if (!entries.exists(designation)) throw 'Unknown $kind "$designation"; known: ${order.join(", ")}';
		return entries.get(designation);
	}

	public function exists(designation:String):Bool return entries.exists(designation);

	public function designations():Array<String> return order.copy();
}
