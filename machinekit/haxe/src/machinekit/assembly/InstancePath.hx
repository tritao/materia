package machinekit.assembly;

/** A slash-separated occurrence path; each segment is a local instance name. */
abstract InstancePath(String) to String {
	public inline function new(name:String) this = segment(name);

	public static function of(path:String):InstancePath {
		if (path == null || path.length == 0) throw "Instance path needs a name";
		for (part in path.split("/")) segment(part);
		return cast path;
	}

	public static function segment(name:String):String {
		if (name == null || name.length == 0 || name.indexOf("/") >= 0)
			throw 'Instance name "$name" must be nonempty and cannot contain "/"';
		return name;
	}

	public function segments():Array<String> return (this : String).split("/");

	public function parent():Null<InstancePath> {
		var at = (this : String).lastIndexOf("/");
		return at < 0 ? null : of((this : String).substr(0, at));
	}

	public function child(name:String):InstancePath return of((this : String) + "/" + segment(name));
}
