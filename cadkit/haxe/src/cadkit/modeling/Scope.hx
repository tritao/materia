package cadkit.modeling;

/** Explicit resource scope for fluent modeling. own() registers a model; release() transfers it out. */
class Scope {
	private final models:Array<Model>;
	private var closed:Bool;

	public function new() {
		models = [];
		closed = false;
	}

	public function own<T:Model>(model:T):T {
		if (closed) {
			model.close();
			throw "scope is closed";
		}
		if (models.indexOf(model) < 0)
			models.push(model);
		return model;
	}

	public function release<T:Model>(model:T):T {
		models.remove(model);
		return model;
	}

	public function close():Void {
		if (closed)
			return;
		closed = true;
		var i = models.length;
		while (i > 0) {
			i--;
			models[i].close();
		}
		models.resize(0);
	}

	public static function run<T:Model>(callback:Scope->T):T {
		var scope = new Scope();
		try {
			var result = callback(scope);
			scope.release(result);
			scope.close();
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}
}
