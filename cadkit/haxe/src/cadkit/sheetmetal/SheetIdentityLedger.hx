package cadkit.sheetmetal;

/** Authored monotonic identity allocator shared by one document sheet chain, never by its derived graph copies. */
class SheetIdentityLedger {
	var nextWall:Int;
	var nextBend:Int;
	final wallOwners:Map<Int, String>;
	final bendOwners:Map<Int, String>;
	public function new(nextWall:Int = 2, nextBend:Int = 1) {
		if (nextWall < 2 || nextBend < 1)
			throw new SheetDiagnosis("sheet.invalid-identity-ledger", "Sheet counters must follow the root wall identity");
		this.nextWall = nextWall; this.nextBend = nextBend;
		wallOwners = new Map(); bendOwners = new Map();
		wallOwners.set(1, "root");
	}
	public function wallCounter():Int { return nextWall; }
	public function bendCounter():Int { return nextBend; }
	public function reserveWalls(owner:String, count:Int):Array<Int> {
		checkCapacity(nextWall, count);
		var result:Array<Int> = [];
		for (i in 0...count) {
			var id = nextWall++; result.push(id); wallOwners.set(id, owner + ":" + i);
		}
		return result;
	}
	public function reserveBends(owner:String, count:Int):Array<Int> {
		checkCapacity(nextBend, count);
		var result:Array<Int> = [];
		for (i in 0...count) {
			var id = nextBend++; result.push(id); bendOwners.set(id, owner + ":" + i);
		}
		return result;
	}
	public function claimWalls(owner:String, ids:Array<Int>):Void { claim(owner, ids, nextWall, wallOwners); }
	public function claimBends(owner:String, ids:Array<Int>):Void { claim(owner, ids, nextBend, bendOwners); }
	static function checkCapacity(next:Int, count:Int):Void {
		if (count <= 0 || count > 2147483647 - next)
			throw new SheetDiagnosis("sheet.identity-exhausted", "Sheet identity counter cannot reserve the requested outputs");
	}
	static function claim(owner:String, ids:Array<Int>, next:Int, owners:Map<Int, String>):Void {
		for (i in 0...ids.length) {
			var id = ids[i];
			var label = owner + ":" + i;
			if (id <= 0 || id >= next || (owners.exists(id) && owners.get(id) != label))
				throw new SheetDiagnosis("sheet.invalid-identity-ledger", "Saved sheet output identity is duplicated or outside its allocator", [label, Std.string(id)]);
			owners.set(id, label);
		}
	}
}
