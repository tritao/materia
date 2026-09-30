import cadkit.solve.JacobianCheck;

/** The finite-difference oracle accepts correct Jacobians and points at a wrong entry. */
class JacobianCheckSmoke {
	public static function run():Void {
		var seed = 12345;
		for (_ in 0...20) {
			var x = [for (_ in 0...4) {
				seed = (seed * 1103515245 + 12345) & 0x7fffffff;
				(seed / 0x7fffffff) * 20 - 10;
			}];
			var result = JacobianCheck.compare(residual, jacobian, x);
			check(result.passed, 'a correct Jacobian passes: $result');
		}

		var wrong = JacobianCheck.compare(residual, x -> {
			var j = jacobian(x);
			j[1 * 4 + 1] = -j[1 * 4 + 1];
			return j;
		}, [1.5, -2, 3, 0.25]);
		check(!wrong.passed && wrong.row == 1 && wrong.column == 1, 'a flipped sign is found at (1, 1): $wrong');

		var threw = false;
		try JacobianCheck.compare(residual, x -> [0.0], [1, 2, 3, 4]) catch (_:Dynamic) threw = true;
		check(threw, "a Jacobian of the wrong size is refused");
	}

	/** Products, a transcendental term and a distance, like real constraint rows. */
	static function residual(x:Array<Float>):Array<Float> {
		var dx = x[2] - x[0], dy = x[3] - x[1];
		return [x[0] * x[1] - 3, Math.sin(x[0]) + x[1] * x[1], Math.sqrt(dx * dx + dy * dy) - 2];
	}

	static function jacobian(x:Array<Float>):Array<Float> {
		var dx = x[2] - x[0], dy = x[3] - x[1];
		var length = Math.sqrt(dx * dx + dy * dy);
		return [
			x[1], x[0], 0, 0,
			Math.cos(x[0]), 2 * x[1], 0, 0,
			-dx / length, -dy / length, dx / length, dy / length
		];
	}

	static function check(value:Bool, message:String):Void {
		if (!value) throw message;
	}
}
