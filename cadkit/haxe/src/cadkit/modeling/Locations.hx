package cadkit.modeling;

class Locations {
	public static function grid(nx:Int, ny:Int, dx:Float, dy:Float, centered:Bool = true):Array<Location> {
		if (nx < 1 || ny < 1 || !Math.isFinite(dx) || !Math.isFinite(dy))
			throw "invalid grid";
		var result:Array<Location> = [];
		for (y in 0...ny)
			for (x in 0...nx)
				result.push(Location.translation(new Vector((x - (centered ? (nx - 1) / 2 : 0)) * dx, (y - (centered ? (ny - 1) / 2 : 0)) * dy)));
		return result;
	}

	public static function polar(count:Int, radius:Float, start:Float = 0, sweep:Float = 6.283185307179586, rotate:Bool = true):Array<Location> {
		if (count < 1 || !Math.isFinite(radius) || radius < 0 || !Math.isFinite(start) || !Math.isFinite(sweep))
			throw "invalid polar pattern";
		var full = Math.abs(Math.abs(sweep) - 2 * Math.PI) < 1e-10;
		var divisor = full ? count : (count > 1 ? count - 1 : 1);
		var result:Array<Location> = [];
		for (i in 0...count) {
			var a = start + sweep * i / divisor;
			var location = Location.translation(new Vector(radius * Math.cos(a), radius * Math.sin(a)));
			if (rotate)
				location = location.compose(Location.rotation(Axis.Z(), a));
			result.push(location);
		}
		return result;
	}
}
