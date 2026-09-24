import CadKit;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Scope;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.modeling.Curve;
import sys.FileSystem;

private typedef ExcavatorHole = {
	var x:Float;
	var z:Float;
	var radius:Float;
}

/** One separately generated B-rep component in the excavator example. */
class ExcavatorComponent {
	public final name:String;
	public final part:Part;

	public function new(name:String, part:Part) {
		this.name = name;
		this.part = part;
	}

	public function close():Void part.close();
}

/**
	Code-authored, multi-part excavator geometry for the assembly workflow.
	All dimensions are millimetres. The example deliberately builds independent
	OCCT solids rather than importing a mesh or collapsing the mechanism into one
	shape. The parts use a provisional shared mechanism coordinate space; an
	assembly document will own explicit placements and joints in a later stage.
**/
class ProceduralExcavator {
	static final SIDE_PLANE_NORMAL:Vector = Vector.Y().scale(-1);

	public static function build(?progress:String->Void):Array<ExcavatorComponent> {
		var report = progress == null ? function(message:String) {} : progress;
		var result:Array<ExcavatorComponent> = [];
		try {
			report("Building Base");
			result.push(new ExcavatorComponent("Base", buildBase()));
			report("Built Base");
			report("Building BasePin");
			result.push(new ExcavatorComponent("BasePin", buildBasePin()));
			report("Built BasePin");
			report("Building Boom");
			result.push(new ExcavatorComponent("Boom", buildBoom()));
			report("Built Boom");
			report("Building Stick");
			result.push(new ExcavatorComponent("Stick", buildStick()));
			report("Built Stick");
			report("Building Bucket");
			result.push(new ExcavatorComponent("Bucket", buildBucket()));
			report("Built Bucket");
			report("Building BucketLink1");
			result.push(new ExcavatorComponent("BucketLink1", buildBucketLink(true)));
			report("Built BucketLink1");
			report("Building BucketLink2");
			result.push(new ExcavatorComponent("BucketLink2", buildBucketLink(false)));
			report("Built BucketLink2");
			report("Building hydraulic cylinders");
			result.push(new ExcavatorComponent("BoomCylinderOuter", buildCylinderOuter(
				new Vector(-14, 0, 53), extendLine(new Vector(-14, 0, 53), new Vector(11, 0, 128), 2.15),
				10.1, 6.2, 30)));
			result.push(new ExcavatorComponent("BoomCylinderInner", buildCylinderInner(
				new Vector(11, 0, 128), new Vector(77, 0, 295), 6.0, 8.5, 15)));
			result.push(new ExcavatorComponent("StickCylinderOuter", buildCylinderOuter(
				new Vector(39, 0, 174), new Vector(209, 0, 134), 9.8, 5.8, 30)));
			result.push(new ExcavatorComponent("StickCylinderInner", buildCylinderInner(
				new Vector(84, 0, 132), new Vector(257, 0, 92), 6.1, 7.5, 15)));
			result.push(new ExcavatorComponent("BucketCylinderOuter", buildCylinderOuter(
				new Vector(87, 0, 150), new Vector(245, 0, 83), 6.2, 3.6, 15)));
			result.push(new ExcavatorComponent("BucketCylinderInner", buildCylinderInner(
				new Vector(128, 0, 119), new Vector(291, 0, 52), 3.8, 6.0, 15)));
			report("Built hydraulic cylinders");
			if (result.length != 13)
				throw "procedural excavator component inventory must contain 13 occurrences";
			return result;
		} catch (error:Dynamic) {
			for (component in result) component.close();
			throw error;
		}
	}

	static function buildBase():Part {
		var scope = new Scope();
		try {
			var pieces:Array<Part> = [];
			pieces.push(scope.own(boxAt(52, 96, 16, 0, 0, 10)));
			pieces.push(scope.own(boxAt(42, 42, 80, -5, 0, 26)));
			// Track rails and their end drums make the undercarriage read as a
			// tracked vehicle instead of a plain support block.
			for (side in [-1, 1]) {
				var y = side * 54.0;
				pieces.push(scope.own(boxAt(88, 12, 10, 0, y, 0)));
				for (x in [-30.0, 0.0, 30.0])
					pieces.push(scope.own(cylinderBetween(new Vector(x, y + 6, 8),
						new Vector(x, y - 6, 8), 6.5)));
			}
			var result = fuseAll(scope, pieces);
			scope.release(result);
			scope.close();
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	static function buildBasePin():Part {
		return cylinderBetween(new Vector(-1, 50, 85), new Vector(-1, -50, 85), 10);
	}

	static function buildBoom():Part {
		var anchorX = -1.0;
		var anchorZ = 85.0;
		var scaleX = 4.28;
		var scaleZ = 1.37;
		var outline = scaleOutline([
			new Vector(-30, 65), new Vector(-18, 61), new Vector(0, 65),
			new Vector(13, 88), new Vector(25, 112), new Vector(29, 139),
			new Vector(35, 165), new Vector(46, 181), new Vector(57, 190),
			new Vector(68, 194), new Vector(72, 203), new Vector(50, 210),
			new Vector(-15, 100)
		], anchorX, anchorZ, scaleX, scaleZ);
		var plate = profilePlate(57, outline, []);
		return addBossPairs(plate, [
			{ x: anchorX, z: anchorZ, radius: 30.0 },
			{ x: anchorX + (52 - anchorX) * scaleX, z: anchorZ + (190 - anchorZ) * scaleZ, radius: 30.0 }
		], 61, [
			hole(anchorX, anchorZ, 14),
			hole(anchorX + (52 - anchorX) * scaleX, anchorZ + (190 - anchorZ) * scaleZ, 14),
			hole(anchorX + (21 - anchorX) * scaleX, anchorZ + (119 - anchorZ) * scaleZ, 9),
			hole(anchorX + (5 - anchorX) * scaleX, anchorZ + (85 - anchorZ) * scaleZ, 4.5),
			hole(anchorX + (26 - anchorX) * scaleX, anchorZ + (132 - anchorZ) * scaleZ, 4.5),
			hole(anchorX + (35 - anchorX) * scaleX, anchorZ + (158 - anchorZ) * scaleZ, 4.5),
			hole(anchorX + (43 - anchorX) * scaleX, anchorZ + (171 - anchorZ) * scaleZ, 4.5)
		]);
	}

	static function buildStick():Part {
		var anchorX = 49.0;
		var anchorZ = 184.0;
		var scaleX = 3.42;
		var scaleZ = 1.41;
		var outline = scaleOutline([
			new Vector(42, 185), new Vector(53, 196), new Vector(84, 174),
			new Vector(139, 137), new Vector(153, 124), new Vector(150, 111),
			new Vector(136, 120), new Vector(79, 155), new Vector(49, 171)
		], anchorX, anchorZ, scaleX, scaleZ);
		var scope = new Scope();
		try {
			var plate = scope.own(profilePlate(32, outline, [
				hole(anchorX, anchorZ, 9.75),
				hole(mapProfilePoint(144, 124, anchorX, anchorZ, scaleX, scaleZ).x,
					mapProfilePoint(144, 124, anchorX, anchorZ, scaleX, scaleZ).z, 9.75),
				hole(mapProfilePoint(85, 157, anchorX, anchorZ, scaleX, scaleZ).x,
					mapProfilePoint(85, 157, anchorX, anchorZ, scaleX, scaleZ).z, 6.3),
				hole(320, 108, 5),
				hole(290, 116, 4)
			]));
			var eye = scope.own(pinEye(anchorX, anchorZ, 16.5, 9.75, 32));
			var result = scope.own(plate.combine(eye));
			scope.release(result);
			scope.close();
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	static function buildBucket():Part {
		// Thin cheeks, a cross barrel, faceted shell web, cutting edge, and
		// separated teeth keep the bucket a single detailed solid.
		var anchorX = 137.0;
		var anchorZ = 119.0;
		var outline = scaleOutline([
			new Vector(130, 125), new Vector(145, 117), new Vector(166, 99),
			new Vector(173, 93), new Vector(178, 81),
			new Vector(184, 72), new Vector(181, 58), new Vector(169, 68),
			new Vector(155, 86), new Vector(139, 94), new Vector(126, 106)
		], anchorX, anchorZ, 1.75, 1.46);
		var scope = new Scope();
		try {
			var leftCheek = scope.own(profilePlate(2, outline, []));
			var leftPlaced = scope.own(leftCheek.translated(new Vector(0, -48, 0)));
			var rightCheek = scope.own(profilePlate(2, outline, []));
			var rightPlaced = scope.own(rightCheek.translated(new Vector(0, 48, 0)));
			var pieces:Array<Part> = [leftPlaced, rightPlaced];
			var barrelOuter = scope.own(cylinderBetween(new Vector(anchorX, 51.7, anchorZ),
				new Vector(anchorX, -51.7, anchorZ), 9));
			var barrelInner = scope.own(cylinderBetween(new Vector(anchorX, 52.7, anchorZ),
				new Vector(anchorX, -52.7, anchorZ), 6));
			pieces.push(scope.own(barrelOuter.subtract(barrelInner)));
			pieces.push(scope.own(boxAt(18, 100, 9, 202, 0, 29)));
			pieces.push(scope.own(boxAt(18, 98, 2, 198, 0, 42)));
			for (y in [-36.0, -18.0, 0.0, 18.0, 36.0])
				pieces.push(scope.own(boxAt(13, 10, 22, 211, y, 10)));
			var result = fuseAll(scope, pieces);
			// Re-cut the pivot bores after joining the cheek, barrel, shell, and lip.
			var secondPin = mapProfilePoint(165, 91, anchorX, anchorZ, 1.75, 1.46);
			var boreA = scope.own(cylinderBetween(new Vector(anchorX, 51, anchorZ),
				new Vector(anchorX, -51, anchorZ), 6));
			var boreB = scope.own(cylinderBetween(new Vector(secondPin.x, 51, secondPin.z),
				new Vector(secondPin.x, -51, secondPin.z), 5.2));
			var cutA = scope.own(result.subtract(boreA));
			var cutB = scope.own(cutA.subtract(boreB));
			var cutC = scope.own(cutB.subtract(scope.own(cylinderBetween(
				new Vector(165, 51, 68), new Vector(165, -51, 68), 3.2))));
			var cutD = scope.own(cutC.subtract(scope.own(cylinderBetween(
				new Vector(183, 51, 52), new Vector(183, -51, 52), 3.2))));
			var cutE = scope.own(cutD.subtract(scope.own(cylinderBetween(
				new Vector(150, 51, 96), new Vector(150, -51, 96), 3.2))));
			scope.release(cutE);
			scope.close();
			return cutE;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	static function buildBucketLink(first:Bool):Part {
		var z1 = first ? 112.0 : 105.0;
		var z2 = first ? 151.0 : 158.0;
		var anchorX = 129.5;
		var anchorZ = (z1 + z2) / 2;
		var scaleX = first ? 0.72 : 1.14;
		var scaleZ = first ? 1.1 : 1.03;
		var bossLength = first ? 27.0 : 47.0;
		var bossRadius = first ? 7.4 : 7.8;
		var holeRadius = first ? 4.5 : 4.0;
		var sourceOutline = first ? [
			new Vector(111, z1 - 7), new Vector(121, z1 - 9),
			new Vector(143, z2 - 5), new Vector(148, z2 + 3),
			new Vector(139, z2 + 9), new Vector(116, z1 + 8)
		] : [
			new Vector(111, z1 - 7), new Vector(121, z1 - 9),
			new Vector(130, z1 + 4), new Vector(138, z2 - 18),
			new Vector(143, z2 - 5), new Vector(148, z2 + 3),
			new Vector(143, z2 + 7), new Vector(139, z2 + 9),
			new Vector(116, z1 + 8), new Vector(112, z1 + 1)
		];
		var plate = profilePlate(first ? 20 : 5,
			scaleOutline(sourceOutline, anchorX, anchorZ, scaleX, scaleZ), []);
		return addBossPairs(plate, [
			{ x: mapProfilePoint(116, z1, anchorX, anchorZ, scaleX, scaleZ).x,
				z: mapProfilePoint(116, z1, anchorX, anchorZ, scaleX, scaleZ).z, radius: bossRadius },
			{ x: mapProfilePoint(143, z2, anchorX, anchorZ, scaleX, scaleZ).x,
				z: mapProfilePoint(143, z2, anchorX, anchorZ, scaleX, scaleZ).z, radius: bossRadius }
		], bossLength, [
			hole(mapProfilePoint(116, z1, anchorX, anchorZ, scaleX, scaleZ).x,
				mapProfilePoint(116, z1, anchorX, anchorZ, scaleX, scaleZ).z, holeRadius),
			hole(mapProfilePoint(143, z2, anchorX, anchorZ, scaleX, scaleZ).x,
				mapProfilePoint(143, z2, anchorX, anchorZ, scaleX, scaleZ).z, holeRadius)
		]);
	}

	static function buildCylinderOuter(start:Vector, end:Vector, outerRadius:Float, boreRadius:Float,
		eyeWidth:Float):Part {
		var scope = new Scope();
		try {
			var outer = scope.own(cylinderBetween(start, end, outerRadius));
			var inner = scope.own(cylinderBetween(start, end, boreRadius));
			var tube = scope.own(outer.subtract(inner));
			var eye = scope.own(pinEye(start.x, start.z, outerRadius * 1.35,
				Math.min(boreRadius, 4.5), eyeWidth));
			var withEye = scope.own(tube.combine(eye));
			var bore = scope.own(cylinderBetween(new Vector(start.x, eyeWidth / 2 + 2, start.z),
				new Vector(start.x, -eyeWidth / 2 - 2, start.z), Math.min(boreRadius, 4.5)));
			var result = scope.own(withEye.subtract(bore));
			scope.release(result);
			scope.close();
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	static function buildCylinderInner(start:Vector, end:Vector, rodRadius:Float, eyeRadius:Float,
		eyeWidth:Float):Part {
		var direction = end.subtract(start).normalized();
		var rodStart = start.subtract(direction.scale(2));
		var rodEnd = end.add(direction.scale(5));
		var scope = new Scope();
		try {
			var rod = scope.own(cylinderBetween(rodStart, rodEnd, rodRadius));
			var eye = scope.own(pinEye(end.x, end.z, eyeRadius,
				Math.min(rodRadius * 0.75, 3.2), eyeWidth));
			var withEye = scope.own(rod.combine(eye));
			var bore = scope.own(cylinderBetween(new Vector(end.x, eyeWidth / 2 + 2, end.z),
				new Vector(end.x, -eyeWidth / 2 - 2, end.z), Math.min(rodRadius * 0.75, 3.2)));
			var result = scope.own(withEye.subtract(bore));
			scope.release(result);
			scope.close();
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	static function profilePlate(thickness:Float, outline:Array<Vector>, holes:Array<ExcavatorHole>):Part {
		var plane = new Plane(new Vector(0, thickness / 2, 0), Vector.X(), SIDE_PLANE_NORMAL);
		var points = [for (point in outline) plane.toWorld(new Vector(point.x, point.y, 0))];
		var boundary = Curve.polyline(points, true);
		var holeCurves:Array<Curve> = [];
		var sketch:Null<Sketch> = null;
		try {
			for (item in holes) {
				var center = plane.toWorld(new Vector(item.x, item.z, 0));
				holeCurves.push(Curve.circle(item.radius,
					new Plane(center, Vector.X(), SIDE_PLANE_NORMAL)));
			}
			sketch = Sketch.face(boundary, holeCurves, plane);
			var result = sketch.extrude(thickness);
			sketch.close();
			boundary.close();
			for (curve in holeCurves) curve.close();
			return result;
		} catch (error:Dynamic) {
			if (sketch != null) sketch.close();
			boundary.close();
			for (curve in holeCurves) curve.close();
			throw error;
		}
	}

	static function scaleOutline(outline:Array<Vector>, anchorX:Float, anchorZ:Float,
		scaleX:Float, scaleZ:Float):Array<Vector> {
		return [for (point in outline) new Vector(
			anchorX + (point.x - anchorX) * scaleX,
			anchorZ + (point.y - anchorZ) * scaleZ
		)];
	}

	static function mapProfilePoint(x:Float, z:Float, anchorX:Float, anchorZ:Float,
		scaleX:Float, scaleZ:Float):Vector {
		return new Vector(anchorX + (x - anchorX) * scaleX, 0,
			anchorZ + (z - anchorZ) * scaleZ);
	}

	static function extendLine(start:Vector, end:Vector, factor:Float):Vector {
		return start.add(end.subtract(start).scale(factor));
	}

	static function addBossPairs(plate:Part, bosses:Array<{x:Float, z:Float, radius:Float}>,
		bossLength:Float, holes:Array<ExcavatorHole>):Part {
		var scope = new Scope();
		try {
			var parts:Array<Part> = [scope.own(plate)];
			for (boss in bosses)
				parts.push(scope.own(cylinderBetween(new Vector(boss.x, bossLength / 2, boss.z),
					new Vector(boss.x, -bossLength / 2, boss.z), boss.radius)));
			var withBosses = fuseAll(scope, parts);
			var result = withBosses;
			for (item in holes) {
				var cutter = scope.own(cylinderBetween(new Vector(item.x, bossLength / 2 + 1, item.z),
					new Vector(item.x, -bossLength / 2 - 1, item.z), item.radius));
				var cut = scope.own(result.subtract(cutter));
				result = cut;
			}
			scope.release(result);
			scope.close();
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	static function fuseAll(scope:Scope, parts:Array<Part>):Part {
		if (parts.length == 0) throw "cannot fuse an empty part list";
		var current = scope.own(parts[0]);
		for (index in 1...parts.length) {
			current = scope.own(current.combine(parts[index]));
		}
		return current;
	}

	static function boxAt(width:Float, depth:Float, height:Float, x:Float, y:Float, bottom:Float):Part {
		var box = Part.box(width, depth, height);
		try {
			var moved = box.translated(new Vector(x, y, bottom));
			box.close();
			return moved;
		} catch (error:Dynamic) {
			box.close();
			throw error;
		}
	}

	static function cylinderBetween(start:Vector, end:Vector, radius:Float):Part {
		var axis = end.subtract(start);
		var length = axis.length();
		var direction = axis.normalized();
		var referenceAxis = Math.abs(direction.dot(Vector.Y())) < 0.9 ? Vector.Y() : Vector.X();
		var reference = direction.cross(referenceAxis).normalized();
		var cylinder = Part.cylinder(radius, length);
		try {
			var result = cylinder.placed(new Location(new Plane(start, reference, direction)));
			cylinder.close();
			return result;
		} catch (error:Dynamic) {
			cylinder.close();
			throw error;
		}
	}

	static function pinEye(x:Float, z:Float, outside:Float, inside:Float, width:Float):Part {
		var scope = new Scope();
		try {
			var outer = scope.own(cylinderBetween(new Vector(x, width / 2, z), new Vector(x, -width / 2, z), outside));
			var inner = scope.own(cylinderBetween(new Vector(x, width / 2 + 1, z), new Vector(x, -width / 2 - 1, z), inside));
			var result = scope.own(outer.subtract(inner));
			scope.release(result);
			scope.close();
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	static function hole(x:Float, z:Float, radius:Float):ExcavatorHole
		return {x: x, z: z, radius: radius};

	public static function main():Void {
		var output = Sys.args().length == 0 ? "procedural-excavator" : Sys.args()[0];
		if (!FileSystem.exists(output)) FileSystem.createDirectory(output);
		var components = build();
		var metrics:Array<Dynamic> = [];
		try {
			for (component in components) {
				var shape = component.part.shape;
				var physical = component.part.massProperties();
				var bounds = shape.bounds();
				var minimum = bounds.get_min();
				var maximum = bounds.get_max();
				metrics.push({
					name: component.name,
					solids: component.part.solidCount(),
					faces: shape.subshapeCount(CadKit.ShapeKind.Face),
					edges: shape.subshapeCount(CadKit.ShapeKind.Edge),
					volume: physical.volume,
					surfaceArea: physical.surfaceArea,
					bounds: [minimum.get_x(), minimum.get_y(), minimum.get_z(),
						maximum.get_x(), maximum.get_y(), maximum.get_z()]
				});
				component.part.exportStep(output + "/" + component.name + ".step");
			}
			Sys.println(haxe.Json.stringify({componentCount: components.length, components: metrics}, null, "  "));
		} catch (error:Dynamic) {
			for (component in components) component.close();
			throw error;
		}
		for (component in components) component.close();
	}
}
