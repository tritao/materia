import cadkit.modeling.AssemblyModel;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord;

/** A posed excavator mechanism. Coordinates are millimetres in each part's CAD frame. */
class ProceduralExcavatorAssembly {
	static inline var DEG:Float = Math.PI / 180;

	public static function build():AssemblyRecord {
		var model = new AssemblyModel();
		for (name in ["Base", "BasePin", "Boom", "Stick", "Bucket", "BucketLink1", "BucketLink2",
			"BoomCylinderOuter", "BoomCylinderInner", "StickCylinderOuter", "StickCylinderInner",
			"BucketCylinderOuter", "BucketCylinderInner"])
			model.add(id(name));

		point(model, "Base", "boom", -1, 85);
		point(model, "Base", "pin", -1, 85);
		point(model, "Base", "boomCylinder", -14, 53);
		point(model, "BasePin", "center", -1, 85);
		point(model, "Boom", "base", -1, 85);
		point(model, "Boom", "tip", 225.84, 228.85);
		point(model, "Boom", "boomCylinder", 114.56, 149.39);
		point(model, "Boom", "stickCylinder", 187.32, 202.82);
		point(model, "Stick", "base", 49, 184);
		point(model, "Stick", "tip", 373.9, 99.4);
		point(model, "Stick", "stickCylinder", 172.12, 145.93);
		point(model, "Stick", "bucketCylinder", 290, 116);
		point(model, "Stick", "link", 320, 108);
		point(model, "Bucket", "base", 137, 119);
		point(model, "Bucket", "link", 186, 78.12);
		point(model, "Bucket", "bucketCylinder", 186, 78.12);

		model.mate("base-pin", "fixed", id("Base"), "pin", id("BasePin"), "center");
		model.mate("boom-hinge", "revolute", id("Base"), "boom", id("Boom"), "base", -20 * DEG);
		model.mate("stick-hinge", "revolute", id("Boom"), "tip", id("Stick"), "base", 0);
		model.mate("bucket-hinge", "revolute", id("Stick"), "tip", id("Bucket"), "base", 35 * DEG);

		// Two links close the bucket linkage between its bored pin and a stick datum.
		var link1Root = {x: 119.78, z: 110.05}, link1Tip = {x: 139.22, z: 152.95};
		var link2Root = {x: 114.11, z: 104.205}, link2Tip = {x: 144.89, z: 158.795};
		point(model, "BucketLink1", "root", link1Root.x, link1Root.z);
		point(model, "BucketLink1", "tip", link1Tip.x, link1Tip.z);
		point(model, "BucketLink2", "root", link2Root.x, link2Root.z);
		point(model, "BucketLink2", "tip", link2Tip.x, link2Tip.z);
		var start = model.worldPoint(id("Stick"), "link");
		var end = model.worldPoint(id("Bucket"), "link");
		var length1 = distance(link1Root.x, link1Root.z, link1Tip.x, link1Tip.z);
		var length2 = distance(link2Root.x, link2Root.z, link2Tip.x, link2Tip.z);
		var middle = circleIntersection(start.x, start.z, end.x, end.z, length1, length2);
		mateAimed(model, "link-one-hinge", id("Stick"), "link", id("BucketLink1"),
			"root", link1Root.x, link1Root.z, link1Tip.x, link1Tip.z, middle.x, middle.z);
		mateAimed(model, "link-two-hinge", id("BucketLink1"), "tip", id("BucketLink2"),
			"root", link2Root.x, link2Root.z, link2Tip.x, link2Tip.z, end.x, end.z);
		model.constrain("bucket-link-closure", "revolute", id("Bucket"), "link",
			id("BucketLink2"), "tip");

		cylinder(model, "Boom", "Base", "boomCylinder", "Boom", "boomCylinder",
			"BoomCylinderOuter", "BoomCylinderInner", -14, 53, 39.75, 214.25,
			11, 128, 77, 295);
		cylinder(model, "Stick", "Boom", "stickCylinder", "Stick", "stickCylinder",
			"StickCylinderOuter", "StickCylinderInner", 39, 174, 209, 134,
			84, 132, 257, 92);
		cylinder(model, "Bucket", "Stick", "bucketCylinder", "Bucket", "bucketCylinder",
			"BucketCylinderOuter", "BucketCylinderInner", 87, 150, 245, 83,
			128, 119, 291, 52);

		return model.record();
	}

	static function point(model:AssemblyModel, part:String, name:String, x:Float, z:Float):Void
		model.connector(id(part), name, AssemblyFrames.translation(x, 0, z));

	static function cylinder(model:AssemblyModel, label:String, support:String, supportPin:String,
			target:String, targetPin:String, outer:String, inner:String,
			outerX:Float, outerZ:Float, outerEndX:Float, outerEndZ:Float,
			innerX:Float, innerZ:Float, innerEndX:Float, innerEndZ:Float):Void {
		var outerId = id(outer), innerId = id(inner);
		point(model, outer, "hinge", outerX, outerZ);
		var outerSlide = planarSlide(outerX, outerZ, outerEndX - outerX, outerEndZ - outerZ);
		model.connector(outerId, "slide", outerSlide);
		model.connector(innerId, "slide", planarSlide(innerEndX, innerEndZ,
			innerEndX - innerX, innerEndZ - innerZ));
		point(model, inner, "eye", innerEndX, innerEndZ);
		var end = model.worldPoint(id(target), targetPin);
		mateAimed(model, label + "-cylinder-hinge", id(support), supportPin, outerId, "hinge",
			outerX, outerZ, outerEndX, outerEndZ, end.x, end.z);
		var slideWorld = AssemblyFrames.compose(model.pose(outerId), outerSlide);
		var slideAxis = AssemblyFrames.transformPoint(slideWorld, 0, 1, 0);
		var travel = (end.x - slideWorld.x) * (slideAxis.x - slideWorld.x) +
			(end.z - slideWorld.z) * (slideAxis.z - slideWorld.z);
		model.mate(label + "-cylinder-slide", "prismatic", outerId, "slide", innerId,
			"slide", travel);
		model.constrain(label + "-cylinder-rod-pin", "revolute", id(target), targetPin,
			innerId, "eye");
	}

	/** Shared slide frame twist keeps both eye axes parallel to the pin axes. */
	static function planarSlide(x:Float, z:Float, dx:Float, dz:Float):materia.project.AssemblyRecord.AssemblyFrame {
		if (distance(0, 0, dx, dz) < 1e-9) throw "Cylinder slide has zero length";
		var base = AssemblyFrames.alongY(0, 0, 0, 1, 0, 0);
		var rotation = AssemblyFrames.compose(AssemblyFrames.turnY(-Math.atan2(dz, dx)), base);
		return AssemblyFrames.compose(AssemblyFrames.translation(x, 0, z), rotation);
	}

	static function mateAimed(model:AssemblyModel, joint:String, parent:String,
			parentConnector:String, child:String, childConnector:String,
			localX:Float, localZ:Float, localEndX:Float, localEndZ:Float,
			targetX:Float, targetZ:Float):Void {
		var start = model.worldPoint(parent, parentConnector);
		var localAngle = Math.atan2(localEndZ - localZ, localEndX - localX);
		var worldAngle = Math.atan2(targetZ - start.z, targetX - start.x);
		var parentPose = model.pose(parent);
		var parentYaw = 2 * Math.atan2(parentPose.qy, parentPose.qw);
		model.mate(joint, "revolute", parent, parentConnector, child, childConnector,
			localAngle - worldAngle - parentYaw);
	}

	static function circleIntersection(ax:Float, az:Float, bx:Float, bz:Float,
			firstRadius:Float, secondRadius:Float):{x:Float, z:Float} {
		var dx = bx - ax, dz = bz - az, length = Math.sqrt(dx * dx + dz * dz);
		if (length < 1e-9 || length > firstRadius + secondRadius ||
			length < Math.abs(firstRadius - secondRadius))
			throw "Bucket linkage cannot reach its two pins";
		var ux = dx / length, uz = dz / length;
		var along = (firstRadius * firstRadius - secondRadius * secondRadius + length * length) / (2 * length);
		var height = Math.sqrt(Math.max(0, firstRadius * firstRadius - along * along));
		var one = {x: ax + along * ux - height * uz, z: az + along * uz + height * ux};
		var two = {x: ax + along * ux + height * uz, z: az + along * uz - height * ux};
		return one.z > two.z ? one : two;
	}

	static function distance(ax:Float, az:Float, bx:Float, bz:Float):Float {
		var dx = bx - ax, dz = bz - az;
		return Math.sqrt(dx * dx + dz * dz);
	}

	static function id(name:String):String return "excavator/" + name;
}
