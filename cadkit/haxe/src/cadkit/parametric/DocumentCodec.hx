package cadkit.parametric;

import CadKit;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.features.SketchFeature;
import haxe.Json;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FaceFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.ChamferFeature;
import cadkit.parametric.features.RevolveFeature;
import cadkit.parametric.features.TransformFeature;

/** Versioned JSON persistence for the Haxeon parametric document layer. */
class DocumentCodec {
	public static inline var FORMAT:String = "cadkit.document";
	public static inline var VERSION:Int = 1;

	public static function encode(document:Document):String {
		var encodedFeatures:Array<Dynamic> = [];
		for (index in 0...document.featureCount())
			encodedFeatures.push(encodeFeature(document.featureAt(index)));
		return Json.stringify({
			format: FORMAT,
			version: VERSION,
			features: encodedFeatures
		});
	}

	public static function decode(text:String):Document {
		var document:Null<Document> = null;
		try {
			var root:Dynamic = Json.parse(text);
			if (stringField(root, "format") != FORMAT)
				throw new ParametricError("unsupported document format");
			if (intField(root, "version") != VERSION)
				throw new ParametricError("unsupported document version");

			var records:Array<Dynamic> = cast requiredField(root, "features");
			document = new Document();
			var pendingReferences:Array<Dynamic> = [];
			for (record in records) {
				var featureId = intField(record, "id");
				var featureType = stringField(record, "type");
				var feature:Feature;
				if (featureType == "sketch") {
					var planeRecord = requiredField(record, "plane");
					feature = document.add(new SketchFeature(
						stringField(record, "profile"),
						numberField(record, "width"),
						numberField(record, "height"),
						new Plane(
							decodeVector(requiredField(planeRecord, "origin")),
							decodeVector(requiredField(planeRecord, "xDirection")),
							decodeVector(requiredField(planeRecord, "normal")))));
				} else if (featureType == "box") {
					feature = document.add(new BoxFeature(
						numberField(record, "width"),
						numberField(record, "depth"),
						numberField(record, "height")));
				} else if (featureType == "cylinder") {
					feature = document.add(new CylinderFeature(
						numberField(record, "radius"),
						numberField(record, "height")));
				} else if (featureType == "face") {
					feature = document.add(new FaceFeature(
						requiredFeature(document, intField(record, "source")),
						intField(record, "index"),
						optionalFaceFingerprint(record)));
				} else if (featureType == "extrude") {
					feature = document.add(new ExtrudeFeature(
						requiredFeature(document, intField(record, "source")),
						numberField(record, "x"),
						numberField(record, "y"),
						numberField(record, "z")));
				} else if (featureType == "revolve") {
					feature = document.add(new RevolveFeature(
						requiredFeature(document, intField(record, "source")),
						numberField(record, "originX"),
						numberField(record, "originY"),
						numberField(record, "originZ"),
						numberField(record, "axisX"),
						numberField(record, "axisY"),
						numberField(record, "axisZ"),
						numberField(record, "angle")));
				} else if (featureType == "fillet") {
					var filletSource = requiredFeature(document, intField(record, "source"));
					feature = document.add(new FilletFeature(
						filletSource,
						numberField(record, "radius"),
						null,
						optionalEdgeFingerprints(record)));
				} else if (featureType == "chamfer") {
					var chamferSource = requiredFeature(document, intField(record, "source"));
					feature = document.add(new ChamferFeature(
						chamferSource,
						numberField(record, "distance"),
						null,
						optionalEdgeFingerprints(record)));
				} else if (featureType == "transform") {
					feature = document.add(new TransformFeature(
						requiredFeature(document, intField(record, "source")),
						numberField(record, "x"),
						numberField(record, "y"),
						numberField(record, "z")));
				} else if (featureType == "boolean") {
					feature = document.add(new BooleanFeature(
						requiredFeature(document, intField(record, "first")),
						requiredFeature(document, intField(record, "second")),
						booleanOperation(stringField(record, "operation"))));
				} else {
					throw new ParametricError("unsupported feature type: " + featureType);
				}

				if (feature.id.toInt() != featureId)
					throw new ParametricError("feature IDs must be contiguous and ordered");
				var rawReferences:Array<Dynamic> = cast requiredField(record, "references");
				pendingReferences.push({
					feature: feature,
					references: rawReferences
				});
			}

			for (pending in pendingReferences) {
				var pendingFeature:Feature = cast Reflect.field(pending, "feature");
				var references:Array<Dynamic> = cast Reflect.field(pending, "references");
				for (reference in references)
					decodeReference(pendingFeature, reference);
			}

			document.recompute();
			return document;
		} catch (error:Dynamic) {
			if (document != null)
				document.close();
			throw new ParametricError("document decode failed: " + Std.string(error));
		}
	}

	private static function encodeFeature(feature:Feature):Dynamic {
		var featureType = feature.serializationType();
		var excluded:Array<TopologyReference> = [];
		if (featureType == "fillet") {
			var selectedFillet:FilletFeature = cast feature;
			excluded = selectedFillet.edgeReferences;
		} else if (featureType == "chamfer") {
			var selectedChamfer:ChamferFeature = cast feature;
			excluded = selectedChamfer.edgeReferences;
		}

		var references:Array<Dynamic> = [];
		for (index in 0...feature.topologyReferenceCount()) {
			var reference = feature.topologyReferenceAt(index);
			if (reference.state != ReferenceState.Closed && !containsReference(excluded, reference))
				references.push(encodeReference(reference));
		}

		if (featureType == "sketch") {
			var sketch:SketchFeature = cast feature;
			return {
				id: feature.id.toInt(),
				type: featureType,
				profile: sketch.profile,
				width: sketch.width.value,
				height: sketch.height.value,
				plane: {
					origin: encodeVector(sketch.plane.origin),
					xDirection: encodeVector(sketch.plane.xDirection),
					normal: encodeVector(sketch.plane.normal)
				},
				references: references
			};
		} else if (featureType == "box") {
			var box:BoxFeature = cast feature;
			return {
				id: feature.id.toInt(),
				type: featureType,
				width: box.width.value,
				depth: box.depth.value,
				height: box.height.value,
				references: references
			};
		} else if (featureType == "cylinder") {
			var cylinder:CylinderFeature = cast feature;
			return {
				id: feature.id.toInt(),
				type: featureType,
				radius: cylinder.radius.value,
				height: cylinder.height.value,
				references: references
			};
		} else if (featureType == "face") {
			var face:FaceFeature = cast feature;
			return {
				id: feature.id.toInt(),
				type: featureType,
				source: face.source.id.toInt(),
				index: face.index,
				fingerprint: encodeFingerprint(face.fingerprintData()),
				references: references
			};
		} else if (featureType == "extrude") {
			var extrude:ExtrudeFeature = cast feature;
			return {
				id: feature.id.toInt(),
				type: featureType,
				source: extrude.source.id.toInt(),
				x: extrude.x.value,
				y: extrude.y.value,
				z: extrude.z.value,
				references: references
			};
		} else if (featureType == "revolve") {
			var revolve:RevolveFeature = cast feature;
			return {
				id: feature.id.toInt(),
				type: featureType,
				source: revolve.source.id.toInt(),
				originX: revolve.originX.value,
				originY: revolve.originY.value,
				originZ: revolve.originZ.value,
				axisX: revolve.axisX.value,
				axisY: revolve.axisY.value,
				axisZ: revolve.axisZ.value,
				angle: revolve.angle.value,
				references: references
			};
		} else if (featureType == "fillet") {
			var fillet:FilletFeature = cast feature;
			var filletRecord:Dynamic = {
				id: feature.id.toInt(),
				type: featureType,
				source: fillet.source.id.toInt(),
				radius: fillet.radius.value,
				references: references,
				edges: null
			};
			appendEdgeFingerprints(filletRecord, fillet.edgeReferences);
			return filletRecord;
		} else if (featureType == "chamfer") {
			var chamfer:ChamferFeature = cast feature;
			var chamferRecord:Dynamic = {
				id: feature.id.toInt(),
				type: featureType,
				source: chamfer.source.id.toInt(),
				distance: chamfer.distance.value,
				references: references,
				edges: null
			};
			appendEdgeFingerprints(chamferRecord, chamfer.edgeReferences);
			return chamferRecord;
		} else if (featureType == "transform") {
			var transform:TransformFeature = cast feature;
			return {
				id: feature.id.toInt(),
				type: featureType,
				source: transform.source.id.toInt(),
				x: transform.x.value,
				y: transform.y.value,
				z: transform.z.value,
				references: references
			};
		} else if (featureType == "boolean") {
			var booleanFeature:BooleanFeature = cast feature;
			return {
				id: feature.id.toInt(),
				type: featureType,
				first: booleanFeature.first.id.toInt(),
				second: booleanFeature.second.id.toInt(),
				operation: encodeBooleanOperation(booleanFeature.operation),
				references: references
			};
		}
		throw new ParametricError("unsupported feature type: " + featureType);
	}

	private static function encodeReference(reference:TopologyReference):Dynamic {
		return {
			kind: shapeKindName(reference.kind),
			fingerprint: encodeFingerprint(reference.fingerprintData())
		};
	}

	private static function containsReference(
		references:Array<TopologyReference>,
		candidate:TopologyReference):Bool {
		for (reference in references)
			if (reference == candidate)
				return true;
		return false;
	}

	private static function appendEdgeFingerprints(
		record:Dynamic,
		references:Array<TopologyReference>):Void {
		if (references.length == 0)
			return;
		var encoded:Array<Dynamic> = [];
		for (reference in references)
			encoded.push(encodeFingerprint(reference.fingerprintData()));
		Reflect.setField(record, "edges", encoded);
	}

	private static function decodeReference(feature:Feature, record:Dynamic):Void {
		var fingerprintRecord:Dynamic = requiredField(record, "fingerprint");
		var kind = shapeKind(stringField(record, "kind"));
		var fingerprint = decodeFingerprint(fingerprintRecord, kind);
		TopologyReference.fromFingerprint(feature, kind, fingerprint);
	}

	private static function optionalFaceFingerprint(record:Dynamic):Null<TopologyFingerprint> {
		var value:Dynamic = Reflect.field(record, "fingerprint");
		if (value == null)
			return null;
		return decodeFingerprint(value, CadKit.ShapeKind.Face);
	}

	private static function optionalEdgeFingerprints(
		record:Dynamic):Null<Array<TopologyFingerprint>> {
		var value:Dynamic = Reflect.field(record, "edges");
		if (value == null)
			return null;
		var records:Array<Dynamic> = cast value;
		if (records.length == 0)
			throw new ParametricError("document edge selection must not be empty");
		var result:Array<TopologyFingerprint> = [];
		for (fingerprint in records)
			result.push(decodeFingerprint(fingerprint, CadKit.ShapeKind.Edge));
		return result;
	}

	private static function encodeFingerprint(
		fingerprint:Null<TopologyFingerprint>):Dynamic {
		if (fingerprint == null)
			return null;
		return {
			surface: surfaceKindName(fingerprint.surfaceKind),
			curve: curveKindName(fingerprint.curveKind),
			x: fingerprint.x,
			y: fingerprint.y,
			z: fingerprint.z,
			dx: fingerprint.dx,
			dy: fingerprint.dy,
			dz: fingerprint.dz,
			measure: fingerprint.measure
		};
	}

	private static function decodeFingerprint(
		record:Dynamic,
		kind:CadKit.ShapeKind):TopologyFingerprint {
		return TopologyFingerprint.fromData(
			kind,
			surfaceKind(stringField(record, "surface")),
			curveKind(stringField(record, "curve")),
			numberField(record, "x"),
			numberField(record, "y"),
			numberField(record, "z"),
			numberField(record, "dx"),
			numberField(record, "dy"),
			numberField(record, "dz"),
			numberField(record, "measure"));
	}

	private static function encodeVector(value:Vector):Dynamic {
		return {x: value.x, y: value.y, z: value.z};
	}

	private static function decodeVector(value:Dynamic):Vector {
		return new Vector(numberField(value, "x"), numberField(value, "y"), numberField(value, "z"));
	}

	private static function requiredFeature(document:Document, featureId:Int):Feature {
		var feature = document.featureById(featureId);
		if (feature == null)
			throw new ParametricError("feature dependency is missing: " + featureId);
		return feature;
	}

	private static function requiredField(value:Dynamic, name:String):Dynamic {
		var result = Reflect.field(value, name);
		if (result == null)
			throw new ParametricError("document field is missing: " + name);
		return result;
	}

	private static function stringField(value:Dynamic, name:String):String {
		var result:Dynamic = requiredField(value, name);
		if (!Std.isOfType(result, String))
			throw new ParametricError("document field is not a string: " + name);
		return cast result;
	}

	private static function numberField(value:Dynamic, name:String):Float {
		var result:Float = cast requiredField(value, name);
		if (!Math.isFinite(result))
			throw new ParametricError("document field is not finite: " + name);
		return result;
	}

	private static function intField(value:Dynamic, name:String):Int {
		var result = numberField(value, name);
		var integer = Std.int(result);
		if (result != integer)
			throw new ParametricError("document field is not an integer: " + name);
		return integer;
	}

	private static function encodeBooleanOperation(operation:BooleanOperation):String {
		if (operation == BooleanOperation.Fuse)
			return "fuse";
		if (operation == BooleanOperation.Cut)
			return "cut";
		if (operation == BooleanOperation.Common)
			return "common";
		throw new ParametricError("unsupported boolean operation");
	}

	private static function booleanOperation(name:String):BooleanOperation {
		if (name == "fuse")
			return BooleanOperation.Fuse;
		if (name == "cut")
			return BooleanOperation.Cut;
		if (name == "common")
			return BooleanOperation.Common;
		throw new ParametricError("unsupported boolean operation: " + name);
	}

	private static function shapeKindName(kind:CadKit.ShapeKind):String {
		if (kind == CadKit.ShapeKind.Face)
			return "face";
		if (kind == CadKit.ShapeKind.Edge)
			return "edge";
		if (kind == CadKit.ShapeKind.Vertex)
			return "vertex";
		throw new ParametricError("unsupported topology reference kind");
	}

	private static function shapeKind(name:String):CadKit.ShapeKind {
		if (name == "face")
			return CadKit.ShapeKind.Face;
		if (name == "edge")
			return CadKit.ShapeKind.Edge;
		if (name == "vertex")
			return CadKit.ShapeKind.Vertex;
		throw new ParametricError("unsupported topology reference kind: " + name);
	}

	private static function surfaceKindName(kind:CadKit.SurfaceKind):String {
		if (kind == CadKit.SurfaceKind.Unknown)
			return "unknown";
		if (kind == CadKit.SurfaceKind.Plane)
			return "plane";
		if (kind == CadKit.SurfaceKind.Cylinder)
			return "cylinder";
		if (kind == CadKit.SurfaceKind.Cone)
			return "cone";
		if (kind == CadKit.SurfaceKind.Sphere)
			return "sphere";
		if (kind == CadKit.SurfaceKind.Torus)
			return "torus";
		if (kind == CadKit.SurfaceKind.Bezier)
			return "bezier";
		if (kind == CadKit.SurfaceKind.BSpline)
			return "bspline";
		throw new ParametricError("unsupported surface kind");
	}

	private static function surfaceKind(name:String):CadKit.SurfaceKind {
		if (name == "unknown")
			return CadKit.SurfaceKind.Unknown;
		if (name == "plane")
			return CadKit.SurfaceKind.Plane;
		if (name == "cylinder")
			return CadKit.SurfaceKind.Cylinder;
		if (name == "cone")
			return CadKit.SurfaceKind.Cone;
		if (name == "sphere")
			return CadKit.SurfaceKind.Sphere;
		if (name == "torus")
			return CadKit.SurfaceKind.Torus;
		if (name == "bezier")
			return CadKit.SurfaceKind.Bezier;
		if (name == "bspline")
			return CadKit.SurfaceKind.BSpline;
		throw new ParametricError("unsupported surface kind: " + name);
	}

	private static function curveKindName(kind:CadKit.CurveKind):String {
		if (kind == CadKit.CurveKind.Unknown)
			return "unknown";
		if (kind == CadKit.CurveKind.Line)
			return "line";
		if (kind == CadKit.CurveKind.Circle)
			return "circle";
		if (kind == CadKit.CurveKind.Ellipse)
			return "ellipse";
		if (kind == CadKit.CurveKind.Hyperbola)
			return "hyperbola";
		if (kind == CadKit.CurveKind.Parabola)
			return "parabola";
		if (kind == CadKit.CurveKind.Bezier)
			return "bezier";
		if (kind == CadKit.CurveKind.BSpline)
			return "bspline";
		if (kind == CadKit.CurveKind.Offset)
			return "offset";
		throw new ParametricError("unsupported curve kind");
	}

	private static function curveKind(name:String):CadKit.CurveKind {
		if (name == "unknown")
			return CadKit.CurveKind.Unknown;
		if (name == "line")
			return CadKit.CurveKind.Line;
		if (name == "circle")
			return CadKit.CurveKind.Circle;
		if (name == "ellipse")
			return CadKit.CurveKind.Ellipse;
		if (name == "hyperbola")
			return CadKit.CurveKind.Hyperbola;
		if (name == "parabola")
			return CadKit.CurveKind.Parabola;
		if (name == "bezier")
			return CadKit.CurveKind.Bezier;
		if (name == "bspline")
			return CadKit.CurveKind.BSpline;
		if (name == "offset")
			return CadKit.CurveKind.Offset;
		throw new ParametricError("unsupported curve kind: " + name);
	}
}
