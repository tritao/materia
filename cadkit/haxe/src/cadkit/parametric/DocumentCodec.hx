package cadkit.parametric;

import CadKit;
import cadkit.parametric.features.WireFeature;
import cadkit.parametric.features.PolylineFeature;
import cadkit.parametric.features.LoftFeature;
import cadkit.parametric.features.SweepFeature;
import cadkit.parametric.features.OffsetFeature;
import cadkit.parametric.features.ShellFeature;
import cadkit.parametric.features.ProjectFeature;
import cadkit.parametric.features.GridFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.features.PolarPatternFeature;
import cadkit.parametric.features.HoleFeature;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SolverSettings;
import haxe.Json;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.PocketFeature;
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
		var encodedParameters:Array<Dynamic> = [];
		for (parameter in document.namedParameters()) {
			var bindings:Array<Dynamic> = [];
			for (binding in parameter.bindings())
				bindings.push({
					feature: binding.ownerFeature().id.toInt(),
					parameter: binding.name
				});
			if (bindings.length == 0)
				throw new ParametricError("named parameter has no bindings: " + parameter.name);
			encodedParameters.push({
				name: parameter.name,
				value: parameter.value,
				bindings: bindings
			});
		}
		return Json.stringify({
			format: FORMAT,
			version: VERSION,
			features: encodedFeatures,
			parameters: encodedParameters,
			output: document.outputFeature().id.toInt()
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
				var modeling = decodeModelingFeature(featureType, record, document);
				if (modeling != null) {
					feature = document.add(modeling);
				} else if (featureType == "constrained-sketch") {
					feature = document.add(decodeConstrainedSketch(record, document));
				} else if (featureType == "sketch") {
					var planeRecord = requiredField(record, "plane");
					feature = document.add(new SketchFeature(stringField(record, "profile"), numberField(record, "width"), numberField(record, "height"),
						new Plane(decodeVector(requiredField(planeRecord, "origin")), decodeVector(requiredField(planeRecord, "xDirection")),
							decodeVector(requiredField(planeRecord, "normal")))));
				} else if (featureType == "box") {
					feature = document.add(new BoxFeature(numberField(record, "width"), numberField(record, "depth"), numberField(record, "height")));
				} else if (featureType == "cylinder") {
					feature = document.add(new CylinderFeature(numberField(record, "radius"), numberField(record, "height")));
				} else if (featureType == "face") {
					feature = document.add(new FaceFeature(requiredFeature(document, intField(record, "source")), intField(record, "index"),
						optionalFaceFingerprint(record)));
				} else if (featureType == "extrude") {
					var extrudeSource = requiredFeature(document, intField(record, "source"));
					var encodedAmount:Dynamic = Reflect.field(record, "amount");
					feature = encodedAmount == null
						? document.add(new ExtrudeFeature(extrudeSource, numberField(record, "x"), numberField(record, "y"), numberField(record, "z")))
						: document.add(ExtrudeFeature.along(extrudeSource, finiteNumber(encodedAmount, "amount"),
							new Vector(numberField(record, "x"), numberField(record, "y"), numberField(record, "z")),
							optionalBool(record, "reversed", false), optionalBool(record, "symmetric", false)));
				} else if (featureType == "pocket") {
					feature = document.add(new PocketFeature(requiredFeature(document, intField(record, "target")),
						requiredFeature(document, intField(record, "profile")), stringField(record, "mode"), numberField(record, "depth")));
				} else if (featureType == "revolve") {
					feature = document.add(new RevolveFeature(requiredFeature(document, intField(record, "source")), numberField(record, "originX"),
						numberField(record, "originY"), numberField(record, "originZ"), numberField(record, "axisX"), numberField(record, "axisY"),
						numberField(record, "axisZ"), numberField(record, "angle")));
				} else if (featureType == "fillet") {
					var filletSource = requiredFeature(document, intField(record, "source"));
					feature = document.add(new FilletFeature(filletSource, numberField(record, "radius"), null, optionalEdgeFingerprints(record),
						optionalSelection(record)));
				} else if (featureType == "chamfer") {
					var chamferSource = requiredFeature(document, intField(record, "source"));
					feature = document.add(new ChamferFeature(chamferSource, numberField(record, "distance"), null, optionalEdgeFingerprints(record),
						optionalSelection(record)));
				} else if (featureType == "transform") {
					feature = document.add(new TransformFeature(requiredFeature(document, intField(record, "source")), numberField(record, "x"),
						numberField(record, "y"), numberField(record, "z")));
				} else if (featureType == "boolean") {
					feature = document.add(new BooleanFeature(requiredFeature(document, intField(record, "first")),
						requiredFeature(document, intField(record, "second")), booleanOperation(stringField(record, "operation"))));
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

			var parameterRecords:Dynamic = Reflect.field(root, "parameters");
			if (parameterRecords != null) {
				var records:Array<Dynamic> = cast parameterRecords;
				for (record in records) {
					var named = document.defineParameter(stringField(record, "name"), numberField(record, "value"));
					var bindings:Array<Dynamic> = cast requiredField(record, "bindings");
					if (bindings.length == 0)
						throw new ParametricError("named parameter has no bindings: " + named.name);
					for (binding in bindings) {
						var feature = requiredFeature(document, intField(binding, "feature"));
						named.bind(feature.parameter(stringField(binding, "parameter")));
					}
				}
			}
			var output:Dynamic = Reflect.field(root, "output");
			if (output != null)
				document.setOutput(requiredFeature(document, integerValue(output, "output")));

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

		var modeling = encodeModelingFeature(feature, references);
		if (modeling != null)
			return modeling;
		if (featureType == "constrained-sketch") {
			return encodeConstrainedSketch(cast feature, references);
		} else if (featureType == "sketch") {
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
				amount: extrude.amount == null ? null : extrude.amount.value,
				reversed: extrude.reversed,
				symmetric: extrude.symmetric,
				references: references
			};
		} else if (featureType == "pocket") {
			var pocket:PocketFeature = cast feature;
			return {
				id: feature.id.toInt(),
				type: featureType,
				target: pocket.target.id.toInt(),
				profile: pocket.profile.id.toInt(),
				mode: pocket.mode,
				depth: pocket.depth.value,
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
				selection: encodeSelection(fillet.selection),
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
				selection: encodeSelection(chamfer.selection),
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

	private static function encodeConstrainedSketch(feature:ConstrainedSketchFeature, references:Array<Dynamic>):Dynamic {
		var sketch = feature.sketch();
		var points:Array<Dynamic> = [];
		var entities:Array<Dynamic> = [];
		var constraints:Array<Dynamic> = [];
		for (point in sketch.points())
			points.push({id: point.id, x: point.x, y: point.y});
		for (entity in sketch.entities())
			entities.push({
				id: entity.id,
				kind: entity.kind,
				first: entity.first,
				second: entity.second,
				radius: entity.radius,
				startAngle: entity.startAngle,
				endAngle: entity.endAngle,
				clockwise: entity.clockwise,
				construction: entity.construction
			});
		for (constraint in sketch.constraints()) {
			var value = constraint.value;
			if (constraint.kind == "distance" || constraint.kind == "radius" || constraint.kind == "angle")
				value = feature.dimension(constraint.id).value;
			constraints.push({
				id: constraint.id,
				kind: constraint.kind,
				first: constraint.first,
				second: constraint.second,
				third: constraint.third,
				value: value
			});
		}
		return {
			id: feature.id.toInt(),
			type: "constrained-sketch",
			references: references,
			units: sketch.units,
			plane: {
				origin: encodeVector(sketch.plane.origin),
				xDirection: encodeVector(sketch.plane.xDirection),
				normal: encodeVector(sketch.plane.normal)
			},
			settings: {
				tolerance: sketch.settings.tolerance,
				rankTolerance: sketch.settings.rankTolerance,
				maxIterations: sketch.settings.maxIterations,
				initialDamping: sketch.settings.initialDamping
			},
			points: points,
			entities: entities,
			constraints: constraints,
			support: feature.support == null ? null : feature.support.id.toInt(),
			supportSelection: encodeSelection(feature.supportSelection),
			supportXDirection: feature.supportXDirection == null ? null : encodeVector(feature.supportXDirection),
			supportOffset: feature.supportOffset,
			supportFlipped: feature.supportFlipped
		};
	}

	private static function decodeConstrainedSketch(record:Dynamic, document:Document):ConstrainedSketchFeature {
		var plane = requiredField(record, "plane");
		var settings = requiredField(record, "settings");
		var sketch = new ConstrainedSketch(new Plane(decodeVector(requiredField(plane, "origin")),
			decodeVector(requiredField(plane, "xDirection")), decodeVector(requiredField(plane, "normal"))),
			stringField(record, "units"), new SolverSettings(numberField(settings, "tolerance"),
				numberField(settings, "rankTolerance"), intField(settings, "maxIterations"), numberField(settings, "initialDamping")));
		var pointRecords:Array<Dynamic> = cast requiredField(record, "points");
		for (value in pointRecords)
			sketch.addPoint(new SketchPoint(stringField(value, "id"), numberField(value, "x"), numberField(value, "y")));
		var entityRecords:Array<Dynamic> = cast requiredField(record, "entities");
		for (value in entityRecords) {
			var entity = switch (stringField(value, "kind")) {
				case "line": SketchEntity.line(stringField(value, "id"), stringField(value, "first"), stringField(value, "second"),
					boolField(value, "construction"));
				case "circle": SketchEntity.circle(stringField(value, "id"), stringField(value, "first"), numberField(value, "radius"),
					boolField(value, "construction"));
				case "arc": SketchEntity.arc(stringField(value, "id"), stringField(value, "first"), numberField(value, "radius"),
					numberField(value, "startAngle"), numberField(value, "endAngle"), boolField(value, "clockwise"),
					boolField(value, "construction"));
				default: throw new ParametricError("unsupported constrained sketch entity");
			};
			sketch.addEntity(entity);
		}
		var constraintRecords:Array<Dynamic> = cast requiredField(record, "constraints");
		for (value in constraintRecords)
			sketch.addConstraint(SketchConstraint.raw(stringField(value, "id"), stringField(value, "kind"),
				stringField(value, "first"), optionalString(value, "second"), optionalString(value, "third"), numberField(value, "value")));
		var supportValue:Dynamic = Reflect.field(record, "support");
		if (supportValue == null)
			return new ConstrainedSketchFeature(sketch);
		return new ConstrainedSketchFeature(sketch, requiredFeature(document, integerValue(cast supportValue, "support")),
			decodeSelection(requiredField(record, "supportSelection")), decodeVector(requiredField(record, "supportXDirection")),
			numberField(record, "supportOffset"), boolField(record, "supportFlipped"));
	}

	private static function optionalString(record:Dynamic,name:String):Null<String> { var value:Dynamic=Reflect.field(record,name);if(value==null)return null;if(!Std.isOfType(value,String))throw new ParametricError("document field is not a string: "+name);return cast value; }

	private static function decodeModelingFeature(type:String, record:Dynamic, document:Document):Null<Feature> {
		if (type == "wire")
			return new WireFeature(requiredFeature(document, intField(record, "source")));
		if (type == "polyline") {
			var raw:Array<Dynamic> = cast requiredField(record, "points");
			var points:Array<Vector> = [];
			for (point in raw)
				points.push(decodeVector(point));
			return new PolylineFeature(points, boolField(record, "closed"));
		}
		if (type == "loft") {
			var raw:Array<Dynamic> = cast requiredField(record, "sections");
			var sections:Array<Feature> = [];
			for (item in raw)
				sections.push(requiredFeature(document, intField(item, "id")));
			return new LoftFeature(sections, boolField(record, "ruled"));
		}
		if (type == "sweep")
			return new SweepFeature(requiredFeature(document, intField(record, "profile")), requiredFeature(document, intField(record, "path")));
		if (type == "offset")
			return new OffsetFeature(requiredFeature(document, intField(record, "source")), numberField(record, "distance"));
		if (type == "shell")
			return new ShellFeature(requiredFeature(document, intField(record, "source")), numberField(record, "thickness"),
				decodeSelection(requiredField(record, "selection")));
		if (type == "project")
			return new ProjectFeature(requiredFeature(document, intField(record, "source")), requiredFeature(document, intField(record, "target")),
				decodeVector(requiredField(record, "direction")));
		if (type == "grid")
			return new GridFeature(requiredFeature(document, intField(record, "source")), intField(record, "columns"), intField(record, "rows"),
				numberField(record, "spacingX"), numberField(record, "spacingY"));
		if (type == "linear-pattern") {
			var rawSecondDirection:Dynamic = Reflect.field(record, "secondDirection");
			return new LinearPatternFeature(requiredFeature(document, intField(record, "source")), numberField(record, "count"),
				numberField(record, "spacing"), decodeVector(requiredField(record, "direction")), numberField(record, "secondCount"),
				numberField(record, "secondSpacing"), rawSecondDirection == null ? null : decodeVector(rawSecondDirection));
		}
		if (type == "polar-pattern")
			return new PolarPatternFeature(requiredFeature(document, intField(record, "source")), numberField(record, "count"),
				numberField(record, "radius"), numberField(record, "angularSpan"), decodeVector(requiredField(record, "axisOrigin")),
				decodeVector(requiredField(record, "axisDirection")), decodeVector(requiredField(record, "radialDirection")),
				boolField(record, "orientInstances"), numberField(record, "startAngle"));
		if (type == "hole") {
			var rawIncludedAngle:Dynamic = Reflect.field(record, "includedAngle");
			return new HoleFeature(requiredFeature(document, intField(record, "target")),
				decodeSelection(requiredField(record, "selection")), decodeVector(requiredField(record, "xDirection")),
				stringField(record, "style"), stringField(record, "mode"), numberField(record, "x"), numberField(record, "y"),
				numberField(record, "diameter"), numberField(record, "depth"), numberField(record, "recessDiameter"),
				numberField(record, "recessDepth"), numberField(record, "offset"), boolField(record, "flipped"),
				rawIncludedAngle == null ? Math.PI / 2 : finiteNumber(rawIncludedAngle, "includedAngle"));
		}
		return null;
	}

	private static function encodeModelingFeature(feature:Feature, references:Array<Dynamic>):Dynamic {
		var type = feature.serializationType();
		if (type == "wire") {
			var value:WireFeature = cast feature;

			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				source: value.source.id.toInt()
			};
		}
		if (type == "polyline") {
			var value:PolylineFeature = cast feature;
			var points:Array<Dynamic> = [];
			for (point in value.points())
				points.push(encodeVector(point));
			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				points: points,
				closed: value.closed
			};
		}
		if (type == "loft") {
			var value:LoftFeature = cast feature;
			var sections:Array<Dynamic> = [];
			for (section in value.dependencyFeatures())
				sections.push({id: section.id.toInt()});
			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				sections: sections,
				ruled: value.ruled
			};
		}
		if (type == "sweep") {
			var value:SweepFeature = cast feature;

			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				profile: value.profile.id.toInt(),
				path: value.path.id.toInt()
			};
		}
		if (type == "offset") {
			var value:OffsetFeature = cast feature;

			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				source: value.source.id.toInt(),
				distance: value.distance.value
			};
		}
		if (type == "shell") {
			var value:ShellFeature = cast feature;

			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				source: value.source.id.toInt(),
				thickness: value.thickness.value,
				selection: encodeSelection(value.selection)
			};
		}
		if (type == "project") {
			var value:ProjectFeature = cast feature;

			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				source: value.source.id.toInt(),
				target: value.target.id.toInt(),
				direction: encodeVector(new Vector(value.x.value, value.y.value, value.z.value))
			};
		}
		if (type == "grid") {
			var value:GridFeature = cast feature;

			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				source: value.source.id.toInt(),
				columns: value.columns,
				rows: value.rows,
				spacingX: value.spacingX.value,
				spacingY: value.spacingY.value
			};
		}
		if (type == "linear-pattern") {
			var value:LinearPatternFeature = cast feature;

			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				source: value.source.id.toInt(),
				count: value.count.value,
				spacing: value.spacing.value,
				direction: encodeVector(value.direction),
				secondCount: value.secondCount.value,
				secondSpacing: value.secondSpacing.value,
				secondDirection: value.secondDirection == null ? null : encodeVector(value.secondDirection)
			};
		}
		if (type == "polar-pattern") {
			var value:PolarPatternFeature = cast feature;

			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				source: value.source.id.toInt(),
				count: value.count.value,
				radius: value.radius.value,
				startAngle: value.startAngle.value,
				angularSpan: value.angularSpan.value,
				axisOrigin: encodeVector(value.axisOrigin),
				axisDirection: encodeVector(value.axisDirection),
				radialDirection: encodeVector(value.radialDirection),
				orientInstances: value.orientInstances
			};
		}
		if (type == "hole") {
			var value:HoleFeature = cast feature;

			return {
				id: feature.id.toInt(),
				type: type,
				references: references,
				target: value.target.id.toInt(),
				selection: encodeSelection(value.selection),
				xDirection: encodeVector(value.xDirection),
				offset: value.offset,
				flipped: value.flipped,
				style: value.style,
				mode: value.mode,
				x: value.x.value,
				y: value.y.value,
				diameter: value.diameter.value,
				depth: value.depth.value,
				recessDiameter: value.recessDiameter.value,
				recessDepth: value.recessDepth.value,
				includedAngle: value.includedAngle.value
			};
		}
		return null;
	}

	private static function encodeSelection(selection:Null<SelectionRecipe>):Dynamic {
		if (selection == null)
			return null;
		return {
			kind: selection.kind,
			geometry: selection.geometry,
			parallel: selection.parallel == null ? null : encodeVector(selection.parallel),
			position: selection.position,
			axis: encodeVector(selection.axis),
			expectedCount: selection.expectedCount,
			tolerance: selection.tolerance
		};
	}

	private static function optionalSelection(record:Dynamic):Null<SelectionRecipe> {
		var value:Dynamic = Reflect.field(record, "selection");
		return value == null ? null : decodeSelection(value);
	}

	private static function decodeSelection(record:Dynamic):SelectionRecipe {
		var parallel:Dynamic = Reflect.field(record, "parallel");
		return new SelectionRecipe(stringField(record, "kind"), stringField(record, "geometry"), parallel == null ? null : decodeVector(parallel),
			stringField(record, "position"), decodeVector(requiredField(record, "axis")), intField(record, "expectedCount"), numberField(record, "tolerance"));
	}

	private static function boolField(record:Dynamic, name:String):Bool {
		var value:Dynamic = requiredField(record, name);
		if (!Std.isOfType(value, Bool))
			throw new ParametricError("document field is not a boolean: " + name);
		return cast value;
	}

	private static function optionalBool(record:Dynamic, name:String, fallback:Bool):Bool {
		var value:Dynamic = Reflect.field(record, name);
		if (value == null)
			return fallback;
		if (!Std.isOfType(value, Bool))
			throw new ParametricError("document field is not a boolean: " + name);
		return cast value;
	}

	private static function encodeReference(reference:TopologyReference):Dynamic {
		return {
			kind: shapeKindName(reference.kind),
			fingerprint: encodeFingerprint(reference.fingerprintData())
		};
	}

	private static function containsReference(references:Array<TopologyReference>, candidate:TopologyReference):Bool {
		for (reference in references)
			if (reference == candidate)
				return true;
		return false;
	}

	private static function appendEdgeFingerprints(record:Dynamic, references:Array<TopologyReference>):Void {
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

	private static function optionalEdgeFingerprints(record:Dynamic):Null<Array<TopologyFingerprint>> {
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

	private static function encodeFingerprint(fingerprint:Null<TopologyFingerprint>):Dynamic {
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

	private static function decodeFingerprint(record:Dynamic, kind:CadKit.ShapeKind):TopologyFingerprint {
		return TopologyFingerprint.fromData(kind, surfaceKind(stringField(record, "surface")), curveKind(stringField(record, "curve")),
			numberField(record, "x"), numberField(record, "y"), numberField(record, "z"), numberField(record, "dx"), numberField(record, "dy"),
			numberField(record, "dz"), numberField(record, "measure"));
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

	private static function finiteNumber(value:Dynamic, name:String):Float {
		var result:Float = cast value;
		if (!Math.isFinite(result))
			throw new ParametricError("document field is not finite: " + name);
		return result;
	}

	private static function intField(value:Dynamic, name:String):Int {
		var result = numberField(value, name);
		return integerValue(result, name);
	}

	private static function integerValue(result:Float, name:String):Int {
		if (!Math.isFinite(result))
			throw new ParametricError("document field is not finite: " + name);
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
