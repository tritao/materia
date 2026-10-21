package materia.tooling;

import haxe.Json;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.ToolingEnums.CoatedFace;
import materia.tooling.ToolingEnums.StockDirection;
import materia.casting.InsertSpec.InsertRole;
import materia.casting.InsertSpec.InsertFinish;
import haxe.crypto.Sha256;
import materia.assembly.AssemblyCodec;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingPrism;
import materia.casting.CastingPrism.CastingPoint2;
import materia.tooling.casting.CastingToolingInput;
import materia.casting.CastingFeature;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.casting.CastingSolid;
import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.tooling.casting.CastingSetup;
import materia.tooling.casting.CastingSetup.CoreStrategy;
import materia.tooling.casting.CastingSetup.CastingFeatureStrategy;

/** Strict, detached and deterministic records. No native geometry or inventory effects.
 * Named collections are sorted by ID; profile vertices retain their geometric order.
 * Inputs are explicitly millimetres; other units require conversion before this boundary.
 */
class ToolingCodec {
	public static inline var FORMAT = "materia.casting-tooling";
	public static inline var VERSION = 11;
	public static inline var MAX_RECORDS = 512;
	public static inline var MAX_TEXT_BYTES = 2097152;

	public static function encode(document:ToolingDocument):String {
		var text = canonical({format: FORMAT, version: VERSION, document: snapshot(document)}) + "\n";
		checkSize(text);
		return text;
	}

	public static function decode(text:String):ToolingDocument {
		checkSize(text);
		var raw:Dynamic = Json.parse(text);
		object(raw, ["format", "version", "document"], "tooling");
		if (string(raw, "format") != FORMAT || integer(raw, "version") != VERSION)
			throw "Unsupported tooling format or version";
		return readDocument(Reflect.field(raw, "document"));
	}

	/** Persist source geometry once when a native source component needs no mold setup. */
	public static function encodeInput(input:CastingToolingInput):String {
		var text = canonical({format:FORMAT+"-source",version:VERSION,input:inputSnapshot(input)})+"\n";
		checkSize(text);
		return text;
	}

	public static function decodeInput(text:String):CastingToolingInput {
		checkSize(text);
		var raw:Dynamic = Json.parse(text);
		object(raw,["format","version","input"],"tooling source");
		if (string(raw,"format") != FORMAT+"-source" || integer(raw,"version") != VERSION)
			throw "Unsupported tooling source format or version";
		return readInput(Reflect.field(raw,"input"));
	}

	/** Accepts typed records through exactly the same validation as decoded JSON. */
	public static function snapshot(document:ToolingDocument):ToolingDocument
		return readDocument(document);

	static function readDocument(raw:Dynamic):ToolingDocument {
		object(raw, ["input", "setup", "recipe"], "document");
		var input = readInput(Reflect.field(raw, "input"));
		var document:ToolingDocument = {
			input: input,
			setup: readSetup(Reflect.field(raw, "setup"), input),
			recipe: readRecipe(Reflect.field(raw, "recipe"))
		};
		materia.tooling.casting.CastingStrategies.resolve(document);
		return document;
	}

	public static function inputFingerprint(input:CastingToolingInput):String
		return Sha256.encode(canonical(inputSnapshot(input)));

	public static function recipeFingerprint(recipe:ToolingRecipe):String
		return Sha256.encode(canonical(recipeSnapshot(recipe)));

	public static function documentFingerprint(document:ToolingDocument):String
		return Sha256.encode(encode(document));

	public static function inputSnapshot(input:CastingToolingInput):CastingToolingInput
		return readInput(input);

	static function readInput(raw:Dynamic):CastingToolingInput {
		object(raw, ["id", "revision", "lengthUnit", "envelope", "features", "inserts", "datumScheme", "interfaces", "interfaceContext"], "input");
		if (string(raw, "lengthUnit") != "mm")
			throw "Tooling input requires explicit mm units";
		var envelope:Array<CastingPrism> = [for (p in array(raw, "envelope")) readPrism(p)];
		if (envelope.length == 0)
			throw "Tooling input needs an envelope";
		var features = materia.casting.CastingFeatureCodec.snapshot(array(raw,"features"));
		var inserts:Array<CastingToolingInsert> = [for (i in array(raw, "inserts")) readInsert(i)];
		unique(envelope, p -> p.id, "envelope");
		unique(features, f -> f.id, "features");
		unique(inserts, i -> i.id, "inserts");
		var byId = new Map<String, CastingToolingInsert>();
		for (i in inserts)
			byId.set(i.id, i);
		var referenced = new Map<String, Bool>();
		for (f in features) {
			if (f.kind == InsertRecess) {
				var i = f.owner == null ? null : byId.get(f.owner);
				if (i == null || referenced.exists(f.owner))
					throw 'Invalid insert reference on ${f.id}';
				if (canonical(f.geometry.solid) != canonical(materia.casting.CastingFeature.CastingGeometrySolid.Insert(i.body)) || !sameFrame(f.geometry.pose, i.pose))
					throw 'Insert recess ${f.id} does not match insert ${i.id}';

				referenced.set(i.id, true);
			}
		}
		for (i in inserts)
			if (!referenced.exists(i.id))
				throw 'Missing recess for insert ${i.id}';
		materia.casting.CastingCavities.derive(envelope, features);
		var interfaces = materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(Reflect.field(raw, "interfaces"));
		var datumScheme = Reflect.field(raw, "datumScheme") == null ? null
			: materia.casting.FoundationDatumScheme.FoundationDatums.snapshot(Reflect.field(raw, "datumScheme"));
		if (datumScheme != null) {
			if (datumScheme.foundationId != string(raw, "id")) throw "Foundation datum scheme belongs to a different source";
			materia.casting.FoundationDatumScheme.FoundationDatums.validateSource(datumScheme,envelope,features,
				[for (insert in inserts) {id:insert.id,pose:insert.pose,holes:insert.holes,interfaces:insert.interfaces}], interfaces);
		}
		var context=Reflect.field(raw,"interfaceContext")==null ? null
			: materia.casting.CastingInterfaceContext.CastingInterfaceContexts.snapshot(Reflect.field(raw,"interfaceContext"));
		if (context!=null) {
			validateInterfaceBindings(context,interfaces,inserts);
		}
		return {
			id: string(raw, "id"),
			revision: revision(raw),
			lengthUnit: ToolingLengthUnit.Millimetres,
			datumScheme: datumScheme,
			interfaceContext: context,
			interfaces: interfaces,
			envelope: envelope,
			features: features,
			inserts: inserts
		};
	}

	static function validateInterfaceBindings(context:materia.casting.CastingInterfaceContext,
			interfaces:Array<materia.casting.InterfaceRequirement>,inserts:Array<CastingToolingInsert>):Void {
		if (context.inserts.length!=inserts.length) throw "Interface context must bind every source insert";
		validateInterfaceOwner(context,context.bodyMember,materia.assembly.AssemblyFrames.identity(),interfaces);
		for (insert in inserts) {
			var bindings=context.inserts.filter(binding -> binding.id==insert.id);
			if (bindings.length!=1) throw "Missing interface context insert binding: "+insert.id;
			validateInterfaceOwner(context,bindings[0].member,insert.pose,insert.interfaces);
		}
	}

	static function validateInterfaceOwner(context:materia.casting.CastingInterfaceContext,member:String,pose:AssemblyFrame,
			requirements:Array<materia.casting.InterfaceRequirement>):Void {
		var count=0;
		for (source in context.sources) switch source {
			case Precision(owner,requirement,sourcePose):
				if (owner!=member) continue;
				count++;
				var expected=requirements.filter(expected -> expected.id==requirement.id);
				if (expected.length!=1 || canonical(materia.casting.InterfaceRequirement.InterfaceRequirements.records(expected))
					!=canonical(materia.casting.InterfaceRequirement.InterfaceRequirements.records([requirement])) || !sameFrame(pose,sourcePose))
					throw "Interface context differs from source requirement or pose: "+member+"/"+requirement.id;
			case Axis(_,_,_):
		}
		if (count!=requirements.length) throw "Interface context omits source requirements: "+member;
	}

	static function readPrism(raw:Dynamic):CastingPrism {
		object(raw, ["id", "profile", "height", "pose"], "prism");
		var profile:Array<CastingPoint2> = [
			for (p in array(raw, "profile", 64)) {
				object(p, ["x", "y"], "profile point");
				{
					x: number(p, "x"),
					y: number(p, "y")
				};
			}
		];
		if (profile.length < 3)
			throw "Prism needs at least three profile vertices";
		// Every other vertex must lie strictly left of every directed edge. This
		// rejects clockwise, degenerate, concave and self-intersecting profiles.
		for (n in 0...profile.length) {
			var a = profile[n], b = profile[(n + 1) % profile.length];
			for (k in 0...profile.length)
				if (k != n && k != (n + 1) % profile.length) {
					var c = profile[k],
						cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
					if (!Math.isFinite(cross) || cross <= 0)
						throw "Prism profile must be strictly convex and CCW";
				}
		}
		return {
			id: string(raw, "id"),
			profile: profile,
			height: positive(raw, "height"),
			pose: frame(raw, "pose")
		};
	}

	static function readInsert(raw:Dynamic):CastingToolingInsert {
		object(raw, [
			"id",
			"body",
			"holes",
			"origin", "interfaces",
			"pose",
			"role",
			"label",
			"positionTolerance", "material",
			"finish"
		], "insert");
		var source = materia.casting.InsertSpec.InsertSpecCodec.snapshot({
			body: Reflect.field(raw, "body"), holes: Reflect.field(raw, "holes"), origin: Reflect.field(raw, "origin"), interfaces: Reflect.field(raw,"interfaces")
		});
		var body = source.body;
		var material=materia.casting.InsertMaterial.InsertMaterials.snapshot(Reflect.field(raw,"material"));
		switch source.origin {
			case Fabricated(manufacturing):
				if (manufacturing.material.grade!=material.grade) throw "Insert manufacturing grade differs from source stock";
			case Catalogue(_):
		}
		var boreWidth = switch InsertBodies.mouth(body) {
			case Box(w, d, _): Math.min(w, d);
			case Cylinder(diameter, _): diameter;
		};
		var holes = source.holes;
		for (hole in holes) {
			var diameter = hole.relief == null ? hole.diameter : hole.relief.diameter;
			if (hole.counterbore != null) diameter = Math.max(diameter, hole.counterbore.diameter);
			if (diameter >= boreWidth) throw "Insert bore consumes its seat";
		}
		return {
			id: string(raw, "id"),
			body: body,
			holes: holes,
			origin: source.origin,
			interfaces: source.interfaces,
			pose: frame(raw, "pose"),
			role: insertRole(string(raw, "role")),
			label: string(raw, "label"),
			positionTolerance: positive(raw, "positionTolerance"),
			material: material,
			finish: insertFinish(string(raw, "finish"))
		};
	}

	static function readSetup(raw:Dynamic, input:CastingToolingInput):CastingSetup {
		object(raw, [
			"id",
			"revision",
			"sourceFingerprint",
			"sourceToMold",
			"fillHeightMm",
			"freeboardMm",
			"features"
		], "setup");
		var fingerprint = string(raw, "sourceFingerprint");
		if (fingerprint != inputFingerprint(input))
			throw "Stale tooling setup: source fingerprint does not match";
		var byId = new Map<String, CastingFeature>();
		for (f in input.features)
			if (f.kind != InsertRecess) {
				byId.set(f.id, f);
				if (f.derivedFrom != null)
					byId.set(f.derivedFrom, f);
			}
		var used = new Map<String, Bool>();
		var features:Array<CastingFeatureStrategy> = [];
		for (f in array(raw, "features")) {
			object(f, ["featureId", "strategy", "withdrawal", "priority"], "feature strategy");
			var id = string(f, "featureId");
			if (!byId.exists(id) || used.exists(id))
				throw 'Unknown, duplicate or insert-recess strategy $id';
			used.set(id, true);
			var strategy:CoreStrategy = switch string(f, "strategy") {
				case "sacrificial-eps":
					SacrificialEPS;
				case "reusable-former":
					ReusableFormer;
				case "unsupported":
					Unsupported;
				default:
					throw "Unsupported core strategy token";
			};
			var direction:Null<{
				var x:Float;
				var y:Float;
				var z:Float;
			}> = null;
			if (Reflect.field(f, "withdrawal") != null) {
				var v = Reflect.field(f, "withdrawal");
				object(v, ["x", "y", "z"], "withdrawal");
				direction = {x: number(v, "x"), y: number(v, "y"), z: number(v, "z")};
				var norm = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
				if (Math.abs(norm - 1) > 1e-8)
					throw "Withdrawal direction must be unit length";
			}
			if (strategy == ReusableFormer && direction == null)
				throw "Reusable former needs a withdrawal direction";
			var priority = integer(f, "priority");
			features.push({
				featureId: id,
				strategy: strategy,
				withdrawal: direction,
				priority: priority
			});
		}
		features.sort((a, b) -> Reflect.compare(a.featureId, b.featureId));
		return {
			id: string(raw, "id"),
			revision: revision(raw),
			sourceFingerprint: fingerprint,
			fillHeightMm: number(raw, "fillHeightMm"),
			freeboardMm: nonnegative(raw, "freeboardMm"),
			sourceToMold: frame(raw, "sourceToMold"),
			features: features
		};
	}

	public static function recipeSnapshot(recipe:ToolingRecipe):ToolingRecipe
		return readRecipe(recipe);

	static function readRecipe(raw:Dynamic):ToolingRecipe {
		object(raw, [
			"featureRules",
			"id",
			"revision",
			"panel",
			"core",
			"jointClearance",
			"linerThickness",
			"routerDiameter",
			"adhesiveLineThickness",
			"mixDensityKgM3",
			"steelDensityKgM3",
			"handlingLoadFactor",
			"coreWireAxis",
			"hardware",
			"jointLiningAllowance",
			"jointMachiningAllowance",
			"minimumLigament",
			"resinSpec",
			"releaseSpec",
			"adhesiveSpec",
			"processEvidence",
			"designReview"
		], "recipe");
		var limits = Reflect.field(raw, "designReview");
		object(limits, [
			"minimumCornerRadiusMm",
			"minimumInsertCoverMm",
			"minimumInsertEdgeMm",
			"minimumFixingAccessMm",
			"minimumLiftingPoints",
			"removalClearanceMm"
		], "casting review limits");
		function optionalLimit(key:String):Null<Float>
			return Reflect.field(limits, key) == null ? null : nonnegative(limits, key);
		var lifting:Null<Int> = Reflect.field(limits, "minimumLiftingPoints") == null ? null : integer(limits, "minimumLiftingPoints");
		if (lifting != null && lifting < 0)
			throw "Invalid minimum lifting points";
		var p = Reflect.field(raw, "panel"), c = Reflect.field(raw, "core");
		var hw = Reflect.field(raw, "hardware");
		object(hw, [
			"plugDepth",
			"plugMaterial",
			"sealMargin",
			"sealMaterial",
			"minimumBridgeWidth",
			"locatingPinDiameter",
			"supportScrewDiameter",
			"supportClearanceDiameter",
			"secondaryPinTravel",
			"padDepth",
			"labelOffset",
			"datumThickness",
			"datumPostWidth",
			"formerFlangeThickness",
			"formerFlangeMargin",
			"fixturePinDiameter",
			"fixturePinEmbed",
			"fixturePinSpacing",
			"fixtureSlotToolDiameter"
		], "tooling hardware");
		var hardware:materia.tooling.ToolingRecipe.ToolingHardware = {
			plugDepth: positive(hw, "plugDepth"),
			plugMaterial: string(hw, "plugMaterial"),
			sealMargin: nonnegative(hw, "sealMargin"),
			sealMaterial: string(hw, "sealMaterial"),
			minimumBridgeWidth: positive(hw, "minimumBridgeWidth"),
			locatingPinDiameter: positive(hw, "locatingPinDiameter"),
			supportScrewDiameter: positive(hw, "supportScrewDiameter"),
			supportClearanceDiameter: positive(hw, "supportClearanceDiameter"),
			secondaryPinTravel: positive(hw, "secondaryPinTravel"),
			padDepth: positive(hw, "padDepth"),
			labelOffset: positive(hw, "labelOffset"),
			datumThickness: positive(hw, "datumThickness"),
			datumPostWidth: positive(hw, "datumPostWidth"),
			formerFlangeThickness: positive(hw, "formerFlangeThickness"),
			formerFlangeMargin: positive(hw, "formerFlangeMargin"),
			fixturePinDiameter: positive(hw, "fixturePinDiameter"),
			fixturePinEmbed: positive(hw, "fixturePinEmbed"),
			fixturePinSpacing: positive(hw, "fixturePinSpacing"),
			fixtureSlotToolDiameter: positive(hw, "fixtureSlotToolDiameter")
		};
			if (hardware.supportClearanceDiameter <= hardware.supportScrewDiameter)
			throw "Hardware clearance must exceed screw diameter";
		object(p, [
			"materialSpec",
			"nominalThickness",
			"measuredThickness",
			"thicknessVariation",
			"width",
			"height",
			"coatedFace",
			"allowedRotations",
			"directionConstraint"
		], "panel stock");
		object(c, [
			"materialSpec",
			"width",
			"depth",
			"height",
			"densityKgM3",
			"compressionStrengthPa",
			"compressionStrainFraction",
			"maximumTemperatureC",
			"evidence"
		], "core stock");
		var thickness = positive(p, "measuredThickness"),
			variation = nonnegative(p, "thicknessVariation");
		if (variation >= thickness)
			throw "Panel thickness variation consumes stock";
		var face = coatedFace(string(p, "coatedFace"));
		var rotations:Array<Int> = [];
		for (value in array(p, "allowedRotations", 2)) {
			var rotation = integer({value: value}, "value");
			if ((rotation != 0 && rotation != 90) || rotations.indexOf(rotation) >= 0)
				throw "Invalid or duplicate panel stock rotation";
			rotations.push(rotation);
		}
		rotations.sort(Reflect.compare);
		var direction = stockDirection(string(p, "directionConstraint"));
		if (rotations.length == 0 || (direction == "width-aligned" && rotations.indexOf(90) >= 0))
			throw "Invalid panel direction constraint";
		if (nonnegative(raw, "jointClearance") + 1e-8 < variation + nonnegative(raw, "jointLiningAllowance") + nonnegative(raw, "jointMachiningAllowance"))
			throw "Joint clearance below stock variation and allowances";
		if (positive(raw, "handlingLoadFactor") < 1)
			throw "Handling load factor must be at least one";
		var strength = nullableNumber(c, "compressionStrengthPa", true),
			strain = nullableNumber(c, "compressionStrainFraction", true);
		if ((strength == null) != (strain == null) || (strain != null && strain >= 1))
			throw "Compression strength requires a strain fraction between zero and one";
		return {
			featureRules: materia.tooling.casting.CastingStrategies.snapshotRules(Reflect.field(raw, "featureRules")),
			id: string(raw, "id"),
			revision: revision(raw),
			panel: {
				materialSpec: string(p, "materialSpec"),
				nominalThickness: positive(p, "nominalThickness"),
				measuredThickness: thickness,
				thicknessVariation: variation,
				width: positive(p, "width"),
				height: positive(p, "height"),
				coatedFace: face,
				allowedRotations: rotations,
				directionConstraint: direction
			},
			core: {
				materialSpec: string(c, "materialSpec"),
				width: positive(c, "width"),
				depth: positive(c, "depth"),
				height: positive(c, "height"),
				densityKgM3: nullableNumber(c, "densityKgM3", true),
				compressionStrengthPa: strength,
				compressionStrainFraction: strain,
				maximumTemperatureC: nullableNumber(c, "maximumTemperatureC", false),
				evidence: evidence(c, "evidence")
			},
			adhesiveLineThickness: positive(raw, "adhesiveLineThickness"),
			mixDensityKgM3: positive(raw, "mixDensityKgM3"),
			steelDensityKgM3: positive(raw, "steelDensityKgM3"),
			handlingLoadFactor: positive(raw, "handlingLoadFactor"),
			coreWireAxis: switch string(raw, "coreWireAxis") {
				case "x":
					materia.tooling.ToolingRecipe.CoreWireAxis.X;
				case "y":
					materia.tooling.ToolingRecipe.CoreWireAxis.Y;
				case "z":
					materia.tooling.ToolingRecipe.CoreWireAxis.Z;
				default:
					throw "Invalid core wire axis";
			},
			hardware: hardware,
			jointLiningAllowance: nonnegative(raw, "jointLiningAllowance"),
			jointMachiningAllowance: nonnegative(raw, "jointMachiningAllowance"),
			jointClearance: nonnegative(raw, "jointClearance"),
			linerThickness: nonnegative(raw, "linerThickness"),
			routerDiameter: positive(raw, "routerDiameter"),
			minimumLigament: positive(raw, "minimumLigament"),
			resinSpec: string(raw, "resinSpec"),
			releaseSpec: string(raw, "releaseSpec"),
			adhesiveSpec: string(raw, "adhesiveSpec"),
			processEvidence: evidence(raw, "processEvidence"),
			designReview: {
				minimumCornerRadiusMm: optionalLimit("minimumCornerRadiusMm"),
				minimumInsertCoverMm: optionalLimit("minimumInsertCoverMm"),
				minimumInsertEdgeMm: optionalLimit("minimumInsertEdgeMm"),
				minimumFixingAccessMm: optionalLimit("minimumFixingAccessMm"),
				minimumLiftingPoints: lifting,
				removalClearanceMm: optionalLimit("removalClearanceMm")
			}
		};
	}

	public static function coatedFace(value:String):CoatedFace
		return switch value {
			case "positive-z":
				CoatedFace.PositiveZ;
			case "negative-z":
				CoatedFace.NegativeZ;
			case "both":
				CoatedFace.Both;
			case "none":
				CoatedFace.Uncoated;
			default:
				throw "Invalid coated face";
		};

	public static function stockDirection(value:String):StockDirection
		return switch value {
			case "free":
				StockDirection.Free;
			case "width-aligned":
				StockDirection.WidthAligned;
			default:
				throw "Invalid panel direction constraint";
		};

	public static function insertRole(value:String):InsertRole {
		for (role in materia.casting.InsertSpec.InsertRoles.values())
			if ((role : String) == value)
				return role;
		throw "Invalid insert role";
	}

	public static function insertFinish(value:String):InsertFinish {
		for (finish in materia.casting.InsertSpec.InsertFinishes.values())
			if ((finish : String) == value)
				return finish;
		throw "Invalid insert finish";
	}

	static function object(raw:Dynamic, fields:Array<String>, label:String):Void {
		if (raw == null || !Reflect.isObject(raw) || Std.isOfType(raw, Array) || Std.isOfType(raw, String))
			throw 'Expected tooling object: $label';
		for (key in Reflect.fields(raw))
			if (fields.indexOf(key) < 0)
				throw 'Unknown $label field: $key';
		for (key in fields)
			if (!Reflect.hasField(raw, key))
				throw 'Missing $label field: $key';
	}

	static function array(raw:Dynamic, key:String, limit:Int = MAX_RECORDS):Array<Dynamic> {
		var value:Dynamic = Reflect.field(raw, key);
		if (!Std.isOfType(value, Array))
			throw 'Expected tooling array: $key';
		var result:Array<Dynamic> = cast value;
		if (result.length > limit)
			throw 'Too many tooling records: $key';
		return result;
	}

	static function string(raw:Dynamic, key:String):String {
		var value:Dynamic = Reflect.field(raw, key);
		if (!Std.isOfType(value, String))
			throw 'Expected tooling text: $key';
		var result:String = cast value;
		if (StringTools.trim(result).length == 0 || result.length > 2048 || AssemblyCodec.containsNul(result))
			throw 'Invalid tooling text: $key';
		if (key == "id" || key == "featureId" || key == "insertId")
			ToolingIds.validate(result);
		return result;
	}

	static function nullableString(raw:Dynamic, key:String):Null<String>
		return Reflect.field(raw, key) == null ? null : string(raw, key);

	static function number(raw:Dynamic, key:String):Float {
		var value:Dynamic = Reflect.field(raw, key);
		if ((!Std.isOfType(value, Float) && !Std.isOfType(value, Int)) || !Math.isFinite(value))
			throw 'Expected finite tooling number: $key';
		return value;
	}

	static function integer(raw:Dynamic, key:String):Int {
		var n = number(raw, key);
		if (n < 0 || n > 2147483647 || n != Math.floor(n))
			throw 'Expected tooling integer: $key';
		return Std.int(n);
	}

	static function revision(raw:Dynamic):Int {
		var n = integer(raw, "revision");
		if (n < 1)
			throw "Tooling revision must be positive";
		return n;
	}

	static function positive(raw:Dynamic, key:String):Float {
		var n = number(raw, key);
		if (n <= 0)
			throw 'Expected positive tooling dimension: $key';
		return n;
	}

	static function nonnegative(raw:Dynamic, key:String):Float {
		var n = number(raw, key);
		if (n < 0)
			throw 'Expected nonnegative tooling value: $key';
		return n;
	}

	static function nullableNumber(raw:Dynamic, key:String, mustBePositive:Bool):Null<Float>
		return Reflect.field(raw, key) == null ? null : (mustBePositive ? positive(raw, key) : number(raw, key));

	static function evidence(raw:Dynamic, key:String):Array<String> {
		var result:Array<String> = [];
		for (value in array(raw, key, 64)) {
			var text = string({value: value}, "value");
			if (result.indexOf(text) >= 0)
				throw 'Duplicate evidence reference: $text';
			result.push(text);
		}
		result.sort(Reflect.compare);
		return result;
	}

	static function frame(raw:Dynamic, key:String):AssemblyFrame {
		var p = Reflect.field(raw, key);
		object(p, ["x", "y", "z", "qx", "qy", "qz", "qw"], key);
		var result:AssemblyFrame = {
			x: number(p, "x"),
			y: number(p, "y"),
			z: number(p, "z"),
			qx: number(p, "qx"),
			qy: number(p, "qy"),
			qz: number(p, "qz"),
			qw: number(p, "qw")
		};
		AssemblyCodec.validateFrame(result);
		var norm = result.qx * result.qx + result.qy * result.qy + result.qz * result.qz + result.qw * result.qw;
		if (Math.abs(norm - 1) > 1e-8)
			throw "Tooling frame requires a unit quaternion";
		return result;
	}

	static function sameFrame(a:AssemblyFrame, b:AssemblyFrame):Bool {
		if (Math.abs(a.x - b.x) > 1e-8 || Math.abs(a.y - b.y) > 1e-8 || Math.abs(a.z - b.z) > 1e-8)
			return false;
		var dot = a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
		var sign = dot < 0 ? -1.0 : 1.0;
		return Math.abs(a.qx - sign * b.qx) <= 1e-8
			&& Math.abs(a.qy - sign * b.qy) <= 1e-8
			&& Math.abs(a.qz - sign * b.qz) <= 1e-8
			&& Math.abs(a.qw - sign * b.qw) <= 1e-8;
	}

	static function checkSize(text:String):Void {
		if (text == null || text.length > MAX_TEXT_BYTES || haxe.io.Bytes.ofString(text).length > MAX_TEXT_BYTES)
			throw "Tooling document exceeds size limit";
	}

	static function unique<T>(items:Array<T>, key:T->String, label:String):Void {
		var ids = new Map<String, Bool>();
		for (item in items) {
			var id = key(item);
			if (ids.exists(id))
				throw 'Duplicate tooling $label ID: $id';
			ids.set(id, true);
		}
		items.sort((a, b) -> Reflect.compare(key(a), key(b)));
	}

	/** Sort object keys as well as named records so field construction order is irrelevant. */
	public static function canonical(value:Dynamic):String {
		if (Std.isOfType(value,materia.casting.InsertManufacturing.InsertBlank))
			return canonical(materia.casting.InsertManufacturing.InsertManufacturingCodec.blankRecord(cast value));
		if (Std.isOfType(value,materia.casting.InsertManufacturing.InsertInspection))
			return canonical(materia.casting.InsertManufacturing.InsertManufacturingCodec.inspectionRecord(cast value));
		if (Std.isOfType(value,materia.casting.CastingInterfaceContext.InterfaceContextSource))
			return canonical(materia.casting.CastingInterfaceContext.CastingInterfaceContexts.sourceRecord(cast value));
		if (Std.isOfType(value,materia.casting.InterfaceRelationship.InterfaceMember))
			return canonical(materia.casting.InterfaceRelationship.InterfaceRelationships.memberRecord(cast value));
		if (Std.isOfType(value,materia.casting.InterfaceRelationship.InterfaceRelationshipKind))
			return canonical(materia.casting.InterfaceRelationship.InterfaceRelationships.kindRecord(cast value));
		if (Std.isOfType(value,materia.casting.InterfaceRelationship.RelationshipTolerance))
			return canonical(materia.casting.InterfaceRelationship.InterfaceRelationships.toleranceRecord(cast value));
		if (Std.isOfType(value, materia.casting.FoundationDatumScheme.DatumReference))
			return canonical(materia.casting.FoundationDatumScheme.FoundationDatums.referenceRecord(cast value));
		if (Std.isOfType(value, materia.casting.InterfaceRequirement.PrecisionGeometry))
			return canonical(materia.casting.InterfaceRequirement.InterfaceRequirements.geometryRecord(cast value));
		if (Std.isOfType(value, materia.casting.InsertSpec.InsertBody))
			return canonical(materia.casting.InsertSpec.InsertBodyCodec.record(cast value));
		if (Std.isOfType(value, materia.casting.CastingFeature.CastingGeometrySolid))
			return canonical(materia.casting.CastingFeature.CastingGeometrySolids.record(cast value));
		if (Std.isOfType(value, materia.casting.InsertOrigin))
			return canonical(materia.casting.InsertOrigin.InsertOriginCodec.record(cast value));
		if (Std.isOfType(value, CastingSolid))
			return canonical(materia.casting.CastingSolid.CastingSolidCodec.record(cast value));
		if (value == null)
			return "null";
		if (Std.isOfType(value, String) || Std.isOfType(value, Bool) || Std.isOfType(value, Float) || Std.isOfType(value, Int))
			return Json.stringify(value);
		if (Std.isOfType(value, Array)) {
			var values:Array<Dynamic> = cast value;
			return "[" + [for (v in values) canonical(v)].join(",") + "]";
		}
		var keys = Reflect.fields(value);
		keys.sort(Reflect.compare);
		return "{" + [
			for (key in keys)
				Json.stringify(key) + ":" + canonical(Reflect.field(value, key))
		].join(",") + "}";
	}
}
