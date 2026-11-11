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
import haxeon.wire.JsonWire;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SolverSettings;
import haxe.Json;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.threads.ThreadSpec;
import materia.project.ThreadSpecData;
import haxeon.wire.JsonWire;
import cadkit.parametric.features.ImportedShapeFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.PocketFeature;
import cadkit.parametric.features.FaceFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.ChamferFeature;
import cadkit.parametric.features.RevolveFeature;
import cadkit.parametric.features.RotationFeature;
import cadkit.parametric.features.MirrorFeature;
import cadkit.parametric.features.DatumSketchFeature;
import cadkit.parametric.features.LevelExtrudeFeature;
import cadkit.parametric.features.LevelBoxFeature;
import cadkit.parametric.features.DefinitionOutputFeature;
import cadkit.parametric.features.ToolCollectionFeature;
import cadkit.parametric.DocumentId;
import cadkit.parametric.ElementId;
import cadkit.parametric.LevelElement;
import cadkit.parametric.ReferencePlaneElement;
import cadkit.parametric.ElementReference;
import cadkit.parametric.Placement;
import cadkit.parametric.DefinitionId;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.ElementKind;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.PersistentReference;
import cadkit.parametric.RelationshipId;

/** Versioned JSON persistence for the Haxeon parametric document layer. */
class DocumentCodec {
  public static inline var FORMAT:String = "cadkit.document";
  public static inline var VERSION:Int = 18;

  public static function encode(document:Document):String {
    var encodedFeatures:Array<Dynamic> = [];
    for (index in 0...document.featureCount()) {
      var feature = document.featureAt(index);
      var encoded = encodeFeature(feature);
      Reflect.setField(encoded, "active", feature.active);
      if (feature.label != "") Reflect.setField(encoded, "label", feature.label);
      if (feature.group != "") Reflect.setField(encoded, "group", feature.group);
      encodedFeatures.push(encoded);
    }
    var encodedParameters:Array<Dynamic> = [];
    for (parameter in document.namedParameters()) {
      var bindings:Array<Dynamic> = [];
      for (binding in parameter.bindings()) bindings.push({
        feature: binding.ownerFeature().id.toInt(),
        parameter: binding.name
      });
      var encodedParameter:Dynamic = {
        name: parameter.name,
        value: parameter.valueIn(parameter.unit),
        kind: parameter.kind,
        unit: parameter.unit,
        expression: parameter.expression == null ? null : parameter.expression.source,
        bindings : bindings
      };
      // Role metadata only when it says something: editable is the default, an expression is always derived.
      if (parameter.expression == null && parameter.role != ParameterRole.Editable) Reflect.setField(encodedParameter, "role", parameter.role);
      if (parameter.basis != "") Reflect.setField(encodedParameter, "basis", parameter.basis);
      if (parameter.note != "") Reflect.setField(encodedParameter, "note", parameter.note);
      encodedParameters.push(encodedParameter);
    }
    var encodedElements:Array<Dynamic> = [];
    for (element in document.allElements()) {
      var placement = encodePlacement(element.localPlacement);
      var parent = element.placementParent == null ? null : encodeElementReference(element.placementParent);
      if (element.kind == "geometry") {
        var geometry:Feature = cast element.output;
        encodedElements.push({
          id: element.id.value,
          name: element.name,
          kind: element.kind,
          output: geometry.id.toInt(),
          placement: placement,
          parent: parent,
          properties: encodeProperties(element.properties())
        });
      } else if (element.kind == "instance") {
        var instance:InstanceElement = cast element;
        var overrides:Array<Dynamic> = [];
        for (name in instance.overrideNames()) overrides.push({name: name, value: instance.typedOverrideValue(name)});
        encodedElements.push({
          id: element.id.value,
          name: element.name,
          kind: element.kind,
          definition: instance.definitionId.value,
          overrides: overrides,
          placement: placement,
          parent: parent,
          properties: encodeProperties(element.properties())
        });
      } else if (element.kind == "level") {
        var level:LevelElement = cast element;
        encodedElements.push({
          id: level.id.value,
          name: level.name,
          kind: level.kind,
          elevation: level.elevation,
          offset: level.offset,
          relativeTo: level.relativeTo == null ? null : encodeElementReference(level.relativeTo),
          placement : placement,
          parent : parent,
          properties : encodeProperties(element.properties())
        });
      } else if (element.kind == ElementKind.ReferencePlane) {
        var datum:ReferencePlaneElement = cast element;
        encodedElements.push({
          id: datum.id.value,
          name: datum.name,
          kind: datum.kind,
          plane: {
            origin: encodeVector(datum.plane.origin),
            xDirection: encodeVector(datum.plane.xDirection),
            normal: encodeVector(datum.plane.normal)
          },
          placement: placement,
          parent: parent,
          properties: encodeProperties(element.properties())
        });
      } else if (element.kind == ElementKind.Object) {
        encodedElements.push({
          id: element.id.value,
          name: element.name,
          kind: element.kind,
          placement: placement,
          parent: parent,
          properties: encodeProperties(element.properties())
        });
      } else {
        throw new ParametricError("unsupported structural element kind: " + element.kind);
      }
    }
    var encodedDefinitions:Array<Dynamic> = [];
    for (definition in document.allDefinitions()) {
      var inputs:Array<Dynamic> = [];
      for (input in definition.inputs()) inputs.push({
        name: input.name,
        kind: input.kind,
        unit: input.unit,
        value: input.isNumeric() ? UnitConversion.fromCanonical(input.defaultValue,
        input.kind, input.unit) : input.defaultValue,
        allowedValues : input.allowedValues,
        editedByUser : input.editedByUser
      });
      var outputs:Array<Dynamic> = [];
      for (output in definition.outputs()) outputs.push({name: output.name, purpose: output.purpose});
      var subgraph:Dynamic = null;
      var graphPayload = definition.subgraph;
      if (graphPayload != null) {
        var inputBindings:Array<Dynamic> = [];
        for (input in definition.inputs()) inputBindings.push({
          input: input.name,
          parameter: graphPayload.parameterName(input.name)
        });
        var outputFeatures:Array<Dynamic> = [];
        for (output in definition.outputs()) outputFeatures.push({
          output: output.name,
          feature: graphPayload.featureId(output.name)
        });
        subgraph = {graph: graphPayload.graph, inputBindings: inputBindings, outputFeatures: outputFeatures};
      }
      encodedDefinitions.push({
        id: definition.id.value,
        name: definition.name,
        recipe: definition.recipe,
        revision: definition.revision,
        inputs: inputs,
        outputs: outputs,
        subgraph: subgraph,
        properties: encodeProperties(definition.properties())
      });
    }
    var output = document.outputFeatureOrNull();
    var encodedRelationships:Array<Dynamic> = [];
    for (relationship in document.allRelationships()) encodedRelationships.push({
      id: relationship.id.value,
      type: relationship.typeName,
      source: encodeElementReference(relationship.source),
      target: encodeElementReference(relationship.target),
      properties: encodeProperties(relationship.properties())
    });
    return Json.stringify({
      format: FORMAT,
      version: VERSION,
      lengthUnit: document.lengthUnit,
      documentId: document.id.value,
      implicitOutput: document.implicitOutputEnabled,
      features: encodedFeatures,
      parameters: encodedParameters,
      definitions: encodedDefinitions,
      elements: encodedElements,
      relationships: encodedRelationships,
      output: output == null ? null : output.id.toInt()
    });
  }

  public static function decode(text:String, clone:Bool = false, evaluate:Bool = true):Document {
    AssemblyMemberEvaluator.register();
    var document:Null<Document> = null;
    try {
      var root:Dynamic = Json.parse(text);
      if (stringField(root, "format") != FORMAT) throw new ParametricError("unsupported document format");
      var version = intField(root, "version");
      if (version != VERSION) throw new ParametricError('schema v$version is unsupported; expected v$VERSION');
      var implicitOutput = optionalBool(root, "implicitOutput", true);
      var lengthUnit = stringField(root, "lengthUnit");

      var records:Array<Dynamic> = cast requiredField(root, "features");
      var sourceDocumentId = optionalString(root, "documentId");
      var targetDocument:Document = clone ? new Document(null, implicitOutput, lengthUnit) : new Document(
        new DocumentId(stringField(root, "documentId")),
        implicitOutput,
        lengthUnit
      );
      document = targetDocument;
      var decodeElementReferenceInDocument = function(value:Dynamic) {
        var reference = decodeElementReference(value);
        if (clone
          && sourceDocumentId != null && reference.documentId.value == sourceDocumentId) return new ElementReference(
            targetDocument.id,
            reference.elementId
          );
        return reference;
      };
      var remapDocumentId = function(value:String) {
        return clone && sourceDocumentId != null && value == sourceDocumentId ? targetDocument.id.value : value;
      };
      var pendingReferences:Array<Dynamic> = [];
      for (record in records) {
        var featureId = intField(record, "id");
        var featureType = stringField(record, "type");
        var codec = FeatureTypeRegistry.codec(featureType);
        if (codec == null) throw new ParametricError("unsupported feature type: " + featureType);
        var feature = document.add(codec.decode(requiredField(record, "data"), new FeatureCodecContext(document, remapDocumentId)));
        if (feature.serializationType() != featureType)
          throw new ParametricError('registered feature decoded as ${feature.serializationType()}, not $featureType');

        if (feature.id.toInt() != featureId) throw new ParametricError("feature IDs must be contiguous and ordered");
        if (!optionalBool(record, "active", true)) feature.restoreActive(false);
        var savedLabel = optionalString(record, "label");
        if (savedLabel != null) feature.label = savedLabel;
        var savedGroup = optionalString(record, "group");
        if (savedGroup != null) feature.group = savedGroup;
        var rawReferences:Array<Dynamic> = cast requiredField(record, "references");
        pendingReferences.push({feature: feature, references: rawReferences});
      }

      for (pending in pendingReferences) {
        var pendingFeature:Feature = cast Reflect.field(pending, "feature");
        var references:Array<Dynamic> = cast Reflect.field(pending, "references");
        for (reference in references) decodeReference(pendingFeature, reference);
      }

      var rawParameterRecords:Dynamic = Reflect.field(root, "parameters");
      var parameterRecords:Array<Dynamic> = rawParameterRecords == null ? [] : cast rawParameterRecords;
      for (record in parameterRecords) {
        var rawKind:Dynamic = Reflect.field(record, "kind");
        if (rawKind == null) document.defineParameter(stringField(record, "name"), numberField(record, "value"));
        else document.defineTypedParameter(
          stringField(record, "name"),
          numberField(record, "value"),
          stringField(record, "kind"),
          stringField(record, "unit")
        );
      }
      for (record in parameterRecords) {
        var rawExpression:Dynamic = Reflect.field(record, "expression");
        if (rawExpression != null) {
          if (!Std.isOfType(rawExpression,
            String)) throw new ParametricError("document field is not a string: expression");
          document.installExpression(stringField(record, "name"), cast rawExpression);
        }
      }
      for (record in parameterRecords) {
        var role = optionalString(record, "role"), basis = optionalString(record, "basis"), note = optionalString(record, "note");
        if (role != null || basis != null || note != null) {
          var described = document.parameter(stringField(record, "name"));
          described.describe(role == null ? described.role : role, basis == null ? "" : basis, note == null ? "" : note);
        }
      }
      for (record in parameterRecords) {
        var named = document.parameter(stringField(record, "name"));
        var bindings:Array<Dynamic> = cast requiredField(record, "bindings");
        for (binding in bindings) {
          var feature = requiredFeature(document, intField(binding, "feature"));
          named.bind(feature.parameter(stringField(binding, "parameter")));
        }
      }
      var rawOutput:Dynamic = Reflect.field(root, "output");
      if (rawOutput != null) document.setOutput(requiredFeature(document, integerValue(rawOutput, "output")));
      var definitionRecords:Array<Dynamic> = cast requiredField(root, "definitions");
      for (definitionRecord in definitionRecords) {
        var inputRecords:Array<Dynamic> = cast requiredField(definitionRecord, "inputs");
        var inputs:Array<DefinitionInput> = [];
        for (inputRecord in inputRecords) {
          var kind = stringField(inputRecord, "kind");
          var value:Dynamic = requiredField(inputRecord, "value");
          var rawAllowed:Dynamic = Reflect.field(inputRecord, "allowedValues");
          inputs.push(new DefinitionInput(
            stringField(inputRecord, "name"),
            kind,
            stringField(inputRecord, "unit"),
            value,
            rawAllowed == null ? null : cast rawAllowed,
            optionalBool(inputRecord, "editedByUser", false)
          ));
        }
        var outputRecords:Array<Dynamic> = cast requiredField(definitionRecord, "outputs");
        var outputs:Array<DefinitionOutput> = [];
        for (outputRecord in outputRecords) {
          var purpose = stringField(outputRecord, "purpose");
          outputs.push(new DefinitionOutput(stringField(outputRecord, "name"), purpose));
        }
        var subgraph:Null<DefinitionSubgraph> = null;
        var rawSubgraph:Dynamic = Reflect.field(definitionRecord, "subgraph");
        if (rawSubgraph != null) {
          var inputBindings = new Map<String, String>();
          var rawInputBindings:Array<Dynamic> = cast requiredField(rawSubgraph, "inputBindings");
          for (binding in rawInputBindings) {
            var inputName = stringField(binding, "input");
            if (inputBindings.exists(inputName)) throw new ParametricError("duplicate subgraph input binding: "
              + inputName);
            inputBindings.set(inputName, stringField(binding, "parameter"));
          }
          var outputFeatures = new Map<String, Int>();
          var rawOutputFeatures:Array<Dynamic> = cast requiredField(rawSubgraph, "outputFeatures");
          for (output in rawOutputFeatures) {
            var outputName = stringField(output, "output");
            if (outputFeatures.exists(outputName)) throw new ParametricError("duplicate subgraph output binding: "
              + outputName);
            outputFeatures.set(outputName, intField(output, "feature"));
          }
          subgraph = new DefinitionSubgraph(stringField(rawSubgraph, "graph"), inputBindings, outputFeatures);
        }
        var definition = document.installDefinition(
          new DefinitionId(stringField(definitionRecord, "id")),
          stringField(definitionRecord, "name"),
          stringField(definitionRecord, "recipe"),
          inputs,
          outputs,
          subgraph
        );
        definition.restoreRevision(intField(definitionRecord, "revision"));
        for (propertyRecord in optionalPropertyRecords(definitionRecord)) {
          var property = decodeTypedProperty(propertyRecord, remapDocumentId);
          definition.restoreProperty(property.name, property);
        }
      }
      var elementRecords:Array<Dynamic> = cast requiredField(root, "elements");
      for (elementRecord in elementRecords) {
        var kind = stringField(elementRecord, "kind");
        var eid = new ElementId(stringField(elementRecord, "id"));
        var ename = stringField(elementRecord, "name");
        if (kind == "geometry") document.installElement(
          ename,
          requiredFeature(document, intField(elementRecord, "output")),
          eid
        );
        else if (kind == "instance") {
          var overrideRecords:Array<Dynamic> = cast requiredField(elementRecord, "overrides");
          var overrides = new Map<String, Dynamic>();
          for (overrideRecord in overrideRecords) overrides.set(
            stringField(overrideRecord, "name"),
            requiredField(overrideRecord, "value")
          );
          document.installInstance(ename, eid, new DefinitionId(stringField(elementRecord, "definition")), overrides);
        } else if (kind == "level") {
          var relative:Dynamic = Reflect.field(elementRecord, "relativeTo");
          document.installLevel(
            ename,
            eid,
            numberField(elementRecord, "elevation"),
            numberField(elementRecord, "offset"),
            relative == null ? null : decodeElementReferenceInDocument(relative)
          );
        } else if (kind == "reference-plane") {
          var p = requiredField(elementRecord, "plane");
          document.installReferencePlane(
            ename,
            eid,
            new Plane(
              decodeVector(requiredField(p, "origin")),
              decodeVector(requiredField(p, "xDirection")),
              decodeVector(requiredField(p, "normal"))
            )
          );
        } else if (kind == ElementKind.Object) {
          document.installObject(ename, eid);
        } else throw new ParametricError("unsupported element kind: " + kind);
      }
      for (elementRecord in elementRecords) {
        var loaded = document.element(new ElementId(stringField(elementRecord, "id")));
        var rawParent:Dynamic = Reflect.field(elementRecord, "parent");
        document.restoreElementPlacement(
          loaded,
          decodePlacement(requiredField(elementRecord, "placement")),
          rawParent == null ? null : decodeElementReferenceInDocument(rawParent)
        );
        for (propertyRecord in optionalPropertyRecords(elementRecord)) {
          var property = decodeTypedProperty(propertyRecord, remapDocumentId);
          loaded.restoreProperty(property.name, property);
        }
        document.worldPlacement(loaded);
      }
      var rawRelationshipRecords:Dynamic = Reflect.field(root, "relationships");
      if (rawRelationshipRecords != null && !Std.isOfType(rawRelationshipRecords,
        Array)) throw new ParametricError("document relationships field is not an array");
      var relationshipRecords:Array<Dynamic> = rawRelationshipRecords == null ? [] : cast rawRelationshipRecords;
      for (relationshipRecord in relationshipRecords) {
        var relationship = document.installRelationship(
          new RelationshipId(stringField(relationshipRecord, "id")),
          stringField(relationshipRecord, "type"),
          decodeElementReferenceInDocument(requiredField(relationshipRecord, "source")),
          decodeElementReferenceInDocument(requiredField(relationshipRecord, "target"))
        );
        for (propertyRecord in optionalPropertyRecords(relationshipRecord)) {
          var property = decodeTypedProperty(propertyRecord, remapDocumentId);
          relationship.restoreProperty(property.name, property);
        }
      }
      document.validatePersistentReferences();

      if (evaluate) document.recompute();
      return document;
    } catch (error:Dynamic) {
      if (document != null) document.close();
      var detail:Dynamic = Reflect.field(error, "message");
      throw new ParametricError("document decode failed: " + (detail == null ? Std.string(error) : Std.string(detail)));
    }
  }

  private static function encodeFeature(feature:Feature):Dynamic {
    var featureType = feature.serializationType();
    // Every feature type is saved by its registered codec; the references it keeps itself are not saved again.
    var registered = FeatureTypeRegistry.codec(featureType);
    if (registered == null) throw new ParametricError("unsupported feature type: " + featureType);
    var excluded = registered.ownedReferences(feature);

    var references:Array<Dynamic> = [];
    for (index in 0...feature.topologyReferenceCount()) {
      var reference = feature.topologyReferenceAt(index);
      if (reference.state != ReferenceState.Closed && !containsReference(excluded,
        reference)) references.push(encodeReference(reference));
    }

    if (feature.document == null) throw new ParametricError("registered feature is not attached: " + featureType);
    return {
      id: feature.id.toInt(),
      type: featureType,
      data: registered.encode(feature, new FeatureCodecContext(feature.document)),
      references: references
    };
  }

  private static function encodeElementReference(value:ElementReference):Dynamic return {
    document: value.documentId.value,
    element: value.elementId.value
  };

  private static function decodeElementReference(value:Dynamic):ElementReference return new ElementReference(
    new DocumentId(stringField(value, "document")),
    new ElementId(stringField(value, "element"))
  );

  private static function encodePlacement(value:Placement):Dynamic {
    var p = value.location.plane;
    return {origin: encodeVector(p.origin), xDirection: encodeVector(p.xDirection), normal: encodeVector(p.normal)};
  }

  private static function decodePlacement(value:Dynamic):Placement return new Placement(new Plane(
    decodeVector(requiredField(value, "origin")),
    decodeVector(requiredField(value, "xDirection")),
    decodeVector(requiredField(value, "normal"))
  ));

  private static function encodeProperties(values:Array<TypedProperty>):Array<Dynamic> {
    var result:Array<Dynamic> = [];
    for (property in values) {
      var value:Dynamic = property.value;
      if (Std.isOfType(value, PersistentReference)) {
        var reference:PersistentReference = cast value;
        value = {document: reference.documentId, targetType: reference.targetType, targetId: reference.targetId};
      } else if (Std.isOfType(value, Placement)) {
        value = encodePlacement(cast value);
      }
      result.push({
        name: property.name,
        type: property.type,
        unit: property.unit,
        tokenDomain: property.tokenDomain,
        metadata: property.metadata,
        value: value
      });
    }
    return result;
  }

  private static function optionalPropertyRecords(record:Dynamic):Array<Dynamic> {
    var raw:Dynamic = Reflect.field(record, "properties");
    if (raw == null) return [];
    if (!Std.isOfType(raw, Array)) throw new ParametricError("document properties field is not an array");
    return cast raw;
  }

  private static function decodeTypedProperty(record:Dynamic, remapDocumentId:String -> String):TypedProperty {
    var name = stringField(record, "name");
    var type = stringField(record, "type");
    var unit = optionalString(record, "unit");
    var tokenDomain = optionalString(record, "tokenDomain");
    var metadata:Dynamic = Reflect.field(record, "metadata");
    if (!Reflect.hasField(record, "value")) throw new ParametricError("property record has no value field: " + name);
    var rawValue:Dynamic = Reflect.field(record, "value");
    var value:Dynamic = switch type {
      case TypedProperty.TypeBoolean:
        if (!Std.isOfType(rawValue, Bool)) throw new ParametricError("boolean property value is malformed: " + name);
        rawValue;
      case TypedProperty.TypeInteger:
        integerValue(cast rawValue, name);
      case TypedProperty.TypeText, TypedProperty.TypeToken:
        if (!Std.isOfType(rawValue, String)) throw new ParametricError("text property value is malformed: " + name);
        rawValue;
      case QuantityKind.Scalar, QuantityKind.Length, QuantityKind.Angle, QuantityKind.Area, QuantityKind.Volume:
        finiteNumber(rawValue, name);
      case TypedProperty.TypeElementReference,
        TypedProperty.TypeDefinitionReference, TypedProperty.TypeFeatureReference:
        var targetType = switch type {
          case TypedProperty.TypeElementReference:
            PersistentReference.ElementTarget;
          case TypedProperty.TypeDefinitionReference:
            PersistentReference.DefinitionTarget;
          default:
            PersistentReference.FeatureTarget;
        };
        if (stringField(rawValue, "targetType")
          != targetType) throw new ParametricError("property reference target type does not match its property type: "
          + name);
        new PersistentReference(
          remapDocumentId(stringField(rawValue, "document")),
          targetType,
          stringField(rawValue, "targetId")
        );
      case TypedProperty.TypePlacement:
        decodePlacement(rawValue);
      default:
        rawValue;
    };
    return new TypedProperty(name, type, value, unit, tokenDomain, metadata);
  }

  /**
		A record's name, if today's naming rules made it. A name from other rules could equal a different element's
		name now, so it is dropped: the reference resolves by geometry and takes its current name (TN-D13).
	*/
  private static function currentName(record:Dynamic):Null<String> {
    var name = optionalString(record, "name");
    var scheme:Dynamic = Reflect.field(record, "naming");
    if (name == null || scheme == null) return null;
    var schemeValue:Float = cast scheme;
    if (Std.int(schemeValue) != cadkit.Shape.namingScheme()) return null;
    return name;
  }

  private static function optionalString(record:Dynamic, name:String):Null<String> {
    var value:Dynamic = Reflect.field(record, name);
    if (value == null) return null;
    if (!Std.isOfType(value, String)) throw new ParametricError("document field is not a string: " + name);
    return cast value;
  }

  private static function encodeSelection(selection:Null<SelectionRecipe>):Dynamic {
    if (selection == null) return null;
    return {
      kind: selection.kind,
      geometry: selection.geometry,
      parallel: selection.parallel == null ? null : encodeVector(selection.parallel),
      position : selection.position,
      axis : encodeVector(selection.axis),
      expectedCount : selection.expectedCount,
      tolerance : selection.tolerance
    };
  }

  private static function optionalSelection(record:Dynamic):Null<SelectionRecipe> {
    var value:Dynamic = Reflect.field(record, "selection");
    return value == null ? null : decodeSelection(value);
  }

  private static function decodeSelection(record:Dynamic):SelectionRecipe {
    var parallel:Dynamic = Reflect.field(record, "parallel");
    return new SelectionRecipe(
      stringField(record, "kind"),
      stringField(record, "geometry"),
      parallel == null ? null : decodeVector(parallel),
      stringField(record, "position"),
      decodeVector(requiredField(record, "axis")),
      intField(record, "expectedCount"),
      numberField(record, "tolerance")
    );
  }

  private static function boolField(record:Dynamic, name:String):Bool {
    var value:Dynamic = requiredField(record, name);
    if (!Std.isOfType(value, Bool)) throw new ParametricError("document field is not a boolean: " + name);
    return cast value;
  }

  private static function optionalBool(record:Dynamic, name:String, fallback:Bool):Bool {
    var value:Dynamic = Reflect.field(record, name);
    if (value == null) return fallback;
    if (!Std.isOfType(value, Bool)) throw new ParametricError("document field is not a boolean: " + name);
    return cast value;
  }

  private static function encodeReference(reference:TopologyReference):Dynamic {
    return {kind: shapeKindName(reference.kind), fingerprint: encodeFingerprint(reference.fingerprintData())};
  }

  private static function containsReference(references:Array<TopologyReference>, candidate:TopologyReference):Bool {
    for (reference in references) if (reference == candidate) return true;
    return false;
  }

  private static function appendEdgeFingerprints(record:Dynamic, references:Array<TopologyReference>):Void {
    if (references.length == 0) return;
    var encoded:Array<Dynamic> = [];
    for (reference in references) encoded.push(encodeFingerprint(reference.fingerprintData()));
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
    if (value == null) return null;
    return decodeFingerprint(value, CadKit.ShapeKind.Face);
  }

  private static function optionalSupportFaceFingerprint(record:Dynamic):Null<TopologyFingerprint> {
    var value:Dynamic = Reflect.field(record, "supportFaceFingerprint");
    if (value == null) return null;
    return decodeFingerprint(value, CadKit.ShapeKind.Face);
  }

  private static function optionalEdgeFingerprints(record:Dynamic):Null<Array<TopologyFingerprint>> {
    var value:Dynamic = Reflect.field(record, "edges");
    if (value == null) return null;
    var records:Array<Dynamic> = cast value;
    if (records.length == 0) throw new ParametricError("document edge selection must not be empty");
    var result:Array<TopologyFingerprint> = [];
    for (fingerprint in records) result.push(decodeFingerprint(fingerprint, CadKit.ShapeKind.Edge));
    return result;
  }

  public static function encodeFingerprint(fingerprint:Null<TopologyFingerprint>):Dynamic {
    if (fingerprint == null) return null;
    var record:Dynamic = {
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
    // Where the edge position was taken.
    if (fingerprint.kind == CadKit.ShapeKind.Edge) Reflect.setField(
      record,
      "anchor",
      fingerprint.midpoint ? "midpoint" : "start"
    );
    // The element's topological name and the naming rules that made it (TN-D13).
    if (fingerprint.name != null) {
      Reflect.setField(record, "name", fingerprint.name);
      Reflect.setField(record, "naming", cadkit.Shape.namingScheme());
    }
    return record;
  }

  public static function decodeFingerprint(record:Dynamic, kind:CadKit.ShapeKind):TopologyFingerprint {
    if (kind == CadKit.ShapeKind.Edge && Reflect.field(record, "anchor") != "midpoint" && Reflect.field(
      record,
      "anchor"
    ) != "start") throw new ParametricError("Edge fingerprint requires a stated anchor");
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
      numberField(record, "measure"),
      Reflect.field(record, "anchor") == "midpoint",
      currentName(record)
    );
  }

  private static function encodeVector(value:Vector):Dynamic {
    return {x: value.x, y: value.y, z: value.z};
  }

  private static function decodeVector(value:Dynamic):Vector {
    return new Vector(numberField(value, "x"), numberField(value, "y"), numberField(value, "z"));
  }

  private static function requiredFeature(document:Document, featureId:Int):Feature {
    var feature = document.featureById(featureId);
    if (feature == null) throw new ParametricError("feature dependency is missing: " + featureId);
    return feature;
  }

  private static function requiredField(value:Dynamic, name:String):Dynamic {
    var result = Reflect.field(value, name);
    if (result == null) throw new ParametricError("document field is missing: " + name);
    return result;
  }

  private static function stringField(value:Dynamic, name:String):String {
    var result:Dynamic = requiredField(value, name);
    if (!Std.isOfType(result, String)) throw new ParametricError("document field is not a string: " + name);
    return cast result;
  }

  private static function numberField(value:Dynamic, name:String):Float {
    var result:Float = cast requiredField(value, name);
    if (!Math.isFinite(result)) throw new ParametricError("document field is not finite: " + name);
    return result;
  }

  private static function finiteNumber(value:Dynamic, name:String):Float {
    var result:Float = cast value;
    if (!Math.isFinite(result)) throw new ParametricError("document field is not finite: " + name);
    return result;
  }

  private static function intField(value:Dynamic, name:String):Int {
    var result = numberField(value, name);
    return integerValue(result, name);
  }

  private static function integerValue(result:Float, name:String):Int {
    if (!Math.isFinite(result)) throw new ParametricError("document field is not finite: " + name);
    var integer = Std.int(result);
    if (result != integer) throw new ParametricError("document field is not an integer: " + name);
    return integer;
  }

  private static function encodeBooleanOperation(operation:BooleanOperation):String {
    if (operation == BooleanOperation.Fuse) return "fuse";
    if (operation == BooleanOperation.Cut) return "cut";
    if (operation == BooleanOperation.Common) return "common";
    throw new ParametricError("unsupported boolean operation");
  }

  private static function booleanOperation(name:String):BooleanOperation {
    if (name == "fuse") return BooleanOperation.Fuse;
    if (name == "cut") return BooleanOperation.Cut;
    if (name == "common") return BooleanOperation.Common;
    throw new ParametricError("unsupported boolean operation: " + name);
  }

  private static function shapeKindName(kind:CadKit.ShapeKind):String {
    if (kind == CadKit.ShapeKind.Face) return "face";
    if (kind == CadKit.ShapeKind.Edge) return "edge";
    if (kind == CadKit.ShapeKind.Vertex) return "vertex";
    throw new ParametricError("unsupported topology reference kind");
  }

  private static function shapeKind(name:String):CadKit.ShapeKind {
    if (name == "face") return CadKit.ShapeKind.Face;
    if (name == "edge") return CadKit.ShapeKind.Edge;
    if (name == "vertex") return CadKit.ShapeKind.Vertex;
    throw new ParametricError("unsupported topology reference kind: " + name);
  }

  private static function surfaceKindName(kind:CadKit.SurfaceKind):String {
    if (kind == CadKit.SurfaceKind.Unknown) return "unknown";
    if (kind == CadKit.SurfaceKind.Plane) return "plane";
    if (kind == CadKit.SurfaceKind.Cylinder) return "cylinder";
    if (kind == CadKit.SurfaceKind.Cone) return "cone";
    if (kind == CadKit.SurfaceKind.Sphere) return "sphere";
    if (kind == CadKit.SurfaceKind.Torus) return "torus";
    if (kind == CadKit.SurfaceKind.Bezier) return "bezier";
    if (kind == CadKit.SurfaceKind.BSpline) return "bspline";
    throw new ParametricError("unsupported surface kind");
  }

  private static function surfaceKind(name:String):CadKit.SurfaceKind {
    if (name == "unknown") return CadKit.SurfaceKind.Unknown;
    if (name == "plane") return CadKit.SurfaceKind.Plane;
    if (name == "cylinder") return CadKit.SurfaceKind.Cylinder;
    if (name == "cone") return CadKit.SurfaceKind.Cone;
    if (name == "sphere") return CadKit.SurfaceKind.Sphere;
    if (name == "torus") return CadKit.SurfaceKind.Torus;
    if (name == "bezier") return CadKit.SurfaceKind.Bezier;
    if (name == "bspline") return CadKit.SurfaceKind.BSpline;
    throw new ParametricError("unsupported surface kind: " + name);
  }

  private static function curveKindName(kind:CadKit.CurveKind):String {
    if (kind == CadKit.CurveKind.Unknown) return "unknown";
    if (kind == CadKit.CurveKind.Line) return "line";
    if (kind == CadKit.CurveKind.Circle) return "circle";
    if (kind == CadKit.CurveKind.Ellipse) return "ellipse";
    if (kind == CadKit.CurveKind.Hyperbola) return "hyperbola";
    if (kind == CadKit.CurveKind.Parabola) return "parabola";
    if (kind == CadKit.CurveKind.Bezier) return "bezier";
    if (kind == CadKit.CurveKind.BSpline) return "bspline";
    if (kind == CadKit.CurveKind.Offset) return "offset";
    throw new ParametricError("unsupported curve kind");
  }

  private static function curveKind(name:String):CadKit.CurveKind {
    if (name == "unknown") return CadKit.CurveKind.Unknown;
    if (name == "line") return CadKit.CurveKind.Line;
    if (name == "circle") return CadKit.CurveKind.Circle;
    if (name == "ellipse") return CadKit.CurveKind.Ellipse;
    if (name == "hyperbola") return CadKit.CurveKind.Hyperbola;
    if (name == "parabola") return CadKit.CurveKind.Parabola;
    if (name == "bezier") return CadKit.CurveKind.Bezier;
    if (name == "bspline") return CadKit.CurveKind.BSpline;
    if (name == "offset") return CadKit.CurveKind.Offset;
    throw new ParametricError("unsupported curve kind: " + name);
  }
}
