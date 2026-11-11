package cadkit.sheetmetal;

import CadKit;
import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyReference;

/**
 * Persistent edge selections and authored output identities shared by flange and hem features. An edge is chosen either by
 * picking its wall-side face (an editor pick, saved as a face fingerprint) or by name: the wall and the edge's identity in
 * that wall's outline, which code that records a document knows without evaluating it.
 */
class SheetEdgeFeature extends SheetFeature {
  public final source:SheetFeature;
  final references:Array<TopologyReference>;
  var pendingFaces:Null<Array<Shape>>;
  var pendingFingerprints:Null<Array<TopologyFingerprint>>;
  final named:Array<SheetSelectedEdge>;
  var outputWallIds:Array<Int>;
  var outputBendIds:Array<Int>;
  final restoreIdentities:Bool;

  public function new(
    source:SheetFeature,
    selectedFaces:Null<Array<Shape>>,
    ?savedFingerprints:Array<TopologyFingerprint>,
    ?savedWallIds:Array<Int>,
    ?savedBendIds:Array<Int>,
    ?namedEdges:Array<SheetSelectedEdge>
  ) {
    super();
    var sources = (selectedFaces == null ? 0 : 1) + (savedFingerprints == null ? 0 : 1) + (namedEdges == null ? 0 : 1);
    if (sources != 1) throw new ParametricError("sheet edge feature requires exactly one of side faces, saved face fingerprints or named edges");
    if ((selectedFaces != null && selectedFaces.length == 0) || (savedFingerprints != null
      && savedFingerprints.length == 0) || (namedEdges != null && namedEdges.length == 0)) throw new ParametricError("sheet edge selection must not be empty");
    var count:Int;
    if (selectedFaces != null) count = selectedFaces.length;
    else if (savedFingerprints != null) count = savedFingerprints.length;
    else if (namedEdges != null) count = namedEdges.length;
    else throw new ParametricError("sheet edge feature has no edge selection");
    named = namedEdges == null ? [] : [for (edge in namedEdges) {wall: edge.wall, identity: edge.identity}];
    if ((savedWallIds == null) != (savedBendIds
      == null)) throw new ParametricError("saved sheet edge identities must cover every selected edge");
    if (savedWallIds != null && savedWallIds.length
      != count) throw new ParametricError("saved sheet edge wall identities must cover every selected edge");
    if (savedBendIds != null && savedBendIds.length
      != count) throw new ParametricError("saved sheet edge bend identities must cover every selected edge");
    restoreIdentities = savedWallIds != null;
    outputWallIds = savedWallIds == null ? [] : savedWallIds.copy();
    outputBendIds = savedBendIds == null ? [] : savedBendIds.copy();
    this.source = source;
    references = [];
    pendingFaces = null;
    pendingFingerprints = savedFingerprints == null ? null : savedFingerprints.copy();
    if (selectedFaces != null) {
      var owned:Array<Shape> = [];
      try {
        for (face in selectedFaces) {
          if (face == null || face.kind()
            != CadKit.ShapeKind.Face) throw new ParametricError("sheet edge selection requires wall-side faces");
          owned.push(face.cloneShape());
        }
      } catch (error:Dynamic) {
        for (face in owned) face.close();
        throw error;
      }
      pendingFaces = owned;
    }
  }
  override public function onAttached():Void {
    if (pendingFaces != null) {
      for (face in pendingFaces) references.push(TopologyReference.fromShape(this, face, source));
      pendingFaces = null;
    } else if (pendingFingerprints != null) {
      for (fingerprint in pendingFingerprints) references.push(TopologyReference.fromFingerprint(
        this,
        CadKit.ShapeKind.Face,
        fingerprint,
        source
      ));
      pendingFingerprints = null;
    }
    var identities = identityLedger();
    var owner = "feature:" + id.toInt();
    if (restoreIdentities) {
      identities.claimWalls(owner, outputWallIds);
      identities.claimBends(owner, outputBendIds);
    } else {
      outputWallIds = identities.reserveWalls(owner, references.length + named.length);
      outputBendIds = identities.reserveBends(owner, references.length + named.length);
    }
  }
  override public function identityLedger():SheetIdentityLedger {
    return source.identityLedger();
  }
  public function wallIds():Array<Int> {
    return outputWallIds.copy();
  }
  public function bendIds():Array<Int> {
    return outputBendIds.copy();
  }
  public function faceReferences():Array<TopologyReference> {
    return references.copy();
  }
  /** The edges chosen by name; empty when they were picked as faces. */
  public function namedEdges():Array<SheetSelectedEdge> {
    return [for (edge in named) {wall: edge.wall, identity: edge.identity}];
  }
  override public function dependencies():Array<FeatureId> {
    return [source.id];
  }
  override public function dependencyFeatures():Array<Feature> {
    return [source];
  }
  public function selectedEdges(context:EvaluationContext, graph:Sheet):Array<SheetSelectedEdge> {
    if (named.length > 0) {
      var seen:Map<String, Bool> = new Map();
      for (edge in named) {
        var key = edge.wall + ":" + edge.identity;
        if (seen.exists(key)) throw new SheetDiagnosis("sheet.duplicate-edge-selection", "Sheet edge feature selects one edge more than once", [key]);
        seen.set(key, true);
        attachment(graph, edge);
      }
      return namedEdges();
    }
    var shape = context.shape(source);
    var selected:Array<SheetSelectedEdge> = [];
    var seen:Map<String, Bool> = new Map();
    for (reference in references) {
      var face = reference.resolveFor(shape);
      try {
        var edge = SheetEdgeSelection.identify(graph, face);
        var identity = graph.wall(edge.wall).outline.identity(edge.edge.loop, edge.edge.segment);
        var key = edge.wall + ":" + identity;
        if (seen.exists(key)) throw new SheetDiagnosis(
          "sheet.duplicate-edge-selection",
          "Sheet edge feature selects one edge more than once",
          [key]
        );
        seen.set(key, true);
        selected.push({wall: edge.wall, identity: identity});
        face.close();
      } catch (error:Dynamic) {
        face.close();
        throw error;
      }
    }
    return selected;
  }
  public static function attachment(graph:Sheet, selection:SheetSelectedEdge):WallEdge {
    var identities = graph.wall(selection.wall).outline.identities();
    var attachment:Null<WallEdge> = null;
    for (i in 0...identities.length) for (j in 0...identities[i].length) if (identities[i][j]
      == selection.identity) attachment = new WallEdge(
        j,
        i
      );
    if (attachment == null) throw new SheetDiagnosis(
      "sheet.edge-selection",
      "An earlier graph edit removed the selected edge",
      [selection.identity]
    );
    return attachment;
  }
  override public function onClose():Void {
    if (pendingFaces != null) for (face in pendingFaces) face.close();
    pendingFaces = null;
    pendingFingerprints = null;
    super.onClose();
  }
}

/** Selection roles survive graph edits independently of a contour's current indices. */
typedef SheetSelectedEdge = {
  var wall:Int;
  var identity:String;
}
