package app;

import CadKit;
import cadkit.Shape;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyResolver;

class CadModelTopology {
  public static function faceFingerprint(source:Shape, index:Int):TopologyFingerprint {
    var face=source.faces().at(index);
    try {
      var shape=face.cloneShape();
      try {
        var result=TopologyFingerprint.capture(shape);
        shape.close();
        face.close();
        return result;
      } catch(error:Dynamic) {
        shape.close();
        throw error;
      }
    } catch(error:Dynamic) {
      face.close();
      throw error;
    }
  }

  public static function remapFace(source:Shape, fingerprint:TopologyFingerprint):Int {
    var resolution=TopologyResolver.resolve(source,fingerprint,CadKit.ShapeKind.Face);
    return resolution.state==ReferenceState.Resolved?resolution.index:-1;
  }
}
