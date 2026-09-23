package app;

import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.TopologyFingerprint;
import nativekit.scene.GeometryData;

/** Modeling-family adapter used by the editor's long-lived CAD document session. */
interface CadSessionModel {
  public function getDocument():Document;
  public function encode():String;
  public function close():Void;
  public function geometryFor(source:Shape):GeometryData;
  public function collisionBoundsFor(source:Shape):CadCollisionBounds;
  public function tessellationSeconds():Float;
  public function geometryConversionSeconds():Float;
  public function sceneDimensions():CadModelDimensions;
  public function setSceneDimensions(width:Float, height:Float, depth:Float):Void;
  public function featureNames():Array<String>;
  public function faceFingerprint(index:Int):TopologyFingerprint;
  public function remapFace(fingerprint:TopologyFingerprint):Int;
  public function exportStep(path:String):Void;
}
