package cadkit.parametric.features;
import cadkit.Geometry;
import cadkit.parametric.*;

class LevelExtrudeFeature extends Feature {
	public final source:Feature; public final base:ElementReference; public final top:ElementReference;
	public final baseOffset:Parameter; public final topOffset:Parameter;
	public function new(source,base,top,baseOffset=0.0,topOffset=0.0){super();this.source=source;this.base=base;this.top=top;this.baseOffset=new Parameter(this,"level-extrude.baseOffset",baseOffset,-1e300,false,1e300,ParameterKind.Length);this.topOffset=new Parameter(this,"level-extrude.topOffset",topOffset,-1e300,false,1e300,ParameterKind.Length);}
	override public function serializationType():String return "level-extrude";
	override public function dependencies():Array<FeatureId> return [source.id];
	override public function dependencyFeatures():Array<Feature> return [source];
	override public function datumDependencies():Array<String> return [base.elementId.value,top.elementId.value];
	override public function elementReferences():Array<ElementReference> return [base, top];
	override public function evaluate(context:EvaluationContext):EvaluationResult { var low=context.owner().levelElevation(base)+baseOffset.value;var high=context.owner().levelElevation(top)+topOffset.value;if(high<=low)throw new ParametricError("top level must be above base level");var moved=context.shape(source).translate(Geometry.vec3(0,0,low));try{var result=EvaluationResult.fromOperation(moved.extrudeOperation(Geometry.vec3(0,0,high-low)));moved.close();return result;}catch(e:Dynamic){moved.close();throw e;} }
}
