package cadkit.parametric.features;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.parametric.*;

class DatumSketchFeature extends Feature {
	public final profile:String; public final datum:ElementReference; public final offset:Parameter;
	public final width:Parameter; public final height:Parameter;
	public function new(profile:String,width:Float,height:Float,datum:ElementReference,offset:Float=0) { super();this.profile=profile;this.datum=datum;this.offset=new Parameter(this,"datum-sketch.offset",offset,-1e300,false,1e300,ParameterKind.Length);this.width=new Parameter(this,"datum-sketch.width",width,0,false,1e300,ParameterKind.Length);this.height=new Parameter(this,"datum-sketch.height",height,0,false,1e300,ParameterKind.Length); }
	override public function serializationType():String return "datum-sketch";
	override public function datumDependencies():Array<String> return [datum.elementId.value];
	override public function elementReferences():Array<ElementReference> return [datum];
	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var element=context.owner().resolveElement(datum); var plane:Plane;
		if(element.kind=="level") plane=new Plane(new Vector(0,0,context.owner().levelElevation(datum)+offset.value),Vector.X(),Vector.Z());
		else if(element.kind=="reference-plane") { var source:ReferencePlaneElement=cast element;plane=new Plane(source.plane.origin.add(source.plane.normal.scale(offset.value)),source.plane.xDirection,source.plane.normal); }
		else throw new ParametricError("datum sketch requires a level or reference-plane");
		var sketch=profile=="rectangle"?Sketch.rectangle(width.value,height.value,plane):profile=="circle"?Sketch.circle(width.value,plane):Sketch.slot(width.value,height.value,plane);
		var result=EvaluationResult.fromShape(sketch.shape.cloneShape());sketch.close();return result;
	}
}
