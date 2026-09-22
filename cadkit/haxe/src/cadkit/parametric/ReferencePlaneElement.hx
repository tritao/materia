package cadkit.parametric;

import cadkit.modeling.Plane;

class ReferencePlaneElement extends Element {
	public var plane(default, null):Plane;
	public function new(document:Document,id:ElementId,name:String,plane:Plane) { super(document,id,name,"reference-plane"); this.plane=plane; }
	public function setPlane(value:Plane):Void document.setReferencePlane(this,value);
	public function restorePlane(value:Plane):Void { plane=value; document.datumChanged(this); }
}
