import cadkit.parametric.*;
import cadkit.parametric.features.*;

class TwoLevelDatumBuilding {
	public final document:Document; public final ground:LevelElement; public final upper:LevelElement;
	public final walls:Array<Element>; public final slab:Element; public final roof:Element;
	public function new(){document=new Document();ground=document.createLevel("Ground",0);upper=document.createLevel("Upper",3000);walls=[];var g=new ElementReference(document.id,ground.id);var u=new ElementReference(document.id,upper.id);
		for(i in 0...4){var p=document.add(new SketchFeature("rectangle",i<2?8000:200,i<2?200:5600));var wall=document.add(new LevelExtrudeFeature(p,g,u,200,0));walls.push(document.createElement("Wall "+(i+1),wall));}
		var sp=document.add(new SketchFeature("rectangle",8000,6000));var sf=document.add(new LevelExtrudeFeature(sp,g,g,0,200));slab=document.createElement("Slab",sf);
		var rp=document.add(new SketchFeature("rectangle",8200,6200));var rf=document.add(new LevelExtrudeFeature(rp,u,u,0,250));roof=document.createElement("Offset roof",rf);document.setOutput(rf);document.recompute();}
	public function setUpper(value:Float):Void{upper.setElevation(value);document.recompute();}
	public function close():Void document.close();
}
