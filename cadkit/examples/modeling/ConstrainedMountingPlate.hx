import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** A single constrained profile drives the plate outline and its four holes. */
class ConstrainedMountingPlate {
	public final document:Document;
	public var profile(default,null):ConstrainedSketchFeature;
	public var extrusion(default,null):ExtrudeFeature;
	public var finish(default,null):FilletFeature;

	public function new() {
		document=DocumentBuilder.build(function(builder){
			var width=builder.dimension("plate.width",80),height=builder.dimension("plate.height",50);
			var clearance=builder.dimension("holes.clearance",10),radius=builder.dimension("holes.radius",3);
			var thickness=builder.dimension("plate.thickness",6),fillet=builder.dimension("fillet.radius",2);
			var dimensions:Map<String,cadkit.parametric.NamedParameter> = new Map();
			dimensions.set("width",width);dimensions.set("height",height);dimensions.set("clearance.x",clearance);dimensions.set("clearance.y",clearance);dimensions.set("hole.radius",radius);
			profile=builder.constrainedSketch(makeSketch(width.value,height.value,clearance.value,radius.value),dimensions);
			extrusion=builder.extrude(profile,thickness);
			finish=builder.fillet(extrusion,fillet,new SelectionRecipe("edge","line",Vector.Z(),"ends",Vector.X(),4));
			builder.output(finish);
		});
	}

	private static function makeSketch(width:Float,height:Float,clearance:Float,radius:Float):ConstrainedSketch {
		var s=new ConstrainedSketch(),hw=width/2,hh=height/2;
		for(p in [new SketchPoint("q0",-hw,-hh),new SketchPoint("q1",hw,-hh),new SketchPoint("q2",hw,hh),new SketchPoint("q3",-hw,hh),
			new SketchPoint("xa",-1,0),new SketchPoint("xb",1,0),new SketchPoint("ya",0,-1),new SketchPoint("yb",0,1),
			new SketchPoint("bl",-hw+clearance,-hh+clearance),new SketchPoint("br",hw-clearance,-hh+clearance),new SketchPoint("tl",-hw+clearance,hh-clearance),new SketchPoint("tr",hw-clearance,hh-clearance),
			new SketchPoint("leftAnchor",-hw,-hh+clearance),new SketchPoint("bottomAnchor",-hw+clearance,-hh)])s.addPoint(p);
		s.addEntity(SketchEntity.line("bottom","q0","q1")).addEntity(SketchEntity.line("right","q1","q2"))
			.addEntity(SketchEntity.line("top","q2","q3")).addEntity(SketchEntity.line("left","q3","q0"))
			.addEntity(SketchEntity.line("xaxis","xa","xb",true)).addEntity(SketchEntity.line("yaxis","ya","yb",true))
			.addEntity(SketchEntity.line("clearanceHorizontal","leftAnchor","bl",true)).addEntity(SketchEntity.line("clearanceVertical","bottomAnchor","bl",true));
		for(id in ["bl","br","tl","tr"])s.addEntity(SketchEntity.circle("hole."+id,id,radius));
		for(id in ["xa","xb","ya","yb"])s.addConstraint(SketchConstraint.fixed("fixed."+id,id));
		s.addConstraint(SketchConstraint.horizontal("bottom.horizontal","bottom")).addConstraint(SketchConstraint.horizontal("top.horizontal","top"))
			.addConstraint(SketchConstraint.vertical("left.vertical","left")).addConstraint(SketchConstraint.vertical("right.vertical","right"))
			.addConstraint(SketchConstraint.distance("width","q0","q1",width)).addConstraint(SketchConstraint.distance("height","q1","q2",height))
			.addConstraint(SketchConstraint.symmetric("bottom.centered","q0","q1","yaxis")).addConstraint(SketchConstraint.symmetric("right.centered","q1","q2","xaxis"));
		s.addConstraint(SketchConstraint.pointOn("left.anchor","leftAnchor","left")).addConstraint(SketchConstraint.horizontal("clearance.horizontal","clearanceHorizontal"))
			.addConstraint(SketchConstraint.distance("clearance.x","leftAnchor","bl",clearance))
			.addConstraint(SketchConstraint.pointOn("bottom.anchor","bottomAnchor","bottom")).addConstraint(SketchConstraint.vertical("clearance.vertical","clearanceVertical"))
			.addConstraint(SketchConstraint.distance("clearance.y","bottomAnchor","bl",clearance))
			.addConstraint(SketchConstraint.symmetric("holes.bottom","bl","br","yaxis")).addConstraint(SketchConstraint.symmetric("holes.left","bl","tl","xaxis"))
			.addConstraint(SketchConstraint.symmetric("holes.top","tl","tr","yaxis"));
		s.addConstraint(SketchConstraint.radius("hole.radius","hole.bl",radius));
		for(id in ["br","tl","tr"])s.addConstraint(SketchConstraint.equal("hole.equal."+id,"hole.bl","hole."+id));
		return s;
	}

	/** Grouped edit; validation or recompute failure restores parameters and the last solid. */
	public function resize(width:Float,height:Float,clearance:Float,radius:Float):Void {
		if(width<=2*(clearance+radius)||height<=2*(clearance+radius))throw new ParametricError("constraints width, height, clearance.x, clearance.y, and hole.radius conflict");
		var transaction=document.beginTransaction();
		try {document.parameter("plate.width").set(width);document.parameter("plate.height").set(height);document.parameter("holes.clearance").set(clearance);document.parameter("holes.radius").set(radius);document.recompute();}
		catch(error:Dynamic){transaction.cancel();throw error;}transaction.commit();
	}

	public function close():Void document.close();
}
