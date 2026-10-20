package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Cross slide with a bottom Z datum and optional through-Y nut tunnel. */
class MillCrossSaddle extends MachineComponent {
	public static inline var BLOCK_SEAT:Float = 12;
	public static inline var BLOCK_SCREW_LENGTH:Float = 16;
	public static inline var WRENCH_HEIGHT:Float = 34;
	static var recipe:Null<ComponentType>;

	public static function parameters():Array<ComponentParameter> return [
		P.length("width", 450),
		P.length("depth", 174),
		P.length("height", 80),
		P.flag("ribbed", false), P.flag("blockAccess",false), P.flag("nutBoss",false), P.flag("nutServiceAccess",false), P.length("guideSpacing",220), P.length("blockSpan",50.7), P.length("blockHolePitchB",26),P.length("blockHolePitchC",26), P.length("blockClearance",4.5),
		P.length("crossRailSpacing",160), P.length("crossRailLength",450), P.length("crossRailWidth",15),
		P.length("bore", 54), P.length("boreZ", 40), P.flag("motorPocket", true), P.flag("sidePockets", false)
	];

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.cross-saddle", parameters(), v -> new MillCrossSaddle(v));
		return recipe;
	}

	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values);
		if (v.number("bore") + 20 > v.number("height") || v.number("bore") >= v.number("width")) throw "Saddle tunnel leaves insufficient material";
		if ((v.boolean("sidePockets") || v.boolean("ribbed")) && (v.number("width") < 450 || v.number("depth") < 174 || v.number("height") < 56))
			throw "Saddle side pockets need continuous chords and end webs";
		if (v.boolean("ribbed") && (v.number("guideSpacing")<160 || v.number("guideSpacing")>v.number("width")-80 ||
			v.number("blockHolePitchC")<=0 || v.number("blockHolePitchC")>v.number("guideSpacing")/2 || v.number("blockHolePitchB")<=0 ||
			v.number("blockSpan")<=v.number("blockHolePitchB")/2+17)) throw "Ribbed saddle cannot retain its guide pads and bolt wells";
		if (v.boolean("blockAccess") && (!v.boolean("ribbed") || v.boolean("sidePockets") ||
			v.number("blockSpan")+v.number("blockHolePitchB")/2+10>=v.number("depth")/2 ||
			v.number("guideSpacing")/2-(v.number("blockHolePitchC")+20)/2-12<=45))
			throw "Block access pockets need ribbed guide pads, end clearance and retained central webs";
		if(v.boolean("nutBoss") && (!v.boolean("ribbed") || !v.boolean("blockAccess") || v.number("height")<72 ||
			v.number("boreZ")!=0 || v.number("guideSpacing")/2-(v.number("blockHolePitchC")+20)/2<MillYNutMount.BOSS_WIDTH/2+12))
			throw "Nut boss needs the underside screw datum and separated guide service webs";
		if(v.boolean("nutServiceAccess") && !v.boolean("nutBoss")) throw "Nut service notch requires reinforced nut boss";
		super("MILL-CROSS-SADDLE", "Machined cross saddle", "cast iron", false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Face, Solids.axial(0, 0, v.number("height")));
		if(v.boolean("nutServiceAccess")) addFacet(makeInterfaces());
	}

	function makeInterfaces():machinekit.design.DesignInterfaces {
		var holes:Array<machinekit.design.DesignInterfaces.HoleInterface> = [], d=parameterNumber("depth"), h=parameterNumber("height");
		for(p in MillYNutMount.anchors()) holes.push({id:'anchor${p.x}-${p.y}',frame:F.fromRotationMatrix(p.x,-d/2,p.y,[1,0,0,0,0,1,0,-1,0]),radius:2.5,depth:24.5,minWall:5,minClearance:0.4,counterbore:null,approach:null,
			thread:{spec:machinekit.standard.FastenerThreads.coarse(6).mating(),tapDrill:5,usableDepth:22,cylindricalDepth:23,point:1.5,stock:MillYNutMount.BOSS_DEPTH,minEngagement:9,minTipGap:2,minBack:3}});
		var material:Array<machinekit.design.DesignRequirement.DesignEnvelope> = [];
		for(x in [-16.0,16.0]) material.push({id:'notch-cheek$x',frame:F.translation(x,-d/2+8,36),volume:BoxVolume(10,16,h-40)});
		for(side in [-1,1]) material.push({id:'cross-rail-land$side',frame:F.translation(0,side*parameterNumber("crossRailSpacing")/2,h-3),volume:BoxVolume(parameterNumber("crossRailLength"),parameterNumber("crossRailWidth"),3)});
		return new machinekit.design.DesignInterfaces(holes,[],material);
	}
	override public function componentType():Null<ComponentType> return recipeType();

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var guide = parameterNumber("guideSpacing")/2;
		var w = parameterNumber("width"), d = parameterNumber("depth"), h = parameterNumber("height");
		var padHalf=Math.max(20,(parameterNumber("blockHolePitchC")+parameterNumber("blockClearance")+8)/2);
		var tools:Array<Part> = [];
		if (parameterNumber("bore") > 0) tools.push(Part.cylinderAlongY(parameterNumber("bore") / 2, -d / 2 - 1, d / 2 + 1, 0, parameterNumber("boreZ")));
		for (x in [-35.0, 35.0]) for (z in [-20.0, 20.0]) tools.push(Part.cylinderAlongY(2.5, d / 2 - 16, d / 2 + 1, x, h / 2 + z));
		if (values().boolean("motorPocket")) {
			var bore = Part.cylinderSpan(33, -226, -64);
			tools.push(PosedParts.moved(bore, F.fromRotationMatrix(0, 0, 40, [0,0,1, 1,0,0, 0,1,0]))); bore.close();
			tools.push(Part.box(12, 84, 82).translated(new Vector(-201, 0, -1)));
			tools.push(Part.box(17, 40, 42).translated(new Vector(-218.5, 0, 40)));
			for (y in [-32.0, 32.0]) for (z in [10.0, 70.0]) {
				var hole = Part.cylinderSpan(2.5, -196, -182);
				tools.push(PosedParts.moved(hole, F.fromRotationMatrix(0, y, z, [0,0,1, 1,0,0, 0,1,0]))); hole.close();
			}
		}
		for (x in [-32.0, 32.0]) tools.push(Part.cylinderAlongY(2.5, -d / 2 - 1, -d / 2 + 16, x, 12));
		// Shallow side pockets retain continuous top/bottom chords, end webs and the central nut attachment.
		if (values().boolean("sidePockets")) for (side in [-1, 1])
			for (interval in [[-205.0,-115.0],[-105.0,-55.0],[55.0,130.0],[140.0,205.0]])
				tools.push(Part.box(interval[1]-interval[0], 13, h-28)
					.translated(new Vector((interval[0]+interval[1])/2, side*(d/2-5.5), 14)));
		if (values().boolean("ribbed")) {
			// Side-entry service pockets are centered on the real block pattern.
			if (values().boolean("blockAccess")) {
				var pocketWidth=parameterNumber("blockHolePitchC")+20;
				var inner=parameterNumber("blockSpan")-parameterNumber("blockHolePitchB")/2-10;
				for (sx in [-1,1]) for (sy in [-1,1])
					tools.push(accessPocket(sx*guide,pocketWidth,sy<0?-d/2-1:inner,sy<0?-inner:d/2+1,h));
				// Separate lightening cells leave full-height webs beside each guide pad.
				for (side in [-1,1]) for (interval in [[-w/2+40,-guide-pocketWidth/2-12],[-guide+pocketWidth/2+12,-45.0],
					[45.0,guide-pocketWidth/2-12],[guide+pocketWidth/2+12,w/2-20]])
					if (interval[1]>interval[0]) tools.push(Part.box(interval[1]-interval[0],d/2-28,h-28)
						.translated(new Vector((interval[0]+interval[1])/2,side*(d/4+14),14)));
			} else
			// Four bottom guide pads, continuous X guide chords, transverse end
			// webs and the central nut spine remain joined by a 14 mm sole.
			for (side in [-1,1]) for (interval in [[-w/2+40,-guide-padHalf],[-guide+padHalf,-45.0],[45.0,guide-padHalf],[guide+padHalf,w/2-20]])
				tools.push(Part.box(interval[1]-interval[0],d/2-28,h-28)
					.translated(new Vector((interval[0]+interval[1])/2,side*(d/4+14),14)));
			for (interval in [[-w/2-1,-guide-padHalf],[-guide+padHalf,-45.0],[45.0,guide-padHalf],[guide+padHalf,w/2+1]])
				tools.push(Part.box(interval[1]-interval[0],d+2,5).translated(new Vector((interval[0]+interval[1])/2,0,-1)));
			for (x in [-guide,guide]) tools.push(Part.box(2*padHalf,24,5).translated(new Vector(x,0,-1)));
			for (sx in [-1,1]) for (sy in [-1,1])
				for (dx in [-parameterNumber("blockHolePitchC")/2,parameterNumber("blockHolePitchC")/2])
				for (dy in [-parameterNumber("blockHolePitchB")/2,parameterNumber("blockHolePitchB")/2]) {
				var x = sx*parameterNumber("guideSpacing")/2+dx, y = sy*parameterNumber("blockSpan")+dy;
				tools.push(Part.cylinderSpan(parameterNumber("blockClearance")/2,-1,values().boolean("blockAccess")?BLOCK_SEAT+1:h+1,x,y));
				if (!values().boolean("blockAccess")) tools.push(Part.cylinderSpan(5,12,h+1,x,y));
			}
		}
		var body=Part.box(w,d,h);
		if (values().boolean("nutBoss")) {
			var pieces=[body,Part.box(MillYNutMount.BOSS_WIDTH,MillYNutMount.BOSS_DEPTH,h)
				.translated(new Vector(0,-d/2+MillYNutMount.BOSS_DEPTH/2,0))];
			for(sx in [-1,1]) for(sy in [-1,1]) pieces.push(Part.prism([
				new Vector(sx*46,sy*d/2),new Vector(sx*58,sy*d/2),
				new Vector(sx*(guide-(parameterNumber("blockHolePitchC")+20)/2),sy*40),
				new Vector(sx*(guide-(parameterNumber("blockHolePitchC")+20)/2-12),sy*40)],BLOCK_SEAT,h-14));
			// Add the load-transfer ribs after lightening cuts, then retain all interfaces.
			var relieved=Solids.cut(body,tools); pieces[0]=relieved;
			body=Solids.union(pieces); tools=[];
			tools.push(Part.cylinderAlongY(parameterNumber("bore")/2,-d/2-1,d/2+1,0,parameterNumber("boreZ")));
			if(values().boolean("nutServiceAccess")) for(tool in machinekit.design.DesignInterfaces.of(this).drillingTools()) tools.push(tool);
			else for(p in MillYNutMount.anchors()) tools.push(Part.cylinderAlongY(2.5,-d/2-1,-d/2+22,p.x,p.y));
			for(p in MillYNutMount.pilots()) {
				if(p.x<0) tools.push(Part.cylinderAlongY(3.05,-d/2-1,-d/2+10,p.x,p.y));
				else tools.push(Solids.union([Part.cylinderAlongY(3.05,-d/2-1,-d/2+10,p.x-0.2,p.y),
					Part.cylinderAlongY(3.05,-d/2-1,-d/2+10,p.x+0.2,p.y),
					Part.box(0.4,11,6.1).translated(new Vector(p.x,-d/2+4.5,p.y-3.05))]));
			}
			if (values().boolean("nutServiceAccess")) {
				tools.push(Part.box(20,2,h-23).translated(new Vector(0,-d/2,24)));
				tools.push(Part.cylinderSpan(10,24,h+1,0,-d/2+1));
			} else tools.push(Part.cylinderSpan(5,24,h+1,0,-d/2+5));
		}
		var raw=Solids.cut(body,tools);
		if (!values().boolean("blockAccess") && !values().boolean("nutBoss")) return raw;
		try { var result=raw.refined(); raw.close(); return result; } catch(e:Dynamic) { raw.close(); throw e; }
	}
	/** Rounded XZ profile extruded from the side face to the inner bolt row. */
	static function accessPocket(x:Float,width:Float,y0:Float,y1:Float,height:Float):Part {
		var floor=BLOCK_SEAT, top=height-14, radius=4.0, depth=y1-y0;
		var parts=[Part.box(width-2*radius,depth,top-floor).translated(new Vector(x,(y0+y1)/2,floor)),
			Part.box(width,depth,top-floor-2*radius).translated(new Vector(x,(y0+y1)/2,floor+radius))];
		for (sx in [-1,1]) for (z in [floor+radius,top-radius])
			parts.push(Part.cylinderAlongY(radius,y0,y1,x+sx*(width/2-radius),z));
		return Solids.union(parts);
	}
}
