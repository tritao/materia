package machinekit.hobbing;

import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.FoundationSizing;
import machinekit.structural.InsertHoles;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertAnchorage.InsertAnchorSelection;
import materia.casting.InsertSpec.InsertRole;

typedef HobberSizedFoundation = {
	var body:ComponentValues;
	var placements:Array<CastInsertPlacement>;
}

/** Resolve source dimensions before realizing the final foundation. Hole limits
 * and demand are design rules; compound capacity remains a workshop input. */
class HobberFoundationSizing {
	static inline final LIFT_THREAD_MM=12;
	static inline final LIFT_MAXIMUM_WIDTH_MM=40;
	static inline final LIFT_DEPTH_MM=100;
	static inline final LIFT_THICKNESS_MM=30;
	static inline final LIFT_POINT_COUNT=4;
	static inline final COLUMN_LIFT_START_SIDE_MM=60;
	static inline final COLUMN_LIFT_X_MM=160;
	static inline final COLUMN_LIFT_Y_MM=60;
	static inline final COLUMN_LIFT_Y_STEP_MM=1;
	static inline final COLUMN_LIFT_MAXIMUM_Y_SHIFT_MM=20;

	public static function resolve(kind:String):HobberSizedFoundation {
		var body=FoundationLayout.bodyValues(kind);
		var rows=HobberMountingHoles.derive(kind,FoundationLayout.placements(kind));
		var rule=InsertAnchorSelection.sourceDemandRule("hobber-"+kind+"-source-anchors");
		var failures:Array<String>=[];
		for(size in rule.allowedNominalDiametersMm) {
			var single=FoundationSizing.singleThreadRule(rule,size);
			try {
				var placed=lifting(kind,body,rows,single);
				// Catalogue sockets retain their retention while their real mass
				// remains in the complete lifting-demand assembly.
				return {body:body,placements:FoundationSizing.anchors(placed,single,body)};
			} catch(error:Dynamic) {
				if(Std.isOfType(error,machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect))throw error;
				failures.push(size+" mm: "+Std.string(error));
			}
		}
		throw "No common source anchor thread fits "+kind+": "+failures.join("; ");
	}

	static function blank(width:Float,depth:Float):CastInsert {
		var choice=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(LIFT_THREAD_MM).mating());
		return new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(width,depth,LIFT_THICKNESS_MM),
			InsertHoles.tappedBlind(choice,LIFT_THICKNESS_MM))
			.setToken("role",LiftingMount).setToken("label","foundation lifting connection")
			.setToken("finish",materia.casting.InsertSpec.InsertFinish.AsSupplied));
	}

	static function lifting(kind:String,body:ComponentValues,rows:Array<CastInsertPlacement>,
		rule:materia.casting.InsertAnchorage.InsertAnchorPatternRule):Array<CastInsertPlacement> {
		var choice=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(LIFT_THREAD_MM).mating());
		var policy=machinekit.design.ThreadedHolePolicy.resolve(choice.thread,C45Bright);
		var minimumWidth=Std.int(Math.ceil(choice.thread.majorDiameter+2*policy.minWall));
		var existing=rows.filter(row->row.part.parameterToken("role")==LiftingMount);
		var result=rows.filter(row->row.part.parameterToken("role")!=LiftingMount);
		// Preserve each existing service centre and its exposed face. The policy
		// requires a thicker column blank; steel grows inward from that face.
		var massRows=result.concat([for(row in existing) {
			id:row.id,part:blank(LIFT_MAXIMUM_WIDTH_MM,LIFT_DEPTH_MM),openings:row.openings,
			pose:F.compose(row.pose,F.translation(0,0,row.part.boxDimensions(row.id).height-LIFT_THICKNESS_MM))
		}]);
		var mass=FoundationSizing.asCastMass(new MineralFoundation(body,massRows));
		if(mass.mass<=0 || !Math.isFinite(mass.mass) || mass.unaccounted.length>0)
			throw "Hobber lifting layout requires known source mass: "+kind;
		var dimensions=FoundationLayout.dimensions(kind);
		var sites=massRows.filter(row->row.part.parameterToken("role")==LiftingMount);
		var missing=LIFT_POINT_COUNT-sites.length;
		if(missing<0)throw "Unexpected source lifting count: "+kind;
		if(missing>0) {
			var proposed=InsertAnchorSelection.liftingPositions({x:-dimensions[0]/2,y:-dimensions[1]/2,z:0,
				width:dimensions[0],depth:dimensions[1],height:dimensions[2]},{x:mass.centreOfMass.x,y:mass.centreOfMass.y,z:mass.centreOfMass.z},LIFT_POINT_COUNT);
			var centreX=0.0,centreY=0.0;
			for(site in sites){centreX+=site.pose.x;centreY+=site.pose.y;}
			centreX/=sites.length;centreY/=sites.length;
			proposed.sort((a,b)->{
				var da=(a.x-centreX)*(a.x-centreX)+(a.y-centreY)*(a.y-centreY);
				var db=(b.x-centreX)*(b.x-centreX)+(b.y-centreY)*(b.y-centreY);
				var distance=Reflect.compare(db,da);
				return distance==0 ? Reflect.compare(a.y,b.y) : distance;
			});
			proposed=proposed.slice(0,missing);
			if(proposed.length!=missing)throw "No CoM-based source lifting positions: "+kind;
			for(index in 0...proposed.length) {
				var point=proposed[index];
				if(kind=="bed")point.y=(point.y<0 ? -1 : 1)*FoundationLayout.LIFT_CENTRE_Y;
				else {point.x=index==0 ? COLUMN_LIFT_X_MM : -COLUMN_LIFT_X_MM;point.y=COLUMN_LIFT_Y_MM;}
				Sys.println("Hobber source lifting proposal "+kind+": "+point.x+", "+point.y+", "+point.z);
				sites.push({id:"liftAdditional"+(index+1),part:kind=="column" ? blank(COLUMN_LIFT_START_SIDE_MM,COLUMN_LIFT_START_SIDE_MM)
						: blank(LIFT_MAXIMUM_WIDTH_MM,LIFT_DEPTH_MM),
					pose:F.translation(point.x,point.y,point.z-LIFT_THICKNESS_MM),
					openings:[materia.casting.CastingPatterns.face("body.z-max")]});
			}
		}
		var resin=new machinekit.structural.MineralCasting(body).geometry();
		var columnPair:Null<{width:Float,depth:Float,shift:Float,turn:Int}>=null;
		var additionalOptions:Null<Array<{width:Float,depth:Float}>>=null;
		try {
			var neighbours=FoundationSizing.anchors(result,rule,body);
			for(site in sites) {
				var chosen:CastInsertPlacement=null,chosenHardware:CastInsertPlacement=null,failures:Array<String>=[];
				var additional=kind=="column" && StringTools.startsWith(site.id,"liftAdditional");
				var stock=site.part.boxDimensions(site.id);
				var footprints:Array<{width:Float,depth:Float}> = [];
				if(additional) {
					if(additionalOptions==null) {
						var cover=rule.coverDiameters*rule.allowedNominalDiametersMm[0];
						var half=dimensions[0]/2-Math.abs(site.pose.x);
						var envelope=materia.casting.CastingEnvelope.build(dimensions[0],dimensions[1],dimensions[2]);
						var cuts=materia.casting.CastingPatterns.decode(body.token("features"));
						for(cavity in materia.casting.CastingCavities.derive(envelope,cuts))
							for(box in materia.casting.CastingCavities.boxes(cavity)) {
								if(box.z+box.height<=site.pose.z || box.z>=site.pose.z+stock.height)continue;
								if(box.y+box.depth<=site.pose.y-COLUMN_LIFT_MAXIMUM_Y_SHIFT_MM-stock.depth/2
									|| box.y>=site.pose.y+stock.depth/2)continue;
								if(box.x+box.width<=site.pose.x)half=Math.min(half,site.pose.x-box.x-box.width);
								else if(box.x>=site.pose.x)half=Math.min(half,box.x-site.pose.x);
							}
						var maximumWidth=Std.int(Math.floor(2*(half-cover)+1e-7));
						additionalOptions=[];
						for(width in minimumWidth...maximumWidth+1)for(depth in minimumWidth...Std.int(stock.depth)+1) {
							var prototype=blank(width,depth);
							try {
								machinekit.tooling.InsertAnchorGeometry.derive(site.id,prototype,rule,FootprintOnly,
									mass.mass*rule.gravityMps2/Math.min(rule.liftingLoadPaths,LIFT_POINT_COUNT));
								additionalOptions.push({width:width,depth:depth});
							} catch(error:Dynamic) {
								if(Std.isOfType(error,machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect))throw error;
							}
						}
						additionalOptions.sort((a,b)->a.width*a.depth!=b.width*b.depth
							? Reflect.compare(a.width*a.depth,b.width*b.depth) : Reflect.compare(a.width,b.width));
					}
					footprints=additionalOptions;
				} else for(width in minimumWidth...Std.int(stock.width)+1)footprints.push({width:width,depth:stock.depth});
				var shifts=additional ? Std.int(COLUMN_LIFT_MAXIMUM_Y_SHIFT_MM/COLUMN_LIFT_Y_STEP_MM)+1 : 1;
				for(step in 0...shifts) {
					var shift=step*COLUMN_LIFT_Y_STEP_MM;
					if(additional && columnPair!=null && shift!=columnPair.shift)continue;
					for(footprint in footprints) {
						var width=footprint.width,depth=footprint.depth;
						if(additional && columnPair!=null && (width!=columnPair.width || depth!=columnPair.depth))continue;
						for(turn in additional ? [0] : [0,1]) {
							if(additional && columnPair!=null && turn!=columnPair.turn)continue;
							var shifted=F.compose(site.pose,F.translation(0,-shift,0));
							var pose=turn==0 ? shifted : F.compose(shifted,
								F.fromRotationMatrix(0,0,0,[0.0,-1,0,1,0,0,0,0,1]));
							var candidate:CastInsertPlacement={id:site.id,part:blank(width,depth),pose:pose,openings:site.openings};
							try {
								machinekit.tooling.InsertAnchorGeometry.checkSourceClearance(candidate,neighbours,resin,dimensions,rule.coverDiameters*rule.allowedNominalDiametersMm[0]);
								var anchored=machinekit.tooling.InsertAnchorGeometry.derive(candidate.id,candidate.part,rule,
									Casting(resin,pose),mass.mass*rule.gravityMps2/Math.min(rule.liftingLoadPaths,LIFT_POINT_COUNT));
								chosen=candidate;chosenHardware={id:site.id,part:anchored,pose:pose,openings:site.openings};
								if(additional)columnPair={width:width,depth:depth,shift:shift,turn:turn};
								break;
							} catch(error:Dynamic) {
								if(Std.isOfType(error,machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect))throw error;
								failures.push(width+" x "+depth+" mm, turn "+turn+", Y shift "+shift+": "+Std.string(error));
							}
						}
						if(chosen!=null)break;
					}
					if(chosen!=null)break;
				}
				if(chosen==null)throw "No source lifting footprint fits "+kind+"/"+site.id+": "+failures.join("; ");
				result.push(chosen);neighbours.push(chosenHardware);
			}
		} catch(error:Dynamic) {resin.close();throw error;}
		resin.close();return result;
	}


}
