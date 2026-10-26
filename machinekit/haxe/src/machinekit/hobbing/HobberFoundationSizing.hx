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
								checkBlank(candidate,neighbours,resin,dimensions,rule.coverDiameters*rule.allowedNominalDiametersMm[0]);
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

	/** Proud steel is permitted only beyond the declared casting face. Inside
 * the envelope it must remain in resin and clear all other source inserts. */
	static function checkBlank(candidate:CastInsertPlacement,others:Array<CastInsertPlacement>,
		resin:cadkit.modeling.Part,dimensions:Array<Float>,minimumClearance:Float):Void {
		// The hobber's sheet/plate frames are signed axis permutations. Their
		// box-to-box distance is exact, so reject insufficient source lands before
		// constructing native solids. Catalogue/socket geometry keeps the native path.
		function aligned(pose:materia.assembly.AssemblyRecord.AssemblyFrame):Bool {
			for(axis in [[1.0,0,0],[0.0,1,0],[0.0,0,1]]) {
				var v=F.transformVector(pose,axis[0],axis[1],axis[2]);
				for(value in [v.x,v.y,v.z])if(Math.abs(value)>1e-9 && Math.abs(Math.abs(value)-1)>1e-9)return false;
			}
			return true;
		}
		var ownGeometry:materia.casting.CastingFeature.CastingGeometry={solid:Insert(candidate.part.bodySpec()),pose:candidate.pose};
		if(aligned(candidate.pose))for(other in others) {
			var geometry:materia.casting.CastingFeature.CastingGeometry={solid:Insert(other.part.bodySpec()),pose:other.pose};
			if(!materia.casting.CastingFeature.CastingGeometrySolids.isBox(geometry.solid) || !aligned(other.pose))continue;
			var a=materia.casting.CastingRegionBounds.box(Placed(ownGeometry));
			var b=materia.casting.CastingRegionBounds.box(Placed(geometry));
			if(a==null || b==null)throw "Missing exact source plate bounds";
			var dx=Math.max(0,Math.max(a.x-b.x-b.width,b.x-a.x-a.width));
			var dy=Math.max(0,Math.max(a.y-b.y-b.depth,b.y-a.y-a.depth));
			var dz=Math.max(0,Math.max(a.z-b.z-b.height,b.z-a.z-a.height));
			if(Math.sqrt(dx*dx+dy*dy+dz*dz)+1e-7<minimumClearance)
				throw "Lifting blank lacks casting cover to source insert: "+candidate.id+" / "+other.id;
		}
		var owned:Array<cadkit.modeling.Part>=[];
		function own(part:cadkit.modeling.Part):cadkit.modeling.Part {owned.push(part);return part;}
		try {
			var local=own(machinekit.structural.InsertBodies.part(candidate.part.bodySpec()));
			var steel=own(machinekit.assembly.PosedParts.moved(local,candidate.pose));
			var envelope=materia.casting.CastingEnvelope.build(dimensions[0],dimensions[1],dimensions[2]);
			var feature:materia.casting.CastingFeature={id:"lifting-candidate",kind:InsertRecess,
				geometry:ownGeometry,openings:candidate.openings,draft:{degrees:0,axis:{x:0,y:0,z:1}},
				cornerRadius:0,owner:candidate.id,adjustable:[],derivedFrom:null};
			var cavity=materia.casting.CastingCavities.derive(envelope,[feature])[0];
			var embedded=own(machinekit.structural.CastingCavityCad.part(cavity));
			var missing=own(embedded.subtract(resin));
			if(missing.volume()>1e-6)throw "Lifting blank meets a source cavity: "+candidate.id;
			var faces=machinekit.tooling.InsertAnchorGeometry.planarBoundary(steel);
			for(other in others) {
				var spec=other.part.insertSpec();
				switch spec.anchorage {
					case Bolted(_,anchors):for(anchor in anchors) {
						var hole=spec.holes.filter(hole->hole.id==anchor.holeId)[0];
						var frame=F.compose(other.pose,materia.casting.InsertAnchorage.InsertAnchorageCodec.bearingFrame(hole,anchor));
						var screw=anchor.screw,axis=F.transformVector(frame,0,0,1);
						var origin=F.transformPoint(frame,0,0,0),flange=F.transformPoint(frame,0,0,-screw.flangeThicknessMm);
						var distance=materia.casting.InsertAnchorage.InsertAnchorScreen.inside(origin,faces) ? 0.0 : Math.POSITIVE_INFINITY;
						for(face in faces) {
							distance=Math.min(distance,materia.casting.InsertAnchorage.InsertAnchorScreen.cylinderFace(
								origin,axis,screw.thread.majorDiameter/2,screw.lengthMm,face));
							distance=Math.min(distance,materia.casting.InsertAnchorage.InsertAnchorScreen.cylinderFace(
								flange,axis,screw.flangeDiameterMm/2,screw.flangeThicknessMm,face));
							distance=Math.min(distance,materia.casting.InsertAnchorage.InsertAnchorScreen.hexFace(
								frame,screw.headAcrossFlatsMm,-screw.headHeightMm,-screw.flangeThicknessMm,face));
						}
						if(distance+1e-7<minimumClearance)
							throw "Lifting blank lacks casting cover to source anchor: "+candidate.id+" / "+other.id+"/"+anchor.id;
					}
					case Undeclared,CatalogueRetention:
				}
			}
			// Confirm the screened choice with the complete neighbouring steel,
			// including the actual anchor hardware, once all analytic checks pass.
			for(other in others) {
				var spec=other.part.insertSpec();
				var blanks=[own(machinekit.structural.InsertBodies.part(other.part.bodySpec()))];
				for(hardware in machinekit.tooling.InsertAnchorGeometry.components(spec)) {
					var local=own(hardware.part.geometry());local.shape.setThreadedFaces([]);
					blanks.push(own(machinekit.assembly.PosedParts.moved(local,hardware.pose)));
				}
				var blank=own(cadkit.modeling.Part.fuseAll(blanks));
				var placed=own(machinekit.assembly.PosedParts.moved(blank,other.pose));
				var common=own(steel.intersect(placed));
				if(common.volume()>1e-6 || steel.distance(placed)+1e-7<minimumClearance) {
					if(materia.casting.CastingFeature.CastingGeometrySolids.isBox(
						materia.casting.CastingFeature.CastingGeometrySolid.Insert(spec.body)) && aligned(other.pose))
						throw new machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect(
							"Lifting clearance screen disagrees with native steel/anchor geometry: "+candidate.id+" / "+other.id);
					throw "Lifting blank lacks casting cover to source insert: "+candidate.id+" / "+other.id;
				}
			}
		} catch(error:Dynamic) {for(part in owned)part.close();throw error;}
		for(part in owned)part.close();
	}
}
