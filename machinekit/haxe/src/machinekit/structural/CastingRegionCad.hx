package machinekit.structural;

import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Model;
import cadkit.modeling.Scope;
import materia.casting.CastingRegion;
import materia.casting.CastingRegion.CastingRegionCodec;

/** Native evaluation of neutral finished-region expressions. All temporaries
 * close on success and failure; zero-volume contacts are not solid material.
 */
class CastingRegionCad {
	public static function part(region:CastingRegion):Part {
		var expression=CastingRegionCodec.snapshot(region),scope=new Scope();
		try {
			var result=build(scope,expression);
			scope.release(result);
			scope.close();
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	static function build(scope:Scope,expression:CastingRegion):Part {
		return switch expression {
			case Placed(geometry):
				var local=scope.own(switch geometry.solid {
					case Cut(solid): CastingSolids.part(solid);
					case Prism(profile,height): CastingProfileCad.prism(profile,0,height);
					case Insert(body): InsertBodies.part(body);
				});
				scope.own(machinekit.assembly.PosedParts.moved(local,geometry.pose));
			case Drafted(id,opening,pose,depth,degrees):
				scope.own(CastingProfileCad.drafted(materia.casting.CastingDraftSections.sections(id,opening,pose,depth,degrees)));
			case Union(children):
				var operands:Array<Part> =[];
				for (child in children) {
					var operand=CastingRegionCad.build(scope,child);
					if (!operand.isEmpty()) operands.push(operand);
				}
				scope.own(operands.length==0 ? new Part(Shape.compound([])) : Part.fuseAll(operands));
			case Intersection(children):
				var result=CastingRegionCad.build(scope,children[0]);
				for (index in 1...children.length) {
					if (result.isEmpty()) break;
					var tool=CastingRegionCad.build(scope,children[index]);
					var common=scope.own(new Model(result.shape.common(tool.shape)));
					var solids:Array<Shape> =[];
					for (solidIndex in 0...common.shape.subshapeCount(CadKit.ShapeKind.Solid))
						solids.push(scope.own(new Model(common.shape.subshape(CadKit.ShapeKind.Solid,solidIndex))).shape);
					result=scope.own(new Part(Shape.compound(solids)));
				}
				result;
			case Difference(body,cuts):
				var source=CastingRegionCad.build(scope,body),tools:Array<Part> =[];
				for (cut in cuts) {
					var tool=CastingRegionCad.build(scope,cut);
					if (!tool.isEmpty()) tools.push(tool);
				}
				scope.own(source.isEmpty() || tools.length==0 ? new Part(source.shape.cloneShape()) : source.subtractAll(tools));
		};
	}
}
