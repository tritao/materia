package cadkit.parametric;
import cadkit.modeling.Plane;
class LevelElevationChange implements DocumentChange {
	final level:LevelElement; final before:Float; final after:Float;
	public function new(l,b,a){level=l;before=b;after=a;} public function undo():Void level.restoreElevation(before); public function redo():Void level.restoreElevation(after);
}
class ReferencePlaneChange implements DocumentChange {
	final datum:ReferencePlaneElement; final before:Plane; final after:Plane;
	public function new(d,b,a){datum=d;before=b;after=a;} public function undo():Void datum.restorePlane(before); public function redo():Void datum.restorePlane(after);
}
