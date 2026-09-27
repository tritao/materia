package machinekit.component;

import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** MachineKit helpers that consume parts, plus its joint-axis convention. */
class Solids {
	/** Fuse all parts and close each input. */
	public static function union(parts:Array<Part>):Part {
		var result:Part = null;
		try {
			result = Part.fuseAll(parts);
			for (part in parts) part.close();
			return result;
		} catch (error:Dynamic) {
			if (result != null) result.close();
			for (part in parts) if (part != null) part.close();
			throw error;
		}
	}

	/** Subtract all tools and close the base and tools. */
	public static function cut(base:Part, tools:Array<Part>):Part {
		if (tools.length == 0) return base;
		var result:Part = null;
		try {
			result = base.subtractAll(tools);
			base.close();
			for (tool in tools) tool.close();
			return result;
		} catch (error:Dynamic) {
			if (result != null) result.close();
			base.close();
			for (tool in tools) if (tool != null) tool.close();
			throw error;
		}
	}

	/** Frame at a point whose +Y joint axis points along CAD +Z. */
	public static function axial(x:Float, y:Float, z:Float):AssemblyFrame
		return AssemblyFrames.alongY(x, y, z, 0, 0, 1);
}
