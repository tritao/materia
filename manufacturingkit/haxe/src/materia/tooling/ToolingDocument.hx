package materia.tooling;

import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.CastingSetup;

/** Authored manufacturing inputs, separate from sheet plans and inventory.
 * Schema version and format are supplied by ToolingCodec's outer envelope.
 */
typedef ToolingDocument = {
	var input:CastingToolingInput;
	var setup:CastingSetup;
	var recipe:ToolingRecipe;
}
