package cadkit.sketch;

/** One Jacobian row as (variable, entry) pairs; a variable may repeat and its entries then add. */
typedef SketchSparseRow = {index:Array<Int>, value:Array<Float>};
