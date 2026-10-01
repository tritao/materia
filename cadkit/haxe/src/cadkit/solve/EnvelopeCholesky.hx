package cadkit.solve;

/**
	Sparse symmetric positive-definite solves for constraint systems. A
	reverse Cuthill-McKee ordering keeps each row's nonzeros near the diagonal,
	so a matrix is stored by rows within its envelope (`envelope[row][column −
	first[row]]` for `first[row] <= column <= row`) and Cholesky fills nothing
	outside it. Chains and grids, the shape of most sketches, keep a narrow band.
*/
class EnvelopeCholesky {
	/**
		RCM over an adjacency list (symmetric, no self loops). Returns each
		vertex's row (`position`) and each row's first nonzero column (`first`).
	*/
	public static function order(neighbours:Array<Array<Int>>):{position:Array<Int>, first:Array<Int>} {
		var k = neighbours.length;
		var order:Array<Int> = [], visited = [for (_ in 0...k) false];
		while (order.length < k) {
			// Start each component from a vertex of least degree.
			var start = -1;
			for (v in 0...k)
				if (!visited[v] && (start < 0 || neighbours[v].length < neighbours[start].length))
					start = v;
			visited[start] = true;
			var head = order.length;
			order.push(start);
			while (head < order.length) {
				var v = order[head++];
				var next = [for (w in neighbours[v]) if (!visited[w]) w];
				next.sort((a, b) -> neighbours[a].length - neighbours[b].length);
				for (w in next) {
					visited[w] = true;
					order.push(w);
				}
			}
		}
		order.reverse();
		var position = [for (_ in 0...k) 0];
		for (row in 0...k)
			position[order[row]] = row;
		var first = [for (row in 0...k) row];
		for (v in 0...k)
			for (w in neighbours[v]) {
				var row = position[v], column = position[w];
				if (column < first[row])
					first[row] = column;
			}
		return {position: position, first: first};
	}

	/** A zero matrix with the given envelope. */
	public static function zero(first:Array<Int>):Array<Array<Float>>
		return [for (row in 0...first.length) [for (_ in first[row]...(row + 1)) 0.0]];

	/**
		In-place Cholesky; L keeps the envelope. False when a pivot is not
		positive. `pivots`, when given, receives each squared diagonal of L
		(the Schur-complement pivots), for rank tests.
	*/
	public static function factor(envelope:Array<Array<Float>>, first:Array<Int>, ?pivots:Array<Float>):Bool {
		for (row in 0...envelope.length) {
			var rowFirst = first[row];
			for (column in rowFirst...(row + 1)) {
				var sum = envelope[row][column - rowFirst];
				var from = rowFirst > first[column] ? rowFirst : first[column];
				for (k in from...column)
					sum -= envelope[row][k - rowFirst] * envelope[column][k - first[column]];
				if (column == row) {
					if (!(sum > 0))
						return false;
					if (pivots != null)
						pivots.push(sum);
					envelope[row][column - rowFirst] = Math.sqrt(sum);
				} else
					envelope[row][column - rowFirst] = sum / envelope[column][column - first[column]];
			}
		}
		return true;
	}

	/** Solves L Lᵀ y = b for a factored envelope, in its own (RCM) order. */
	public static function solve(envelope:Array<Array<Float>>, first:Array<Int>, b:Array<Float>):Array<Float> {
		var k = envelope.length, y = b.copy();
		for (row in 0...k) {
			var sum = y[row], rowFirst = first[row];
			for (column in rowFirst...row)
				sum -= envelope[row][column - rowFirst] * y[column];
			y[row] = sum / envelope[row][row - rowFirst];
		}
		var row = k - 1;
		while (row >= 0) {
			var rowFirst = first[row];
			y[row] /= envelope[row][row - rowFirst];
			for (column in rowFirst...row)
				y[column] -= envelope[row][column - rowFirst] * y[row];
			row--;
		}
		return y;
	}
}
