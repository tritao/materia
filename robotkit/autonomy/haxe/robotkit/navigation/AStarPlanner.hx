package robotkit.navigation;

import robotkit.mobile.Pose2;

/**
 * Deterministic 8-connected A* over a Costmap2.
 *
 * The robot is wherever it is, so the start cell is always accepted. A start
 * inside a blocked but not lethal cell (the costmap's discretization margin,
 * where the robot can stand without touching anything) plans an escape: the
 * route may pass through further blocked, non-lethal cells, at a penalty, only
 * until it first reaches a traversable cell, and never re-enters blocked cells
 * after that. A start in a traversable cell plans exactly as if escapes did not
 * exist.
 */
class AStarPlanner implements Planner {
  public final costmap:Costmap2;

  static final SQRT_TWO:Float = 1.4142135623730951;
  static final DIRECTIONS:Array<Array<Int>> = [
    [-1, -1], [0, -1], [1, -1], [-1, 0], [1, 0], [-1, 1], [0, 1], [1, 1]
  ];

  public function new(costmap:Costmap2) {
    if (costmap == null) throw "AStarPlanner requires a costmap";
    this.costmap = costmap;
  }

  public function plan(start:Pose2, goal:Pose2):Path {
    if (start == null || goal == null) throw "AStarPlanner requires start and goal poses";
    var startCell = costmap.grid.worldToCell(start);
    var goalCell = costmap.grid.worldToCell(goal);
    if (startCell == null) throw "AStarPlanner start lies outside the costmap";
    if (goalCell == null) throw "AStarPlanner goal lies outside the costmap";
    var startIndex = index(cast startCell);
    var goalIndex = index(cast goalCell);
    if (!costmap.isTraversable(goalCell.x, goalCell.y))
      throw "AStarPlanner goal lies in a blocked cell";

    var count = costmap.grid.width * costmap.grid.height;
    var scores:Array<Float> = [for (_ in 0...count) 1.0e300];
    var parents:Array<Int> = [for (_ in 0...count) -1];
    var closed:Array<Bool> = [for (_ in 0...count) false];
    var open:Array<OpenNode> = [];
    scores[startIndex] = 0.0;
    push(open, new OpenNode(startIndex, 0.0,
      heuristic(startCell.x, startCell.y, goalCell.x, goalCell.y)));

    var escapeCost = Math.max(10.0, 2.0 * costmap.inflationCostWeight);
    var found = false;
    while (open.length > 0) {
      var current = pop(open);
      if (closed[current.index] || current.score != scores[current.index])
        continue;
      if (current.index == goalIndex) {
        found = true;
        break;
      }
      closed[current.index] = true;
      var currentX = current.index % costmap.grid.width;
      var currentY = Std.int(current.index / costmap.grid.width);
      // Only a cell still inside the start's blocked margin may step into
      // other blocked cells, and never into a lethal one.
      var escaping = !costmap.isTraversable(currentX, currentY);
      for (direction in DIRECTIONS) {
        var nextX = currentX + direction[0];
        var nextY = currentY + direction[1];
        if (!enterable(nextX, nextY, escaping)) continue;
        var diagonal = direction[0] != 0 && direction[1] != 0;
        if (diagonal && (!enterable(currentX + direction[0], currentY, escaping) ||
            !enterable(currentX, currentY + direction[1], escaping)))
          continue;
        var nextIndex = nextY * costmap.grid.width + nextX;
        if (closed[nextIndex]) continue;
        var step = costmap.grid.resolutionMeters * (diagonal ? SQRT_TWO : 1.0);
        var cellCost = costmap.isTraversable(nextX, nextY)
          ? costmap.cellCost(nextX, nextY) : escapeCost;
        var candidate = scores[current.index] + step * (1.0 + cellCost);
        if (candidate < scores[nextIndex]) {
          scores[nextIndex] = candidate;
          parents[nextIndex] = current.index;
          var estimate = heuristic(nextX, nextY, goalCell.x, goalCell.y) *
            costmap.grid.resolutionMeters;
          push(open, new OpenNode(nextIndex, candidate, candidate + estimate));
        }
      }
    }
    if (!found) throw "AStarPlanner could not find a traversable route";
    return buildPath(start, goal, reconstruct(parents, startIndex, goalIndex));
  }

  function index(cell:GridCell2):Int return cell.y * costmap.grid.width + cell.x;

  function enterable(x:Int, y:Int, escaping:Bool):Bool
    return costmap.isTraversable(x, y) || (escaping && !costmap.isLethal(x, y));

  function heuristic(x:Int, y:Int, goalX:Int, goalY:Int):Float {
    var dx = Math.abs(goalX - x);
    var dy = Math.abs(goalY - y);
    var diagonal = Math.min(dx, dy);
    return diagonal * SQRT_TWO + Math.max(dx, dy) - diagonal;
  }

  function reconstruct(parents:Array<Int>, startIndex:Int,
      goalIndex:Int):Array<Int> {
    var reversed = [goalIndex];
    var current = goalIndex;
    while (current != startIndex) {
      current = parents[current];
      if (current < 0) throw "AStarPlanner route reconstruction failed";
      reversed.push(current);
    }
    reversed.reverse();
    return reversed;
  }

  function buildPath(start:Pose2, goal:Pose2, cells:Array<Int>):Path {
    var positions:Array<Pose2> = [start];
    // The exact start pose is already inside its start cell. Inserting that
    // cell's center can add a short backwards segment when the robot starts off
    // center, so only append the subsequent cell centers.
    for (offset in 1...cells.length) {
      var index = cells[offset];
      var x = index % costmap.grid.width;
      var y = Std.int(index / costmap.grid.width);
      appendDistinct(positions, costmap.grid.cellCenter(x, y));
    }
    appendDistinct(positions, goal);
    if (positions.length < 2)
      throw "AStarPlanner start and goal are the same pose";
    var oriented:Array<Pose2> = [];
    for (index in 0...positions.length) {
      var point = positions[index];
      var yaw:Float;
      if (index == 0) {
        yaw = start.yaw;
      } else if (index == positions.length - 1) {
        yaw = goal.yaw;
      } else {
        var next = positions[index + 1];
        yaw = Math.atan2(next.y - point.y, next.x - point.x);
      }
      oriented.push(new Pose2(point.x, point.y, yaw));
    }
    return new Path(oriented, costmap.grid.frameId);
  }

  function appendDistinct(positions:Array<Pose2>, point:Pose2):Void {
    var previous = positions[positions.length - 1];
    var dx = point.x - previous.x;
    var dy = point.y - previous.y;
    if (dx != 0.0 || dy != 0.0) positions.push(point);
  }

  function push(heap:Array<OpenNode>, node:OpenNode):Void {
    heap.push(node);
    var child = heap.length - 1;
    while (child > 0) {
      var parent = Std.int((child - 1) / 2);
      if (!less(heap[child], heap[parent])) break;
      var swap = heap[parent];
      heap[parent] = heap[child];
      heap[child] = swap;
      child = parent;
    }
  }

  function pop(heap:Array<OpenNode>):OpenNode {
    var result = heap[0];
    var last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      var parent = 0;
      while (true) {
        var left = parent * 2 + 1;
        var right = left + 1;
        var smallest = parent;
        if (left < heap.length && less(heap[left], heap[smallest])) smallest = left;
        if (right < heap.length && less(heap[right], heap[smallest])) smallest = right;
        if (smallest == parent) break;
        var swap = heap[parent];
        heap[parent] = heap[smallest];
        heap[smallest] = swap;
        parent = smallest;
      }
    }
    return result;
  }

  function less(left:OpenNode, right:OpenNode):Bool {
    if (left.estimate != right.estimate) return left.estimate < right.estimate;
    if (left.score != right.score) return left.score < right.score;
    return left.index < right.index;
  }
}

private class OpenNode {
  public final index:Int;
  public final score:Float;
  public final estimate:Float;

  public function new(index:Int, score:Float, estimate:Float) {
    this.index = index;
    this.score = score;
    this.estimate = estimate;
  }
}
