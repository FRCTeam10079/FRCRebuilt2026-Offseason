// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfinding;

/**
 * Represents a node position on the pathfinding grid.
 *
 * <p>Grid coordinates are integer indices where (0,0) is at the origin of the field. Each cell
 * represents a NODE_SIZE_METERS × NODE_SIZE_METERS area.
 *
 * @param x X index in the grid (column)
 * @param y Y index in the grid (row)
 */
public record GridPosition(int x, int y) implements Comparable<GridPosition> {

  /**
   * Calculate the Euclidean distance to another grid position.
   *
   * @param other The other grid position
   * @return The distance in grid units
   */
  public double distanceTo(GridPosition other) {
    return Math.hypot(other.x - x, other.y - y);
  }

  /**
   * Check if this position is adjacent to another (including diagonals).
   *
   * @param other The other grid position
   * @return True if the positions are adjacent
   */
  public boolean isAdjacentTo(GridPosition other) {
    int dx = Math.abs(other.x - x);
    int dy = Math.abs(other.y - y);
    return dx <= 1 && dy <= 1 && !(dx == 0 && dy == 0);
  }

  @Override
  public int compareTo(GridPosition o) {
    if (x == o.x) {
      return Integer.compare(y, o.y);
    } else {
      return Integer.compare(x, o.x);
    }
  }

  @Override
  public String toString() {
    return "GridPosition(" + x + ", " + y + ")";
  }
}
