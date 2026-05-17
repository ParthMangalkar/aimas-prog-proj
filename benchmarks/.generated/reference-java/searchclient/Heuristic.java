package searchclient;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;

public abstract class Heuristic
        implements Comparator<State> {

    // BFS distance maps: one per goal cell. distanceFromGoal[i][r][c] = shortest
    // path distance from goal cell i to cell (r,c), respecting walls only.
    private int[][] goalRows;
    private int[][] goalCols;
    private char[] goalTypes;       // 'A'-'Z' for box goals, '0'-'9' for agent goals
    private int[][][] distFromGoal; // distFromGoal[goalIndex][row][col]
    private int numGoals;

    private int rows;
    private int cols;

    public Heuristic(State initialState) {
        this.rows = State.walls.length;
        this.cols = State.walls[0].length;

        // Collect all goal positions
        ArrayList<int[]> goalPositions = new ArrayList<>();
        ArrayList<Character> goalTypesList = new ArrayList<>();

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                char goal = State.goals[r][c];
                if (('A' <= goal && goal <= 'Z') || ('0' <= goal && goal <= '9')) {
                    goalPositions.add(new int[]{r, c});
                    goalTypesList.add(goal);
                }
            }
        }

        this.numGoals = goalPositions.size();
        this.goalRows = new int[numGoals][];
        this.goalCols = new int[numGoals][];
        this.goalTypes = new char[numGoals];
        this.distFromGoal = new int[numGoals][rows][cols];

        // Precompute BFS from each goal cell
        for (int i = 0; i < numGoals; i++) {
            int gr = goalPositions.get(i)[0];
            int gc = goalPositions.get(i)[1];
            this.goalRows[i] = new int[]{gr};
            this.goalCols[i] = new int[]{gc};
            this.goalTypes[i] = goalTypesList.get(i);

            // Initialize distances to max
            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    distFromGoal[i][r][c] = Integer.MAX_VALUE;
                }
            }

            // BFS from goal cell (gr, gc)
            bfsFrom(i, gr, gc);
        }
    }

    private void bfsFrom(int goalIndex, int startR, int startC) {
        ArrayDeque<int[]> queue = new ArrayDeque<>();
        distFromGoal[goalIndex][startR][startC] = 0;
        queue.add(new int[]{startR, startC});

        while (!queue.isEmpty()) {
            int[] cur = queue.poll();
            int r = cur[0], c = cur[1];
            int nextDist = distFromGoal[goalIndex][r][c] + 1;

            // N, S, E, W
            int[] dr = {-1, 1, 0, 0};
            int[] dc = {0, 0, 1, -1};

            for (int d = 0; d < 4; d++) {
                int nr = r + dr[d];
                int nc = c + dc[d];
                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols
                        && !State.walls[nr][nc]
                        && distFromGoal[goalIndex][nr][nc] == Integer.MAX_VALUE) {
                    distFromGoal[goalIndex][nr][nc] = nextDist;
                    queue.add(new int[]{nr, nc});
                }
            }
        }
    }

    public int h(State s) {
        int total = 0;

        // --- Box goals: for each box-goal, find closest matching box ---
        for (int i = 0; i < numGoals; i++) {
            char goalType = goalTypes[i];

            if ('A' <= goalType && goalType <= 'Z') {
                int gr = goalRows[i][0];
                int gc = goalCols[i][0];

                // Check if this goal is already satisfied
                if (s.boxes[gr][gc] == goalType) {
                    continue;
                }

                // Find the closest matching box using precomputed BFS distance
                int minDist = Integer.MAX_VALUE;
                for (int r = 0; r < rows; r++) {
                    for (int c = 0; c < cols; c++) {
                        if (s.boxes[r][c] == goalType) {
                            int d = distFromGoal[i][r][c];
                            if (d < minDist) {
                                minDist = d;
                            }
                        }
                    }
                }
                if (minDist < Integer.MAX_VALUE) {
                    total += minDist;
                }
            }
        }

        // --- Agent goals: BFS distance from agent to its goal ---
        for (int i = 0; i < numGoals; i++) {
            char goalType = goalTypes[i];

            if ('0' <= goalType && goalType <= '9') {
                int agentIdx = goalType - '0';
                if (agentIdx < s.agentRows.length) {
                    int gr = goalRows[i][0];
                    int gc = goalCols[i][0];

                    // Check if already satisfied
                    if (s.agentRows[agentIdx] == gr && s.agentCols[agentIdx] == gc) {
                        continue;
                    }

                    int d = distFromGoal[i][s.agentRows[agentIdx]][s.agentCols[agentIdx]];
                    if (d < Integer.MAX_VALUE) {
                        total += d;
                    }
                }
            }
        }

        // --- Agent-to-nearest-unplaced-box bonus ---
        // For each agent, add the distance to its nearest unplaced box of matching color.
        // This reflects that the agent must walk to a box before pushing/pulling it.
        for (int a = 0; a < s.agentRows.length; a++) {
            int minBoxDist = Integer.MAX_VALUE;
            boolean hasUnplacedBox = false;

            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    char box = s.boxes[r][c];
                    if (box == 0) continue;

                    // Only consider boxes that match this agent's color
                    if (State.boxColors[box - 'A'] != State.agentColors[a]) continue;

                    // Check if this box is already at a matching goal
                    if (State.goals[r][c] == box) continue;

                    hasUnplacedBox = true;

                    // Manhattan distance from agent to this box (fast, always admissible)
                    int d = Math.abs(s.agentRows[a] - r) + Math.abs(s.agentCols[a] - c);
                    if (d < minBoxDist) {
                        minBoxDist = d;
                    }
                }
            }

            if (hasUnplacedBox && minBoxDist < Integer.MAX_VALUE) {
                total += minBoxDist;
            }
        }

        return total;
    }

    /**
     * Goal count heuristic: counts the number of unsatisfied goals.
     * A box goal is unsatisfied if the goal cell does not contain the correct box letter.
     * An agent goal is unsatisfied if the agent is not on its goal cell.
     */
    public int hGoalCount(State s) {
        int count = 0;
        for (int i = 0; i < numGoals; i++) {
            char goalType = goalTypes[i];
            int gr = goalRows[i][0];
            int gc = goalCols[i][0];

            if ('A' <= goalType && goalType <= 'Z') {
                // Box goal: check if the correct box is at this goal cell
                if (s.boxes[gr][gc] != goalType) {
                    count++;
                }
            } else if ('0' <= goalType && goalType <= '9') {
                // Agent goal: check if the agent is at this goal cell
                int agentIdx = goalType - '0';
                if (agentIdx < s.agentRows.length) {
                    if (s.agentRows[agentIdx] != gr || s.agentCols[agentIdx] != gc) {
                        count++;
                    }
                }
            }
        }
        return count;
    }

    public abstract int f(State s);

    @Override
    public int compare(State s1, State s2) {
        return this.f(s1) - this.f(s2);
    }
}

class HeuristicAStar
        extends Heuristic {
    public HeuristicAStar(State initialState) {
        super(initialState);
    }

    @Override
    public int f(State s) {
        return s.g() + this.h(s);
    }

    @Override
    public String toString() {
        return "A* evaluation";
    }
}

class HeuristicWeightedAStar
        extends Heuristic {
    private int w;

    public HeuristicWeightedAStar(State initialState, int w) {
        super(initialState);
        this.w = w;
    }

    @Override
    public int f(State s) {
        return s.g() + this.w * this.h(s);
    }

    @Override
    public String toString() {
        return String.format("WA*(%d) evaluation", this.w);
    }
}

class HeuristicGreedy
        extends Heuristic {
    public HeuristicGreedy(State initialState) {
        super(initialState);
    }

    @Override
    public int f(State s) {
        return this.h(s);
    }

    @Override
    public String toString() {
        return "greedy evaluation";
    }
}

class HeuristicGoalCountGreedy
        extends Heuristic {
    public HeuristicGoalCountGreedy(State initialState) {
        super(initialState);
    }

    @Override
    public int f(State s) {
        return this.hGoalCount(s);
    }

    @Override
    public String toString() {
        return "greedy goal count evaluation";
    }
}

class HeuristicGoalCountAStar
        extends Heuristic {
    public HeuristicGoalCountAStar(State initialState) {
        super(initialState);
    }

    @Override
    public int f(State s) {
        return s.g() + this.hGoalCount(s);
    }

    @Override
    public String toString() {
        return "A* goal count evaluation";
    }
}
