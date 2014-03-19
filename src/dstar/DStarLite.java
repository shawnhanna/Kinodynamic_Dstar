package dstar;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

/**
 * 
 * @author daniel beard http://danielbeard.wordpress.com
 *         http://github.com/paintstripper
 * 
 *         Copyright (C) 2012 Daniel Beard
 * 
 *         Permission is hereby granted, free of charge, to any person obtaining
 *         a copy of this software and associated documentation files (the
 *         "Software"), to deal in the Software without restriction, including
 *         without limitation the rights to use, copy, modify, merge, publish,
 *         distribute, sublicense, and/or sell copies of the Software, and to
 *         permit persons to whom the Software is furnished to do so, subject to
 *         the following conditions:
 * 
 *         The above copyright notice and this permission notice shall be
 *         included in all copies or substantial portions of the Software.
 * 
 *         THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *         EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *         MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *         NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 *         BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 *         ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *         CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *         SOFTWARE.
 * 
 */
@SuppressWarnings("serial")
public class DStarLite implements java.io.Serializable {

	// Private Member variables
	private List<State> path = new ArrayList<State>();
	private double C1;
	private double k_m;
	private State s_start = new State();
	private State s_goal = new State();
	private State s_last = new State();
	private int maxSteps;
	private PriorityQueue<State> openList = new PriorityQueue<State>();
	// Change back to private****
	public HashMap<State, CellInfo> cellHash = new HashMap<State, CellInfo>();
	private HashMap<State, Float> openHash = new HashMap<State, Float>();

	// Constants
	private double M_SQRT2 = Math.sqrt(2.0);

	BufferedImage bufImg;

	// Default constructor
	public DStarLite() {
		maxSteps = 10000000;
		C1 = 1;
	}

	// Calculate Keys
	public void CalculateKeys() {

	}

	/*
	 * Initialise Method
	 * 
	 * @params start and goal coordinates
	 */
	public void init(int sX, int sY, int gX, int gY) {
		cellHash.clear();
		path.clear();
		openHash.clear();
		while (!openList.isEmpty())
			openList.poll();

		k_m = 0;

		s_start.x = sX;
		s_start.y = sY;
		s_goal.x = gX;
		s_goal.y = gY;

		CellInfo tmp = new CellInfo();
		tmp.g = 0;
		tmp.rhs = 0;
		tmp.cost = C1;

		cellHash.put(s_goal, tmp);

		tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(s_start, s_goal);
		tmp.cost = C1;
		cellHash.put(s_start, tmp);
		s_start = calculateKey(s_start);

		s_last = s_start;

	}

	/**
	 * CalculateKey(state u) As per [S. Koenig, 2002]
	 */
	private State calculateKey(State u) {
		double val = Math.min(getRHS(u), getG(u));

		u.k.setFirst(val + heuristic(u, s_start) + k_m);
		u.k.setSecond(val);

		return u;
	}

	/**
	 * Returns the rhs value for state u.
	 */
	private double getRHS(State u) {
		if (u == s_goal)
			return 0;

		// if the cellHash doesn't contain the State u
		if (cellHash.get(u) == null)
			return heuristic(u, s_goal);
		return cellHash.get(u).rhs;
	}

	/**
	 * Returns the g value for the state u.
	 */
	private double getG(State u) {
		// if the cellHash doesn't contain the State u
		if (cellHash.get(u) == null)
			return heuristic(u, s_goal);
		return cellHash.get(u).g;
	}

	/**
	 * Pretty self explanatory, the heuristic we use is the 8-way distance
	 * scaled by a constant C1 (should be set to <= min cost)
	 */
	private double heuristic(State a, State b) {
		return eightCondist(a, b) * C1;
	}

	/**
	 * Returns the 8-way distance between state a and state b
	 */
	private double eightCondist(State a, State b) {
		double min = Math.abs(a.x - b.x);
		double max = Math.abs(a.y - b.y);
		if (min > max) {
			double temp;
			temp = min;
			min = max;
			max = temp;
		}
		return ((M_SQRT2 - 1.0) * min + max);

	}

	public boolean replan() {
		path.clear();

		int res = computeShortestPath();
		if (res < 0) {
			System.out.println("No Path to Goal");
			return false;
		}

		
		LinkedList<State> n = new LinkedList<State>();
		State cur = s_start;

		// No path
		if (getG(s_start) == Double.POSITIVE_INFINITY) {
			System.out.println("No Path to Goal");
			return false;
		}

		//Generate path
		while (cur.neq(s_goal)) {
			path.add(cur);
			n = new LinkedList<State>();
			n = getSucc(cur);

			if (n.isEmpty()) {
				System.out.println("No Path to Goal");
				return false;
			}

			double cmin = Double.POSITIVE_INFINITY;
			double tmin = 0;
			State smin = new State();

			for (State i : n) {
				double val = cost(cur, i);
				double val2 = trueDist(i, s_goal) + trueDist(s_start, i);
				val += getG(i);

				if (close(val, cmin)) {
					if (tmin > val2) {
						tmin = val2;
						cmin = val;
						smin = i;
					}
				} else if (val < cmin) {
					tmin = val2;
					cmin = val;
					smin = i;
				}
			}
			n.clear();
			cur = new State(smin);
			// cur = smin;
		}
		path.add(s_goal);
		return true;
	}

	/**
	 * As per [S. Koenig,2002] except for two main modifications: 1. We stop
	 * planning after a number of steps, 'maxsteps' we do this because this
	 * algorithm can plan forever if the start is surrounded by obstacles 2. We
	 * lazily remove states from the open list so we never have to iterate
	 * through it.
	 */
	private int computeShortestPath() {
		LinkedList<State> s = new LinkedList<State>();

		if (openList.isEmpty())
			return 1;

		int k = 0;
		boolean test = false;
		while ((!openList.isEmpty())
				&& (openList.peek().lt(s_start = calculateKey(s_start)))
				|| (test = (getRHS(s_start) != getG(s_start)))) {

			if (k++ > maxSteps) {
				System.out.println("At maxsteps");
				return -1;
			}
			// System.out.println("At step: "+k);

			State u;


			// lazy remove
			while (true) {
				if (openList.isEmpty())
					return 1;
				u = openList.poll();

				if (!isValid(u))
					continue;
				if (!(u.lt(s_start)) && (!test))
					return 2;
				break;
			}
//			System.out.println("Evaluating: "+u);
			openHash.remove(u);

			State k_old = new State(u);

			if (k_old.lt(calculateKey(u))) { // u is out of date
				insert(u);
			} else if (getG(u) > getRHS(u)) { // needs update (got better)
				setG(u, getRHS(u));
				s = getPred(u);
				for (State i : s) {
					updateVertex(i);
				}
			} else { // g <= rhs, state has got worse
				setG(u, Double.POSITIVE_INFINITY);
				s = getPred(u);

				for (State i : s) {
					updateVertex(i);
				}
				updateVertex(u);
			}
		} // while
		return 0;
	}

	/*
	 * Returns a list of successor states for state u, since this is an 8-way
	 * graph this list contains all of a cells neighbours. Unless the cell is
	 * occupied, in which case it has no successors.
	 */
	private LinkedList<State> getSucc(State u) {
		LinkedList<State> s = new LinkedList<State>();
		State tempState;

		if (occupied(u))
			return s;

		// Generate the successors, starting at the immediate right,
		// Moving in a clockwise manner
		tempState = new State(u.x + 1, u.y, new Pair<Double, Double>(-1.0, -1.0));
		s.add(tempState);
		tempState = new State(u.x + 1, u.y + 1, new Pair<Double, Double>(-1.0, -1.0));
		s.add(tempState);
		tempState = new State(u.x, u.y + 1, new Pair<Double, Double>(-1.0, -1.0));
		s.add(tempState);
		tempState = new State(u.x - 1, u.y + 1, new Pair<Double, Double>(-1.0, -1.0));
		s.add(tempState);
		tempState = new State(u.x - 1, u.y, new Pair<Double, Double>(-1.0, -1.0));
		s.add(tempState);
		tempState = new State(u.x - 1, u.y - 1, new Pair<Double, Double>(-1.0, -1.0));
		s.add(tempState);
		tempState = new State(u.x, u.y - 1, new Pair<Double, Double>(-1.0, -1.0));
		s.add(tempState);
		tempState = new State(u.x + 1, u.y - 1, new Pair<Double, Double>(-1.0, -1.0));
		s.add(tempState);

		return s;
	}

	/*
	 * Returns a list of all the predecessor states for state u. Since this is
	 * for an 8-way connected graph, the list contains all the neighbours for
	 * state u. Occupied neighbours are not added to the list
	 */
	private LinkedList<State> getPred(State u) {
		LinkedList<State> s = new LinkedList<State>();
		State tempState;

		tempState = new State(u.x + 1, u.y, new Pair<Double, Double>(-1.0, -1.0));
		if (!occupied(tempState))
			s.add(tempState);
		tempState = new State(u.x + 1, u.y + 1, new Pair<Double, Double>(-1.0, -1.0));
		if (!occupied(tempState))
			s.add(tempState);
		tempState = new State(u.x, u.y + 1, new Pair<Double, Double>(-1.0, -1.0));
		if (!occupied(tempState))
			s.add(tempState);
		tempState = new State(u.x - 1, u.y + 1, new Pair<Double, Double>(-1.0, -1.0));
		if (!occupied(tempState))
			s.add(tempState);
		tempState = new State(u.x - 1, u.y, new Pair<Double, Double>(-1.0, -1.0));
		if (!occupied(tempState))
			s.add(tempState);
		tempState = new State(u.x - 1, u.y - 1, new Pair<Double, Double>(-1.0, -1.0));
		if (!occupied(tempState))
			s.add(tempState);
		tempState = new State(u.x, u.y - 1, new Pair<Double, Double>(-1.0, -1.0));
		if (!occupied(tempState))
			s.add(tempState);
		tempState = new State(u.x + 1, u.y - 1, new Pair<Double, Double>(-1.0, -1.0));
		if (!occupied(tempState))
			s.add(tempState);

		return s;
	}

	/*
	 * Update the position of the agent/robot. This does not force a replan.
	 */
	public void updateStart(int x, int y) {
		s_start.x = x;
		s_start.y = y;

		k_m += heuristic(s_last, s_start);

		s_start = calculateKey(s_start);
		s_last = s_start;

	}

	/*
	 * This is somewhat of a hack, to change the position of the goal we first
	 * save all of the non-empty nodes on the map, clear the map, move the goal
	 * and add re-add all of the non-empty cells. Since most of these cells are
	 * not between the start and goal this does not seem to hurt performance too
	 * much. Also, it frees up a good deal of memory we are probably not going
	 * to use.
	 */
	public void updateGoal(int x, int y) {
		List<Pair<ipoint2, Double>> toAdd = new ArrayList<Pair<ipoint2, Double>>();
		Pair<ipoint2, Double> tempPoint;

		for (Map.Entry<State, CellInfo> entry : cellHash.entrySet()) {
			if (!close(entry.getValue().cost, C1)) {
				tempPoint = new Pair<ipoint2, Double>(new ipoint2(entry.getKey().x,
						entry.getKey().y), entry.getValue().cost);
				toAdd.add(tempPoint);
			}
		}

		cellHash.clear();
		openHash.clear();

		while (!openList.isEmpty())
			openList.poll();

		k_m = 0;

		s_goal.x = x;
		s_goal.y = y;

		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = 0;
		tmp.cost = C1;

		cellHash.put(s_goal, tmp);

		tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(s_start, s_goal);
		tmp.cost = C1;
		cellHash.put(s_start, tmp);
		s_start = calculateKey(s_start);

		s_last = s_start;

		Iterator<Pair<ipoint2, Double>> iterator = toAdd.iterator();
		while (iterator.hasNext()) {
			tempPoint = iterator.next();
			updateCell(tempPoint.first().x, tempPoint.first().y,
					tempPoint.second());
		}

	}

	/*
	 * As per [S. Koenig, 2002]
	 */
	private void updateVertex(State u) {
		LinkedList<State> s = new LinkedList<State>();

		if (u.neq(s_goal)) {
			s = getSucc(u);
			double tmp = Double.POSITIVE_INFINITY;
			double tmp2;

			for (State i : s) {
				tmp2 = getG(i) + cost(u, i);
				if (tmp2 < tmp)
					tmp = tmp2;
			}
			if (!close(getRHS(u), tmp))
				setRHS(u, tmp);
		}

		if (!close(getG(u), getRHS(u)))
			insert(u);
	}

	/**
	 * Returns true if state u is on the open list or not by checking if it is
	 * in the hash table.
	 */
	private boolean isValid(State u) {
		if (openHash.get(u) == null)
			return false;
		if (!close(keyHashCode(u), openHash.get(u)))
			return false;
		if (!inBounds(u))
			return false;
		return true;
	}

	/**
	 * Sets the G value for state u
	 */
	private void setG(State u, double g) {
		makeNewCell(u);
		cellHash.get(u).g = g;
	}

	/**
	 * Sets the rhs value for state u
	 */
	private void setRHS(State u, double rhs) {
		makeNewCell(u);
		cellHash.get(u).rhs = rhs;
	}

	/**
	 * Checks if a cell is in the hash table, if not it adds it in.
	 */
	private void makeNewCell(State u) {
		if (cellHash.get(u) != null)
			return;
		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(u, s_goal);
		tmp.cost = C1;
		cellHash.put(u, tmp);
	}

	/**
	 * updateCell as per [S. Koenig, 2002]
	 */
	public void updateCell(int x, int y, double val) {
		State u = new State();
		u.x = x;
		u.y = y;

		if ((u.eq(s_start)) || (u.eq(s_goal)))
			return;

		makeNewCell(u);
		cellHash.get(u).cost = val;
		updateVertex(u);
	}

	/*
	 * Inserts state u into openList and openHash
	 */
	private void insert(State u) {
//		if(!inBounds(u))
//			return;
//		System.out.println("Inserting state: "+u);
		// iterator cur
		float csum;

		u = calculateKey(u);
		// cur = openHash.find(u);
		csum = keyHashCode(u);

		// return if cell is already in list. TODO: this should be
		// uncommented except it introduces a bug, I suspect that there is a
		// bug somewhere else and having duplicates in the openList queue
		// hides the problem...
//		if (openHash.get(u) != null)
//			return;

		openHash.put(u, csum);
		openList.add(u);
	}

	private boolean inBounds(State u) {
		return !(u.x < 0 || u.y < 0 || u.x > 999 || u.y > 999); 
	}

	/*
	 * Returns the key hash code for the state u, this is used to compare a
	 * state that has been updated
	 */
	private float keyHashCode(State u) {
		return (float) (u.k.first() + 119993 * u.k.second());
	}

	/*
	 * Returns true if the cell is occupied (non-traversable), false otherwise.
	 * Non-traversable are marked with a cost < 0
	 */
	private boolean occupied(State u) {
		// if the cellHash does not contain the State u
		if (cellHash.get(u) == null)
			return false;
		return (cellHash.get(u).cost < 0);
	}

	/**
	 * Euclidean cost between state a and state b
	 */
	private double trueDist(State a, State b) {
		float x = a.x - b.x;
		float y = a.y - b.y;
		return Math.sqrt(x * x + y * y);
	}

	/*
	 * Returns the cost of moving from state a to state b. This could be either
	 * the cost of moving off state a or onto state b, we went with the former.
	 * This is also the 8-way cost.
	 */
	private double cost(State a, State b) {
		int xd = Math.abs(a.x - b.x);
		int yd = Math.abs(a.y - b.y);
		double scale = 1;

		if (xd + yd > 1)
			scale = M_SQRT2;

		if (cellHash.containsKey(a) == false)
			return scale * C1;
		return scale * cellHash.get(a).cost;
	}

	/*
	 * Returns true if x and y are within 10E-5, false otherwise
	 */
	private boolean close(double x, double y) {
//		if (x == Double.POSITIVE_INFINITY && y == Double.POSITIVE_INFINITY)
//			return true;
		return (Math.abs(x - y) < 0.00001);
	}

	public List<State> getPath() {
		return path;
	}

	public static void main(String[] args) {
		DStarLite pf = new DStarLite();
		pf.init(50, 50, 503, 50);
		
		//add bounds
		for (int i=0; i<1000; i++)
		{
			pf.updateCell(0, i, -1);
			pf.updateCell(i, 0, -1);
			pf.updateCell(1000, i, -1);
			pf.updateCell(i, 1000, -1);
		}
		// add obstacles
		// Vertical obstacle
		for (int i = 0; i < 400; i++) {
			pf.updateCell(300, i, -1);
		}
		// pf.addRectObstcle(500, 500, 100, 200);

		System.out.println("Start node: x: " + pf.s_start.x + ", y: "
				+ pf.s_start.y);
		System.out.println("End node:   x: " + pf.s_goal.x + ", y: "
				+ pf.s_goal.y);

		//Add a horizontal obstacle
		for (int i = 50; i < 400; i++) {
			pf.updateCell(i, 300, -1);
		}
		
		// Time the first planning
		long begin = System.currentTimeMillis();
		pf.replan();
		long end = System.currentTimeMillis();

		System.out.println("Initial plan time: " + (end - begin) + "ms");
/*
		begin = System.currentTimeMillis();
		pf.replan();
		end = System.currentTimeMillis();
		
		System.out.println("replan time: " + (end - begin) + "ms");
*/
//		List<State> path = pf.getPath();
//		for (State i : path) {
//			System.out.println("x: " + i.x + " y: " + i.y + " | ");
//		}
		System.out.println();

		pf.drawImage();
	}

	/**
	 * @author Shawn
	 */

	/**
	 * Add a rectangular obstacle to the map
	 * 
	 * @param x
	 * @param y
	 * @param width
	 * @param height
	 * @return true if valid parameters are passed in
	 */
	public boolean addRectObstcle(int x, int y, int width, int height) {
		if (x < 0 || y < 0 || height < 0 || width < 0)
			return false;

		for (int i = x; i < x + width; i++) {
			for (int j = 0; j < y + height; j++) {
				updateCell(i, j, -1);
			}
		}
		return true;
	}

	public double maxGCost() {
		double max = -100;
		for (Map.Entry<State, CellInfo> entry : cellHash.entrySet()) {
			State s = entry.getKey();
			double cost = s.k.first();
			if (cost > max && cost != Double.POSITIVE_INFINITY)
				max = cost;
		}
		return max;
	}

	public double maxRHSCost() {
		double max = -100;
		for (Map.Entry<State, CellInfo> entry : cellHash.entrySet()) {
			State s = entry.getKey();
			double cost = s.k.second();
			if (cost > max && cost != Double.POSITIVE_INFINITY)
				max = cost;
		}
		return max;
	}

	public double maxCost() {
		double max = -100;
		for (Map.Entry<State, CellInfo> entry : cellHash.entrySet()) {
			double cost = entry.getValue().cost;
			if (cost > max && cost != Double.POSITIVE_INFINITY)
				max = cost;
		}
		return max;
	}

	public void drawImage() {
		// Image
		bufImg = new BufferedImage(1000, 1000, BufferedImage.TYPE_INT_RGB);

		Graphics2D g2d = (Graphics2D) bufImg.getGraphics();
		g2d.setBackground(Color.WHITE);

		double max = maxRHSCost();
		System.out.println("Max = " + max);

		System.out.println("Number of entries in cellhash: " + cellHash.size());
		// Display costs in map??
		for (Map.Entry<State, CellInfo> entry : cellHash.entrySet()) {
			State s = entry.getKey();
			double cost = s.k.second();
			if (cost > 0) {
				if (cost != Double.POSITIVE_INFINITY) {
					g2d.setColor(new Color(0, (int) (cost * 255 / max),
							(int) (cost * 255 / max)));
					g2d.drawLine(s.x, s.y, s.x, s.y);
				}
			}
		}

		// Draw start and end
		System.out.println("start = " + s_start);
		System.out.println("goal  = " + s_goal);
		g2d.setColor(Color.GREEN);
		g2d.fillOval(s_start.x - 5, s_start.y - 5, 10, 10);
		g2d.setColor(Color.GRAY);
		g2d.fillOval(s_goal.x - 5, s_goal.y - 5, 10, 10);

		// Draw path
		g2d.setColor(Color.PINK);
		List<State> path = getPath();
		State last = null;
		for (State s : path) {
			if (last != null) {
				g2d.drawLine(s.x, s.y, last.x, last.y);
			}
			last = s;
		}

		// Draw the obstacles
		g2d.setColor(Color.RED);
		for (Map.Entry<State, CellInfo> entry : cellHash.entrySet()) {
			State s = entry.getKey();
			double cost = entry.getValue().cost;
			if (cost < 0) {
				g2d.drawLine(s.x, s.y, s.x, s.y);
			}
		}

		JFrame frame = new JFrame("Astar Boat Demo");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(bufImg.getWidth(), bufImg.getHeight());
		frame.setResizable(false);
		ImageIcon icon = new ImageIcon(bufImg);
		JLabel label = new JLabel(icon);
		frame.getContentPane().add(label);
		frame.setVisible(true);
	}
}

@SuppressWarnings("serial")
class CellInfo implements java.io.Serializable {
	/**
	 * G and RHS are cost estimates to the goal from this cell RHS is
	 * "better informed"
	 */
	public double g = 0;
	public double rhs = 0;

	/**
	 * cost is the cost of going onto the cell Higher costs will give less
	 * incentive to go to For instance, a rocky terrain might have cost 5, while
	 * pavement has cost 1
	 * 
	 * cells with cost of -1 are unreachable (obstacles)
	 */
	public double cost = 0;
}

class ipoint2 {
	public int x = 0;
	public int y = 0;

	// default constructor
	public ipoint2() {

	}

	// overloaded constructor
	public ipoint2(int x, int y) {
		this.x = x;
		this.y = y;
	}
}
