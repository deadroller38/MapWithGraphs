package roadgraph;

import java.util.LinkedList;

import geography.GeographicPoint;

/**
 * Class representing a path from start position to goal position
 */

class MapPath {

	GeographicPoint start;
	GeographicPoint goal;
	LinkedList<GeographicPoint> path;
	
	public MapPath(GeographicPoint s, GeographicPoint e, LinkedList<GeographicPoint> p) {
		start = s;
		goal = e;
		path = p;
	}
	
	GeographicPoint getStart() {
		return start;
	}
	
	GeographicPoint getGoal() {
		return goal;
	}
	
	LinkedList<GeographicPoint> getPath () {
		return path;
	}
}
