package objects.agents;


import java.io.Serializable;
import java.util.ArrayList;

import objects.roles.OfficerRole;
import sim.EmergentCrime;
import sim.EngDBasic;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.field.geo.GeomVectorField;
import sim.field.network.Edge;
import sim.field.network.Network;
import sim.util.geo.MasonGeometry;
import objects.agents.TrafficAgent;
import objects.network.GeoNode;
import objects.network.ListEdge;

import com.vividsolutions.jts.geom.*;
import com.vividsolutions.jts.linearref.LengthIndexedLine;


public class Agent_SWise extends TrafficAgent implements Serializable {

	
	private static final long serialVersionUID = 1L;

	////////// Objects ///////////////////////////////////////
	EngDBasic world;
	
	Stoppable stopper = null;
	boolean removed = false;
	boolean alive = true;

	Coordinate currentGoal = null;
	

	////////// Movement Rules ////////////////////////////////
	// Movement rules determine how the Agent moves through the physical space. The Agent
	// may be restricted to only utilize roads; may be allowed on walkable areas; may
	// or may not be obeying traffic laws. 
	
	int currentMovementRule = 0;
	
	public static int movementRule_roadsOnly = 0;
	public static int movementRule_roadsOnlyNoLaws = 1;
	public static int movementRule_roadsAndPaths = 2;
	public static int movementRule_roadsAndPathsNoLaws = 3;
	
	////////// Speed ////////////////////////////////////////
	// Determines how quickly the Agent can maximally move.
	
	double speed = 400;// ~3 pmh

	////////// Activities ////////////////////////////////////
	// The Agent may be engaged in a number of different activities which determine
	// its status. The types of activities may vary depending on the specific role
	// the Agent is currently playing.
	
	int currentActivity = 0;
	
	public static int activity_travel = 100;
	public static int activity_work = 200;
	public static int activity_relax = 300;
	public static int activity_sleep = 400;
	
	////////// Attributes ///////////////////////////////////

	OfficerRole myRole = null;
	
	String myID;
	
	Coordinate base, work;
	
	Stoppable observer = null;
	
	// Time checks
	double lastMove = -1;

	// Knowledge
	public Network roadNetwork = null;
	ArrayList <ArrayList<Edge>> familiarPaths = new ArrayList <ArrayList <Edge>> ();	
	
	////////// Parameters ///////////////////////////////////

	double size = 3;
	Coordinate targetDestination = null;
		
	public Agent_SWise(String id, Coordinate position, Coordinate base, EmergentCrime world, GeomVectorField layer, Network network){		
		this(id, position, base, world, 2000, layer, network);
	}
	
	/**
	 * Constructor
	 * @param position
	 * @param base
	 * @param world
	 */
	public Agent_SWise(String id, Coordinate position, Coordinate base, EmergentCrime world, double speed, 
			GeomVectorField layer, Network network){

		super((new GeometryFactory()).createPoint(position));
		
		myID = id;
		this.world = world;
		this.isMovable = true;
		this.space = layer;

		this.speed = speed;
		this.minSpeed = 0;
		
		if(base != null){
			Coordinate homePoint = world.snapPointToRoadNetwork(base);
			this.base = homePoint;
		}

		
		edge = world.getClosestEdge(position);
		
		if(edge == null){
			System.out.println(this.myID + "\tINIT_ERROR: no nearby edge");
			return;
		}
			
		GeoNode n1 = (GeoNode) edge.getFrom();
		GeoNode n2 = (GeoNode) edge.getTo();
		
		if(n1.geometry.getCoordinate().distance(position) <= n2.geometry.getCoordinate().distance(position))
			node = n1;
		else 
			node = n2;

		if(work == null)
			this.work = null;
		else {
			Coordinate workPoint = world.snapPointToRoadNetwork(work);
			this.work = (Coordinate)workPoint.clone();
		}
		
		segment = new LengthIndexedLine((LineString)((MasonGeometry)edge.info).geometry);
		startIndex = segment.getStartIndex();
		endIndex = segment.getEndIndex();
		currentIndex = segment.indexOf(position);

/*		observer = world.schedule.scheduleRepeating(new Steppable (){
			private static final long serialVersionUID = 1L;

			@Override
			public void step(SimState state) {
				if(currentActivity == activity_sleep) return;
			}
		}, 50, 12);
	*/	
		currentActivity = activity_sleep;
		
		space.addGeometry(this);
		
		roadNetwork = network;
	}

	/**
	 * 
	 * @param resolution
	 * @return 1 for success, -1 for failure
	 */	
	public int navigate(double resolution){
		myLastSpeed = -1;
		
		if(path != null){
			double time = 1;//speed;
			while(path != null && time > 0){
				time = move(time, speed, resolution);
			}
			
			if(segment != null)
				updateLoc(segment.extractPoint(currentIndex));				

			if(time < 0){
				return -1;
			}
			else
				return 1;
		}
		return -1;		
	}
	
	/**
	 * 
	 * @param time - a positive amount of time, representing the period of time agents 
	 * 				are allocated for movement
	 * @param obstacles - set of spaces which are obstacles to the agent
	 * @return the amount of time left after moving, negated if the movement failed
	 */
	protected double move(double time, double mySpeed, double resolution){
		
		// if we're at the end of the edge and we have more edges, move onto the next edge
		if(arrived() ){
			
			// clean up any edge we leave
			if(edge != null && edge.getClass().equals(ListEdge.class))
				((ListEdge)edge).removeElement(this);

			// if we have arrived and there is no other edge in the path, we have finished our journey: 
			// reset the path and return the remaining time
			if(goalPoint == null && path.size() == 0 && (currentIndex <= startIndex || currentIndex >= endIndex )){
				path = null;
				return time;
			}
			
			// make sure that there is another edge in the path
			if(path.size() > 0) { 

				// take the next edge
				Edge newEdge = path.remove(path.size() - 1);				
				edge = newEdge;

				// make sure it's open
				// if it's not, return an error!
				if(((MasonGeometry)newEdge.info).getStringAttribute("open").equals("CLOSED")){
					updateLoc(node.geometry.getCoordinate());
					edge = newEdge;
					path = null;
					return -1;
				}				

				// stop at traffic lights if that's your rule!
				if(currentMovementRule % 2 == 0 && node.hasAttribute("delay")){
					time = Math.min(time, .01); // TODO: traffic light influence 
				}

				// change our positional node to be the Node toward which we're moving
				node = (GeoNode) edge.getOtherNode(node);
				
				// format the edge's geometry so that we can move along it conveniently
				LineString ls = (LineString)((MasonGeometry)edge.info).geometry;

				// set up the segment and coordinates
				segment = new LengthIndexedLine(ls);
				startIndex = segment.getStartIndex();
				endIndex = segment.getEndIndex();
				currentIndex = segment.project(this.geometry.getCoordinate());
				
				
				// if that was the last edge and we have a goal point, resize the expanse
				if(path.size() == 0 && goalPoint != null){ 
					double goalIndex = segment.project(goalPoint);
					if(currentIndex < goalIndex)
						endIndex = goalIndex;
					else
						startIndex = goalIndex;
				}
				
				// make sure we're moving in the correct direction along the Edge
				if(node.equals(edge.to())){
					direction = 1;
					currentIndex = Math.max(currentIndex, startIndex);
				} else {
					direction = -1;
					currentIndex = Math.min(currentIndex, endIndex);
				}

				if(edge.getClass().equals(ListEdge.class))
					((ListEdge)edge).addElement(this);

			}
						

		}
		
		// otherwise, we're on an Edge and moving forward!

		// set our speed
		double speed;
		if(edge != null && edge.getClass().equals(ListEdge.class)){
			
			// Each car has a certain amount of space: wants to preserve a following distance. 
			// If the amount of following distance is less than 20 meters (~ 6 car lengths) it'll slow
			// proportionately
			double val = Math.min(1, ((ListEdge)edge).lengthPerElement() / 20); 
			speed = Math.max( mySpeed * val, minSpeed);
		}
		else
			speed = mySpeed;

		myLastSpeed = speed;
		
		// construct a new current index which reflects the speed and direction of travel
		double proposedCurrentIndex = currentIndex + time * speed * direction;
		
		// great! It works! Move along!
		currentIndex = proposedCurrentIndex;
				
		if( direction < 0 ){
			if(currentIndex < startIndex){
				time = (startIndex - currentIndex) / speed; // convert back to time
				currentIndex = startIndex;
			}
			else
				time = 0;
		}
		else if(currentIndex > endIndex){
			time = (currentIndex - endIndex) / speed; // convert back to time
			currentIndex = endIndex;
		}
		else
			time = 0;

		// don't overshoot if we're on the last bit!
		if(goalPoint != null && path.size() == 0){
			double idealIndex = segment.indexOf(goalPoint);
			if((direction == 1 && idealIndex <= currentIndex) || (direction == -1 && idealIndex >= currentIndex)){
				currentIndex = idealIndex;
				time = 0;
				startIndex = endIndex = currentIndex;
			}
		}

		updateLoc(segment.extractPoint(currentIndex));
		
		if(path.size() == 0 && arrived()){
			path = null;
		}
		return time;
	}
	
	void pickDefaultActivity(){
		int time = (int) world.schedule.getTime();
		world.schedule.scheduleOnce(time + (60 / world.temporalResolution_minutesPerTick), 100, this); // check again in an hour
	}
	
	/**
	 * Return the timestep that will correspond with the next instance of the given hour:minute combination
	 * 
	 * Basically: find how many days so far, make it that many days + 
	 * 
	 * @param desiredHour - the hour to find
	 * @param desiredMinuteBlock - the minute to find
	 * @return
	 */
	int getTime(int desiredHour, int desiredMinuteBlock){

		int result = 0;
		
		// the current time in the day
		int time = (int)(world.schedule.getTime());
		int numDaysSoFar = (int) Math.floor(time / 288);
		int currentTime = time % 288;

		int goalTime = desiredHour * 12 + desiredMinuteBlock;
		
		if(goalTime < currentTime)
			result = 288 * (numDaysSoFar + 1) + goalTime;
		else
			result = 288 * numDaysSoFar + goalTime;
		
		return result;
	}
	
	@Override
	public void step(SimState state) {
		
		////////// Initial Checks ///////////////////////////////////////////////
		
		if(removed)
			return;
		
		// make sure I'm only being called once per tick
		if(lastMove >= state.schedule.getTime()) return;
				
		////////// BEHAVIOR //////////////////////////////////////////////////////
		
		if(targetDestination != null){
			if(path == null){
				headFor(targetDestination);
			}
			this.navigate(world.spatialResolution);
		}

		updateStatus();

		// update this Agent's information, and possibly remove them from the simulation if they've
		// exited the bounds of the world
		lastMove = state.schedule.getTime();
	}
	
	public void updateStatus(){
		if(myRole != null) 
			myRole.activate(world.schedule.getTime());
	}

	public void updateStatus(int i){
	}

	/**
	 * Set up a course to take the Agent to the given coordinates
	 * 
	 * @param place - the target destination
	 * @return 1 for success, -1 for a failure to find a path, -2 for failure based on the provided destination or current position
	 */
	public int headFor(Coordinate place) {

		//TODO: MUST INCORPORATE ROAD NETWORK STUFF
		if(place == null){
			System.out.println("ERROR: can't move toward nonexistant location");
			return -1;
		}
		
		// first, record from where the agent is starting
		startPoint = this.geometry.getCoordinate();
		goalPoint = null;

		if(!(edge.getTo().equals(node) || edge.getFrom().equals(node))){
			System.out.println( (int)world.schedule.getTime() + "\t" + this.myID + "\tMOVE_ERROR_mismatch_between_current_edge_and_node");
			return -2;
		}

		// FINDING THE GOAL //////////////////

		// set up goal information
		targetDestination = world.snapPointToRoadNetwork(place);
		
		GeoNode destinationNode = world.getClosestGeoNode(targetDestination, EmergentCrime.spatialResolution);//place);
		if(destinationNode == null){
			System.out.println((int)world.schedule.getTime() + "\t" + this.myID + "\tMOVE_ERROR_invalid_destination_node");
			return -2;
		}

		// be sure that if the target location is not a node but rather a point along an edge, that
		// point is recorded
		if(destinationNode.geometry.getCoordinate().distance(targetDestination) > EmergentCrime.spatialResolution)
			goalPoint = targetDestination;
		else
			goalPoint = null;


		// FINDING A PATH /////////////////////

		path = pathfinder.astarPath(node, destinationNode, roadNetwork);

		// if it fails, give up
		if (path == null){
			return -1;
		}

		// CHECK FOR BEGINNING OF PATH ////////

		// we want to be sure that we're situated on the path *right now*, and that if the path
		// doesn't include the link we're on at this moment that we're both
		// 		a) on a link that connects to the startNode
		// 		b) pointed toward that startNode
		// Then, we want to clean up by getting rid of the edge on which we're already located

		// Make sure we're in the right place, and face the right direction
		if (edge.getTo().equals(node))
			direction = 1;
		else if (edge.getFrom().equals(node))
			direction = -1;
		else {
			System.out.println((int)world.schedule.getTime() + "\t" + this.myID + "MOVE_ERROR_mismatch_between_current_edge_and_node_2");
			return -2;
		}

		// reset stuff
		if(path.size() == 0 && targetDestination.distance(geometry.getCoordinate()) > world.spatialResolution){
			path.add(edge);
			node = (GeoNode) edge.getOtherNode(node); // because it will look for the other side in the navigation!!! Tricky!!
		}

		// CHECK FOR END OF PATH //////////////

		// we want to be sure that if the goal point exists and the Agent isn't already on the edge 
		// that contains it, the edge that it's on is included in the path
		if (goalPoint != null) {

			ListEdge myLastEdge = world.getClosestEdge(goalPoint);
			
			if(myLastEdge == null){
				System.out.println((int)world.schedule.getTime() + "\t" + this.myID + "\tMOVE_ERROR_goal_point_is_too_far_from_any_edge");
				return -2;
			}
			
			// make sure the point is on the last edge
			Edge lastEdge;
			if (path.size() > 0)
				lastEdge = path.get(0);
			else
				lastEdge = edge;

			Point goalPointGeometry = world.fa.createPoint(goalPoint);
			if(!lastEdge.equals(myLastEdge) && ((MasonGeometry)lastEdge.info).geometry.distance(goalPointGeometry) > EmergentCrime.spatialResolution){
				if(lastEdge.getFrom().equals(myLastEdge.getFrom()) || lastEdge.getFrom().equals(myLastEdge.getTo()) 
						|| lastEdge.getTo().equals(myLastEdge.getFrom()) || lastEdge.getTo().equals(myLastEdge.getTo()))
					path.add(0, myLastEdge);
				else{
					System.out.println((int)world.schedule.getTime() + "\t" + this.myID + "\tMOVE_ERROR_goal_point_edge_is_not_included_in_the_path");
					return -2;
				}
			}
			
		}

		// set up the coordinates
		this.startIndex = segment.getStartIndex();
		this.endIndex = segment.getEndIndex();

		return 1;
	}



	/**
	 * Check the tile in which this Agent finds itself to see if there's something on fire near it
	 * 
	 * (Could easily be extended to check for other things as well)
	 */
	void observe(){
		if(removed == true) return;

	}
		
	////////////////////////////////////////////////////////////////////////////////////////////////
	/////// end METHODS ////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////

	
	////////////////////////////////////////////////////////////////////////////////////////////////
	/////// UTILITIES //////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	

	

	public boolean equals(Object o){
		if(!(o instanceof Agent_SWise)) return false;
		else 
			return ((Agent_SWise)o).myID.equals(myID);
	}
	
	public int hashCode(){ return myID.hashCode(); }

	public String toString(){ return myID; }
	
	public Coordinate getBase(){ return base; }
	public Coordinate getWork(){ return work; }
	
	public void setSpeed(double speed){ this.speed = speed; }
	public double getSpeed(){ return speed; }

	public void setActivity(int i){ this.currentActivity = i; }
	public int getActivity(){ return this.currentActivity; }
	
	public void setMovementRule(int i){ this.currentMovementRule = i; }
	public int getMovementRule(){ return this.currentMovementRule; }

	public Coordinate getTargetDestination(){ return this.targetDestination; }

	public GeoNode getNode() {return node;}
		
	void stepWrapper(){ this.step(world); }

	public boolean arrivedAtGoal(){
		if(currentGoal == null) return true;
		else if(geometry.getCoordinate().distance(currentGoal) > EmergentCrime.spatialResolution) return false;
		else return true;
	}
	
	public Coordinate getGoal(){ return currentGoal; }
	public void setCurrentGoal(Coordinate c){
		headFor(c);
		currentGoal = (Coordinate) c.clone();
	}
	
	public ArrayList <Edge> getPath(){
		return path;
	}
	
	public void moveTo(Coordinate c){
		this.updateLoc(c);
	}
	
	public void schedule(double time){ world.schedule.scheduleOnce(time, this);}
	public void scheduleIn(double timeDiff){ world.schedule.scheduleOnce(world.schedule.getTime() + timeDiff, this);}
	
	public double getTime(){ return world.schedule.getTime(); }
	public EmergentCrime getWorld(){ return world; }
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////
	/////// end UTILITIES //////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	
}