package objects.agents;


import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Random;

import sim.EngDBasicCopy;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.field.network.Edge;
import sim.field.network.Network;
import sim.util.Bag;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;
import objects.agents.TrafficAgent;
import objects.NetworkUtilities;
import objects.network.GeoNode;
import objects.network.ListEdge;

import com.vividsolutions.jts.geom.*;
import com.vividsolutions.jts.linearref.LengthIndexedLine;


/**
 * Agent object. Contains attributes, makes decisions, communicates, moves, etc. 
 * 
 */
public class AgentCopy extends TrafficAgent implements Serializable {

	
	private static final long serialVersionUID = 1L;

	////////// Objects ///////////////////////////////////////
	EngDBasicCopy world;
	
	Stoppable stopper = null;
	boolean removed = false;
	boolean alive = true;

	////////// Activities ////////////////////////////////////

	int currentActivity = 0;
	
	public static int activity_travel = 1;
	public static int activity_work = 2;
	public static int activity_relax = 3;
	public static int activity_sleep = 4;
	public static int activity_evacuate = 5;
	
	////////// Attributes ///////////////////////////////////

	String myID;
	
	Coordinate home, work; // work/school or whatever
	
	Stoppable observer = null;
	Stoppable mediaUser = null;
	
	// Time checks
	double lastMove = -1;
	double lastContact = -1;
	int lastSocialMediaCheckin = -1;

	// weighted by familiarity (and more familiar with major highways, etc)
	// this should be updated when Agent finds out about fires!
	public Network familiarRoadNetwork = null;
	ArrayList <ArrayList<Edge>> familiarPaths = new ArrayList <ArrayList <Edge>> ();


	// mapping of other agents, relationship with agents
	HashMap <AgentCopy, Integer> intimateTies = new HashMap <AgentCopy, Integer> ();
	HashMap <AgentCopy, Integer> recentIntimateContact = new HashMap <AgentCopy, Integer> ();
	HashMap <Integer, Integer> sentimentSignal = new HashMap <Integer, Integer> ();
	
	double size = 3;
	
	Coordinate targetDestination = null;

	double stress = 0; // between 0 and 10, 10 being super stressed out in a bad way
	
	////////// Parameters ///////////////////////////////////

	double communication_success_prob = .8;
	double contact_success_prob = .5;
	double tweet_prob = .1;
	double retweet_prob = .1;
	
	double comfortDistance = 10000;
	double observationDistance = 1000;
	
	double decayParam = .5;
	
	////////// END Parameters ///////////////////////////////
	
		
	public GeoNode getNode() {return node;}
	
	/**
	 * Default Wrapper Constructor: provides the default parameters
	 * 
	 * @param id - unique string identifying the Agent
	 * @param homeCoord - Coordinate indicating the initial position of the Agent
	 * @param homeCoord2 - Coordinate indicating the Agent's home location
	 * @param workCoord - Coordinate indicating the Agent's workplace
	 * @param world - reference to the containing Hotspots instance
	 */
	public AgentCopy(String id, String homeCoord, String homeCoord2, String workCoord, EngDBasicCopy world){		
		
	}
	
	/**
	 * Specialized constructor: use to specify parameters for an Agent
	 * 
	 * @param id - unique string identifying the Agent
	 * @param position - Coordinate indicating the initial position of the Agent
	 * @param home - Coordinate indicating the Agent's home location
	 * @param work - Coordinate indicating the Agent's workplace
	 * @param world - reference to the containing Hotspots instance
	 * @param communication_success_prob - probability of successfully communicating information 
	 * 		to another Agent 
	 * @param contact_success_prob - probability of successfully getting in touch with a distant
	 * 		Agent upon trying to activate the intimate social ties
	 * @param tweet_prob - probability of generating a Tweet upon activation
	 * @param retweet_prob - probability of retweeting other information upon activation
	 * @param comfortDistance - distance to dangerous obstacle mitigating Agent behavior
	 * @param observationDistance - distance to dangerous obstace within which Agent perceives it
	 * @param decayParam - parameter indicating rate of decay of influence of stressful information
	 * 		on the Agents' stress level
	 * @param speed - speed at which the Agent moves through the environment (m per 5 min)
	 */
	public AgentCopy(String id, Coordinate position, Coordinate home, Coordinate work, EngDBasicCopy world, 
			double communication_success_prob, double contact_success_prob, double tweet_prob, 
			double retweet_prob, double comfortDistance, double observationDistance, double decayParam, double speed){

		super((new GeometryFactory()).createPoint(position));
		
		myID = id;
		this.world = world;
		this.isMovable = true;
		this.space = world.agentsPortrayal;

		this.communication_success_prob = communication_success_prob;
		this.contact_success_prob = contact_success_prob;
		this.tweet_prob = tweet_prob;
		this.retweet_prob = retweet_prob;
		this.comfortDistance = comfortDistance;
		this.observationDistance = observationDistance;
		this.decayParam = decayParam;
		this.speed = speed;
		this.minSpeed = 650; // ~5mph

		
		// LOCALIZE THE AGENT INITIALLY
		
		// find the closest edge to the Agent initially (for ease of path-planning)
		//edge = world.getClosestEdge(position);
		
		// if no such edge exists, there is a problem with the setup
		if(edge == null){ 
			System.out.println(this.myID + "\tINIT_ERROR");
			return;
		}

		// figure out the closest GeoNode to the Agent's initial position
		GeoNode n1 = (GeoNode) edge.getFrom();
		GeoNode n2 = (GeoNode) edge.getTo();
		
		if(n1.geometry.getCoordinate().distance(position) <= n2.geometry.getCoordinate().distance(position))
			node = n1;
		else 
			node = n2;

		// do all the setup regarding the Agent's position on the road segment
		segment = new LengthIndexedLine((LineString)((MasonGeometry)edge.info).geometry);
		startIndex = segment.getStartIndex();
		endIndex = segment.getEndIndex();
		currentIndex = segment.indexOf(position);

		// SCHEDULE THE AGENT'S VARIOUS PROCESSES
		
		// schedule the Agent to check in and make decisions at the beginning of the simulation (with
		// ordering 100 so that it runs after the wildfire, etc)
		world.schedule.scheduleOnce(this, 100);
		
		// add the Agent to the space
		space.addGeometry(this);
		
		// set the Agent to not initially be evacuating
		this.addIntegerAttribute("Evacuating", 0);		
	}

	/**
	 * Navigate
	 * 
	 * Attempt to move along the existing path. If the road is impassible, a RoadClosure 
	 * is raised and the Agent attempts to replan its path to its target. If that replanning 
	 * fails, the Agent defaults to trying to wander toward its target.
	 */
	public int navigate(double resolution){
		
		
		myLastSpeed = -1; // reset this for accuracy in reporting
		
		// attempt to utilize the superclass's movement method
		int moveSuccess = super.navigate(resolution);
		
		// if the move didn't succeed, but the Agent still has a path (meaning that it
		// hasn't arrived at its destination) the Agent knows that there's a problem with
		// the road and registers a ROAD CLOSURE.
		if(moveSuccess < 0 && path != null){
			
			this.familiarRoadNetwork.removeEdge(edge);

			// try to plan a new path to the target destination
			int headForSuccess = headFor(this.targetDestination, familiarRoadNetwork);
			
			// if pathplanning failed, switch to wandering
			if(headForSuccess == -1){ 
				
				// pick the road that gets you closest to your target destination, and try to plan again from there!
				for(Object o: world.roadNodesLayer.getObjectsWithinDistance(node, 1000)){
	                GeoNode other = (GeoNode) o;
	                headFor(other.geometry.getCoordinate(), world.roadLayer);
	                if(path != null) // if the Agent has found a path, great! Go from there
	                	return 1;
				}
				
				// if none of the major roads provided a way out, the attempt to move has failed
				return -1;
			}
			
			// report on the success of the Agent in trying to move
			return headForSuccess;
		}
		
		// otherwise the attempt at moving was successful!
		else return 1;
	}
	
	/**
	 * Return the timestep that will correspond with the next instance of the given hour:minute combination
	 * 
	 * @param desiredHour - the hour to find
	 * @param desiredMinuteBlock - the minute to find
	 * @return the timestep of the next hour:minute combination
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
	
	/**
	 * Check in on the Agent and run its decision tree. Schedule when next to check in.
	 */
	@Override
	public void step(SimState state) {
		
		////////// Initial Checks ///////////////////////////////////////////////
		
		if(removed)
			return;
		
		// make sure the Agent is only being called once per tick
		if(lastMove >= state.schedule.getTime()) return;
		
		////////// Cleanup ////////////////////////////////////////////////////

		// update this Agent's information, and possibly remove them from the simulation if they've
		// exited the bounds of the world
		lastMove = state.schedule.getTime();
		if(currentActivity == activity_evacuate && path == null && !world.agentsPortrayal.MBR.contains(this.geometry.getCoordinate())){
			removeMe();
			System.out.println((int)world.schedule.getTime() + "\t" + this.myID + "\tEVACUATED");
			return;
		}
		
	}
	
	/**
	 * Tidies up after the Agent and removes all possible traces of it from the simulation
	 */
	void removeMe(){
		
		// internal record-keeping
		removed = true;
		observer.stop();
		lastMove = world.schedule.getTime() + 1;
		
		// takes the Agent out of the environment
		if(edge != null && edge instanceof ListEdge) 
			((ListEdge)edge).removeElement(this);
		world.agentsLayer.remove(this);
		
		// finally, reset position information
		this.updateLoc(new Coordinate(0,0)); // take me off the map, essentially
		world.resetAgentLayer();
		return;
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////
	/////// METHODS ////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////

	
		
	////////////////////////////////////////////////////////////////////////////////////////////////
	/////// end METHODS ////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////

	
	////////////////////////////////////////////////////////////////////////////////////////////////
	/////// UTILITIES //////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////

	int headFor(Coordinate place, Network roadNetwork) {

		// first, record from where the agent is starting
		startPoint = this.geometry.getCoordinate();
		goalPoint = null;

		// set up the coordinates
		this.startIndex = segment.getStartIndex();
		this.endIndex = segment.getEndIndex();

		return 1;
	}
		

	/**
	 * Comparator
	 */
	public boolean equals(Object o){
		if(!(o instanceof AgentCopy)) return false;
		else 
			return ((AgentCopy)o).myID.equals(myID);
	}
	
	/** HashCode */
	public int hashCode(){ return myID.hashCode(); }

	public String toString(){ return myID; }
	
	// GETTERS
	public Coordinate getHome(){ return home; }
	public Coordinate getWork(){ return work; }
	public int getActivity(){ return this.currentActivity; }
	public double getValence(){ return this.stress; }
	
	
	
	/**  Wrapper around step, so that it can be called from other functions */
	void stepWrapper(){ this.step(world); }

	////////////////////////////////////////////////////////////////////////////////////////////////
	/////// end UTILITIES //////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
 
}