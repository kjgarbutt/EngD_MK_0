package sim;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

import objects.NetworkUtilities;
import objects.network.GeoNode;
import objects.network.ListEdge;
import objects.agents.Agent;
import sim.engine.SimState;
import sim.field.geo.GeomGridField;
import sim.field.geo.GeomGridField.GridDataType;
import sim.field.geo.GeomVectorField;
import sim.field.network.Network;
import sim.io.geo.ArcInfoASCGridImporter;
import sim.io.geo.ShapeFileImporter;
import sim.util.Bag;
import sim.util.geo.AttributeValue;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.planargraph.Node;

import ec.util.MersenneTwisterFast;

/**
*
* A simple back to basics model that locates agents on Norfolk's road network and makes them
* move from A to B, then they change direction and head back to the start.
* The process repeats until the user quits. The number of agents, their start and end points
* is determined by data in a .csv (/data/...) and assigned by the user under 'goals'
* (approx. Line 84).
*
* @author KJGarbutt
*
*/

public class EngDBasic extends SimState	{

	////////////////////////////////////////////////////////////////
	///////////////////// MODEL PARAMETERS /////////////////////////
	////////////////////////////////////////////////////////////////

	private static final long serialVersionUID = 1L;

	public int grid_width = 1300;
	public int grid_height = 600;
	public static double resolution = 5;// the granularity of the simulation
				// (fiddle around with this to merge nodes into one another)
	double speed = -1;

	///////////////////// Data Sources //////////////////////////////

	String dirName = "src/data/";

	//public static String communicatorFilename = "communicatorEvents.txt";
	public static String agentFilename = "NorfolkITNAGENT.csv";

	//String record_speeds_filename = "data/speeds",
		//record_info_filename = "data/info";

	//BufferedWriter record_speeds, record_sentiment, record_heatmap;
	//public BufferedWriter record_info;

	///////////////////// END Data Sources ///////////////////////

	//////////////////// Containers //////////////////////////////
	public GeomVectorField roadLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField baseLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField networkLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField networkEdgeLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField roadNodesLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField agentsLayer = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField flood3 = new GeomVectorField(grid_width, grid_height);
	public GeomVectorField flood2 = new GeomVectorField(grid_width, grid_height);

	////////////////////// Network ///////////////////////////////
	public GeomPlanarGraph network = new GeomPlanarGraph();		// Stores road network connections
	public GeomVectorField junctions = new GeomVectorField();	// nodes for intersections

	///////////////////// Objects ////////////////////////////////

	public Bag roadNodes = new Bag();
	public Network roads = new Network(false);
	HashMap <MasonGeometry, ArrayList <GeoNode>> localNodes;
	public Bag terminus_points = new Bag();
	public ArrayList <Agent> agents = new ArrayList <Agent> (200000);

	public GeometryFactory fa = new GeometryFactory();

	long mySeed = 0;

	HashMap<Integer, GeomPlanarGraphEdge> idsToEdges =
	        new HashMap<Integer, GeomPlanarGraphEdge>();
	public HashMap<GeomPlanarGraphEdge, ArrayList<Agent>> edgeTraffic =
			new HashMap<GeomPlanarGraphEdge, ArrayList<Agent>>();
    ArrayList<Agent> agentList = new ArrayList<Agent>();

	// Here we force the agents to go to or from work at any time
    public boolean goToWork = true;
    public boolean getGoToWork()	{
        return goToWork;
    }

	//Envelope MBR = null;
	Envelope MBR = roadLayer.getMBR();
	boolean verbose = false;

	public ListEdge goalPoint;

	public GeoNode work;

	public GeoNode home;

	///////////////////// END Objects //////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	/////////////////////////// BEGIN FUNCTIONS ///////////////////////////////
	///////////////////////////////////////////////////////////////////////////

    /**
     * Model Constructor
     * Default constructor function
	 * @param seed
     */
    public EngDBasic(long seed)	{
        super(seed);
        random = new MersenneTwisterFast(12345);
    }

    /**
     * Model Initialization
     * Read in data and set up the simulation
     */
    @Override
    public void start() {
        super.start();
        System.out.println("Reading shapefiles...");

        //////////////////////////////////////////////
		///////////// READING IN DATA ////////////////
		//////////////////////////////////////////////

        try	{
        	readInVectorLayer(baseLayer, dirName + "Final_LSOA.shp", "Boundaries", new Bag());
			readInVectorLayer(roadLayer, dirName + "Final_ITN.shp", "Road Network", new Bag());
			readInVectorLayer(flood3, dirName + "NorfolkFZ3.shp", "Flood Zone 3", new Bag());
			readInVectorLayer(flood2, dirName + "NorfolkFZ2.shp", "Flood Zone 2", new Bag());

			//////////////////////////////////////////////
			////////////////// CLEANUP ///////////////////
			//////////////////////////////////////////////

			// standardize the MBRs so that the visualization lines up

			Envelope MBR = roadLayer.getMBR();
			MBR.expandToInclude(baseLayer.getMBR());

			//this.grid_width = baseLayer.fieldWidth;
			//this.grid_height = baseLayer.fieldHeight;

			MBR.expandToInclude(agentsLayer.getMBR());
			MBR.expandToInclude(flood3.getMBR());
			MBR.expandToInclude(flood2.getMBR());

			//roadLayer = new GeomVectorField(grid_width, grid_height);

			createNetwork();

			// standardize the MBRs so that the visualization lines up
            // and everyone knows what the standard MBR is
			roadLayer.setMBR(MBR);
			baseLayer.setMBR(MBR);
			agentsLayer.setMBR(MBR);
			flood3.setMBR(MBR);
			flood2.setMBR(MBR);

			System.out.println("Finished reading, cleaning and setting MBR.");

			//////////////////////////////////////////////
			////////////////// AGENTS ///////////////////
			//////////////////////////////////////////////

			// set up the agents in the simulation
			setupAgentsFromFile("/data/NorfolkITNAGENT.csv");
			agentsLayer.setMBR(MBR);

			// for each of the Agents, set up relevant, environment-specific information
			int aindex = 0;
			for(Agent a: agents){
				if(a.familiarRoadNetwork == null){

				// the Agent knows about major roads
				Network familiar = roads.cloneGraph();

				// connect the major network to the Agent's location
				//connectToMajorNetwork(a.getNode(), familiar);

				a.familiarRoadNetwork = familiar;

				// add local roads into the network
				//for(Object o: agentsLayer.getObjectsWithinDistance(a, 50)){
					//Agent b = (Agent) o;

					//if(b == a || b.familiarRoadNetwork != null || b.getNode() != a.getNode()) continue;
					//b.familiarRoadNetwork = familiar.cloneGraph();
				}

				// connect the Agent's work into its personal network
				//connectToMajorNetwork(getClosestGeoNode(a.getWork()), a.familiarRoadNetwork);

				// set up its basic paths (fast and quicker and recomputing each time)
				a.setupPaths();

				if(aindex % 100 == 0){ // print report of progress
					System.out.println("..." + aindex + " of " + agents.size());
				}
				aindex++;
			}

        }catch (Exception e) { e.printStackTrace();}
    }

	/**
	 * Finish the simulation and clean up
	 */
	public void finish(){
		super.finish();
    	System.out.println();
    	System.out.println("Simulation ended by user.");
	}

	/**
	 * Given a record file of a set of Agents, create Agents with the assigned characteristics
	 * and add them to the simulation
	 *
	 * @param agentsFilename - the file in which the agent records are stored
	 */
	public void setupAgentsFromFile(String agentsFilename){
		try {
			System.out.println("Reading in agents from " + agentsFilename);

			FileInputStream fstream = new FileInputStream(agentsFilename);

            System.out.println();
            System.out.println("Populating model with Agents: " +agentsFilename);

			// Convert our input stream to a BufferedReader
			BufferedReader agentData = new BufferedReader(new InputStreamReader(fstream));
			String s;

			System.out.println("BEGIN READING IN PEOPLE");

			//HashMap <String, Agent> agentNameMapping = new HashMap <String, Agent> ();

			int indexy = -1;
			// get rid of the header
			agentData.readLine();
			while ((s = agentData.readLine()) != null) {
				String[] bits = s.split(",");

				indexy++;

				// recreate the Agent from the record
				String id = bits[3];
				System.out.println("ID: " +id);
				Integer count = Integer.parseInt(bits[2]);
				System.out.println("Count: " +count);
				String homeCoord = bits[3];
				System.out.println("Home: " +homeCoord);
				String workCoord = bits[4];
				System.out.println("Work: " +workCoord);

				GeomPlanarGraphEdge startingEdge = idsToEdges.get(
	                	(int) Double.parseDouble(homeCoord));
				GeomPlanarGraphEdge goalEdge = idsToEdges.get(
						(int) Double.parseDouble(workCoord));	//reads the .CSV column
						//goals[ random.nextInt(goals.length)]); // uses the hardcoded 'goals' from above

				for (int i = 0; i < count; i++)	{
                	//pop; i++)	{ 	// NO IDEA IF THIS MAKES A DIFFERENCE!?!
                    Agent a = new Agent(id, count, homeCoord, workCoord, startingEdge, goalEdge, this);

                    //boolean successfulStart = a.start(this);
                    //System.out.println("Starting...");

                    //if (!successfulStart)	{
                    //	System.out.println("Main agents added successfully!!");
                    //	continue; // DON'T ADD IT if it's bad
                	//agentNameMapping.put(id, a);

    				this.agents.add(a);
    				agentData.close();
				}

					//Agent a;

					//a = new Agent(id, count, homeCoord, workCoord, this);

				System.out.println("DONE READING IN PEOPLE");
			// clean up
			//}
			}

			} catch (Exception e) {
			System.err.println("File input error: " + agentsFilename);
		}
	}
	
	//////////////////////////////////////////////
	////////// UTILITIES /////////////////////////
	//////////////////////////////////////////////

    /**
     * Create the road network the agents will traverse
     */
    private void createNetwork()	{
    	System.out.println("Creating road network..." +roadLayer);
    	System.out.println();
    	network.createFromGeomField(roadLayer);

    	for (Object o : network.getEdges())	{
            GeomPlanarGraphEdge e = (GeomPlanarGraphEdge) o;

            //System.out.println("idsToEdges = " +idsToEdges);
            idsToEdges.put(e.getIntegerAttribute("ROAD_ID").intValue(), e);
            //System.out.println("idsToEdges = " +idsToEdges);

            e.setData(new ArrayList<Agent>());
        }
    }

    /**
	 * Method to read in a vector layer
	 * @param layer
	 * @param filename
	 * @param layerDescription
	 * @param attributes - optional: include only the given attributes
	 */
	synchronized void readInVectorLayer(GeomVectorField layer, String filename, String layerDescription, Bag attributes){
		try {
				System.out.print("Reading in '" + layerDescription + "' from: " + filename + "...");
				File file = new File(filename);
				if(attributes == null || attributes.size() == 0)
					ShapeFileImporter.read(file.toURL(), layer);
				else
					ShapeFileImporter.read(file.toURL(), layer, attributes);
				System.out.println("done");

		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	/**
	 * set the seed of the random number generator
	 */
	void seedRandom(long number){
		random = new MersenneTwisterFast(number);
		mySeed = number;
	}

	/**
	 *  reset the agent layer's MBR
	 */
	public void resetAgentLayer(){
		this.agentsLayer.setMBR(MBR);
	}

	/**
	 * To run the model without visualization
	 */
	public static void main(String[] args)	{
		//doLoop(EngDBasic.class, args);
		//System.exit(0);
		if(args.length < 8){
			System.out.println("usage error");
			System.exit(0);
		}

		EngDBasic engdbasic = new EngDBasic(System.currentTimeMillis());

		engdbasic.speed = Double.parseDouble(args[7]);

		System.out.println("Loading...");

		engdbasic.start();

		System.out.println("Running...");

		for(int i = 0; i < 288 * 3; i++){
			engdbasic.schedule.step(engdbasic);
		}

		engdbasic.finish();

		System.out.println("...run finished");

		System.exit(0);
    }
}
