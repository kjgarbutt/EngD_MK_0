package sim;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

import objects.agents.Agent;
import objects.agents.ElderlyAgent;
import objects.agents.LimitedActionsAgent;
import objects.agents.NGOAgent;
import objects.network.GeoNode;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.field.geo.GeomVectorField;
import sim.field.network.Network;
import sim.io.geo.ShapeFileImporter;
import sim.util.Bag;
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
 * A simple model that locates agents on Norfolk's road network and makes them
 * move from A to B, then they change direction and head back to the start.
 * The process repeats until the user quits. The number of agents, their start
 * and end points is determined by data in NorfolkITNLSOA.csv and assigned by the
 * user under 'goals' (approx. Line 84).
 *
 * @author KJGarbutt
 *
 */
public class EngDBasicCopy extends SimState	{

	////////////////////////////////////////////////////////////////
	///////////////////// MODEL PARAMETERS /////////////////////////
	////////////////////////////////////////////////////////////////

	private static final long serialVersionUID = -4554882816749973618L;
	public int grid_width = 1300;
	public int grid_height = 600;
	public static double resolution = 5;// the granularity of the simulation
										// (fiddle around with this to merge
										// nodes into one another)

	/////////////// Data Sources ///////////////////////////////////////
	String dirName = "src/data/";

	public static String agentFilename = "NorfolkITNAGENT.csv";

	BufferedWriter record_speeds, record_sentiment, record_heatmap;
	public BufferedWriter record_info;

	//// END Data Sources ////////////////////////
				
	//////////////////// Containers //////////////////////////////
    public GeomVectorField roadLayer = new GeomVectorField();
    public GeomVectorField baseLayer = new GeomVectorField();
    public GeomVectorField flood3Layer = new GeomVectorField();
    public GeomVectorField flood2Layer = new GeomVectorField();
    //public GeomVectorField householdsFZLayer = new GeomVectorField();
    //public GeomVectorField householdsLayer = new GeomVectorField();
    public GeomVectorField agentsLayer = new GeomVectorField();
    public GeomVectorField ngoAgentsLayer = new GeomVectorField();
    public GeomVectorField elderlyAgentsLayer = new GeomVectorField();
    public GeomVectorField limitedActionsAgentsLayer = new GeomVectorField();

    ////////////////////// Network ///////////////////////////////
    public GeomPlanarGraph network = new GeomPlanarGraph();	// Stores road network connections
    public GeomVectorField junctions = new GeomVectorField();	// nodes for intersections

    /////////////// Objects //////////////////////////////////////////////

	public Bag roadNodes = new Bag();
	public Network roads = new Network(false);
	HashMap <MasonGeometry, ArrayList <GeoNode>> localNodes;
	public Bag terminus_points = new Bag();

	public ArrayList <Agent> agents = new ArrayList <Agent> (200000);

	public GeometryFactory fa = new GeometryFactory();

	long mySeed = 0;

	Envelope MBR = null;

	boolean verbose = false;

	/////////////// END Objects //////////////////////////////////////////
		
    ///////////////////// MainAgent //////////////////////////////
    // maps between unique edge IDs and edge structures themselves
    HashMap<Integer, GeomPlanarGraphEdge> idsToEdges =
        new HashMap<Integer, GeomPlanarGraphEdge>();
    public HashMap<GeomPlanarGraphEdge, ArrayList<Agent>> edgeTraffic =
        new HashMap<GeomPlanarGraphEdge, ArrayList<Agent>>();

    public GeomVectorField agent = new GeomVectorField();
    ArrayList<Agent> agentList = new ArrayList<Agent>();

    // Here we force the agents to go to or from work at any time
    public boolean goToWork = true;
    public boolean getGoToWork()	{
        return goToWork;
    }

    ///////////////////// NGOAgent //////////////////////////////
    HashMap<Integer, GeomPlanarGraphEdge> idsToEdges1 =
            new HashMap<Integer, GeomPlanarGraphEdge>();
    public HashMap<GeomPlanarGraphEdge, ArrayList<NGOAgent>> edgeTraffic1 =
            new HashMap<GeomPlanarGraphEdge, ArrayList<NGOAgent>>();

    public GeomVectorField ngoagents = new GeomVectorField();
    ArrayList<NGOAgent> ngoAgentList = new ArrayList<NGOAgent>();

    public boolean goToWork1 = true;
    public boolean getGoToWork1()	{
        return goToWork1;
    }

    //////////////////// ElderlyAgent ///////////////////////////
    HashMap<Integer, GeomPlanarGraphEdge> idsToEdges2 =
            new HashMap<Integer, GeomPlanarGraphEdge>();
    public HashMap<GeomPlanarGraphEdge, ArrayList<ElderlyAgent>> edgeTraffic2 =
            new HashMap<GeomPlanarGraphEdge, ArrayList<ElderlyAgent>>();

    public GeomVectorField elderlyagent = new GeomVectorField();
    ArrayList<ElderlyAgent> elderlyAgentList = new ArrayList<ElderlyAgent>();

    public boolean goToWork2 = true;
    public boolean getGoToWork2()	{
        return goToWork2;
    }

    //////////////////// LimitedActionsAgent //////////////////////
    HashMap<Integer, GeomPlanarGraphEdge> idsToEdges3 =
            new HashMap<Integer, GeomPlanarGraphEdge>();
    public HashMap<GeomPlanarGraphEdge, ArrayList<LimitedActionsAgent>> edgeTraffic3 =
            new HashMap<GeomPlanarGraphEdge, ArrayList<LimitedActionsAgent>>();

    public GeomVectorField limitedactionsagent = new GeomVectorField();
    ArrayList<LimitedActionsAgent> limitedActionsAgentList = new ArrayList<LimitedActionsAgent>();

    public boolean goToWork3 = true;
    public boolean getGoToWork3()	{
        return goToWork3;
    }

    //////////////////// agentGoals //////////////////////

    /**
     * Here we set the 'goals', or destinations, which relate to ROAD_ID in NorfolkITNLSOA.csv/.shp...
     * 30250 = Norfolk & Norwich Hospital
     * 74858 = James Paget Hospital (Does not work!)
     * 18081 = Queen Elizabeth Hospital
     * 46728 = BRC Norwich Office
     * 49307 = Sprowston Fire Station - BRC Fire & Emergency Support
     *
     */

    Integer[] goals =	{	// MainAgent
    		60708, 70353, 75417, 29565, 3715, 15816, 47794, 16561, 70035, 55437, 98, 45, 987, 345, 5643, 234, 21, 8765, 10345
    };

    Integer[] goals1 =	{	// NGOAgent
    		60708, 70353, 75417, 29565, 3715, 15816, 47794, 16561, 70035, 55437
    };

    Integer[] goals2 =	{	// ElderlyAgent
    		60708, 70353, 75417, 29565, 3715, 15816, 47794, 16561, 70035, 55437
    };

    Integer[] goals3 =	{	// LimitedActionsAgent
    		60708, 70353, 75417, 29565, 3715, 15816, 47794, 16561, 70035, 55437
    };


    ///////////////////////////////////////////////////////////////////////////
	/////////////////////////// BEGIN FUNCTIONS ///////////////////////////////
	///////////////////////////////////////////////////////////////////////////

    /**
     * Model Constructor
     */
    public EngDBasicCopy(long seed)	{
        super(seed);
        random = new MersenneTwisterFast(12345);
    }


    /**
     * Model Initialization
     */
    @Override
    public void start() {
        super.start();
        System.out.println("Reading shapefiles...");

		//////////////////////////////////////////////
		///////////// READING IN DATA ////////////////
		//////////////////////////////////////////////

        /*
        src/
        data/
            .shp, .dbf, .csv ...
        sim/
        	app/
        		geo/
        			MK_2/
        				.java files
         */
        try	{
        	readInVectorLayer(baseLayer, dirName + "Final_LSOA.shp", "Boundaries", new Bag());
			readInVectorLayer(roadLayer, dirName + "Final_ITN.shp", "Road Network", new Bag());
			readInVectorLayer(flood3Layer, dirName + "NorfolkFZ3.shp", "Flood Zone 3", new Bag());
			readInVectorLayer(flood2Layer, dirName + "NorfolkFZ2.shp", "Flood Zone 2", new Bag());
	
			//////////////////////////////////////////////
			////////////////// CLEANUP ///////////////////
			//////////////////////////////////////////////
	
			// standardize the MBRs so that the visualization lines up
			Envelope MBR = baseLayer.getMBR();
			MBR.expandToInclude(roadLayer.getMBR());
	
			//this.grid_width = baseLayer.fieldWidth;
			//this.grid_height = baseLayer.fieldHeight;
	
			MBR.expandToInclude(agentsLayer.getMBR());
			MBR.expandToInclude(flood3Layer.getMBR());
			MBR.expandToInclude(flood2Layer.getMBR());
	
			//roadLayer = new GeomVectorField(grid_width, grid_height);
	
			createNetwork();
	
			// standardize the MBRs so that the visualization lines up
			// and everyone knows what the standard MBR is
			roadLayer.setMBR(MBR);
			baseLayer.setMBR(MBR);
			agentsLayer.setMBR(MBR);
			flood3Layer.setMBR(MBR);
			flood2Layer.setMBR(MBR);
			
			System.out.println();
			System.out.println("Finished reading, cleaning and setting MBR.");
			
			//////////////////////////////////////////////
			////////////////// AGENTS ///////////////////
			//////////////////////////////////////////////
	
			// initialize agents using the following source .CSV files			
			populateAgent("/data/NorfolkITNAGENT.csv");
			populateNGO("/data/NorfolkITNNGO.csv");
			populateElderly("/data/NorfolkITNELDERLY.csv");
			populateLimitedActions("/data/NorfolkITNLIMITED.csv");
			System.out.println();
			System.out.println("Starting simulation...");
	
			// standardize the MBRs so that the visualization lines up
			// and everyone knows what the standard MBR is
			roadLayer.setMBR(MBR);
			baseLayer.setMBR(MBR);
			flood3Layer.setMBR(MBR);
			flood2Layer.setMBR(MBR);
			//householdsFZLayer.setMBR(MBR);
			//householdsLayer.setMBR(MBR);
			agentsLayer.setMBR(MBR);
			ngoAgentsLayer.setMBR(MBR);
			elderlyAgentsLayer.setMBR(MBR);
			limitedActionsAgentsLayer.setMBR(MBR);
			
			// Ensure that the spatial index is updated after all the agents move
			schedule.scheduleRepeating( agentsLayer.scheduleSpatialIndexUpdater(),
					Integer.MAX_VALUE, 1.0);
			schedule.scheduleRepeating( ngoAgentsLayer.scheduleSpatialIndexUpdater(),
					Integer.MAX_VALUE, 1.0);
			schedule.scheduleRepeating( elderlyagent.scheduleSpatialIndexUpdater(),
					Integer.MAX_VALUE, 1.0);
			schedule.scheduleRepeating( limitedactionsagent.scheduleSpatialIndexUpdater(),
					Integer.MAX_VALUE, 1.0);
        	
			/**
			 * Steppable that flips Agent paths once everyone reaches their destinations
             */
			Steppable flipper = new Steppable()	{
            	private static final long serialVersionUID = 1L;
            	
            	@Override
            	public void step(SimState state)	{
            		EngDBasicCopy gstate = (EngDBasicCopy) state;
            		
            		// checks to see if anyone has not yet reached destination
					// turning off makes agents move to next edge then back again
            		for (Agent a : gstate.agentList)	{
            			if (!a.reachedDestination)	{	// someone is still moving: let them do so
            				return;
            			}
            		}
            		// Now send everyone back in the opposite direction
            		boolean toWork = gstate.goToWork;
            		gstate.goToWork = !toWork;
                    // otherwise everyone has reached their latest destination:
                    // turn them back
                    // turning off means agents reach first destination and stay there.
                    for (Agent a : gstate.agentList) 	{
                        a.flipPath();
                    }
                    
                    // NGOAgent
                    for (NGOAgent b : gstate.ngoAgentList)	{
                    	if (!b.reachedDestination)	{
                    		return;
                    	}
                    }
                    boolean toWork1 = gstate.goToWork1;
                    gstate.goToWork1 = !toWork1;
                    
                    for (NGOAgent b : gstate.ngoAgentList)	{
                        b.flipPath();
                    }
                     
                    // ElderlyAgent
                    for (ElderlyAgent c : gstate.elderlyAgentList)	{
                        if (!c.reachedDestination)	{
                            return;
                        }
                    }
                    boolean toWork2 = gstate.goToWork2;
                    gstate.goToWork2 = !toWork2;
                    
                    for (ElderlyAgent c : gstate.elderlyAgentList)	{
                        c.flipPath();
                    }
                    
                    // LimitedActionsAgent
                    for (LimitedActionsAgent d : gstate.limitedActionsAgentList)	{
                        if (!d.reachedDestination)	{
                            return;
                        }
                    }
                    boolean toWork3 = gstate.goToWork3;
                    gstate.goToWork3 = !toWork3;
                    
                    for (LimitedActionsAgent d : gstate.limitedActionsAgentList)	{
                        d.flipPath();
                    }
                }
            };
            
            schedule.scheduleRepeating(flipper, 10);
            // 10? Does it repeat 10 times? Appears to go on forever...
            
            // seed the simulation randomly
            seedRandom(System.currentTimeMillis());
            
        } catch (Exception e) {
        	System.out.println();
        	System.out.println("ERROR: issue with Steppable. ");
        	e.printStackTrace();
        }
    }
    
    
    /**
     * Finish the simulation and clean up
     */
    public void finish()	{
    	super.finish();
    	System.out.println();
    	System.out.println("Simulation ended by user.");
        /*
    	System.out.println("Attempting to export agent data...");
        try	{
        	ShapeFileExporter.write("agents", agents);
        } catch (Exception e)	{
        	System.out.println("Export failed.");
        	e.printStackTrace();
        }
        */
    	}
    
    /**
     * Create the road network the agents will traverse
     */
    private void createNetwork()	{
    	System.out.println();
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

        addIntersectionNodes(network.nodeIterator(), junctions);
    }
    
    /**
     * Method to read in a vector layer
	 * @param layer
	 * @param filename
	 * @param layerDescription
	 * @param attributes - optional: include only the given attributes
	 */
    @SuppressWarnings("deprecation")
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
     * Read in the population files and create appropriate populations
     * @param filename
     */
    ////////////////////// MainAgent /////////////////////////////
    public void populateAgent(String filename)	{
    	//System.out.println("Populating model: ");
    	try	{
    		String filePath = EngDBasicCopy.class.getResource(filename).getPath();
    		FileInputStream fstream = new FileInputStream(filePath);
    		System.out.println("Populating model with Main Agents: " +filePath);
    		
    		BufferedReader d = new BufferedReader(new InputStreamReader(fstream));
    		String s;
    		// get rid of the header
    		d.readLine();
    		// read in all data
    		while ((s = d.readLine()) != null)	{
    			String[] bits = s.split(",");
    			
    			int pop = Integer.parseInt(bits[2]);
				//System.out.println();
				System.out.println("Main Agent road segment population (C:Count): " +pop);
				
				String homeTract = bits[3];
				System.out.println("Main Agent homeTract (D:ROAD_ID): " +homeTract);
				
				String workTract = bits[4];
				System.out.println("Main Agent workTract (E:Work): " +workTract);
				
				String id_id = bits[3];
				System.out.println("Main Agent ID_ID (D:ROAD_ID): " +id_id);
				
				String ROAD_ID = bits[3];
				System.out.println("Main Agent road segment (D:ROAD_ID): " +ROAD_ID);
				//System.out.println();
				
				GeomPlanarGraphEdge startingEdge = idsToEdges.get(
						(int) Double.parseDouble(ROAD_ID));
				GeomPlanarGraphEdge goalEdge = idsToEdges.get(
						//(int) Double.parseDouble(workTract));	//reads the .CSV column
						goals[ random.nextInt(goals.length)]); // uses the hardcoded 'goals' from above
				//System.out.println("startingEdge: " +startingEdge);
				//System.out.println("idsToEdges: " +idsToEdges);
				//System.out.println("goalEdge: " +goalEdge);
				//System.out.println("goals: " +goals);
				//System.out.println("homeNode: " +goals);
				//System.out.println();
				
				for (int i = 0; i < pop; i++)	{
					//pop; i++)	{ 	// NO IDEA IF THIS MAKES A DIFFERENCE!?!
					Agent a = new Agent(this, homeTract, workTract, startingEdge, goalEdge);
					/*
					 System.out.println(
					 "Main Agent " + i + ":	" 
	      				+ " Home: " +homeTract + ";	"
	              		+ "	Work: " +workTract + ";	"
	              		+ " Starting Edge: " +startingEdge + ";"
	              		+ "	Pop: " +pop + ";	"
	              		+ "	id_id: " +id_id + ";	"
	              		+ "	Road_ID: " +ROAD_ID);
	              		*/
					boolean successfulStart = a.start(this);
					//System.out.println("Starting...");
					
					if (!successfulStart)	{
						System.out.println("Main agents added successfully!!");
						continue; // DON'T ADD IT if it's bad
						}
					// MasonGeometry newGeometry = new MasonGeometry(a.getGeometry());
					MasonGeometry newGeometry = a.getGeometry();
					newGeometry.isMovable = true;
					agentsLayer.addGeometry(newGeometry);
					agentList.add(a);
					schedule.scheduleRepeating(a);
					}
				}
    		d.close();
    		} catch (Exception e) {
    			System.out.println();
    			System.out.println("ERROR: issue with Agent file: ");
    			e.printStackTrace();
    			}
    	}
    
    //////////////////////// NGOAgent /////////////////////////////
    public void populateNGO(String filename)	{
    	try	{
    		String filePath = EngDBasicCopy.class.getResource(filename).getPath();
    		FileInputStream fstream = new FileInputStream(filePath);
    		System.out.println();
    		System.out.println("Populating model with NGO Agents: " +filePath);
    		
    		BufferedReader d = new BufferedReader(new InputStreamReader(fstream));
    		String s;
    		
    		// get rid of the header
    		d.readLine();
    		// read in all data
    		while ((s = d.readLine()) != null)	{
    			String[] bits = s.split(",");
    			int pop = Integer.parseInt(bits[2]);
    			//System.out.println();
    			System.out.println("NGO Agent road segment population (C:Count): " +pop);
			
    			String homeTract = bits[3];
    			System.out.println("NGO Agent homeTract (D:ROAD_ID): " +homeTract);
			
    			String workTract = bits[4];
    			System.out.println("NGO Agent workTract (E:Work): " +workTract);
			  
    			String id_id = bits[3];
    			System.out.println("NGO Agent ID_ID (D:ROAD_ID): " +id_id);
			
    			String ROAD_ID = bits[3];
    			System.out.println("NGO Agent road segment (D:ROAD_ID): " +ROAD_ID);
    			//System.out.println();
			
    			GeomPlanarGraphEdge startingEdge = idsToEdges.get(
			  		(int) Double.parseDouble(ROAD_ID));
    			GeomPlanarGraphEdge goalEdge = idsToEdges.get(
			  		(int) Double.parseDouble(workTract));
    			//goals1[ random.nextInt(goals1.length)]);
    			
    			for (int i = 0; i < pop; i++)	{
    				//pop; i++)	{ 	// NO IDEA IF THIS MAKES A DIFFERENCE!?!
    				NGOAgent a = new NGOAgent(this, homeTract, workTract, startingEdge, goalEdge);
    				/*
    				 System.out.println(
    				 "NGO Agent " + i + ":	" 
						+ " Home: " +homeTract + ";	"
			  			+ "	Work: " +workTract + ";	"
			  			+ " Starting Edge: " +startingEdge + ";"
			  			+ "	Pop: " +pop + ";	"
			  			+ "	id_id: " +id_id + ";	"
			  			+ "	Road_ID: " +ROAD_ID);
		  			*/
    				boolean successfulStart = a.start(this);
    				
    				if (!successfulStart)	{
    					System.out.println("NGO agents added successfully!!");
    					continue; // DON'T ADD IT if it's bad
    					}
    				
    				MasonGeometry newGeometry = a.getGeometry();
    				newGeometry.isMovable = true;
    				ngoagents.addGeometry(newGeometry);
    				ngoAgentList.add(a);
    				schedule.scheduleRepeating(a);
    				}
    			}
    		d.close();
    		} catch (Exception e) {
    			System.out.println();
    			System.out.println("ERROR: issue with NGOAgent file: ");
    			e.printStackTrace();
    			}
    	}
    
    //////////////////////// ElderlyAgent ///////////////////////////
    public void populateElderly(String filename)	{
    	try	{
    		String filePath = EngDBasicCopy.class.getResource(filename).getPath();
    		FileInputStream fstream = new FileInputStream(filePath);
    		System.out.println();
    		System.out.println("Populating model with Elderly Agents: " +filePath);

    		BufferedReader d = new BufferedReader(new InputStreamReader(fstream));
    		String s;

    		// get rid of the header
    		d.readLine();
    		// read in all data
    		while ((s = d.readLine()) != null)	{
    			String[] bits = s.split(",");

    			int pop = Integer.parseInt(bits[2]);
    			//System.out.println();
    			System.out.println("Elderly Agent road segment population (C:Count): " +pop);

    			String homeTract = bits[3];
    			System.out.println("Elderly Agent homeTract (D:ROAD_ID): " +homeTract);

    			String workTract = bits[4];
    			System.out.println("Elderly Agent workTract (E:Work): " +workTract);

    			String id_id = bits[3];
    			System.out.println("Elderly Agent ID_ID (D:ROAD_ID): " +id_id);

    			String ROAD_ID = bits[3];
    			System.out.println("Elderly Agent road segment (D:ROAD_ID): " +ROAD_ID);
    			//System.out.println();

    			GeomPlanarGraphEdge startingEdge = idsToEdges.get(
    					(int) Double.parseDouble(ROAD_ID));
    			GeomPlanarGraphEdge goalEdge = idsToEdges.get(
    					(int) Double.parseDouble(workTract));
    			//goals2[ random.nextInt(goals2.length)]);

    			for (int i = 0; i < pop; i++)	{
    				//pop; i++)	{ 	// NO IDEA IF THIS MAKES A DIFFERENCE!?!
    				ElderlyAgent a = new ElderlyAgent(this, homeTract, workTract, startingEdge, goalEdge);
    				/*
    				 System.out.println(
    				 "Elderly Agent " + i + ":	" 
    				 + " Home: " +homeTract + ";	"
    				 + "	Work: " +workTract + ";	"
    				 + " Starting Edge: " +startingEdge + ";"
    				 + "	Pop: " +pop + ";	"
    				 + "	id_id: " +id_id + ";	"
    				 + "	Road_ID: " +ROAD_ID);*/
    				boolean successfulStart = a.start(this);
    				if (!successfulStart)	{
    					System.out.println("Elderly agents added successfully!");
    					continue; // DON'T ADD IT if it's bad
    					}
    				MasonGeometry newGeometry = a.getGeometry();
    				newGeometry.isMovable = true;
    				elderlyAgentsLayer.addGeometry(newGeometry);
    				elderlyAgentList.add(a);
    				schedule.scheduleRepeating(a);
    				}
    			}
    		d.close();
    		
    	} catch (Exception e) {
    		System.out.println();
    		System.out.println("ERROR: issue with ElderlyAgent file: ");
    		e.printStackTrace();
    		}
    	}

	//////////////////// LimitedActions Agent ////////////////////////////
	public void populateLimitedActions(String filename)	{
		try	{
			String filePath = EngDBasicCopy.class.getResource(filename).getPath();
			FileInputStream fstream = new FileInputStream(filePath);
			System.out.println();
			System.out.println("Populating model with Limited Actions Agents: " +filePath);
			
			BufferedReader d = new BufferedReader(new InputStreamReader(fstream));
			String s;
			
			// get rid of the header
			d.readLine();
			// read in all data
			while ((s = d.readLine()) != null)	{
				String[] bits = s.split(",");
				
				int pop = Integer.parseInt(bits[2]);
				//System.out.println();
				System.out.println("LimitedActions Agent road segment population (C:Count): " +pop);
				
				String homeTract = bits[3];
				System.out.println("LimitedActions Agent homeTract (D:ROAD_ID): " +homeTract);
				
				String workTract = bits[4];
				System.out.println("LimitedActions Agent workTract (E:Work): " +workTract);
				
				String id_id = bits[3];
				System.out.println("LimitedActions Agent ID_ID (D:ROAD_ID): " +id_id);
				
				String ROAD_ID = bits[3];
				System.out.println("LimitedActions Agent road segment (D:ROAD_ID): " +ROAD_ID);
				//System.out.println();
				
				GeomPlanarGraphEdge startingEdge = idsToEdges.get(
						(int) Double.parseDouble(ROAD_ID));
				GeomPlanarGraphEdge goalEdge = idsToEdges.get(
						(int) Double.parseDouble(workTract));
				//goals3[ random.nextInt(goals3.length)]);
				
				for (int i = 0; i < pop; i++)	{
					//pop; i++)	{ 	// NO IDEA IF THIS MAKES A DIFFERENCE!?!
					LimitedActionsAgent a = new LimitedActionsAgent(this, homeTract, workTract, startingEdge, goalEdge);
					/*System.out.println(
					 	"Limited Actions Agent " + i + ":	" 
	      				+ " Home: " +homeTract + ";	"
	              		+ "	Work: " +workTract + ";	"
	              		//+ " Starting Edge: " +startingEdge + ";"
	              		+ "	Pop: " +pop + ";	"
	              		+ "	id_id: " +id_id + ";	"
	              		+ "	Road_ID: " +ROAD_ID);*/
					boolean successfulStart = a.start(this);
					
					if (!successfulStart)	{
						System.out.println("Limited Actions agents successfully added!");
						continue; // DON'T ADD IT if it's bad	
					}
					
					MasonGeometry newGeometry = a.getGeometry();
					newGeometry.isMovable = true;
					limitedActionsAgentsLayer.addGeometry(newGeometry);
					limitedActionsAgentList.add(a);
					schedule.scheduleRepeating(a);	
				}
			}
			
			d.close();
			
		} catch (Exception e) {
			System.out.println();
			System.out.println("ERROR: issue with LimitedActionaAgents file: ");
			e.printStackTrace();
		}	
	}			

	/** adds nodes corresponding to road intersections to GeomVectorField
	 *
	 * @param nodeIterator Points to first node
	 * @param intersections GeomVectorField containing intersection geometry
	 *
	 * Nodes will belong to a planar graph populated from LineString network.
	 */
	private void addIntersectionNodes(Iterator<?> nodeIterator,
	                                  GeomVectorField intersections)	{
		GeometryFactory fact = new GeometryFactory();
	    Coordinate coord = null;
	    Point point = null;
	    @SuppressWarnings("unused")
	    
	    int counter = 0;
	    
	    while (nodeIterator.hasNext())	{
	    	Node node = (Node) nodeIterator.next();
	        coord = node.getCoordinate();
	        point = fact.createPoint(coord);

	        junctions.addGeometry(new MasonGeometry(point));
	        counter++;
	    }
	}

    /**
     * Main function allows simulation to be run in stand-alone, non-GUI mode
     */
    public static void main(String[] args)	{
        doLoop(EngDBasicCopy.class, args);
        System.exit(0);
    }
}
