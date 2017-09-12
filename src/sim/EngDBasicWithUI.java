package sim;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.JFrame;

import org.jfree.data.xy.XYSeries;

import sim.display.Console;
import sim.display.Controller;
import sim.display.Display2D;
import sim.display.GUIState;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.field.geo.GeomVectorField;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.FieldPortrayal2D;
import sim.portrayal.geo.GeomPortrayal;
import sim.portrayal.geo.GeomVectorFieldPortrayal;
import sim.util.gui.SimpleColorMap;
import sim.util.media.chart.TimeSeriesChartGenerator;
import vis.AttributePolyPortrayal;

/**
 *
 * A simple model that locates agents on Norfolk's road network and makes them
 * move from A to B, then they change direction and head back to the start.
 * The process repeats until the user quits. The number of agents, their start
 * and end points is determined by data in NorfolkITNLSOA.csv and assigned by the
 * user under 'goals' (approx. Line 80 in main file).
 *
 * @author KJGarbutt
 *
 */
public class EngDBasicWithUI extends GUIState	{

	///////////////////////////////////////////////////////////////////////////
	/////////////////////////// DISPLAY FUNCTIONS /////////////////////////////
	///////////////////////////////////////////////////////////////////////////

	EngDBasic sim;
	public Display2D display;
    public JFrame displayFrame;

    private GeomVectorFieldPortrayal map = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal lsoaPortrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal roadsPortrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal agentsLayerPortrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal flood3Portrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal flood2Portrayal = new GeomVectorFieldPortrayal();
    TimeSeriesChartGenerator trafficChart;
    XYSeries maxSpeed;
    XYSeries avgSpeed;
    XYSeries minSpeed;

    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////// BEGIN FUNCTIONS ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    /**
     * Default constructor
     */
    public EngDBasicWithUI(SimState state)	{
	    super(state);
	    sim = (EngDBasic) state;
    }


    /**
     * Called when starting a new run of the simulation. Sets up the portrayals
     * and chart data.
     */
    public void start()	{
        super.start();

        setupPortrayals();
    }


	/** Loads the simulation from a point */
	public void load(SimState state) {
		super.load(state);

		// we now have new grids. Set up the portrayals to reflect that
		setupPortrayals();
	}


	/**
	 * Sets up the portrayals within the map visualization.
	 */
    public void setupPortrayals() {

    	EngDBasic world = (EngDBasic) state;
		map.setField(world.baseLayer);
		map.setPortrayalForAll(new AttributePolyPortrayal(
				new SimpleColorMap(0,13000, new Color(100,80,30), new Color(240,220,200)),
				"DP0010001", new Color(0,0,0,0), true));
		map.setImmutableField(true);

		roadsPortrayal.setField(world.roadLayer);
		roadsPortrayal.setPortrayalForAll(new GeomPortrayal(Color.DARK_GRAY, 0.5, false));

		lsoaPortrayal.setField(world.baseLayer);
        lsoaPortrayal.setPortrayalForAll(new GeomPortrayal(Color.LIGHT_GRAY, true));

        flood3Portrayal.setField(world.flood3);
        flood3Portrayal.setPortrayalForAll(new GeomPortrayal(Color.CYAN, true));

        flood2Portrayal.setField(world.flood2);
        flood2Portrayal.setPortrayalForAll(new GeomPortrayal(Color.BLUE, true));

        agentsLayerPortrayal.setField(world.agentsLayer);
        //agentsLayerPortrayal.setPortrayalForAll(new GeomPortrayal(Color.GREEN, true));
        agentsLayerPortrayal.setPortrayalForAll(new GeomPortrayal(Color.GREEN, 125, true));

		// reset stuff
		// reschedule the displayer
        display.reset();
        display.setBackdrop(Color.WHITE);
        // redraw the display
        display.repaint();
    }


	/**
     * Initializes the simulation visualization. Sets up the display
     * window, the JFrames, and the chart structure.
     */
    public void init(Controller c)	{
        super.init(c);

        /////////////////// MAIN DISPLAY ////////////////////////
        // makes the displayer and visualises the maps
        display = new Display2D(1300, 600, this);
        // display = new Display2D((int)(1.5 * EngDBasic.grid_width), (int)(1.5 * EngDBasic.grid_height), this);
        // turn off clipping
        // display.setClipping(false);

        // Put portrayals in order from bottom layer to top
        //display.attach(map, "Landscape");
        display.attach(lsoaPortrayal, "LSOA");
        display.attach(flood2Portrayal, "FZ2 Zone");
        display.attach(flood3Portrayal, "FZ3 Zone");
        display.attach(roadsPortrayal, "Roads");
        display.attach(agentsLayerPortrayal, "Agents");

        ////////////////////// TIMESTAMP /////////////////////////////
		display.attach(new FieldPortrayal2D()	{
			private static final long serialVersionUID = 1L;

			Font font = new Font("SansSerif", 0, 24);  // keep it around for efficiency
		    SimpleDateFormat ft = new SimpleDateFormat ("yyyy-MM-dd HH:mm zzz");
		    public void draw(Object object, Graphics2D graphics, DrawInfo2D info)	{
		        String s = "";
		        if (state !=null) // if simulation has not begun or has finished, indicate this
		            s = state.schedule.getTimestamp("Before Simulation", "Simulation Finished");
		        graphics.setColor(Color.white);
		        if (state != null){
		        	// specify the timestep here
		        	Date startDate;
					try {
						startDate = ft.parse("2016-06-29 00:00 GMT");
				        Date time = new Date((int)state.schedule.getTime() * 300000 + startDate.getTime());
				        s = ft.format(time);
					} catch (ParseException e) {
						e.printStackTrace();
					}
		        }

		        graphics.drawString(s, (int)info.clip.x + 10,
		                (int)(info.clip.y + 10 + font.getStringBounds(s,graphics.getFontRenderContext()).getHeight()));

		        }
		    }, "Time");

		displayFrame = display.createFrame();
		displayFrame.setTitle("EngD Basic Model");
		c.registerFrame(displayFrame); // register the frame so it appears in the "Display" list
		displayFrame.setVisible(true);
    }


    /**
     * Quits the simulation and cleans up.
     */
    public void quit()	{
    	System.out.println("Model closed.");
    	super.quit();

        if (displayFrame != null)
            displayFrame.dispose();
        displayFrame = null; // let gc
        display = null; // let gc
    }


    /**
     * Main function to run the simulation
     * @param args
     */
    public static void main(String[] args)	{
    	EngDBasicWithUI gui = null;

		try {
			EngDBasic lb = new EngDBasic(12345);	//System.currentTimeMillis());
			gui = new EngDBasicWithUI(lb);
		} catch (Exception ex){
			System.out.println(ex.getStackTrace());
		}

    	Console console = new Console(gui);
        console.setVisible(true);
    }


    /**
     * @return name of the simulation
     */
    public static String getName()	{
        return "EngD Basic Model";
    }


    /**
     * Allows for users to modify the simulation using the model tab
     */
	public Object getSimulationInspectedObject() {
		return state;
	}
}
