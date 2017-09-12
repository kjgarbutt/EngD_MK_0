package sim;

import java.awt.Color;

import javax.swing.JFrame;

import objects.agents.Agent;
import objects.agents.ElderlyAgent;
import objects.agents.LimitedActionsAgent;
import objects.agents.NGOAgent;

import org.jfree.data.xy.XYSeries;

import sim.display.Console;
import sim.display.Controller;
import sim.display.Display2D;
import sim.display.GUIState;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.portrayal.geo.GeomPortrayal;
import sim.portrayal.geo.GeomVectorFieldPortrayal;
import sim.util.media.chart.TimeSeriesChartGenerator;

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
public class EngDBasicWithUICopy extends GUIState	{

	///////////////////////////////////////////////////////////////////////////
	/////////////////////////// DISPLAY FUNCTIONS /////////////////////////////
	///////////////////////////////////////////////////////////////////////////

	private Display2D display;
    private JFrame displayFrame;

    private GeomVectorFieldPortrayal lsoaPortrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal roadsPortrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal flood3Portrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal flood2Portrayal = new GeomVectorFieldPortrayal();
    //private GeomVectorFieldPortrayal HouseholdsFZPortrayal = new GeomVectorFieldPortrayal();
    //private GeomVectorFieldPortrayal HouseholdsPortrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal agentPortrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal ngoAgentPortrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal elderlyagentPortrayal = new GeomVectorFieldPortrayal();
    private GeomVectorFieldPortrayal limactagentPortrayal = new GeomVectorFieldPortrayal();
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
    protected EngDBasicWithUICopy(SimState state)	{
            super(state);
        }

        /**
         * Main function to run the simulation
         * @param args
         */
        public static void main(String[] args)	{
        	EngDBasicWithUICopy simple = new EngDBasicWithUICopy(
        			new EngDBasicCopy(System.currentTimeMillis()));
            Console c = new Console(simple);
            c.setVisible(true);
        }


        /**
         * @return name of the simulation
         */
        public static String getName()	{
            return "EngD ABM Model MK_4";
        }


        /**
         *  This must be included to have model tab, which allows mid-simulation
         *  modification of the coefficients
         */
        public Object getSimulationInspectedObject()	{
            return state;
        }  // non-volatile


        /**
         * Called when starting a new run of the simulation. Sets up the portrayals
         * and chart data.
         */
        public void start()	{
            super.start();

            EngDBasicCopy world = (EngDBasicCopy) state;

            maxSpeed = new XYSeries("Max Speed");
            avgSpeed = new XYSeries("Average Speed");
            minSpeed = new XYSeries("Min Speed");
            trafficChart.removeAllSeries();
            trafficChart.addSeries(maxSpeed, null);
            trafficChart.addSeries(avgSpeed, null);
            trafficChart.addSeries(minSpeed, null);

            state.schedule.scheduleRepeating(new Steppable()	{
				private static final long serialVersionUID = -3749005402522867098L;

				public void step(SimState state)	{
					EngDBasicCopy world = (EngDBasicCopy) state;
                    double maxS = 0, minS = 10000, avgS = 0, count = 0;
                    /////////////// Main Agent ///////////////////////
                    for (Agent a : world.agentList)	{
                        if (a.reachedDestination)	{
                            continue;
                        }
                        count++;
                        double speed = Math.abs(a.speed);
                        avgS += speed;
                        if (speed > maxS)	{
                            maxS = speed;
                        }
                        if (speed < minS)	{
                            minS = speed;
                        }
                    }
                    
                    /////////////// NGO Agent /////////////////////
                    for (NGOAgent b : world.ngoAgentList)	{
                        if (b.reachedDestination)	{
                            continue;
                        }
                        count++;
                        double speed = Math.abs(b.speed);
                        avgS += speed;
                        if (speed > maxS)	{
                            maxS = speed;
                        }
                        if (speed < minS)	{
                            minS = speed;
                        }
                    }
                    /////////////// Elderly Agent ////////////////////
                    for (ElderlyAgent c : world.elderlyAgentList)	{
                        if (c.reachedDestination)	{
                            continue;
                        }
                        count++;
                        double speed = Math.abs(c.speed);
                        avgS += speed;
                        if (speed > maxS)	{
                            maxS = speed;
                        }
                        if (speed < minS)	{
                            minS = speed;
                        }
                    }
                    /////////////// Limited Actions Agent ////////////////////
                    for (LimitedActionsAgent d : world.limitedActionsAgentList)	{
                        if (d.reachedDestination)	{
                            continue;
                        }
                        count++;
                        double speed = Math.abs(d.speed);
                        avgS += speed;
                        if (speed > maxS)	{
                            maxS = speed;
                        }
                        if (speed < minS)	{
                            minS = speed;
                        }
                    }
                    
                    @SuppressWarnings("deprecation")
					double time = state.schedule.time();
                    avgS /= count;
                    maxSpeed.add(time, maxS, true);
                    minSpeed.add(time, minS, true);
                    avgSpeed.add(time, avgS, true);
                }
            });

        	/**
        	 * Sets up the portrayals within the map visualization.
        	 */

            roadsPortrayal.setField(world.roadLayer);
            roadsPortrayal.setPortrayalForAll(new GeomPortrayal(Color.DARK_GRAY, 0.0005, false));

            lsoaPortrayal.setField(world.baseLayer);
            lsoaPortrayal.setPortrayalForAll(new GeomPortrayal(Color.LIGHT_GRAY, true));

            //flood3Portrayal.setField(world.flood3);
            //flood3Portrayal.setPortrayalForAll(new GeomPortrayal(Color.CYAN, true));

            //flood2Portrayal.setField(world.flood2);
            //flood2Portrayal.setPortrayalForAll(new GeomPortrayal(Color.BLUE, true));
            
            //HouseholdsFZPortrayal.setField(world.HouseholdsFZ);
            //HouseholdsFZPortrayal.setPortrayalForAll(new GeomPortrayal(Color.YELLOW, 50, true));
            
            //HouseholdsPortrayal.setField(world.Households);
            //HouseholdsPortrayal.setPortrayalForAll(new GeomPortrayal(Color.ORANGE, 50, true));

            agentPortrayal.setField(world.agentsLayer);
            agentPortrayal.setPortrayalForAll(new GeomPortrayal(Color.GREEN, 125, true));
            
            ngoAgentPortrayal.setField(world.ngoAgentsLayer);
            ngoAgentPortrayal.setPortrayalForAll(new GeomPortrayal(Color.RED, 125, true));
            
            elderlyagentPortrayal.setField(world.elderlyAgentsLayer);
            elderlyagentPortrayal.setPortrayalForAll(new GeomPortrayal(Color.PINK, 125, true));
            
            limactagentPortrayal.setField(world.limitedActionsAgentsLayer);
            limactagentPortrayal.setPortrayalForAll(new GeomPortrayal(Color.ORANGE, 125, true));
            
            //agentPortrayal.setPortrayalForAll(new sim.portrayal.simple.OvalPortrayal2D(Color.GREEN, 1));
            //ngoAgentPortrayal.setPortrayalForAll(new sim.portrayal.simple.RectanglePortrayal2D(Color.RED, 1));
            //elderlyagentPortrayal.setPortrayalForAll(new sim.portrayal.simple.HexagonalPortrayal2D(Color.GRAY, 1));
            //limactagentPortrayal.setPortrayalForAll(new sim.portrayal.simple.OvalPortrayal2D(Color.PINK, 1));
            
            //tractsPortrayal.setPortrayalForAll(new PolyPortrayal());//(Color.GREEN,true));
            //roadsPortrayal.setPortrayalForAll(new RoadPortrayal());//GeomPortrayal(Color.DARK_GRAY,0.001,false));

            display.reset();
            display.setBackdrop(Color.WHITE);
            display.repaint();

        }


        /**
         * Initializes the simulation visualization. Sets up the display
         * window, the JFrames, and the chart structure.
         */
        public void init(Controller c)
        {
            super.init(c);

            /////////////////// MAIN DISPLAY ////////////////////////
            // makes the displayer and visualises the maps
            display = new Display2D(1300, 600, this);
            // turn off clipping
            // display.setClipping(false);

            displayFrame = display.createFrame();
            displayFrame.setTitle("EngD ABM Model MK_2");
            c.registerFrame(displayFrame); // register the frame so it appears in

            // Put portrayals in order from bottom layer to top
            displayFrame.setVisible(true);
            display.attach(lsoaPortrayal, "LSOA");
            display.attach(flood2Portrayal, "FZ2 Zone");
            display.attach(flood3Portrayal, "FZ3 Zone");
            //display.attach(HouseholdsPortrayal, "Households not in FZ");
            //display.attach(HouseholdsFZPortrayal, "Households in FZ");
            display.attach(roadsPortrayal, "Roads");
            display.attach(agentPortrayal, "Agents");
            display.attach(ngoAgentPortrayal, "NGO Agents");
            display.attach(limactagentPortrayal, "Limited Actions Agents");
            display.attach(elderlyagentPortrayal, "Elderly Agents");

            ////////////////////// CHART ///////////////////////////
            trafficChart = new TimeSeriesChartGenerator();
            trafficChart.setTitle("Traffic Stats");
            trafficChart.setYAxisLabel("Speed");
            trafficChart.setXAxisLabel("Time");
            @SuppressWarnings("deprecation")
			JFrame chartFrame = trafficChart.createFrame(this);
            chartFrame.pack();
            c.registerFrame(chartFrame);

        }


        /**
         * Quits the simulation and cleans up.
         */
        public void quit()	{
        	System.out.println("Model closed.");
        	super.quit();

            if (displayFrame != null)	{
                displayFrame.dispose();
            }
            displayFrame = null; // let gc
            display = null; // let gc
        }
    }
