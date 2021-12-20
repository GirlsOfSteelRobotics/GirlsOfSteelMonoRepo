/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package girlsofsteel.commands;

import girlsofsteel.subsystems.Collector;

/**
 * Moves the collector wheel backwards.
 * @author Abby, Sophia, Sonia
 */
public class CollectorWheelReverse extends CommandBase{

    private final Collector m_collector;

    public CollectorWheelReverse(Collector collector) {
        m_collector = collector;
    }

    /**
     * There is nothing in this method.
     * @author Sophia, Sonia
     */
    @Override
    protected void initialize() {

    }

    /**
     * This rolls the collector wheel backwards.
     * It can be used to release the ball.
     * @author Sophia, Sonia
     */
    @Override
    protected void execute() {
        m_collector.collectorWheelReverse(); //use specific method for clarity
    }
    //The wheel reverses to release the ball. Possibly for both gentle and long passes.

       /**
     * This returns false when we want to move the wheel backwards .
     * @return false always
     * @author Sophia, Sonia
     */
       @Override
    protected boolean isFinished() {
        return false;
    }

     /**
     * This stops the collector wheel.
     * @author Sophia, Sonia
     */
     @Override
    protected void end() {
        m_collector.stopCollectorWheel();
        //stops wheel once the command is finished
    }

     /**
     * This calls the end() method to stop the collector wheel
     * @author Sophia, Sonia
     */
     @Override
    protected void interrupted() {
        end();
    }

}
