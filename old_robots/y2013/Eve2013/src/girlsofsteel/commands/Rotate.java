package girlsofsteel.commands;

import girlsofsteel.subsystems.Chassis;

public class Rotate extends CommandBase {

    private final Chassis m_chassis;
    private final boolean m_targetRotate;
    private final double m_theta;
    private double m_desiredTheta;
    private double m_current;
    private double m_difference;

    public Rotate(Chassis chassis, double theta, boolean targetRotate) {
        m_chassis = chassis;
        this.m_targetRotate = targetRotate;
        this.m_theta = theta;
        if(targetRotate){
            m_desiredTheta = theta;
        }else{
            m_desiredTheta = chassis.getGyroAngle() -
                    chassis.getFieldAdjustment() + theta;
        }
    }

    @Override
    protected void initialize() {
        if(m_targetRotate){
            m_desiredTheta = m_theta;
        }else{
            m_desiredTheta = m_chassis.getGyroAngle() -
                m_chassis.getFieldAdjustment() + m_theta;
        }
        m_chassis.startAutoRotation();
        //make desired theta between 0-360
        while(m_desiredTheta < 0){
            m_desiredTheta += 360;
        }
        m_desiredTheta = m_desiredTheta % 360;
        getDifference();
        System.out.println("Initializing + ");
    }

    @Override
    protected void execute() {
        getDifference();
        System.out.println("Gyro: " + m_chassis.getGyroAngle() + "\tCurrent: "
                + m_current + "\tDesired: " + m_desiredTheta + "\tDifference: "
                + m_difference);
        if(m_desiredTheta - m_chassis.ROTATION_THRESHOLD > m_current ||
                m_current > m_desiredTheta + m_chassis.ROTATION_THRESHOLD){
            System.out.print("Rotating...");
            m_chassis.autoRotateTestBot(m_difference);
        }else{
            m_chassis.pauseAutoRotation();
        }
    }

    @Override
    protected boolean isFinished() {
        return !m_chassis.isAutoRotating();
    }

    @Override
    protected void end() {
        System.out.println("Stopped rotation");
        m_chassis.stopAutoRotation();
    }

    @Override
    protected void interrupted() {
        end();
    }

    @SuppressWarnings("PMD.LinguisticNaming")
    private void getDifference(){
        //get current
        m_current = m_chassis.getGyroAngle() - m_chassis.getFieldAdjustment();
        //make it between 0-360
        while(m_current < 0){
            m_current += 360;
        }
        m_current = m_current % 360;
        //calculate & set the difference
        if(m_desiredTheta - m_current < 180 && m_desiredTheta - m_current > - 180){
            m_difference = m_desiredTheta - m_current;
        }else{
            if(m_desiredTheta - m_current < 0){
                m_difference = 360 + m_desiredTheta - m_current;
            }else{
                m_difference = 360 - m_desiredTheta - m_current;
            }
        }
    }

}
