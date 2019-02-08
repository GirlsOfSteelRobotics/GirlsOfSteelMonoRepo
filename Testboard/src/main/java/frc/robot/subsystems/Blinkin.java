package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Blinkin extends Subsystem {
    private Spark leftLightController;
    private Spark rightLightController;

    private static double test = -0.99;

    public enum LightPattern {
        AUTO_DEFAULT, TELEOP_DEFAULT, HATCH_RELEASE,
    }

    public Blinkin() {
        leftLightController = new Spark(RobotMap.LED_LEFT_PORT);
        rightLightController = new Spark(RobotMap.LED_RIGHT_PORT);

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void setLightPattern(LightPattern pattern) {
        System.out.println("Setting patttern to: " + pattern);
        System.out.println(test);
        switch (pattern) {
        case AUTO_DEFAULT:
            leftLightController.set(test);
            rightLightController.set(test);
            break;
        case TELEOP_DEFAULT:
            leftLightController.set(test);
            rightLightController.set(test);
            break;
        case HATCH_RELEASE:
            leftLightController.set(test);
            rightLightController.set(test);
            System.out.println("reset color");
            break;
        default:
            break;
        }
    }

    public void setOutputValue() {

    }

    public void stop() {

    }
}
