package com.gos.crescendo.shuffleboard.super_structure;

import com.gos.crescendo.shuffleboard.Utils;
import javafx.beans.binding.Bindings;
import javafx.beans.binding.DoubleBinding;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Arc;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;

public class SuperStructureController {

    private static final double MAX_WIDTH = 15; // TODO figure out real value
    private static final double MAX_HEIGHT = 15; // TODO figure out real value

    private static final double CHASSIS_HEIGHT = 2;
    private static final double CHASSIS_WIDTH = 8;
    private static final double PIVOT_MOTOR_RADIUS = 1;
    private static final double ARM_RECT_HEIGHT = 6;
    private static final double ARM_RECT_WIDTH = 1;
    private static final double SHOOTER_RECT_HEIGHT = 1;
    private static final double SHOOTER_RECT_WIDTH = 6;
    private static final double SHOOTER_MOTOR_RADIUS = 0.5;
    private static final double INTAKE_MOTOR_RADIUS = 0.5;
    private static final double PIVOT_ANGLE_GOAL_HEIGHT = ARM_RECT_HEIGHT;
    private static final double PIVOT_ANGLE_GOAL_WIDTH = ARM_RECT_WIDTH;


    private static final double CHASSIS_X = 3;
    private static final double CHASSIS_Y = 9;
    private static final double PIVOT_MOTOR_X = 3.5;
    private static final double PIVOT_MOTOR_Y = 9;
    private static final double ARM_RECT_X = 3;
    private static final double ARM_RECT_Y = 3;
    private static final double SHOOTER_RECT_X = 2;
    private static final double SHOOTER_RECT_Y = 2;
    private static final double SHOOTER_MOTOR_X = 1.75;
    private static final double SHOOTER_MOTOR_Y = 2.5;
    private static final double INTAKE_MOTOR_X = 8.25;
    private static final double INTAKE_MOTOR_Y = 2.5;
    private static final double PIVOT_ANGLE_GOAL_X = ARM_RECT_X;
    private static final double PIVOT_ANGLE_GOAL_Y = ARM_RECT_Y;

    @FXML
    private Group m_group;

    @FXML
    private Pane m_pane;
    
    @FXML
    private Rectangle m_Chassis;

    @FXML
    private Circle m_pivotMotor;

    @FXML
    private Rectangle m_ArmRect;

    @FXML
    private Rectangle m_shooterRect;

    @FXML
    private Circle m_shooterMotor;

    @FXML
    private Circle m_IntakeMotor;

    @FXML
    private Rectangle m_pivotAngleGoal;

    private Rotate m_pivotArmRotate;

    private Rotate m_pivotArmGoalRotate;

    @FXML
    public void initialize() {

        ///////////////////////////////////////////////////////////
        // Controls the inches <-> pixels conversion. Don't touch
        double minWidthMultiplier = 1;
        m_pane.setMinHeight(MAX_HEIGHT * minWidthMultiplier);
        m_pane.setMinWidth(MAX_WIDTH * minWidthMultiplier);

        DoubleBinding scaleBinding = Bindings.createDoubleBinding(() -> Math.min(m_pane.getWidth() / MAX_WIDTH, m_pane.getHeight() / MAX_HEIGHT), m_pane.widthProperty(), m_pane.heightProperty());

        Scale scale = new Scale();
        scale.xProperty().bind(scaleBinding);
        scale.yProperty().bind(scaleBinding);

        m_group.getTransforms().add(scale);
        ///////////////////////////////////////////////////////////

        m_Chassis.setX(CHASSIS_X);
        m_Chassis.setY(CHASSIS_Y);
        m_Chassis.setHeight(CHASSIS_HEIGHT);
        m_Chassis.setWidth(CHASSIS_WIDTH);

        m_pivotMotor.setCenterX(PIVOT_MOTOR_X);
        m_pivotMotor.setCenterY(PIVOT_MOTOR_Y);
        m_pivotMotor.setRadius(PIVOT_MOTOR_RADIUS);

        m_ArmRect.setX(ARM_RECT_X);
        m_ArmRect.setY(ARM_RECT_Y);
        m_ArmRect.setHeight(ARM_RECT_HEIGHT);
        m_ArmRect.setWidth(ARM_RECT_WIDTH);

        m_shooterRect.setX(SHOOTER_RECT_X);
        m_shooterRect.setY(SHOOTER_RECT_Y);
        m_shooterRect.setHeight(SHOOTER_RECT_HEIGHT);
        m_shooterRect.setWidth(SHOOTER_RECT_WIDTH);

        m_shooterMotor.setCenterX(SHOOTER_MOTOR_X);
        m_shooterMotor.setCenterY(SHOOTER_MOTOR_Y);
        m_shooterMotor.setRadius(SHOOTER_MOTOR_RADIUS);

        m_IntakeMotor.setCenterX(INTAKE_MOTOR_X);
        m_IntakeMotor.setCenterY(INTAKE_MOTOR_Y);
        m_IntakeMotor.setRadius(INTAKE_MOTOR_RADIUS);

        m_pivotAngleGoal.setX(PIVOT_ANGLE_GOAL_X);
        m_pivotAngleGoal.setY(PIVOT_ANGLE_GOAL_Y);
        m_pivotAngleGoal.setHeight(PIVOT_ANGLE_GOAL_HEIGHT);
        m_pivotAngleGoal.setWidth(PIVOT_ANGLE_GOAL_WIDTH);

        m_pivotArmRotate = new Rotate();
        m_pivotArmRotate.setAngle(0);
        m_pivotArmRotate.pivotXProperty().bind(Bindings.createObjectBinding(() -> m_ArmRect.getX()+ARM_RECT_WIDTH, m_ArmRect.xProperty()));
        m_pivotArmRotate.pivotYProperty().bind(Bindings.createObjectBinding(() -> m_ArmRect.getY()+ARM_RECT_HEIGHT, m_ArmRect.yProperty()));
        m_ArmRect.getTransforms().add(m_pivotArmRotate);
        m_shooterRect.getTransforms().add(m_pivotArmRotate);
        m_IntakeMotor.getTransforms().add(m_pivotArmRotate);
        m_shooterMotor.getTransforms().add(m_pivotArmRotate);

        m_pivotArmGoalRotate = new Rotate();
        m_pivotArmGoalRotate.setAngle(0);
        m_pivotArmGoalRotate.pivotXProperty().bind(Bindings.createObjectBinding(() -> m_ArmRect.getX()+ARM_RECT_WIDTH, m_ArmRect.xProperty()));
        m_pivotArmGoalRotate.pivotYProperty().bind(Bindings.createObjectBinding(() -> m_ArmRect.getY()+ARM_RECT_HEIGHT, m_ArmRect.yProperty()));
        m_pivotAngleGoal.getTransforms().add(m_pivotArmGoalRotate);

    }


    public void updateSuperStructure(SuperStructureData superStructureData) {

        m_pivotMotor.setFill(Utils.getMotorColor(superStructureData.getPivotMotorPercentage()));
        m_pivotArmRotate.setAngle(superStructureData.getPivotMotorAngle());
        m_pivotArmGoalRotate.setAngle(superStructureData.getGoalAngle());

        m_shooterMotor.setFill(Utils.getMotorColor(superStructureData.getShooterMotorPercentage()));

        if (superStructureData.isHasGamePiece()) {
            m_shooterRect.setFill(Color.ORANGE);
        }
        else {
            m_shooterRect.setFill(Color.TRANSPARENT);
        }

        if (superStructureData.getGoalAngle() == Double.MIN_VALUE) {
            m_pivotAngleGoal.setStroke(Color.TRANSPARENT);
        } else {
            m_pivotAngleGoal.setStroke(Color.BLACK);
        }
    }


}
