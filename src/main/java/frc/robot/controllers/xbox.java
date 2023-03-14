package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.Controller;

public class xbox {

    private final XboxController controller;

    public final JoystickButton buttonA, buttonX, buttonY, buttonB;
    public final JoystickButton buttonBack, buttonStart;
    public final JoystickButton leftBumper, rightBumper;
    public final JoystickButton leftStickClick, rightStickClick;

    // D-Pad
    public POVButton dPadUp, dPadRight, dPadDown, dPadLeft;

    public Trigger leftTrigger, rightTrigger;

    /**
     * @param port The port index on the Driver Station that the controller is
     *             plugged into.
     */
    public xbox(int port) {
        controller = new XboxController(port);

        buttonA = createButton(XboxController.Button.kA.value);
        buttonX = createButton(XboxController.Button.kX.value);
        buttonY = createButton(XboxController.Button.kY.value);
        buttonB = createButton(XboxController.Button.kB.value);
        buttonBack = createButton(XboxController.Button.kBack.value);
        buttonStart = createButton(XboxController.Button.kStart.value);

        leftBumper = createButton(XboxController.Button.kLeftBumper.value);
        rightBumper = createButton(XboxController.Button.kRightBumper.value);

        leftStickClick = createButton(XboxController.Button.kLeftStick.value);
        rightStickClick = createButton(XboxController.Button.kRightStick.value);

        dPadUp = new POVButton(controller, 0);
        dPadRight = new POVButton(controller, 90);
        dPadDown = new POVButton(controller, 180);
        dPadLeft = new POVButton(controller, 270);

        leftTrigger = new Trigger(() -> getLeftTrigger() > Controller.kTriggerDeadband);
        rightTrigger = new Trigger(() -> getRightTrigger() > Controller.kTriggerDeadband);
    }

    // The X (left/right) position of the right joystick on the controller from -1.0 to 1.0
    public double getRightStickX() {
        return deadBandCalc(controller.getRightX(), Controller.kXDeadband);
    }

    // The Y (up/down) position of the right joystick on the controller from -1.0 to 1.0
    public double getRightStickY() {
        return deadBandCalc(controller.getRightY(), Controller.kYDeadband);
    }

    // The X (left/right) position of the left joystick on the controller from -1.0 to 1.0
    public double getLeftStickX() {
        return deadBandCalc(controller.getLeftX(), Controller.kXDeadband);
    }

    // The Y (up/down) position of the left joystick on the controller from -1.0 to 1.0
    public double getLeftStickY() {
        return deadBandCalc(controller.getLeftY(), Controller.kYDeadband);
    }

    /** How much the left trigger on the controller is pressed from 0.0 to 1.0 */
    public double getLeftTrigger() {
        return deadBandCalc(controller.getLeftTriggerAxis(), Controller.kTriggerDeadband);
    }

    /** How much the right trigger on the controller is pressed from 0.0 to 1.0 */
    public double getRightTrigger() {
        return deadBandCalc(controller.getRightTriggerAxis(), Controller.kTriggerDeadband);
    }

    /*
     * Utilities
     */
    private JoystickButton createButton(int buttonID) {
        return new JoystickButton(this.controller, buttonID);
    }

    private double deadBandCalc(double x, double deadBand) {
        if (Math.abs(x) > deadBand)
            return x;
        else
            return 0.0;
    }
}
