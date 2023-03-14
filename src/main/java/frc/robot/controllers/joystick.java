package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controller;

public class joystick {

    private final Joystick controller;
    public final JoystickButton button7;
    public final JoystickButton button8;

    /**
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public joystick(int port) {
        controller = new Joystick(port);
        button7 = createButton(7);
        button8 = createButton(8);
    }

    /** The X (left/right) position of the joystick from -1.0 to 1.0 */
    public double getStickX() {
        return deadBandCalc(controller.getX(), Controller.kXDeadband);
    }

    /** The Y (up/down) position of the joystick from -1.0 to 1.0 */
    public double getStickY() {
        return deadBandCalc(controller.getY(), Controller.kYDeadband);
    }

    /** The Z (rotation) position of the joystick from -1.0 to 1.0 */
    public double getStickZ() {
        return deadBandCalc(controller.getZ(), Controller.kZDeadband);
    }

    /*
     * Utilities
     */
    private JoystickButton createButton(int buttonID) {
        return new JoystickButton(this.controller, buttonID);
    }

    private double deadBandCalc(double x, double deadBand) {
        if (Math.abs(x) > deadBand) return x;
        else return 0.0;
    }
}
