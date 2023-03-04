package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CustomConstants;

public class joystick {

    private final Joystick controller;
    public final JoystickButton button7;
    public final JoystickButton button8;
    private final double kDeadzoneStick = CustomConstants.stickDeadband;

    /**
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public joystick(int port) {
        controller = new Joystick(port);
        button7 = createButton(7);
        button8 = createButton(8);
    }

    /** The X (left/right) position of the right joystick on the controller from -1.0 to 1.0 */
    public double getStickX() {
        return deadzone(controller.getX(), kDeadzoneStick);
    }

    /** The Y (up/down) position of the right joystick on the controller from -1.0 to 1.0 */
    public double getStickY() {
        return deadzone(controller.getY(), kDeadzoneStick);
    }

    /** The Y (up/down) position of the right joystick on the controller from -1.0 to 1.0 */
    public double getStickZ() {
        return deadzone(controller.getZ(), kDeadzoneStick * 2);
    }

    private JoystickButton createButton(int buttonID) {
        return new JoystickButton(this.controller, buttonID);
    }

    private double deadzone(double x, double dz) {
        if (Math.abs(x) > dz) return x;
        else return 0.0;
    }
}
