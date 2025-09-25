package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple Tubewheel subsystem. This is a placeholder motor wrapper for the
 * future tube wheel hardware. Change kMotorPort to the correct PWM (or
 * replace PWMSparkMax with your real motor controller) when wiring is known.
 */
public class Tubewheel extends SubsystemBase {
    // Default PWM port; update to your actual port or change to a CAN device.
    private static final int kMotorPort = 0;

    private final PWMSparkMax motor = new PWMSparkMax(kMotorPort);

    public Tubewheel() {
        // Ensure the motor is stopped at init
        motor.set(0.0);
    }

    /** Run the tubewheel at the given speed (-1..1). Returns a command that
     * continuously sets the motor while scheduled. */
    public Command run(double speed) {
        return Commands.run(() -> motor.set(speed), this).withName("TubewheelRun");
    }

    /** Stop the tubewheel immediately. */
    public void stop() {
        motor.set(0.0);
    }
}
