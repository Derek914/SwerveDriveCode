// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    // Max linear speed of the robot (meters/second). Pulled from tuning constants
    // so builders can change top speed in one place. Used to scale the sticks.
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // Max rotational speed (radians/second). Scales right-stick rotation.
    private double MaxAngularRate = RotationsPerSecond.of(1.75).in(RadiansPerSecond);

    /*
     * Swerve request templates (think: "drive modes").
     * We clone/fill these every 20ms with the latest joystick values.
     */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // Add a 10% deadband so tiny stick noise doesn't move the robot
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            // Open-loop is simple and responsive for driver control. Closed-loop is
            // available but not necessary for basic teleop.
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // Telemetry helper publishes robot pose, module states, etc., to dashboard/logs
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Primary driver controller on USB port 0
    private final CommandXboxController joystick = new CommandXboxController(0);

    // Tubewheel subsystem (placeholder hardware)
    private final frc.robot.subsystems.Tubewheel tubewheel = new frc.robot.subsystems.Tubewheel();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        // Wire up all stick/trigger/button behaviors and default driving
        configureBindings();
    }

    private void configureBindings() {
        // Coordinate system note (WPILib convention):
        //  - +X is forward (away from your driver wall)
        //  - +Y is left
        drivetrain.setDefaultCommand(
            // Default drive command (runs every 20ms in teleop). Each loop, we
            // read the sticks and produce a field-centric request.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // While disabled, continuously apply an Idle request. This ensures the
        // configured neutral mode is applied (e.g., coast/brake) and that
        // modules don't command motion while the robot is safe-disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // A: Hold swerve in a braking stance (useful for resisting pushing)
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // B: Point all wheels in the direction of the left stick (live; not a preset)
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Tubewheel on right bumper / right trigger: RB = forward while held, RT = backward while held
        joystick.rightBumper().whileTrue(tubewheel.run(0.8)); // RB
        joystick.rightTrigger().whileTrue(tubewheel.run(-0.8)); // RT (note: returns analog; treated as boolean here)

        // SysId motor characterization (for tuning feedforward/feedback gains).
        // Run each routine exactly once per log; do not spam these during matches.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Left bumper: reset field-centric heading to "forward" now
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Feed drivetrain state to dashboards/logs for driver/mentor visibility
        drivetrain.registerTelemetry(logger::telemeterize);
    }

     public Command getAutonomousCommand() {
        
                // create variable to drive backwards at 25% max speed
                final var backward = drive.withVelocityY(0)
                .withVelocityX(-0.25 * MaxSpeed)
                .withRotationalRate(0);

// create variable to stop
final var idle = new SwerveRequest.Idle();

// Sequence: apply the backward request for x seconds, then go idle.
return Commands.sequence(
drivetrain.applyRequest(() -> backward).withTimeout(2),
drivetrain.applyRequest(() -> idle)
);
    }
}
