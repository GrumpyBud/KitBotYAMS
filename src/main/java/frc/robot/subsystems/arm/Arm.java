package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AdvantageKitYAMS;
import frc.robot.util.SmartMotorIO;
import frc.robot.util.SmartMotorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Arm subsystem using AdvantageKit + YAMS integration.
 *
 * <p>This demonstrates how to use the SmartMotorIO interface with YAMS configuration for a
 * positional arm mechanism while supporting AdvantageKit log replay.
 */
public class Arm extends SubsystemBase {

  // The IO layer - abstracts hardware access for replay support
  private final SmartMotorIO io;

  // AutoLogged inputs - populated by IO layer, logged by AdvantageKit
  private final SmartMotorIOInputsAutoLogged inputs = new SmartMotorIOInputsAutoLogged();

  // Arm limits
  private final Angle softLimitMin = Degrees.of(-20);
  private final Angle softLimitMax = Degrees.of(10);

  public Arm() {
    // Create YAMS config for closed-loop position control
    SmartMotorControllerConfig config =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(
                50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            .withSimClosedLoopController(
                50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            // Feedforward Constants
            .withFeedforward(new ArmFeedforward(0, 0, 0))
            .withSimFeedforward(new ArmFeedforward(0, 0, 0))
            // Gearing from the motor rotor to final shaft
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            // Motor properties
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    // Create the IO using the factory
    // This handles REAL/SIM/REPLAY modes automatically
    io =
        AdvantageKitYAMS.createTalonFX(
            config,
            DCMotor.getFalcon500(1),
            TelemetryVerbosity.HIGH,
            new TalonFX(5)); // Change CAN ID as needed
  }

  /**
   * Alternative constructor for dependency injection (useful for testing or custom IO).
   *
   * @param io The SmartMotorIO implementation to use
   */
  public Arm(SmartMotorIO io) {
    this.io = io;
  }

  /** Get the current arm angle. */
  public Angle getAngle() {
    return Radians.of(inputs.mechanismPositionRad);
  }

  /** Set the arm to a target angle (closed-loop). */
  public void setAngle(Angle angle) {
    // Apply soft limits
    Angle clampedAngle =
        Degrees.of(
            Math.max(
                softLimitMin.in(Degrees), Math.min(softLimitMax.in(Degrees), angle.in(Degrees))));
    io.setP(clampedAngle);
  }

  /** Set duty cycle for manual control. */
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  /** Command to move arm to a specific angle. */
  public Command goToAngle(Angle angle) {
    return run(() -> setAngle(angle)).withName("ArmGoToAngle");
  }

  /** Command to manually control arm with duty cycle. */
  public Command set(double dutyCycle) {
    return run(() -> setDutyCycle(dutyCycle))
        .finallyDo(interrupted -> setDutyCycle(0))
        .withName("ArmManual");
  }

  @Override
  public void periodic() {
    // Update inputs from hardware (or replay log)
    io.updateInputs(inputs);

    // Send to AdvantageKit for logging/replay
    Logger.processInputs("Arm", inputs);

    // Additional logged outputs
    Logger.recordOutput("Arm/AngleDegrees", Radians.of(inputs.mechanismPositionRad).in(Degrees));
    Logger.recordOutput(
        "Arm/VelocityDegPerSec",
        RadiansPerSecond.of(inputs.mechanismVelocityRadPerSec).in(DegreesPerSecond));
  }

  @Override
  public void simulationPeriodic() {
    // YAMS handles simulation physics
    io.simIterate();
  }
}
