package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import java.util.Optional;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;

/**
 * ArmIO implementation backed by a YAMS Arm mechanism.
 *
 * <p><strong>IMPORTANT:</strong> This IO layer is for <strong>LOGGING ONLY</strong>. Use the YAMS
 * Arm directly for control:
 *
 * <pre>{@code
 * // Create YAMS arm
 * Arm arm = new Arm(
 *     new ArmConfig(motorController)
 *         .withLength(Meters.of(0.5))
 *         .withMass(Kilograms.of(3))
 *         .withHardLimit(Degrees.of(-90), Degrees.of(90))
 *         .withStartingPosition(Degrees.of(0))
 * );
 *
 * // Create IO layer for logging
 * ArmIO io = new ArmIOYAMS(arm);
 *
 * // CONTROL via YAMS Arm (returns Commands):
 * arm.setAngle(Degrees.of(45))  // Position command
 * arm.set(0.5)                  // Duty cycle command
 * arm.setVoltage(Volts.of(6))   // Voltage command
 *
 * // LOGGING via IO layer:
 * io.updateInputs(inputs);
 * Logger.processInputs("Arm", inputs);
 * }</pre>
 */
public class ArmIOYAMS implements ArmIO {

  private final Arm arm;
  private final SmartMotorController motor;

  /**
   * Creates a new ArmIOYAMS.
   *
   * @param arm The YAMS Arm mechanism to wrap for logging
   */
  public ArmIOYAMS(Arm arm) {
    this.arm = arm;
    this.motor = arm.getMotor();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.connected = true;

    try {
      // Arm state
      inputs.angleRad = arm.getAngle().in(Radians);
      inputs.velocityRadPerSec = motor.getMechanismVelocity().in(RadiansPerSecond);

      // Rotor state
      inputs.rotorPositionRad = motor.getRotorPosition().in(Radians);
      inputs.rotorVelocityRadPerSec = motor.getRotorVelocity().in(RadiansPerSecond);

      // Setpoint - from YAMS mechanism's current setpoint
      Optional<Angle> setpoint = motor.getMechanismPositionSetpoint();
      inputs.setpointAngleRad = setpoint.map(a -> a.in(Radians)).orElse(inputs.angleRad);

      // Electrical
      inputs.appliedVolts = motor.getVoltage().in(Volts);
      inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
      inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
      inputs.dutyCycle = inputs.appliedVolts / 12.0;

      // Thermal
      inputs.temperatureCelsius = motor.getTemperature().in(Celsius);

      // Limits from config
      var config = arm.getArmConfig();
      inputs.minAngleRad = config.getLowerHardLimit().map(a -> a.in(Radians)).orElse(0.0);
      inputs.maxAngleRad = config.getUpperHardLimit().map(a -> a.in(Radians)).orElse(0.0);

      // Limit triggers
      inputs.atMinLimit = arm.min().getAsBoolean();
      inputs.atMaxLimit = arm.max().getAsBoolean();

      // Gains (best effort)
      try {
        var motorConfig = motor.getConfig();
        motorConfig
            .getClosedLoopController()
            .ifPresent(
                pid -> {
                  inputs.kP = pid.getP();
                  inputs.kI = pid.getI();
                  inputs.kD = pid.getD();
                });
        motorConfig
            .getArmFeedforward()
            .ifPresent(
                ff -> {
                  inputs.kS = ff.getKs();
                  inputs.kV = ff.getKv();
                  inputs.kA = ff.getKa();
                  inputs.kG = ff.getKg();
                });
      } catch (Exception ignored) {
        // Leave gains at defaults if not accessible
      }

    } catch (Exception ex) {
      inputs.connected = false;
      // Zero transient fields on disconnect
      inputs.angleRad = 0.0;
      inputs.velocityRadPerSec = 0.0;
      inputs.appliedVolts = 0.0;
      inputs.statorCurrentAmps = 0.0;
    }
  }

  @Override
  public void simIterate() {
    arm.simIterate();
  }

  @Override
  public void updateVisualization() {
    arm.visualizationUpdate();
  }

  @Override
  public void updateTelemetry() {
    arm.updateTelemetry();
  }

  /** Returns the underlying YAMS Arm for direct control. */
  public Arm getArm() {
    return arm;
  }
}
