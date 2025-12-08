package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import java.util.Optional;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;

/**
 * FlywheelIO implementation backed by a YAMS FlyWheel mechanism.
 *
 * <p><strong>IMPORTANT:</strong> This IO layer is for <strong>LOGGING ONLY</strong>. Use the YAMS
 * FlyWheel directly for control:
 *
 * <pre>{@code
 * // Create YAMS flywheel
 * FlyWheel flywheel = new FlyWheel(
 *     new FlyWheelConfig(motorController)
 *         .withDiameter(Inches.of(4))
 *         .withMass(Kilograms.of(0.5))
 *         .withSoftLimit(RPM.of(0), RPM.of(6000))
 * );
 *
 * // Create IO layer for logging
 * FlywheelIO io = new FlywheelIOYAMS(flywheel);
 *
 * // CONTROL via YAMS FlyWheel (returns Commands):
 * flywheel.setVelocity(RPM.of(3000))  // Velocity command
 * flywheel.set(0.5)                   // Duty cycle command
 * flywheel.setVoltage(Volts.of(6))    // Voltage command
 *
 * // LOGGING via IO layer:
 * io.updateInputs(inputs);
 * Logger.processInputs("Flywheel", inputs);
 * }</pre>
 */
public class FlywheelIOYAMS implements FlywheelIO {

  private final FlyWheel flywheel;
  private final SmartMotorController motor;

  // RPM conversion constant
  private static final double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * Math.PI);

  /**
   * Creates a new FlywheelIOYAMS.
   *
   * @param flywheel The YAMS FlyWheel mechanism to wrap for logging
   */
  public FlywheelIOYAMS(FlyWheel flywheel) {
    this.flywheel = flywheel;
    this.motor = flywheel.getMotor();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.connected = true;

    try {
      // Flywheel state
      double velocityRadPerSec = flywheel.getSpeed().in(RadiansPerSecond);
      inputs.velocityRadPerSec = velocityRadPerSec;
      inputs.velocityRPM = velocityRadPerSec * RAD_PER_SEC_TO_RPM;

      // Rotor state
      inputs.rotorPositionRad = motor.getRotorPosition().in(Radians);
      inputs.rotorVelocityRadPerSec = motor.getRotorVelocity().in(RadiansPerSecond);

      // Setpoint - from YAMS mechanism's current setpoint
      Optional<AngularVelocity> setpoint = motor.getMechanismSetpointVelocity();
      inputs.setpointVelocityRadPerSec = setpoint.map(v -> v.in(RadiansPerSecond)).orElse(0.0);

      // Electrical
      inputs.appliedVolts = motor.getVoltage().in(Volts);
      inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
      inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
      inputs.dutyCycle = inputs.appliedVolts / 12.0;

      // Thermal
      inputs.temperatureCelsius = motor.getTemperature().in(Celsius);

      // Limits from config
      var config = flywheel.getShooterConfig();
      inputs.minVelocityRadPerSec = config.getLowerSoftLimit().in(RadiansPerSecond);
      inputs.maxVelocityRadPerSec = config.getUpperSoftLimit().in(RadiansPerSecond);

      // Flywheel physical properties
      inputs.diameterMeters = config.getDiameter().in(Meters);
      inputs.massKg = config.getMass().in(Kilograms);

      // At setpoint check (within 5% tolerance)
      if (inputs.setpointVelocityRadPerSec != 0) {
        double error = Math.abs(inputs.velocityRadPerSec - inputs.setpointVelocityRadPerSec);
        inputs.atSetpoint = error < Math.abs(inputs.setpointVelocityRadPerSec * 0.05);
      } else {
        inputs.atSetpoint = Math.abs(inputs.velocityRadPerSec) < 1.0; // Near zero
      }

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
            .getFeedforward()
            .ifPresent(
                ff -> {
                  inputs.kS = ff.getKs();
                  inputs.kV = ff.getKv();
                  inputs.kA = ff.getKa();
                });
      } catch (Exception ignored) {
        // Leave gains at defaults if not accessible
      }

    } catch (Exception ex) {
      inputs.connected = false;
      // Zero transient fields on disconnect
      inputs.velocityRadPerSec = 0.0;
      inputs.velocityRPM = 0.0;
      inputs.appliedVolts = 0.0;
      inputs.statorCurrentAmps = 0.0;
    }
  }

  @Override
  public void simIterate() {
    flywheel.simIterate();
  }

  @Override
  public void updateVisualization() {
    flywheel.visualizationUpdate();
  }

  @Override
  public void updateTelemetry() {
    flywheel.updateTelemetry();
  }

  /** Returns the underlying YAMS FlyWheel for direct control. */
  public FlyWheel getFlywheel() {
    return flywheel;
  }
}
