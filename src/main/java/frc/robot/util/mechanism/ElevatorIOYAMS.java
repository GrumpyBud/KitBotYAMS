package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import java.util.Optional;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;

/**
 * ElevatorIO implementation backed by a YAMS Elevator mechanism.
 *
 * <p><strong>Important:</strong> This IO layer is for LOGGING ONLY. Use the YAMS Elevator's
 * built-in Commands for control (e.g., {@code elevator.setHeight()}).
 *
 * <p>The IO layer extracts data from the YAMS Elevator for AdvantageKit logging/replay. Control
 * should go through the YAMS mechanism directly to use its built-in motion profiling, limits, and
 * command factories.
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * public class MyElevator extends SubsystemBase {
 *   private final Elevator elevator; // YAMS mechanism - use for CONTROL
 *   private final ElevatorIO io;     // IO layer - use for LOGGING
 *   private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
 *
 *   public MyElevator() {
 *     elevator = new Elevator(config);
 *     io = new ElevatorIOYAMS(elevator);
 *   }
 *
 *   // Control goes through YAMS mechanism
 *   public Command goToHeight(Distance height) {
 *     return elevator.setHeight(height);
 *   }
 *
 *   // Logging goes through IO layer
 *   public void periodic() {
 *     io.updateInputs(inputs);
 *     Logger.processInputs("Elevator", inputs);
 *   }
 * }
 * }</pre>
 */
public class ElevatorIOYAMS implements ElevatorIO {

  private final Elevator elevator;
  private final SmartMotorController motor;

  /**
   * Creates a new ElevatorIOYAMS.
   *
   * @param elevator The YAMS Elevator mechanism to wrap for logging
   */
  public ElevatorIOYAMS(Elevator elevator) {
    this.elevator = elevator;
    this.motor = elevator.getMotor();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.connected = true;

    try {
      // Elevator state from YAMS mechanism
      inputs.heightMeters = elevator.getHeight().in(Meters);
      inputs.velocityMetersPerSec = elevator.getVelocity().in(MetersPerSecond);

      // Rotor state from motor
      inputs.rotorPositionRad = motor.getRotorPosition().in(Radians);
      inputs.rotorVelocityRadPerSec = motor.getRotorVelocity().in(RadiansPerSecond);

      // Setpoint from motor controller
      Optional<Distance> setpoint = motor.getMechanismPositionSetpoint();
      inputs.setpointHeightMeters = setpoint.map(d -> d.in(Meters)).orElse(inputs.heightMeters);

      // Electrical
      inputs.appliedVolts = motor.getVoltage().in(Volts);
      inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
      inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
      inputs.dutyCycle = inputs.appliedVolts / 12.0;

      // Thermal
      inputs.temperatureCelsius = motor.getTemperature().in(Celsius);

      // Limits from YAMS config
      var config = elevator.getConfig();
      inputs.minHeightMeters = config.getMinimumHeight().map(d -> d.in(Meters)).orElse(0.0);
      inputs.maxHeightMeters = config.getMaximumHeight().map(d -> d.in(Meters)).orElse(0.0);

      // Limit triggers from YAMS
      inputs.atMinLimit = elevator.min().getAsBoolean();
      inputs.atMaxLimit = elevator.max().getAsBoolean();

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
                  inputs.kG = ff.getKg();
                });
      } catch (Exception ignored) {
        // Leave gains at defaults if not accessible
      }

    } catch (Exception ex) {
      inputs.connected = false;
      inputs.heightMeters = 0.0;
      inputs.velocityMetersPerSec = 0.0;
      inputs.appliedVolts = 0.0;
      inputs.statorCurrentAmps = 0.0;
    }
  }

  @Override
  public void simIterate() {
    elevator.simIterate();
  }

  @Override
  public void updateVisualization() {
    elevator.visualizationUpdate();
  }

  @Override
  public void updateTelemetry() {
    elevator.updateTelemetry();
  }

  /** Returns the underlying YAMS Elevator for control and commands. */
  public Elevator getElevator() {
    return elevator;
  }
}
