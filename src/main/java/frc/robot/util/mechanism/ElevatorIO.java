package frc.robot.util.mechanism;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for YAMS Elevator mechanism with AdvantageKit logging support.
 *
 * <p><strong>This IO layer is for LOGGING ONLY.</strong> Use the YAMS Elevator's built-in Commands
 * for control (e.g., {@code elevator.setHeight()}).
 *
 * <p>The separation allows:
 *
 * <ul>
 *   <li>YAMS handles control, motion profiling, limits, and command factories
 *   <li>This IO layer handles AdvantageKit logging and replay
 * </ul>
 *
 * <p>All units are in meters for position and meters/second for velocity.
 */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    // Connection
    public boolean connected = false;

    // Elevator state
    public double heightMeters = 0.0;
    public double velocityMetersPerSec = 0.0;

    // Motor state (rotor side)
    public double rotorPositionRad = 0.0;
    public double rotorVelocityRadPerSec = 0.0;

    // Setpoint (what YAMS is targeting)
    public double setpointHeightMeters = 0.0;

    // Electrical
    public double appliedVolts = 0.0;
    public double dutyCycle = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;

    // Thermal
    public double temperatureCelsius = 0.0;

    // Limits
    public double minHeightMeters = 0.0;
    public double maxHeightMeters = 0.0;
    public boolean atMinLimit = false;
    public boolean atMaxLimit = false;

    // Gains (for tuning visibility)
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kS = 0.0;
    public double kV = 0.0;
    public double kA = 0.0;
    public double kG = 0.0;
  }

  /** Updates the set of loggable inputs from the YAMS mechanism. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Called during simulation to update YAMS physics. */
  default void simIterate() {}

  /** Called to update YAMS mechanism visualization. */
  default void updateVisualization() {}

  /** Called to update YAMS telemetry (NetworkTables). */
  default void updateTelemetry() {}
}
