package frc.robot.util.mechanism;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for YAMS FlyWheel mechanism with AdvantageKit logging support.
 *
 * <p><strong>IMPORTANT:</strong> This IO layer is for <strong>LOGGING ONLY</strong>. Use the YAMS
 * FlyWheel's built-in Commands for control:
 *
 * <pre>{@code
 * // Control via YAMS FlyWheel (returns Commands):
 * flywheel.setVelocity(RPM.of(3000))  // Velocity command
 * flywheel.set(0.5)                   // Duty cycle command
 * flywheel.setVoltage(Volts.of(6))    // Voltage command
 * flywheel.stop()                     // Stop command
 *
 * // Logging via IO layer:
 * io.updateInputs(inputs);
 * Logger.processInputs("Flywheel", inputs);
 * }</pre>
 *
 * <p>The IO layer extracts data from the YAMS FlyWheel for AdvantageKit to log. In REPLAY mode,
 * AdvantageKit fills the inputs from the log file, enabling full replay without hardware.
 */
public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    // Connection
    public boolean connected = false;

    // Flywheel state
    public double velocityRadPerSec = 0.0;
    public double velocityRPM = 0.0;

    // Motor state (rotor side)
    public double rotorPositionRad = 0.0;
    public double rotorVelocityRadPerSec = 0.0;

    // Setpoint
    public double setpointVelocityRadPerSec = 0.0;

    // Electrical
    public double appliedVolts = 0.0;
    public double dutyCycle = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;

    // Thermal
    public double temperatureCelsius = 0.0;

    // Limits
    public double minVelocityRadPerSec = 0.0;
    public double maxVelocityRadPerSec = 0.0;

    // Gains (for tuning visibility)
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kS = 0.0;
    public double kV = 0.0;
    public double kA = 0.0;

    // Flywheel-specific
    public double diameterMeters = 0.0;
    public double massKg = 0.0;

    // Status
    public boolean atSetpoint = false;
  }

  /** Updates the set of loggable inputs from the YAMS FlyWheel. */
  default void updateInputs(FlywheelIOInputs inputs) {}

  // === Simulation ===

  /** Called during simulation to update physics. */
  default void simIterate() {}

  /** Called to update mechanism visualization. */
  default void updateVisualization() {}

  /** Called to update telemetry (if YAMS built-in telemetry is desired). */
  default void updateTelemetry() {}
}
