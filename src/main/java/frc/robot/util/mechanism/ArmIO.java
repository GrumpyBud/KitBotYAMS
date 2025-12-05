package frc.robot.util.mechanism;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for YAMS Arm mechanism with AdvantageKit logging support.
 *
 * <p><strong>IMPORTANT:</strong> This IO layer is for <strong>LOGGING ONLY</strong>. Use the YAMS
 * Arm's built-in Commands for control:
 *
 * <pre>{@code
 * // Control via YAMS Arm (returns Commands):
 * arm.setAngle(Degrees.of(45))  // Position command
 * arm.set(0.5)                  // Duty cycle command
 * arm.setVoltage(Volts.of(6))   // Voltage command
 * arm.hold()                    // Hold position command
 *
 * // Logging via IO layer:
 * io.updateInputs(inputs);
 * Logger.processInputs("Arm", inputs);
 * }</pre>
 *
 * <p>The IO layer extracts data from the YAMS Arm for AdvantageKit to log. In REPLAY mode,
 * AdvantageKit fills the inputs from the log file, enabling full replay without hardware.
 */
public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    // Connection
    public boolean connected = false;

    // Arm state
    public double angleRad = 0.0;
    public double velocityRadPerSec = 0.0;

    // Motor state (rotor side)
    public double rotorPositionRad = 0.0;
    public double rotorVelocityRadPerSec = 0.0;

    // Setpoint
    public double setpointAngleRad = 0.0;

    // Electrical
    public double appliedVolts = 0.0;
    public double dutyCycle = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;

    // Thermal
    public double temperatureCelsius = 0.0;

    // Limits
    public double minAngleRad = 0.0;
    public double maxAngleRad = 0.0;
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

  /** Updates the set of loggable inputs from the YAMS Arm. */
  default void updateInputs(ArmIOInputs inputs) {}

  // === Simulation ===

  /** Called during simulation to update physics. */
  default void simIterate() {}

  /** Called to update mechanism visualization. */
  default void updateVisualization() {}

  /** Called to update telemetry (if YAMS built-in telemetry is desired). */
  default void updateTelemetry() {}
}
