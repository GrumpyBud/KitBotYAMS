package frc.robot.util;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for YAMS SmartMotorController with AdvantageKit logging support. This bridges YAMS's
 * ease of use with AdvantageKit's replay capabilities.
 *
 * <p>The inputs class contains fields based on YAMS TelemetryVerbosity levels: - LOW: position,
 * velocity (mechanism and rotor level) - MID: + voltage, stator current, supply current - HIGH: +
 * temperature, setpoints, limits, gains
 */
public interface SmartMotorIO {
  @AutoLog
  public static class SmartMotorIOInputs {
    // Connection
    public boolean connected = false;

    // Mechanism state (gearbox output; for linear, units are meters/mps)
    public double mechanismPositionRad = 0.0;
    public double mechanismVelocityRadPerSec = 0.0;

    // Rotor state (motor side)
    public double rotorPositionRad = 0.0;
    public double rotorVelocityRadPerSec = 0.0;

    // Commanded setpoints
    public double setpointPosition = 0.0;
    public double setpointVelocity = 0.0;

    // Electrical (MID+)
    public double appliedVolts = 0.0;
    public double dutyCycle = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;

    // Thermal (HIGH)
    public double temperatureCelsius = 0.0;

    // Gains (HIGH, best-effort)
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kS = 0.0;
    public double kV = 0.0;
    public double kA = 0.0;

    // Limits (HIGH)
    public double mechanismLowerLimit = Double.NEGATIVE_INFINITY;
    public double mechanismUpperLimit = Double.POSITIVE_INFINITY;
    public double statorCurrentLimit = 0.0;
    public double supplyCurrentLimit = 0.0;

    // Motion profiling (HIGH)
    public double motionProfileMaxVelocity = 0.0;
    public double motionProfileMaxAcceleration = 0.0;

    // Flags (HIGH)
    public boolean atUpperLimit = false;
    public boolean atLowerLimit = false;
    public boolean overTemperature = false;
    public boolean motorInverted = false;
    public boolean encoderInverted = false;
    public boolean velocityControl = false;
    public boolean hasMotionProfile = false;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(SmartMotorIOInputs inputs) {}

  // === Control Methods ===

  /** Run the motor at the specified duty cycle [-1, 1]. */
  default void setDutyCycle(double dutyCycle) {}

  /** Run the motor at the specified voltage. */
  default void setVoltage(double volts) {}

  /** Run closed-loop position control to the specified mechanism position (radians or meters). */
  default void setPosition(double position) {}

  /** Run closed-loop velocity control to the specified mechanism velocity (rad/s or m/s). */
  default void setVelocity(double velocity) {}

  /** Stop the motor. */
  default void stop() {}

  // === Configuration Methods ===

  /** Set the encoder position (for zeroing). */
  default void setEncoderPosition(double position) {}

  /** Set mechanism position limits. */
  default void setMechanismLimits(double lower, double upper) {}

  /** Update PID gains. */
  default void setPIDGains(double kP, double kI, double kD) {}

  /** Update feedforward gains. */
  default void setFeedforwardGains(double kS, double kV, double kA, double kG) {}

  // === Simulation ===

  /** Called during simulation to update physics. */
  default void simIterate() {}
}
