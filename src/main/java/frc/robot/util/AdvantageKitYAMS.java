package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Factory class for creating AdvantageKit-compatible YAMS motor controllers.
 *
 * <p>This bridges YAMS (Yet Another Mechanism System) with AdvantageKit's IO layer pattern,
 * enabling log replay while retaining YAMS's ease of use for motor configuration and simulation.
 *
 * <p><strong>For simple motors (rollers, intakes):</strong> Use this class with {@link
 * SmartMotorIO}.
 *
 * <p><strong>For YAMS mechanisms (Elevator, Arm, FlyWheel):</strong> Use {@link
 * frc.robot.util.mechanism.MechanismIOFactory} with the corresponding IO interfaces.
 *
 * <p>Example usage for simple motor:
 *
 * <pre>{@code
 * // In your subsystem constructor:
 * SmartMotorIO rollerIO = AdvantageKitYAMS.createTalonFX(
 *     new SmartMotorControllerConfig(this)
 *         .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
 *         .withIdleMode(MotorMode.BRAKE)
 *         .withStatorCurrentLimit(Amps.of(40)),
 *     DCMotor.getFalcon500(1),
 *     TelemetryVerbosity.HIGH,
 *     new TalonFX(14)
 * );
 *
 * // Then use standard AKit pattern:
 * private final SmartMotorIOInputsAutoLogged inputs = new SmartMotorIOInputsAutoLogged();
 *
 * public void periodic() {
 *   rollerIO.updateInputs(inputs);
 *   Logger.processInputs("Roller", inputs);
 * }
 * }</pre>
 *
 * @see frc.robot.util.mechanism.MechanismIOFactory
 * @see frc.robot.util.mechanism.ElevatorIO
 * @see frc.robot.util.mechanism.ArmIO
 * @see frc.robot.util.mechanism.FlywheelIO
 */
public final class AdvantageKitYAMS {

  private AdvantageKitYAMS() {
    // Utility class - no instantiation
  }

  /**
   * Creates a SmartMotorIO for TalonFX motor controller(s) using YAMS.
   *
   * <p>In REAL mode: Creates a TalonFXWrapper-backed implementation that talks to real hardware. In
   * SIM mode: Creates the same implementation, but YAMS handles simulation automatically. In REPLAY
   * mode: Returns an empty implementation (inputs come from the log file).
   *
   * @param config The YAMS SmartMotorControllerConfig
   * @param motor The DCMotor model for simulation (e.g., DCMotor.getFalcon500(1))
   * @param verbosity How much data to log (LOW, MID, HIGH)
   * @param talons The TalonFX motor controller(s)
   * @return A SmartMotorIO implementation appropriate for the current mode
   */
  public static SmartMotorIO createTalonFX(
      SmartMotorControllerConfig config,
      DCMotor motor,
      TelemetryVerbosity verbosity,
      TalonFX... talons) {

    return switch (Constants.currentMode) {
      case REAL, SIM ->
      // In both REAL and SIM, we use the YAMS wrapper
      // YAMS handles simulation internally via TalonFXSimState
      new SmartMotorIOTalonFX(config, motor, verbosity, talons);

      case REPLAY ->
      // In REPLAY mode, return empty implementation
      // AdvantageKit will populate inputs from the log file
      new SmartMotorIO() {};
    };
  }

  /**
   * Creates a SmartMotorIO for TalonFX with default HIGH verbosity and ANGULAR mechanism type.
   *
   * @param config The YAMS SmartMotorControllerConfig
   * @param motor The DCMotor model for simulation
   * @param talons The TalonFX motor controller(s)
   * @return A SmartMotorIO implementation
   */
  public static SmartMotorIO createTalonFX(
      SmartMotorControllerConfig config, DCMotor motor, TalonFX... talons) {
    return createTalonFX(config, motor, TelemetryVerbosity.HIGH, talons);
  }

  /**
   * Creates a SmartMotorIO for a linear mechanism (e.g., elevator, intake with wheels).
   *
   * @param config The YAMS SmartMotorControllerConfig (should have wheel circumference configured)
   * @param motor The DCMotor model for simulation
   * @param verbosity How much data to log
   * @param talons The TalonFX motor controller(s)
   * @return A SmartMotorIO implementation
   */

  // TODO: Add support for other motor controllers as YAMS adds them
  // public static SmartMotorIO createSparkMax(...) { }
  // public static SmartMotorIO createSparkFlex(...) { }
  // public static SmartMotorIO createTalonFXS(...) { }
}
