package frc.robot.util.mechanism;

import frc.robot.Constants;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.velocity.FlyWheel;

/**
 * Factory class for creating AdvantageKit-compatible YAMS mechanism IO layers.
 *
 * <p><strong>IMPORTANT:</strong> The IO layers are for <strong>LOGGING ONLY</strong>. Use the YAMS
 * mechanisms directly for control (they return Commands).
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * // In your subsystem:
 * private final Elevator elevator;  // For CONTROL
 * private final ElevatorIO io;      // For LOGGING
 * private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
 *
 * public MyElevator(Elevator yamsElevator) {
 *   this.elevator = yamsElevator;
 *   this.io = MechanismIOFactory.createElevatorIO(yamsElevator);
 * }
 *
 * public Command goToHeight(Distance height) {
 *   return elevator.setHeight(height);  // YAMS handles control
 * }
 *
 * @Override
 * public void periodic() {
 *   io.updateInputs(inputs);                  // IO handles logging
 *   Logger.processInputs("Elevator", inputs);
 * }
 * }</pre>
 */
public final class MechanismIOFactory {

  private MechanismIOFactory() {
    // Utility class
  }

  /**
   * Creates an ElevatorIO for a YAMS Elevator mechanism.
   *
   * <p>In REAL/SIM mode: Returns a YAMS-backed implementation for logging. In REPLAY mode: Returns
   * an empty implementation (inputs come from log file).
   *
   * @param elevator The YAMS Elevator mechanism (can be null in REPLAY mode)
   * @return An ElevatorIO implementation appropriate for the current mode
   */
  public static ElevatorIO createElevatorIO(Elevator elevator) {
    return switch (Constants.currentMode) {
      case REAL, SIM -> new ElevatorIOYAMS(elevator);
      case REPLAY -> new ElevatorIO() {};
    };
  }

  /**
   * Creates an ElevatorIO for replay mode only.
   *
   * @return An empty ElevatorIO implementation for replay
   */
  public static ElevatorIO createElevatorIOReplay() {
    return new ElevatorIO() {};
  }

  /**
   * Creates an ArmIO for a YAMS Arm mechanism.
   *
   * <p>In REAL/SIM mode: Returns a YAMS-backed implementation for logging. In REPLAY mode: Returns
   * an empty implementation (inputs come from log file).
   *
   * @param arm The YAMS Arm mechanism (can be null in REPLAY mode)
   * @return An ArmIO implementation appropriate for the current mode
   */
  public static ArmIO createArmIO(Arm arm) {
    return switch (Constants.currentMode) {
      case REAL, SIM -> new ArmIOYAMS(arm);
      case REPLAY -> new ArmIO() {};
    };
  }

  /**
   * Creates an ArmIO for replay mode only.
   *
   * @return An empty ArmIO implementation for replay
   */
  public static ArmIO createArmIOReplay() {
    return new ArmIO() {};
  }

  /**
   * Creates a FlywheelIO for a YAMS FlyWheel mechanism.
   *
   * <p>In REAL/SIM mode: Returns a YAMS-backed implementation for logging. In REPLAY mode: Returns
   * an empty implementation (inputs come from log file).
   *
   * @param flywheel The YAMS FlyWheel mechanism (can be null in REPLAY mode)
   * @return A FlywheelIO implementation appropriate for the current mode
   */
  public static FlywheelIO createFlywheelIO(FlyWheel flywheel) {
    return switch (Constants.currentMode) {
      case REAL, SIM -> new FlywheelIOYAMS(flywheel);
      case REPLAY -> new FlywheelIO() {};
    };
  }

  /**
   * Creates a FlywheelIO for replay mode only.
   *
   * @return An empty FlywheelIO implementation for replay
   */
  public static FlywheelIO createFlywheelIOReplay() {
    return new FlywheelIO() {};
  }
}
