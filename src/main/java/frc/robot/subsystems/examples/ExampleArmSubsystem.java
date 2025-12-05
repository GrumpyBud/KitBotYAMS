package frc.robot.subsystems.examples;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.mechanism.ArmIO;
import frc.robot.util.mechanism.ArmIOYAMS;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Example Arm subsystem using AdvantageKit + YAMS integration.
 *
 * <p>This demonstrates the correct pattern:
 *
 * <ul>
 *   <li><strong>YAMS Arm</strong> - handles control, gravity compensation, commands
 *   <li><strong>ArmIO</strong> - handles AdvantageKit logging for replay
 * </ul>
 *
 * <p>Control goes through the YAMS mechanism. Logging goes through the IO layer.
 */
public class ExampleArmSubsystem extends SubsystemBase {

  // YAMS mechanism - use for CONTROL (commands, setpoints)
  private final Arm arm;

  // IO layer - use for LOGGING (AdvantageKit replay)
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  /** Creates a new ExampleArmSubsystem with default hardware configuration. */
  public ExampleArmSubsystem() {
    // Only create hardware in REAL/SIM modes
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      // Create motor controller config with gravity compensation feedforward
      SmartMotorControllerConfig motorConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(100))) // High reduction
              .withIdleMode(MotorMode.BRAKE)
              .withStatorCurrentLimit(Amps.of(40));

      // Create motor controller
      TalonFX talonFX = new TalonFX(16);
      var motorController = new TalonFXWrapper(talonFX, DCMotor.getFalcon500(1), motorConfig);

      // Create YAMS arm - this handles all control logic including gravity compensation
      ArmConfig armConfig =
          new ArmConfig(motorController)
              .withLength(Meters.of(0.5))
              .withMass(Kilograms.of(3))
              .withHardLimit(Degrees.of(-45), Degrees.of(135))
              .withStartingPosition(Degrees.of(0));

      this.arm = new Arm(armConfig);
      this.io = new ArmIOYAMS(arm);
    } else {
      // REPLAY mode - no hardware, no YAMS mechanism
      this.arm = null;
      this.io = new ArmIO() {}; // Empty IO, AKit fills inputs from log
    }
  }

  // === Getters (read from logged inputs for replay compatibility) ===

  /** Returns the current arm angle in radians. */
  public double getAngleRad() {
    return inputs.angleRad;
  }

  /** Returns the current arm angle in degrees. */
  public double getAngleDegrees() {
    return Units.radiansToDegrees(inputs.angleRad);
  }

  /** Returns true if the arm is at the minimum limit. */
  public boolean atMinLimit() {
    return inputs.atMinLimit;
  }

  /** Returns true if the arm is at the maximum limit. */
  public boolean atMaxLimit() {
    return inputs.atMaxLimit;
  }

  // === Commands (delegate to YAMS mechanism) ===

  /** Command to move arm to a specific angle. Uses YAMS built-in control. */
  public Command goToAngle(Angle angle) {
    if (arm != null) {
      return arm.setAngle(angle);
    }
    // In replay mode, return a no-op command
    return runOnce(() -> {});
  }

  /** Command to move arm to a specific angle in degrees. */
  public Command goToAngleDegrees(double angleDegrees) {
    return goToAngle(Degrees.of(angleDegrees));
  }

  /** Command to move arm to stowed position (0 degrees). */
  public Command stow() {
    return goToAngleDegrees(0);
  }

  /** Command to move arm to min limit. */
  public Command goToMin() {
    return goToAngle(Radians.of(inputs.minAngleRad));
  }

  /** Command to move arm to max limit. */
  public Command goToMax() {
    return goToAngle(Radians.of(inputs.maxAngleRad));
  }

  /** Command to run arm with duty cycle. Uses YAMS built-in control. */
  public Command runDutyCycle(double dutyCycle) {
    if (arm != null) {
      return arm.set(dutyCycle);
    }
    return runOnce(() -> {});
  }

  /** Command to hold current position. Uses YAMS built-in hold with gravity compensation. */
  public Command hold() {
    if (arm != null) {
      return arm.hold();
    }
    return runOnce(() -> {});
  }

  @Override
  public void periodic() {
    // Update inputs from YAMS mechanism (or replay log in REPLAY mode)
    io.updateInputs(inputs);

    // Send to AdvantageKit for logging/replay
    Logger.processInputs("Arm", inputs);

    // Additional computed outputs
    Logger.recordOutput("Arm/AngleDegrees", Units.radiansToDegrees(inputs.angleRad));
    Logger.recordOutput(
        "Arm/AtSetpoint",
        Math.abs(inputs.angleRad - inputs.setpointAngleRad) < Units.degreesToRadians(2));
  }

  @Override
  public void simulationPeriodic() {
    // YAMS handles simulation physics (including gravity)
    io.simIterate();
    io.updateVisualization();
  }
}
