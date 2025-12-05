package frc.robot.subsystems.examples;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.mechanism.ElevatorIO;
import frc.robot.util.mechanism.ElevatorIOYAMS;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Example Elevator subsystem using AdvantageKit + YAMS integration.
 *
 * <p>This demonstrates the correct pattern:
 *
 * <ul>
 *   <li><strong>YAMS Elevator</strong> - handles control, motion profiling, commands
 *   <li><strong>ElevatorIO</strong> - handles AdvantageKit logging for replay
 * </ul>
 *
 * <p>Control goes through the YAMS mechanism. Logging goes through the IO layer.
 */
public class ExampleElevatorSubsystem extends SubsystemBase {

  // YAMS mechanism - use for CONTROL (commands, setpoints)
  private final Elevator elevator;

  // IO layer - use for LOGGING (AdvantageKit replay)
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Creates a new ExampleElevatorSubsystem with default hardware configuration. */
  public ExampleElevatorSubsystem() {
    // Only create hardware in REAL/SIM modes
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      // Create motor controller config
      SmartMotorControllerConfig motorConfig =
          new SmartMotorControllerConfig(this)
              .withControlMode(ControlMode.CLOSED_LOOP)
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(10)))
              .withIdleMode(MotorMode.BRAKE)
              .withStatorCurrentLimit(Amps.of(40))
              .withMechanismCircumference(Inches.of(1.5 * Math.PI)); // 1.5" drum diameter

      // Create motor controller
      TalonFX talonFX = new TalonFX(15);
      var motorController = new TalonFXWrapper(talonFX, DCMotor.getFalcon500(1), motorConfig);

      // Create YAMS elevator - this handles all control logic
      ElevatorConfig elevatorConfig =
          new ElevatorConfig(motorController)
              .withDrumRadius(Inches.of(0.75))
              .withStartingHeight(Meters.of(0))
              .withHardLimits(Meters.of(0), Meters.of(1.2))
              .withMass(Kilograms.of(5));

      this.elevator = new Elevator(elevatorConfig);
      this.io = new ElevatorIOYAMS(elevator);
    } else {
      // REPLAY mode - no hardware, no YAMS mechanism
      this.elevator = null;
      this.io = new ElevatorIO() {}; // Empty IO, AKit fills inputs from log
    }
  }

  // === Getters (read from logged inputs for replay compatibility) ===

  /** Returns the current elevator height in meters. */
  public double getHeightMeters() {
    return inputs.heightMeters;
  }

  /** Returns true if the elevator is at the bottom limit. */
  public boolean atBottom() {
    return inputs.atMinLimit;
  }

  /** Returns true if the elevator is at the top limit. */
  public boolean atTop() {
    return inputs.atMaxLimit;
  }

  // === Commands (delegate to YAMS mechanism) ===

  /** Command to move elevator to a specific height. Uses YAMS built-in control. */
  public Command goToHeight(Distance height) {
    if (elevator != null) {
      return elevator.setHeight(height);
    }
    // In replay mode, return a no-op command
    return runOnce(() -> {});
  }

  /** Command to move elevator to bottom. */
  public Command goToBottom() {
    return goToHeight(Meters.of(inputs.minHeightMeters));
  }

  /** Command to move elevator to top. */
  public Command goToTop() {
    return goToHeight(Meters.of(inputs.maxHeightMeters));
  }

  /** Command to run elevator with duty cycle. Uses YAMS built-in control. */
  public Command runDutyCycle(double dutyCycle) {
    if (elevator != null) {
      return elevator.set(dutyCycle);
    }
    return runOnce(() -> {});
  }

  @Override
  public void periodic() {
    // Update inputs from YAMS mechanism (or replay log in REPLAY mode)
    io.updateInputs(inputs);

    // Send to AdvantageKit for logging/replay
    Logger.processInputs("Elevator", inputs);

    // Additional computed outputs
    Logger.recordOutput("Elevator/HeightMeters", inputs.heightMeters);
    Logger.recordOutput(
        "Elevator/AtSetpoint", Math.abs(inputs.heightMeters - inputs.setpointHeightMeters) < 0.02);
  }

  @Override
  public void simulationPeriodic() {
    // YAMS handles simulation physics
    io.simIterate();
    io.updateVisualization();
  }
}
