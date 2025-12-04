package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Roller extends SubsystemBase {
  private SmartMotorControllerConfig rollerConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.OPEN_LOOP)
          .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40));

  private TalonFX rollerTalonFX = new TalonFX(0);

  private SmartMotorController rollerController =
      new TalonFXWrapper(rollerTalonFX, DCMotor.getFalcon500(1), rollerConfig);

  public AngularVelocity getVelocity() {
    return rollerController.getMechanismVelocity();
  }

  public void setDutyCycle(double dutyCycle) {
    rollerController.setDutyCycle(dutyCycle);
  }

  public Command set(double dutyCycle) {
    return this.runOnce(() -> this.setDutyCycle(dutyCycle))
        .finallyDo(() -> this.setDutyCycle(0))
        .withName("SetRoller");
  }

  @Override
  public void periodic() {
    rollerController.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    rollerController.simIterate();
  }
}
