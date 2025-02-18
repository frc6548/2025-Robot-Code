package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakePivotPIDCommand extends Command {
    private Intake IntakeSubsystem;
    private final ProfiledPIDController pidController;
    private final TrapezoidProfile.Constraints m_constraints;
    private final  SimpleMotorFeedforward m_feedforward;
    private static double kMaxVelocity = 1.75;
    private static double kMaxAcceleration = 0.75;
    private static double kP = 0.005;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kS = 0.0;
    private static double kV = 0.0;
    private static double kA = 0.0;

    public IntakePivotPIDCommand(Intake IntakeSubsystem, double setpoint) {
      this.IntakeSubsystem = IntakeSubsystem;
      this.m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
      this.pidController = new ProfiledPIDController(kP, kI, kD, m_constraints);
      this.m_feedforward  = new SimpleMotorFeedforward(kS, kV, kA);
      pidController.setGoal(setpoint);
      }

  @Override
  public void initialize() {
    System.out.println("IntakePivotCommand started!");
    IntakeSubsystem.setPivotMotor(0);
  }

  @Override
  public void execute() {
    // double speed = pidController.calculate(IntakeSubsystem.getPivotEncoder());

    double speed = pidController.calculate(IntakeSubsystem.getPivotEncoder()
    + m_feedforward.calculate(pidController.getSetpoint().velocity));
    IntakeSubsystem.setPivotMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.setIntakeMotor(0);
    System.out.println("IntakePivotCommand ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
