package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPIDCommand extends Command {
    private Elevator elevatorSubsystem;
    private final ProfiledPIDController pidController;
    private final TrapezoidProfile.Constraints m_constraints;
    private final ElevatorFeedforward m_feedforward;
    private static double kMaxVelocity = 1.75;
    private static double kMaxAcceleration = 0.75;
    private static double kP = 0.0;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kS = 0.0;
    private static double kG = 0.1;
    private static double kV = 0.0;

    public ElevatorPIDCommand(Elevator elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
        this.pidController = new ProfiledPIDController(kP, kI, kD, m_constraints);
        this.m_feedforward  = new ElevatorFeedforward(kS, kG, kV);
        pidController.setGoal(setpoint);
      }

  @Override
  public void initialize() {
    System.out.println("ElevatorPIDCommand started!");
    // pidController.reset();
    elevatorSubsystem.setElevatorMotor(0);
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(elevatorSubsystem.getElevatorEncoder1()
            + m_feedforward.calculate(pidController.getSetpoint().velocity));
    elevatorSubsystem.setElevatorMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorMotor(0);
    System.out.println("ElevatorPIDCommand ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
