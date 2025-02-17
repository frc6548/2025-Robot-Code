package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final SparkMax ElevatorMotor1 = new SparkMax(16, MotorType.kBrushless);
    private final SparkMax ElevatorMotor2 = new SparkMax(17, MotorType.kBrushless);
    public final RelativeEncoder ElevatorEncoder1 = ElevatorMotor1.getEncoder();
    public final RelativeEncoder ElevatorEncoder2 = ElevatorMotor2.getEncoder();

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Velocity1", ElevatorEncoder1.getVelocity());
        SmartDashboard.putNumber("Elevator Velocity2", ElevatorEncoder2.getVelocity());
        SmartDashboard.putNumber("Elevator Encoder1", ElevatorEncoder1.getPosition());
        SmartDashboard.putNumber("Elevator Encoder2", ElevatorEncoder2.getPosition());

}
    public void setElevatorMotor(double speed) {
        ElevatorMotor1.set(speed);
        ElevatorMotor2.set(speed);
}
    public double getElevatorEncoder1() {
        return ElevatorEncoder1.getPosition();
}
    public void resetElevatorEncoder1() {
        ElevatorEncoder1.setPosition(0);
}
}