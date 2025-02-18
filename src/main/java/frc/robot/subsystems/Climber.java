package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private final SparkMax leftClimberMotor = new SparkMax(46, MotorType.kBrushless); //16
    private final SparkMax RightClimberMotor = new SparkMax(47, MotorType.kBrushless);  //17
    public final RelativeEncoder ClimberEncoder1 = leftClimberMotor.getEncoder();
    public final RelativeEncoder ClimberEncoder2 = RightClimberMotor.getEncoder();

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Climber Velocity", ClimberEncoder1.getVelocity());
        SmartDashboard.putNumber("Right Climber Velocity", ClimberEncoder2.getVelocity());
        SmartDashboard.putNumber("Left Climber Encoder", ClimberEncoder1.getPosition());
        SmartDashboard.putNumber("Right Climber Encoder", ClimberEncoder2.getPosition());
    }

    public void setClimberMotor(double speed) {
        leftClimberMotor.set(speed);
        RightClimberMotor.set(-speed);
    }

    public double getClimberEncoder1() {
        return ClimberEncoder1.getPosition();
    }
    
    public void resetClimberEncoder1() {
        ClimberEncoder1.setPosition(0);
    }
}
