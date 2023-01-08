package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.constants;

public class drivetrain {
    private static CANSparkMax leftFront;
    private static CANSparkMax leftMiddle;
    private static CANSparkMax leftBack;
    private static  CANSparkMax rightFront;
    private static CANSparkMax rightMiddle;
    private static CANSparkMax rightBack;
    private static MotorControllerGroup rightDrive;
    private static MotorControllerGroup leftDrive;
    private static DifferentialDrive drive;
    public drivetrain(){
        //temp
        leftFront= new CANSparkMax(1, MotorType.kBrushless);
        leftMiddle= new CANSparkMax(2, MotorType.kBrushless);
        leftBack= new CANSparkMax(3, MotorType.kBrushless);
        rightFront= new CANSparkMax(4, MotorType.kBrushless);
        rightMiddle= new CANSparkMax(5, MotorType.kBrushless);
        rightBack= new CANSparkMax(6, MotorType.kBrushless);
        leftDrive = new MotorControllerGroup(leftFront,leftMiddle,leftBack);
        leftDrive.setInverted(true);
        rightDrive = new MotorControllerGroup(rightFront,rightMiddle,rightBack);
        drive = new DifferentialDrive(leftDrive, rightDrive);
    }
    public void move(Double leftValue,Double rightValue){
        drive.tankDrive(leftValue,rightValue);
        
    }
}
