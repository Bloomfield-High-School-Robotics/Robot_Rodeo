package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.*;
import java.math.*;

public class Drivetrain {
    private XboxController control1 = new XboxController(1);
    private XboxController control2 = new XboxController(2);
    private CANSparkMax dt_ll = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax dt_lf = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax dt_lf1 = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax dt_rf = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax dt_rf1 = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax dt_rl = new CANSparkMax(1, MotorType.kBrushless);
    private MotorControllerGroup mc_left = new MotorControllerGroup(dt_ll, dt_lf, dt_lf1);
    private MotorControllerGroup mc_right = new MotorControllerGroup(dt_rl, dt_rf, dt_rf1);
    private DifferentialDrive dt_main = new DifferentialDrive(mc_left, mc_right);  
    private RelativeEncoder dt_enc_1 = dt_ll.getEncoder();
    private RelativeEncoder dt_enc_2 = dt_rl.getEncoder();
    private PIDController pid;
    private final double kEncConstant = Math.PI * 6 / 8.46;
    private final double kP = 0.3; 
    private final double kI = 0.01; 
    private final double kD = 0.08; 
    private int setpoint;
    private Translation2d tTopLeft;
    private Translation2d tTopRight;
    private Translation2d tBottomLeft;
    private Translation2d tBottomRight;
    private Translation2d center;
    private DifferentialDriveKinematics dt_kinematics;
    private DifferentialDriveOdometry dt_odometry;
    private double k_trackWidth = 2.08;
    private Pigeon2 pig = new Pigeon2(7);


    public Drivetrain() {
        mc_left.setInverted(true);

        pid = new PIDController(kP, kI, kD);

        tTopLeft = new Translation2d(-1.13, 1.34);
        tTopRight = new Translation2d(1.13, 1.34);
        tBottomLeft = new Translation2d(-1.13, -1.34);
        tBottomRight = new Translation2d(1.13, -1.34);

        center = new Translation2d(0, 0);

        dt_enc_1.setPositionConversionFactor(kEncConstant/12);
        dt_enc_2.setPositionConversionFactor(kEncConstant/12);

        dt_kinematics = new DifferentialDriveKinematics(k_trackWidth);

        dt_odometry = new DifferentialDriveOdometry(null, dt_enc_1.getPosition(), dt_enc_2.getPosition());
    }

    public void drive() {
        double errorAdjust = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double feet = ((dt_enc_1.getPosition()*kEncConstant)/12);
        errorAdjust *= 0.05;

        if (control1.getBButton()) {
            dt_main.tankDrive(-errorAdjust, errorAdjust);
        }
        else {
            dt_main.tankDrive(control1.getLeftY()*.90, control1.getRightY()*.90);
        }

        //Limelight Pseudo-Code
        /*
        boolean goPID = false;
        double distance = 0; 
        if (control1.getBButton()) {
            goPID = !goPID;
            double y_off = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
            if (goPID = true && ( tx > 1.5 || tx < -1.5) ) {
                double errorAdjust = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
                errorAdjust *= 0.05;
                dt_main.tankDrive(-errorAdjust, errorAdjust);
            }
            if (goPID = true && ( tx < 1.5 || tx > -1.5 ) ) {
                distance = ( h2 - h1 ) / Math.tan( k_limeAngle + y_off );
                double output = pid.calculate(feet, distance);
                if(Math.abs(output) > 1) {
                    dt_main.tankDrive(-0.75, -0.75);
                }
                else {
                    dt_main.tankDrive(-output*.75, -output*.75);
                }
            }
        }
        */    
    }

    public void auton() {
        setpoint = 30; 
        double feet = ((dt_enc_1.getPosition()*kEncConstant)/12);
        double output = pid.calculate(feet, setpoint);
        if(Math.abs(output) > 1){
            dt_main.tankDrive(-0.75, -0.75);
        }
        else{
            dt_main.tankDrive(-output*.75, -output*.75);
        }
    }

    public void getTelem() {
        SmartDashboard.putNumber("Left Y-axis", control1.getLeftY());
        SmartDashboard.putNumber("Right Y-axis", control1.getRightY());
        SmartDashboard.putNumber("Right Trigger", control1.getRightTriggerAxis());
        SmartDashboard.putNumber("Left Trigger", control1.getLeftTriggerAxis());
        SmartDashboard.putNumber("tx",NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
        SmartDashboard.putNumber("ty",NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
        //SmartDashboard.putNumber("ta",NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0));
        SmartDashboard.putNumber("Error tx",NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
        double feet = (dt_enc_1.getPosition()*kEncConstant)/12;
        double rpm = dt_enc_1.getVelocity();
        SmartDashboard.putNumber("Encoder rpm", rpm);
        SmartDashboard.putNumber("Encoder feet", feet);
        SmartDashboard.putNumber("Error for Auton PID", pid.getPositionError());
        SmartDashboard.putNumber("Setpoint", setpoint);
        //SmartDashboard.putNumber("Yaw", pig.getYaw());
        //SmartDashboard.putNumber("Pitch", pig.getPitch());
        //SmartDashboard.putNumber("Roll", pig.getRoll());
    }
}
