package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.*;
import java.math.*;
//import com.ctre.phoenix.sensors.Pigeon2;




public class Robot extends TimedRobot {
  private XboxController control1;
  private XboxController control2;
  private CANSparkMax dt_ll;
  private CANSparkMax dt_lf;
  private CANSparkMax dt_lf1;
  private CANSparkMax dt_rf;
  private CANSparkMax dt_rf1;
  private CANSparkMax dt_rl;
  private MotorControllerGroup mc_left;
  private MotorControllerGroup mc_right;
  private DifferentialDrive dt_main;
  private Double errorAdjust;
  //private Pigeon2 pig;
  private RelativeEncoder dt_enc_1;
  private final double kEncConstant = Math.PI * 6 / 8.46;
  private final double kP = 0.04;


 @Override
  public void robotInit() {
    //pid = new PIDController(5, 0, 15);


    //pig = new Pigeon2(7);


    dt_ll = new CANSparkMax(4, MotorType.kBrushless);
    dt_lf = new CANSparkMax(6, MotorType.kBrushless);
    dt_lf1 = new CANSparkMax(5, MotorType.kBrushless);


    dt_rl = new CANSparkMax(1, MotorType.kBrushless);
    dt_rf = new CANSparkMax(2, MotorType.kBrushless);
    dt_rf1 = new CANSparkMax(3, MotorType.kBrushless);


    control1 = new XboxController(1);
    //control2 = new XboxController(2);


    dt_enc_1 = dt_ll.getEncoder();


    mc_left = new MotorControllerGroup(dt_ll, dt_lf, dt_lf1);
    mc_right  = new MotorControllerGroup(dt_rl, dt_rf, dt_rf1);
    dt_main = new DifferentialDrive(mc_left, mc_right);  
    mc_left.setInverted(true);
   }


  @Override
  public void teleopPeriodic() {
    getTelem();
    getTurn();
    if(control1.getBButton()){
    dt_main.tankDrive(-errorAdjust, errorAdjust);
    }
    else{dt_main.tankDrive(control1.getLeftY()*.5, control1.getRightY()*.5);}
  }
 
  @Override
  public void autonomousInit() {
    dt_enc_1.setPosition(0.0);
  }


  @Override
  public void autonomousPeriodic() {
    int setpoint = 10;
    double feet = (dt_enc_1.getPosition()*kEncConstant)/12;
    double error = setpoint - feet;
    double output = kP * error;


    dt_main.tankDrive(-output, output);
  }


  public void getTurn(){
    final double constant = .025;
    errorAdjust = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) * constant;
  }


  @Override
  public void disabledInit() {
    dt_enc_1.setPosition(0.0);
  }
  //Gets telemetry values
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
    SmartDashboard.putNumber("Encoder feet", feet);
    //SmartDashboard.putNumber("Yaw", pig.getYaw());
    //SmartDashboard.putNumber("Pitch", pig.getPitch());
    //SmartDashboard.putNumber("Roll", pig.getRoll());
  }
}

