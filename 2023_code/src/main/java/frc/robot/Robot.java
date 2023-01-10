package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private XboxController control1;
  private XboxController control2;
  private CANSparkMax dt_ll;
  private CANSparkMax dt_lf;
  private CANSparkMax dt_lf2;
  private CANSparkMax dt_rf;
  private CANSparkMax dt_rf2;
  private CANSparkMax dt_rl;
  private MotorControllerGroup mc_left; 
  private MotorControllerGroup mc_right; 
  private DifferentialDrive dt_main;

 @Override
  public void robotInit() {
    //These are just placeholder ID's
    dt_ll = new CANSparkMax(1, MotorType.kBrushed);
    dt_lf = new CANSparkMax(2, MotorType.kBrushed);
    dt_lf2 = new CANSparkMax(3, MotorType.kBrushed);
    dt_rl = new CANSparkMax(4, MotorType.kBrushed);
    dt_rf = new CANSparkMax(5, MotorType.kBrushed);
    dt_rf2 = new CANSparkMax(6, MotorType.kBrushed);
    control1 = new XboxController(1);
    control2 = new XboxController(2);
    mc_left = new MotorControllerGroup(dt_ll, dt_lf, dt_lf2);
    mc_right  = new MotorControllerGroup(dt_rl, dt_rf, dt_rf2);
    dt_main = new DifferentialDrive(mc_left, mc_right);  
    mc_right.setInverted(true);
   }
  @Override
  public void teleopPeriodic() {
    //Exotic Drivetrain Code
    boolean goReverse = false;
    while (goReverse = false) {
      dt_main.tankDrive(-control1.getLeftY()*.70, control1.getRightY()*.70);
    }
    while (goReverse = true) {
      dt_main.tankDrive(control1.getLeftY()*.70, -control1.getRightY()*.70);
    }
    if (getBButtonPressed()) {
      !goReverse;
    }
  }
  
  @Override
  public void autonomousInit() {
  //Will add later
  }

  @Override
  public void autonomousPeriodic() {
  //Will add later
  }
}
