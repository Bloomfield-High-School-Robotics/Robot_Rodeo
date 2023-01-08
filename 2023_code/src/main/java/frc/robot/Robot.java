
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.drivetrain;
import frc.robot.constants;
public class Robot extends TimedRobot {
  //private vars
  private XboxController control;
  private drivetrain drive;
  //calculated error to be determined in the lineUpTarget function
  private double adjustError;

   @Override
public void robotInit() {
  //new xbox controller 
  control = new XboxController(0);
  //new drivetrain (check drivetrain class in the files of the same folder)
  drive= new drivetrain();
}
  @Override
  public void teleopPeriodic() {
    //limeVals is an array of doubles length of 3. first is the x value read from the limelight, second is the y value, and 3rd is area
    double[] limeVals=limelightPeriodic();
    //if a button is held down and theres a valid target in sight
    if(control.getAButton() && limeVals[3]==1.0){
      //calling the move function from drivetrain class (tank drive) with the errors as the inputs
      drive.move(adjustError,adjustError);
    }
    else{
      //regular tank drive
      drive.move(control.getLeftY(),control.getLeftX());
    }
    
  }
  public double[] limelightPeriodic(){
  //getting values read from limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  //read values periodically
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double v = tv.getDouble(0.0);

  //post to smart dashboard periodically
  SmartDashboard.putNumber("LimelightX", x);
  SmartDashboard.putNumber("LimelightY", y);
  SmartDashboard.putNumber("LimelightArea", area);
  SmartDashboard.putNumber("LimelightValidTarget",v);
  //shove them into a double array and sent them to teleop periodic
  double[] ret = new double[4];
  ret[0]=x;
  ret[1]=y;
  ret[2]=area;
  ret[3]=v;
  return ret;
  }
  public void lineUpTarget(){
    //getting the horizontal error
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //this error is the horizontal distance multiplied by our steer constant in constants.java
    adjustError = constants.STEER_K*tx;
    
  }
}
    

