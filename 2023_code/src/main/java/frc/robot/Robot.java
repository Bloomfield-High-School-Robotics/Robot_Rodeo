
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  //private vars

   @Override
  public void robotInit() {

  }
  @Override
  public void teleopPeriodic() {
    //limeVals is an array of doubles length of 3. first is the x value read from the limelight, second is the y value, and 3rd is area
    Double[] limeVals=limelightPeriodic();
    
  }
  public Double[] limelightPeriodic(){
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
  Double[] ret = new Double[4];
  ret[0]=x;
  ret[1]=y;
  ret[2]=area;
  ret[3]=v;
  return ret;
  }
    @Override
    public void autonomousInit() {
  
    }

  @Override
  public void autonomousPeriodic() {

  }
}

