package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class Mech {
    
    public CANSparkMax intake = new CANSparkMax(10, MotorType.kBrushed);
    private CANSparkMax elevator = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax angle = new CANSparkMax(9, MotorType.kBrushless);
    public CANSparkMax cowCatch = new CANSparkMax(8, MotorType.kBrushless);
    private PIDController ElPid= new PIDController(0.1, 0, 0.1);
    private double kEncElConstant = 2 * Math.PI / 23;
    private RelativeEncoder elEncoder = elevator.getEncoder();
    private double EL_LIMIT = 0.75;
    double endpoint = 2.5;
    double curpoint = 0;

    // Control and Safety Parameters
        // Encoder Boundaries (Note: 42 counts is equal to 1 Revolution)
        // NOTE ! ! ! : Make sure cow catcher and angle are set to "default" positions.
    final int COWENC_MAX = 21; // Assuming Cow Catcher starts folded into robot.
    final int ANGLE_MAX = 21; // Assuming angle starts folded on top of robot.

    public Mech() {

        //output in feet
        elEncoder.setPositionConversionFactor( kEncElConstant / 12 );

    }

    public void Intake(boolean ward, boolean back){
        // triggers
        if (ward) {
            intake.set(1);
        }
        else if(back){
            intake.set(-1);
        }
        else{
            intake.set(0);
        }
    }

    public void Elevator(boolean butt){
        curpoint = elEncoder.getPosition();
        if (butt) {
            double output = ElPid.calculate(curpoint, endpoint);
            if (Math.abs(output) > EL_LIMIT) {
                output = EL_LIMIT;
            }
            elevator.set(output);
        }
        else if (!butt) {
            double output = ElPid.calculate(curpoint, 0);
            if (Math.abs(output) > EL_LIMIT) {
                output = EL_LIMIT;
            }
            elevator.set(output);
        }
        else {
            elevator.set(0);
        }
    }

    public void ElevatorManual(boolean front, boolean back) {
        if (front) {
            elevator.set(.75);
        }
        else if (back) {
            elevator.set(-.75);
        }
        else {
            elevator.set(0);
        }
    }

    public void inAngle(boolean ward, boolean back){
        if(ward){
            angle.set(.25);
        }
        else if(back){
            angle.set(-.25);
        }
        else{
            angle.set(0);
        }
    }
    
    public void cowCatch(boolean ward, boolean back) {
        if (ward && cowCatch.getEncoder().getPosition() < COWENC_MAX) {
            cowCatch.set(0.05);
        }
        else if (back && cowCatch.getEncoder().getPosition() > 0) {
            cowCatch.set(-0.05);
        }
        else { 
            cowCatch.set(0.0);
        }
    }
}