/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.libraries;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class FakeJoystick extends Joystick{
    private double[] axis = {0.0, 0.0, 0.0, 0.0};

    public FakeJoystick(double x, double y, double z, double t) {
        super(-1);
        setX(x);
        setY(y);
        setZ(z);
        setThrottle(t);
    }

    public FakeJoystick() {
        this(0,0,0,0);
    }

    @Override
    public double getRawAxis(int axis) {
        return this.axis[axis];
    }

    public void setX(double x){
        axis[0] = x; 
    }

    public void setY(double y){
        axis[1] = y; 
    }

    public void setZ(double z){
        axis[2] = z; 
    }

    public void setThrottle(double t){
        axis[3] = t; 
    }

    public void setBackwards(double percent){
        setX(0);
        setY(1);
        setZ(0);
        setThrottle(Math.abs(percent));
    }
    
    public void setForwards(double percent){
        setX(0);
        setY(-1);
        setZ(0);
        setThrottle(Math.abs(percent));
    }
    
    public void setLeftShift(double percent){
        setX(-1);
        setY(0);
        setZ(0);
        setThrottle(Math.abs(percent));
    }
    
    public void setRightShift(double percent){
        setX(1);
        setY(0);
        setZ(0);
        setThrottle(Math.abs(percent));
    }
    public void setLeftRotate(double percent){
        setX(0);
        setY(0);
        setZ(-1);
        setThrottle(Math.abs(percent));
    }
    
    public void setRightRotate(double percent){
        setX(0);
        setY(0);
        setZ(1);
        setThrottle(Math.abs(percent));
    }

    public void setStop(){
        setX(0);
        setY(0);
        setZ(0);
        setThrottle(0);
    }
}
