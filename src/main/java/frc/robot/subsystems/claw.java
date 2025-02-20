package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class Claw extends SubsystemBase {
     // All hardware classes already have WPILib integration
     final TalonFX m_intake = new TalonFX(31);
     final TalonFX m_mount = new TalonFX(32);
     
    //only a single neutral motor request is required for the system as it is always the same value
    private final NeutralOut m_brake = new NeutralOut();

 
     final TalonFXSimState m_intakeSim = m_intake.getSimState();
     final TalonFXSimState m_mountSim = m_mount.getSimState();
 
     final VelocityTorqueCurrentFOC m_intakeFXOut = new VelocityTorqueCurrentFOC(0).withSlot(0);
     final PositionTorqueCurrentFOC m_mountFXOut = new PositionTorqueCurrentFOC(0).withSlot(0);
 
     final TalonFXConfiguration m_mountConfig = new TalonFXConfiguration();
     



   
    public final AngularVelocity k_max_wheel_speed = RevolutionsPerSecond.of(1000/60.0);
    

    
    public final AngularVelocity k_max_arm_speed = RevolutionsPerSecond.of(5/60.0);

    public final Angle k_min_angle =  Degrees.of(0);
    public final Angle k_max_angle = Degrees.of(90);

    public final Angle k_load_coral_position = Degrees.of(0);
   



     public Claw(){

      // configure the motor controller
      TalonFXConfiguration m_intakeConfig = new TalonFXConfiguration();

       /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
       m_intakeConfig.Slot0.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
       m_intakeConfig.Slot0.kI = 0; // No output for integrated error
       m_intakeConfig.Slot0.kD = 0; // No output for error derivative
       m_intakeConfig.Slot0.kP = 5; // An error of 1 rotation per second results in 5 A output
       // Peak output of 20 A
       m_intakeConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(20))
       .withPeakReverseTorqueCurrent(Amps.of(-20));

       
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_intake.getConfigurator().apply(m_intakeConfig);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }



         TalonFXConfiguration  m_mountConfig = new TalonFXConfiguration();
       /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    
        m_mountConfig.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
        m_mountConfig.Slot1.kI = 0; // No output for integrated error
        m_mountConfig.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
        // Peak output of 20 A
        m_mountConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(20))
        .withPeakReverseTorqueCurrent(Amps.of(-20));

        
        // status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_mount.getConfigurator().apply(m_mountConfig);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }
        
        //set the position of the mechanism to 0, this is not a control but a delclaration that the position it is in is 0
        m_mount.setPosition(0);


     }



    // internal methond to set the positon using a unit aware object
    private void set_mount_angle(Angle position){

           // gt is greater than 
        if (position.gt( k_max_angle)){
            
            // logger.log(position + " requested is greater than the max position ");
            position = k_max_angle;

        }
        else if (position.lt(k_min_angle)){

            //logger.log(position + " requested is less than the minimum position");
            position = k_min_angle;

        }

        m_mount.setControl(m_mountFXOut.withPosition(position));


  }
 
 // roboto container(or other commands) can call this methond to get acces to a
    // command that will call the internal private method that directly controls the 
    // motor with the postion suppliied when this method was called
    // ie, some part of code wants the position to be X, so it calls 
    // command = m_EX_Subsystem.getposition_commmand(x) and command now
    // contains a command of run(()->set_turntable_posiotn(x))
    // it will then pass/bind this command to the Command Scheduler to run 
    // when approprite( determinted by the binding type and the command type, ie run,...)
    // the command sceduler will then call ()->set_turntable_position(x).
    // the ()-> is needted becuse the Command scheduler cannont provide parameters at 
    // time it would call it, so instead ()-> is a "wrapper" method with no name,
    // called an anonomus function that takes no parameters, what "()" means, but calls
    // the inner method that is "hard coded" with the value x at the time m_EX_Subsystem.getposition_commmand(x)
    // is called, giving you a new run command/anonomus method for every postion, a meathod that writes methods to 
    // call methdods

    //note that this method returns a command and is no a command itself
    public Command get_position_command(Angle position){

        //  \/ run command template returns a run command that does the thing insided      
        return run(()-> set_mount_angle(position));

       }       
    
}
