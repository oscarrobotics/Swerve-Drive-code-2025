package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.*;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class Claw extends SubsystemBase {
     // All hardware classes already have WPILib integration
    final TalonFX m_intake = new TalonFX(61);
    final TalonFX m_mount = new TalonFX(62);
     
    //only a single neutral motor request is required for the system as it is always the same value
    private final NeutralOut m_brake = new NeutralOut();

 
    final TalonFXSimState m_intakeSim = m_intake.getSimState();
    final TalonFXSimState m_mountSim = m_mount.getSimState();
 
    final VelocityTorqueCurrentFOC m_intakeFXOut = new VelocityTorqueCurrentFOC(0).withSlot(0);
    final PositionTorqueCurrentFOC m_mountFXOut = new PositionTorqueCurrentFOC(0).withSlot(0);
 
    final CANcoder m_mount_encoder = new CANcoder(17);


     
    //


   
    public final AngularVelocity k_max_wheel_speed = RevolutionsPerSecond.of(1000/60.0);
    

    
    public final AngularVelocity k_max_arm_speed = RevolutionsPerSecond.of(5/60.0);



    // physical dimentions of the robot
    public final Distance k_intake_length = Inches.of(12);
    public final Distance k_algehook_center_length = Inches.of(20);


    public final Angle k_min_angle =  Degrees.of(0);
    public final Angle k_max_angle = Degrees.of(90);

    public final Angle k_load_coral_position = Degrees.of(0);

    public final Angle k_coral_position_1 = Degrees.of(0);
    public final Angle k_coral_position_2 = Degrees.of(0);
    public final Angle k_coral_position_3 = Degrees.of(0);
    public final Angle k_coral_position_4 = Degrees.of(0);

    public final Angle k_alge_position_1 = Degrees.of(0);
    public final Angle k_alge_position_2 = Degrees.of(0);
    public final Angle k_alge_position_3 = Degrees.of(0);
    public final Angle k_alge_position_4 = Degrees.of(0);

    public final Angle k_process_alge_position = Degrees.of(0);
    public final Angle k_barge_alge_position = Degrees.of(0);

    public boolean s_has_coral = false;
    


    public Claw(){

        // configure the motor controller
        TalonFXConfiguration m_intake_config = new TalonFXConfiguration();

        /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        m_intake_config.Slot0.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
        m_intake_config.Slot0.kI = 0; // No output for integrated error
        m_intake_config.Slot0.kD = 0; // No output for error derivative
        m_intake_config.Slot0.kP = 5; // An error of 1 rotation per second results in 5 A output
        m_intake_config.Slot0.kG = 0.1;
        m_intake_config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // Peak output of 5 A
        m_intake_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(5))
        .withPeakReverseTorqueCurrent(Amps.of(-5));

        //motion magic settings
        m_intake_config.MotionMagic.MotionMagicAcceleration = 10;
        m_intake_config.MotionMagic.MotionMagicJerk = 100;

       

       
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_intake.getConfigurator().apply(m_intake_config);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }



        TalonFXConfiguration  m_mount_config = new TalonFXConfiguration();
       
        m_mount_config.Slot0.kP = 60; // An error of 1 rotation results in 60 A output
        m_mount_config.Slot0.kI = 0; // No output for integrated error
        m_mount_config.Slot0.kD = 6; // A velocity of 1 rps results in 6 A output
        
        // Peak output of 5 A
        m_mount_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(5))
        .withPeakReverseTorqueCurrent(Amps.of(-5));

        
        


        // bind the remote encoder to the mount motor

        CANcoderConfiguration m_mount_encoder_config = new CANcoderConfiguration();
        m_mount_encoder_config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.6));
        m_mount_encoder_config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        
        

        
        m_mount_config.Feedback.FeedbackRemoteSensorID = m_mount_encoder.getDeviceID();
        m_mount_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        m_mount_config.Feedback.SensorToMechanismRatio = 1.0;
        m_mount_config.Feedback.RotorToSensorRatio = 100;
        
        //for motion magic controls
        m_mount_config.MotionMagic.MotionMagicCruiseVelocity = 10;
        m_mount_config.MotionMagic.MotionMagicExpo_kV = 1;
        m_mount_config.MotionMagic.MotionMagicExpo_kA = 1;



        // m_fx.getConfigurator().apply(fx_cfg);
        status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = m_mount_encoder.getConfigurator().apply(m_mount_encoder_config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        
        
        status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = m_mount.getConfigurator().apply(m_mount_config);
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

    private void set_intake_speed(AngularVelocity speed){

        if (speed.gt(k_max_wheel_speed)){
            //logger.log("max wheel speed exceeded")
            speed = k_max_wheel_speed;

        }
        else if(speed.lt(k_max_wheel_speed.unaryMinus())){//unary Minus is negate
            //logger.log("negativce max wheel speed exceeded")
            speed = k_max_wheel_speed.unaryMinus();
        }

        m_intake.setControl(m_intakeFXOut.withVelocity(speed));

    }

    private void stop_intake(){
        m_intake.setControl(m_brake);
    }

    private Current get_intake_current(){

        return m_intake.getTorqueCurrent().getValue();
    }

    private BooleanSupplier intake_curent_exceeded(Current amps){

        BooleanSupplier current_trigger = ()-> get_intake_current().gt(amps);

        return current_trigger;

    }

    private void has_coral_true(){
        s_has_coral = true;
    }

    private void has_coral_false(){
        s_has_coral = false;
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
    public Command position_command(Angle position){

        //  \/ run command template returns a run command that does the thing insided      
        return run(()-> set_mount_angle(position));

    }

    public Command intake_coral_command(){

        return run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(300, RPM));})
            .until(intake_curent_exceeded(Amp.of(10)))
            .beforeStarting(this::has_coral_true)
            .withTimeout(10)
            .andThen(this::stop_intake
            );
    }

    public Command outtake_coral_command(){

        return run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(-300, RPM));})
            .withTimeout(4)
            .andThen(this::stop_intake
            );
    }

    
    
}
