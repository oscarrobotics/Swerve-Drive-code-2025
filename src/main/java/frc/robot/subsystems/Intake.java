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
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
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
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Map;
import java.util.function.BooleanSupplier;


public class Intake extends SubsystemBase {
     // All hardware classes already have WPILib integration
    
    
    final TalonFX m_intake = new TalonFX(62); 
    //only a single neutral motor request is required for the system as it is always the same value
    private final NeutralOut m_brake = new NeutralOut();

 
    final TalonFXSimState m_intakeSim = m_intake.getSimState();
    

    
    final MotionMagicVelocityTorqueCurrentFOC m_intakeFXOut_v_mm = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
    final MotionMagicExpoTorqueCurrentFOC m_intakeFXOut_ep_mm = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);

    
    public final AngularVelocity k_max_wheel_speed = RevolutionsPerSecond.of(1000/60.0);
    
    // physical dimentions of the robot
    public final Distance k_intake_length = Inches.of(12);
    public final Distance k_algehook_center_length = Inches.of(20);


    TalonFXConfiguration m_intake_config = new TalonFXConfiguration();
  
    private final double k_default_intake_ks = 10; // output to overcome static friction
    private final double k_default_intake_kp = 30; // proportional
    private final double k_default_intake_ki = 0; // integral
    private final double k_default_intake_kd = 3; //derivative

    private final double k_default_intake_accel = 1;
    private final double k_default_intake_jerk = 1;
    
    private final double k_intake_current_limit = 30;

    public boolean m_has_coral = false;
    public boolean m_has_alge = false;
   
    private boolean m_coraling_state = true; // weather the robot is supposed to be manpulating the coral or Algae

    private ShuffleboardTab  m_intake_tab = Shuffleboard.getTab("Intake Tuning");

    // private Generi



    public Intake(){

        // configure the motor controller
        

        /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        m_intake_config.Slot0.kS = k_default_intake_ks; // To account for friction, add 2.5 A of static feedforward
        m_intake_config.Slot0.kI = k_default_intake_ki; // No output for integrated error
        m_intake_config.Slot0.kD = k_default_intake_kd; // No output for error derivative
        m_intake_config.Slot0.kP = k_default_intake_kp; // An error of 1 rotation per second results in 5 A output
        
        m_intake_config.Slot0.kA =10; // No output for error derivative
        m_intake_config.Slot0.kV = 10;        // Peak output of 5 A
        m_intake_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(30))
        .withPeakReverseTorqueCurrent(Amps.of(-30));
        //motion magic settings
        m_intake_config.MotionMagic.MotionMagicAcceleration = 300;
        m_intake_config.MotionMagic.MotionMagicJerk = 3000;

        
       
        
       
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_intake.getConfigurator().apply(m_intake_config);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }



        
       
     
            


        // bind the remote encoder to the mount motor

      

     
        
        //for motion magic controls

        
        //set the position of the mechanism to 0, this is not a control but a delclaration that the position it is in is 0
        // m_mount.setPosition(0);

        

    }




    private void set_intake_speed(AngularVelocity speed){

        // if (speed.gt(k_max_wheel_speed)){
        //     //logger.log("max wheel speed exceeded")
        //     speed = k_max_wheel_speed;

        // }
        // else if(speed.lt(k_max_wheel_speed.unaryMinus())){//unary Minus is negate
        //     //logger.log("negativce max wheel speed exceeded")
        //     speed = k_max_wheel_speed.unaryMinus();
        // }

        m_intake.setControl(m_intakeFXOut_v_mm.withVelocity(speed));

    }
    


    private void advance_intake(Distance delta){

        double gear_ratio = 2;
        Distance intake_diameter = Inches.of(4);

        Angle delta_rev = Rotation.of(delta.div(intake_diameter.times(Math.PI)).magnitude());

        Angle cur_position = m_intake.getPosition().getValue();

        m_intake.setPosition(cur_position.plus(delta_rev));




    }
    

    public void doing_coral(){

        m_coraling_state = true;
    }

    public void doing_alge(){

        m_coraling_state = false;
    }

    public BooleanSupplier coraling(){

        return ()-> m_coraling_state;
    }
   


    public BooleanSupplier has_coral(){

        return ()-> m_has_coral;
    }
        




    private void stop_intake(){
        m_intake.setControl(m_brake);
    }

    private Current get_intake_current(){

        return m_intake.getTorqueCurrent().getValue();
    }


    private Timer current_Timer = new Timer();
    private BooleanSupplier intake_curent_exceeded(Current amps , double time){

        BooleanSupplier current_trigger = ()->{

        if (get_intake_current().gt(amps))
            
            current_Timer.start();

        if(get_intake_current().lte(amps)){
            current_Timer.stop();
        }
        if (current_Timer.get()>time){
            current_Timer.stop();
            current_Timer.reset();
            return true;
        }
        else
            return false;

        };
        return current_trigger;

    }

    private void has_coral_true(){
        m_has_coral = true;
    }

    private void has_coral_false(){
        m_has_coral = false;
    }

    private void has_alge_true(){
        m_has_alge = true;
    }

    private void has_alge_false(){
        m_has_alge = false;
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

    

    public Command intake_coral_command(){

        return new ParallelRaceGroup( run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(300, RPM));})
            .until(intake_curent_exceeded(Amp.of(35),3))
            .beforeStarting(this::has_coral_true),
            new WaitCommand(1.5))
            .andThen(this::stop_intake);
    }

    public Command continuous_intake(){
        // i dont think this will properly flip the intake direction, need to test
        return  runEnd(()->set_intake_speed(AngularVelocity.ofBaseUnits(m_coraling_state?300:-300, RPM)),this::stop_intake);
        
    }
    public Command continuous_outake(){

        return  runEnd(()->set_intake_speed(AngularVelocity.ofBaseUnits(m_coraling_state?-300:300, RPM)),this::stop_intake);
        
    }
    

    public Command outtake_coral_command(){

        return run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(-170, RPM));})
            .withTimeout(2)
            .andThen(this::stop_intake
            ).andThen(this::has_coral_false);

        
    }


    public Command auto_outtake_coral_command(){

        return run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(-160, RPM));})
            .withTimeout(2)
            .andThen(this::stop_intake
            ).andThen(this::has_coral_false);

        
    }

    public Command intake_alge_command (){
        return new ParallelRaceGroup( run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(-200, RPM));})
            .until(intake_curent_exceeded(Amp.of(35),2))
            .beforeStarting(this::has_alge_true),
            new WaitCommand(2))
            .andThen(this::stop_intake);

    }

    public Command outtake_alge_command(){

        return run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(400, RPM));})
            .withTimeout(1)
            .andThen(this::stop_intake
            ).andThen(this::has_alge_false);

        
    }

    public Command launch_coral_command(){

        return run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(-170, RPM));})
            .withTimeout(1)
            .andThen(this::stop_intake
            ).andThen(this::has_coral_false);

    }


    

    @Override
    public void periodic() {
        super.periodic();
        publish_intake_data();

    }


    public void publish_intake_data(){

        // put data important for charaterizing the data to the smart dashboard
        double set_point =m_intake.getClosedLoopReference().getValueAsDouble();
        double error = m_intake.getClosedLoopError().getValueAsDouble();
        double tcurrent = m_intake.getTorqueCurrent().getValueAsDouble();
        double velocity = m_intake.getVelocity().getValueAsDouble();
        double acceleration = m_intake.getAcceleration().getValueAsDouble();
        double position = m_intake.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Intake Set Point", set_point);
        SmartDashboard.putNumber("Intake Error", error);
        SmartDashboard.putNumber("Intake Torque Current", tcurrent);
        SmartDashboard.putNumber("Intake Velocity", velocity);
        SmartDashboard.putNumber("Intake Acceleration", acceleration);
        SmartDashboard.putNumber("Intake Position", position);

        // SmartDashboard.putData("Claw Sim", m_mech2d);


        // SmartDashboard.putData("PID_Verification", m_elevator_motor.getClosedLoopSlot()
        
    }

    public void configure_intake_from_dash(){
        // configure the motor from the smart dashboard
        m_intake_config.Slot0.kP = SmartDashboard.getNumber("intake kP", k_default_intake_kp); 
        m_intake_config.Slot0.kI = SmartDashboard.getNumber("intake kI", k_default_intake_ki);
        m_intake_config.Slot0.kD = SmartDashboard.getNumber("intake kD", k_default_intake_kd);
        
       

        m_intake_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(SmartDashboard.getNumber("intake Torque Current", k_intake_current_limit)))
        .withPeakReverseTorqueCurrent(Amps.of(-SmartDashboard.getNumber("intake Torque Current", k_intake_current_limit)));

        // m_intake_config.MotionMagic.MotionMagicCruiseVelocity = SmartDashboard.getNumber("intake Cruise Velocity", k_default_intake_cVelocity);
        // m_mount_config.MotionMagic.MotionMagicExpo_kV = SmartDashboard.getNumber("Elevator kV", k_default_intake_kV);
        // m_mount_config.MotionMagic.MotionMagicExpo_kA = SmartDashboard.getNumber("Elevator kA", k_default_intake_kA);
        
        m_intake_config.MotionMagic.MotionMagicAcceleration = SmartDashboard.getNumber("Intake Accel", k_default_intake_accel);
        m_intake_config.MotionMagic.MotionMagicJerk = SmartDashboard.getNumber("Intake Jerk", k_default_intake_jerk);
        
        m_intake.getConfigurator().apply(m_intake_config);


    } 


    

    
    
}   
