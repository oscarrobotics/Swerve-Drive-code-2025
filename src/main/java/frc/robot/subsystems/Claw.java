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


public class Claw extends SubsystemBase {
     // All hardware classes already have WPILib integration
    
    final TalonFX m_mount = new TalonFX(61); 
    final TalonFX m_intake = new TalonFX(62); 
    //only a single neutral motor request is required for the system as it is always the same value
    private final NeutralOut m_brake = new NeutralOut();

 
    final TalonFXSimState m_intakeSim = m_intake.getSimState();
    final TalonFXSimState m_mountSim = m_mount.getSimState();
 


    final VelocityTorqueCurrentFOC m_intakeFXOut = new VelocityTorqueCurrentFOC(0).withSlot(0);
    final MotionMagicVelocityTorqueCurrentFOC m_intakeFXOut_v_mm = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
    final MotionMagicExpoTorqueCurrentFOC m_intakeFXOut_mm = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);

    final PositionTorqueCurrentFOC m_mountFXOut = new PositionTorqueCurrentFOC(0).withSlot(0);
    final MotionMagicExpoTorqueCurrentFOC m_mountFXOut_mm = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
    
    final CANcoder m_mount_encoder = new CANcoder(17);
    

     
    //


   
    public final AngularVelocity k_max_wheel_speed = RevolutionsPerSecond.of(1000/60.0);
    

    
    public final AngularVelocity k_max_arm_speed = RevolutionsPerSecond.of(5/60.0);



    // physical dimentions of the robot
    public final Distance k_intake_length = Inches.of(12);
    public final Distance k_algehook_center_length = Inches.of(20);


    public final Angle k_min_angle =  Rotation.of(-.15);
    public final Angle k_max_angle = Degrees.of(0.3);

    public final Angle k_load_coral_position = Degrees.of(0);

    public final Angle k_stowed = Rotation.of(0);
    


    public final Angle k_coral_position_1 = Rotation.of(0.28);
    public final Angle k_coral_position_2 = Rotation.of(0.1);
    public final Angle k_coral_position_3 = Rotation.of(0.0);
    public final Angle k_coral_position_4 = Rotation.of(-0.07);

    public final Angle k_alge_position_1 = Rotation.of(0);
    public final Angle k_alge_position_2 = Rotation.of(0);
    public final Angle k_alge_position_3 = Rotation.of(0);
    public final Angle k_alge_position_4 = Rotation.of(0);

    public final Angle k_process_alge_position = Rotation.of(0);
    public final Angle k_barge_alge_position = Rotation.of(0);

    public boolean m_has_coral = false;
    

    TalonFXConfiguration m_intake_config = new TalonFXConfiguration();
    TalonFXConfiguration  m_mount_config = new TalonFXConfiguration();


    private final double k_default_mount_ks = 0;
    private final double k_default_mount_kp = 80;
    private final double k_default_mount_ki = 0;
    private final double k_default_mount_kd = 8;
    private final double k_default_mount_kg = 5;
    private final double k_default_mount_kff = 0;
    // mm_expo gains
    private final double k_default_mount_kV = 10;
    private final double k_default_mount_kA = 10;
    private final double k_default_mount_cVelocity = 0.1; // used for both mm and mm_expo
    
    private final double k_mount_current_limit = 30;



    private final double k_default_intake_ks = 0;
    private final double k_default_intake_kp = 20;
    private final double k_default_intake_ki = 0;
    private final double k_default_intake_kd = 6;

    private final double k_default_intake_accel = 1;
    private final double k_default_intake_jerk = 1;
    private final double k_default_intake_cVelocity = 10; // used for both mm and mm_expo
    
    private final double k_intake_current_limit = 30;

    public Claw(){

        // configure the motor controller
        

        /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        m_intake_config.Slot0.kS = k_default_intake_ks; // To account for friction, add 2.5 A of static feedforward
        m_intake_config.Slot0.kI = k_default_intake_ki; // No output for integrated error
        m_intake_config.Slot0.kD = k_default_intake_kd; // No output for error derivative
        m_intake_config.Slot0.kP = k_default_intake_kp; // An error of 1 rotation per second results in 5 A output
        
        // Peak output of 5 A
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



        
       
        m_mount_config.Slot0.kP = k_default_mount_kp; // An error of 1 rotation results in 60 A output
        m_mount_config.Slot0.kI = k_default_mount_ki; // No output for integrated error
        m_mount_config.Slot0.kD = k_default_mount_kd; // A velocity of 1 rps results in 6 A output
        m_mount_config.Slot0.kG = k_default_mount_kg;
        m_mount_config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        // Peak output of 5 A
        m_mount_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(k_mount_current_limit))
        .withPeakReverseTorqueCurrent(Amps.of(-k_mount_current_limit));

        m_mount_config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        m_mount_config.MotorOutput.NeutralMode=NeutralModeValue.Brake;
            


        // bind the remote encoder to the mount motor

        CANcoderConfiguration m_mount_encoder_config = new CANcoderConfiguration();
        m_mount_encoder_config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.6));
        m_mount_encoder_config.MagnetSensor.withMagnetOffset(-0.339844);
        m_mount_encoder_config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        

        
        m_mount_config.Feedback.FeedbackRemoteSensorID = m_mount_encoder.getDeviceID();
        m_mount_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        m_mount_config.Feedback.SensorToMechanismRatio = 1.0;
        m_mount_config.Feedback.RotorToSensorRatio = 100;
        
        //for motion magic controls
        m_mount_config.MotionMagic.MotionMagicCruiseVelocity = k_default_mount_cVelocity;
        m_mount_config.MotionMagic.MotionMagicExpo_kV = k_default_mount_kV;
        m_mount_config.MotionMagic.MotionMagicExpo_kA = k_default_mount_kA;



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
        // m_mount.setPosition(0);

        SmartDashboard.putNumber("Mount kP", k_default_mount_kp); 
        SmartDashboard.putNumber("Mount kI", k_default_mount_ki);
        SmartDashboard.putNumber("Mount kD", k_default_mount_kd);
        SmartDashboard.putNumber("Mount kG", k_default_mount_kg);
        SmartDashboard.putNumber("Mount kff", k_default_mount_kff);
       
        
        SmartDashboard.putNumber("Mount Current Limit", k_mount_current_limit);
       
        SmartDashboard.putNumber("Mount Cruise Velocity", k_default_mount_cVelocity);
        SmartDashboard.putNumber("Mount kV", k_default_mount_kV);
        SmartDashboard.putNumber("Mount kA", k_default_mount_kA);

        SmartDashboard.putData("Update mount PID", new InstantCommand(this::configure_mount_from_dash));
        SmartDashboard.putData("Update intake PID", new InstantCommand(this::configure_intake_from_dash));

        register();

    }



    // internal methond to set the positon using a unit aware object
    private void set_mount_angle_mm(Angle position){

        //    gt is greater than 
        if (position.gt( k_max_angle)){
            
            // logger.log(position + " requested is greater than the max position ");
            position = k_max_angle;

        }
        else if (position.lt(k_min_angle)){

            //logger.log(position + " requested is less than the minimum position");
            position = k_min_angle;

        }

        m_mount.setControl(m_mountFXOut_mm.withPosition(position));


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


    public BooleanSupplier at_position(double tolerance){
        
        BooleanSupplier position_trigger = ()-> Math.abs(m_mount.getClosedLoopError().getValueAsDouble())<tolerance;
        
        return position_trigger;
    }

    public BooleanSupplier at_position(){

        return at_position(0.005);
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

    private BooleanSupplier intake_curent_exceeded(Current amps){

        BooleanSupplier current_trigger = ()-> get_intake_current().gt(amps);

        return current_trigger;

    }

    private void has_coral_true(){
        m_has_coral = true;
    }

    private void has_coral_false(){
        m_has_coral = false;
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
    public Command set_position_command_mm(Angle position){

        //  \/ run command template returns a run command that does the thing insided      
        return run(()-> set_mount_angle_mm(position));

    }

    public Command intake_coral_command(){

        return new ParallelRaceGroup( run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(300, RPM));})
            .until(intake_curent_exceeded(Amp.of(30)))
            .beforeStarting(this::has_coral_true),
            new WaitCommand(4))
            .andThen(this::stop_intake);
    }

    public Command outtake_coral_command(){

        return run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(-170, RPM));})
            .withTimeout(2)
            .andThen(this::stop_intake
            ).andThen(this::has_coral_false);

        
    }

    public Command launch_coral_command(){

        return run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(-170, RPM));})
            .withTimeout(2)
            .andThen(this::stop_intake
            ).andThen(this::has_coral_false);

    }


    public void publish_mount_data(){

        // put data important for charaterizing the data to the smart dashboard
        double set_point =m_mount.getClosedLoopReference().getValueAsDouble();
        double error = m_mount.getClosedLoopError().getValueAsDouble();
        double tcurrent = m_mount.getTorqueCurrent().getValueAsDouble();
        double velocity = m_mount.getVelocity().getValueAsDouble();
        double acceleration = m_mount.getAcceleration().getValueAsDouble();
        double position = m_mount.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Mount Set Point", set_point);
        SmartDashboard.putNumber("Mount Error", error);
        SmartDashboard.putNumber("Mount Torque Current", tcurrent);
        SmartDashboard.putNumber("Mount Velocity", velocity);
        SmartDashboard.putNumber("Mount Acceleration", acceleration);
        SmartDashboard.putNumber("Mount Position", position);

        // SmartDashboard.putData("Claw Sim", m_mech2d);


        // SmartDashboard.putData("PID_Verification", m_elevator_motor.getClosedLoopSlot()
        
    }

    public void configure_mount_from_dash(){
        // configure the motor from the smart dashboard
        m_mount_config.Slot0.kP = SmartDashboard.getNumber("Mount kP", k_default_mount_kp); 
        m_mount_config.Slot0.kI = SmartDashboard.getNumber("Mount kI", k_default_mount_ki);
        m_mount_config.Slot0.kD = SmartDashboard.getNumber("Mount kD", k_default_mount_kd);
        m_mount_config.Slot0.kG = SmartDashboard.getNumber("Mount kG", k_default_mount_kg);
       

        m_mount_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(SmartDashboard.getNumber("Mount Current Limit", k_mount_current_limit)))
        .withPeakReverseTorqueCurrent(Amps.of(-SmartDashboard.getNumber("Mount Current Limit", k_mount_current_limit)));

        m_mount_config.MotionMagic.MotionMagicCruiseVelocity = SmartDashboard.getNumber("Elevator Cruise Velocity", k_default_mount_cVelocity);
        m_mount_config.MotionMagic.MotionMagicExpo_kV = SmartDashboard.getNumber("Elevator kV", k_default_mount_kV);
        m_mount_config.MotionMagic.MotionMagicExpo_kA = SmartDashboard.getNumber("Elevator kA", k_default_mount_kA);


        
        m_mount.getConfigurator().apply(m_mount_config);


    }

    @Override
    public void periodic() {
        super.periodic();
        publish_intake_data();
        publish_mount_data();
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

        m_intake_config.MotionMagic.MotionMagicCruiseVelocity = SmartDashboard.getNumber("intake Cruise Velocity", k_default_intake_cVelocity);
        // m_mount_config.MotionMagic.MotionMagicExpo_kV = SmartDashboard.getNumber("Elevator kV", k_default_intake_kV);
        // m_mount_config.MotionMagic.MotionMagicExpo_kA = SmartDashboard.getNumber("Elevator kA", k_default_intake_kA);
        
        m_intake_config.MotionMagic.MotionMagicAcceleration = SmartDashboard.getNumber("Intake Accel", k_default_intake_accel);
        m_intake_config.MotionMagic.MotionMagicJerk = SmartDashboard.getNumber("Intake Jerk", k_default_intake_jerk);
        
        m_intake.getConfigurator().apply(m_intake_config);


    }
    

    
    
}
