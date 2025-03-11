package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.databind.deser.impl.FailingDeserializer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Elevator extends SubsystemBase{
    // All hardware classes already have WPILib integration
    final TalonFX m_elevator_motor = new TalonFX(51);
    final TalonFX m_elevator_motor_follower = new TalonFX(52);

    final CANcoder m_elevator_CANcoder = new CANcoder(59);
    

    final TalonFXSimState m_elevator_motorSim = m_elevator_motor.getSimState();

    final PositionTorqueCurrentFOC m_elevator_motorOut = new PositionTorqueCurrentFOC(0);

    final MotionMagicExpoTorqueCurrentFOC m_elevator_motorOut_mm = new MotionMagicExpoTorqueCurrentFOC(0);
    // final MotionMagicExpoTorqueCurrentFOC m_elevator_motorOut_mm = new MotionMagicExpoTorqueCurrentFOC(0);
    // used normal motion magic for to try and get it to work, but elevator was secretly mechnaically bad 
    // so probably will revert back to expo when fixed
    // final MotionMagicTorqueCurrentFOC m_elevator_motorOut_mm = new MotionMagicTorqueCurrentFOC(0);
    // final PositionTorqueCurrentFOC m_elevator_motorOut_mm = new PositionTorqueCurrentFOC(0);
    
    private final NeutralOut m_brake = new NeutralOut();



    //if external cancoder isnt used
    public final double k_elevator_rotations = 3.7;
    public final double k_elevator_ratio = k_elevator_rotations*1;



    public final Angle k_elevator_min_rot = Rotations.of(0);
    public final Angle k_elevator_max_rot = Rotations.of(0.625);

    public final Distance k_min_Distance =  Meters.of(0);
    public final Distance k_max_Distance = Meter.of(3);
    public final Angle k_stowed =  Rotation.of(0.02);

    public final Angle k_coral_level_sense_postion_1 = Rotations.of(0.15);
    public final Angle k_coral_level_sense_postion_2 = Rotations.of(0.35);
    public final Angle k_coral_level_sense_postion_3 = Rotations.of(0.551);
    public final Angle k_coral_level_sense_postion_4 = Rotations.of(0.615);




    // physical Characteristics of robot
    public final Distance min_axel_height = Inches.of(12);
    public final Distance max_axel_height = Inches.of(78);

    public final Mass k_carrage_mass = Kilogram.of(13);

    public final Distance k_windlass_radius = Inches.of(2.4);


    //goal heights
    
    public final Distance k_coral_level_1 = Meters.of(0.48);
    public final Distance k_coral_level_2 = Meters.of(0.81);
    public final Distance k_coral_level_3 = Meters.of(1.21);
    public final Distance k_coral_level_4 = Meters.of(1.83);

    public final Distance k_alge_level_1 = Meters.of(0);
    public final Distance k_alge_level_2 = Meters.of(0);
    public final Distance k_alge_stacked = Meters.of(0.4);
    

    public final Distance k_barge_height = Meters.of(0);

    public final Distance k_processor_height = Meters.of(0);


    // Theses are the gains for the controll loop of the elevator, they control how the elevator moves
    // by breaking up the nessay controll efforts into differnts aspects of the mechanial system suck how 
    // much force is necessary to overcome firction(ks), how much force is necessar to overcom gravity(kG),
    // how much force is necessary/how much effort(related to how fast it should move) it should apply
    // depending on how far it is from the set point(kP), how much force it should apply depending on how fast
    // it is approaching or leaving the set point(kD), and how much force it needs over small forces over time near 
    // while it is near the set point(kI)
    // KV and KA are used to descrive the velocity profile the elevator should follow to get to a position set point
    // when using motikon magic expo, there exact definintion is explained in the CTRE documentation,
    // higher values will make the elevator move slower, and should gradually be adjusted down as to the optimun value
    // cVelocity is cruise velocity, it is the max speed the elevator will move at when using motion magic
    // acceleration and jerk are used to control how fast the elevator can accelerate and decelerate in normal motion magic
    // to generate trapazoidal profiles, they are not used in motion magic expo.
    // At comp, while trying to get the elevator to work, I was mostly just adjusting these values trying to get the 
    // elevator to move, but as it turns out he eclevator was mechanically mechaninly malfunctioning, so no set of values
    // would have worked. the Elevator is 3 stages which means the force required to move it is 3 times greater than the acual
    // weight of the elevator( an issue whenn the force is tranfered though 5mm belt teeth), to mitigate this, the elevator has 
    // contant force sprigne that draw the carage up in the second stage, which "should" transfer back thought the other stages,
    // assinting the motor, unfortunately I belived this relied on an assumption that the string would perfectly transfer the force,
    // and but I belive the string is strecthing absorbing the force some of the spring provide and when lited by the first stage the 
    // stings just shink back down intead of the springs, making the elevator easy to lift from the carrace but not from the first
    // stage where the motor attaches, atleast for the first few inches concelling the issue.  
    private final double k_default_ks = 0;
    private final double k_default_kp = 120;
    private final double k_default_ki = 0;
    private final double k_default_kd = 3;
    private final double k_default_kg = 0;
    private double k_default_kff = 10;
    private double k_default_kff_offset = -1;
    // mm_expo gains
    private final double k_default_kV = 30;
    private final double k_default_kA = 5;
    private final double k_default_cVelocity = 0.2; // used for both mm and mm_expo
    
    // "normal" motion magic gains
    private final double k_default_Acceleration =10; //noma
    private final double k_default_jerk = 10;

    private final double k_current_limit = 70;


    private final ElevatorSim m_elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60(2), 
        3, 
        k_carrage_mass.in(Kilograms), 
        k_windlass_radius.in(Meters),
        min_axel_height.in(Meters), 
        max_axel_height.in(Meters), 
        false, 
        min_axel_height.in(Meters));

    private final Mechanism2d m_mech2d =
        new Mechanism2d(6, max_axel_height.in(Meters));
    private final MechanismRoot2d m_mech2dRoot =
        m_mech2d.getRoot("Elevator Root", 3, 0.0);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90, 6, new Color8Bit(Color.kRed))
        );

    private final TalonFXConfiguration m_elevator_config = new TalonFXConfiguration();

    public Elevator(){
        
        
        
        // motor configuration section
        

        
        m_elevator_config.Slot0.kS = k_default_ks;
        m_elevator_config.Slot0.kP = k_default_kp; // An error of 1 rotation results in 60 A output
        m_elevator_config.Slot0.kI = k_default_ki; // No output for integrated error
        m_elevator_config.Slot0.kD = k_default_kd; // A velocity of 1 rps results in 6 A output
        m_elevator_config.Slot0.kG = k_default_kg;
        m_elevator_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        // Peak output of 20 A
        m_elevator_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(k_current_limit))
        .withPeakReverseTorqueCurrent(Amps.of(-k_current_limit));
        CurrentLimitsConfigs elecurent = new CurrentLimitsConfigs().withStatorCurrentLimit(k_current_limit).withSupplyCurrentLimit(k_current_limit);



        //for motion magic controls
        m_elevator_config.MotionMagic.MotionMagicCruiseVelocity = k_default_cVelocity;
        m_elevator_config.MotionMagic.MotionMagicAcceleration = k_default_Acceleration;
        m_elevator_config.MotionMagic.MotionMagicJerk = k_default_jerk;
        m_elevator_config.MotionMagic.MotionMagicExpo_kV = k_default_kV;
        m_elevator_config.MotionMagic.MotionMagicExpo_kA = k_default_kA;


        // bind the remote encoder to the mount motor

        CANcoderConfiguration m_elevator_CANcoder_config = new CANcoderConfiguration();
        m_elevator_CANcoder_config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.85));
        m_elevator_CANcoder_config.MagnetSensor.withMagnetOffset(Rotations.of(-0.045166));
        m_elevator_CANcoder_config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
       
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_elevator_CANcoder.getConfigurator().apply(m_elevator_CANcoder_config);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }


        
        m_elevator_config.Feedback.FeedbackRemoteSensorID = m_elevator_CANcoder.getDeviceID();
        m_elevator_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        m_elevator_config.Feedback.SensorToMechanismRatio = 1.0;
        m_elevator_config.Feedback.RotorToSensorRatio = 11.71*5.5;
        m_elevator_config.Feedback.RotorToSensorRatio = 11.71*5.5;
        
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_elevator_motor.getConfigurator().apply(m_elevator_config);
        m_elevator_motor.getConfigurator().apply(elecurent);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }

        m_elevator_motor_follower.setControl(new Follower(m_elevator_motor.getDeviceID(), false));

        // m_elevator_motor.setPosition(0);

        SmartDashboard.putData("Elevator Sim", m_mech2d);

        SmartDashboard.putNumber("Elevator kP", k_default_kp); 
        SmartDashboard.putNumber("Elevator kI", k_default_ki);
        SmartDashboard.putNumber("Elevator kD", k_default_kd);
        SmartDashboard.putNumber("Elevator kG", k_default_kg);
        SmartDashboard.putNumber("Elevator kff", k_default_kff);
       
        
        SmartDashboard.putNumber("Elevator Current Limit", k_current_limit);
       
        SmartDashboard.putNumber("Elevator Cruise Velocity", k_default_cVelocity);
        SmartDashboard.putNumber("Elevator kV", k_default_kV);
        SmartDashboard.putNumber("Elevator kA", k_default_kA);

        SmartDashboard.putData("Update Elevator PID", new InstantCommand(this::configure_from_dash));

        register();
        

    }

    private Angle validate_and_convert(Distance claw_height){

        // Takes in a value of Distance representing the claws center axel height from the groung
        // Validates the input to make sure that the hieght is within range 
        // Converts the axel hieght to travel distance and then to the rotaition positon of the motor/sensor
        // Then validate the rotaions to makes sure the motor is not set to an out of range position incase the calculations are bugged


        return Rotations.of(10);
      


    }
    
    public boolean is_stowed(){

        return true;
        // return m_elevator_CANcoder.getAbsolutePosition().getValue().lt(Rotation.of(0.2));
    }

        
        

    
    private void set_elevator_position(Distance posision){

        // gt is greater than 
        if (posision.gt( k_max_Distance)){
         
            // logger.log(position + " requested is greater than the max position ");
            posision = k_max_Distance;

        }
        else if (posision.lt(k_min_Distance)){

            //logger.log(position + " requested is less than the minimum position");
            posision = k_min_Distance;

        }

        //calucaulate the conversion from meters to rotations
        // posision = posision.times(posision.baseUnitMagnitude());
        double ratio = (posision.minus(k_min_Distance)).div(k_max_Distance.minus(k_min_Distance)).baseUnitMagnitude();
    
        Angle output = k_elevator_max_rot.minus(k_elevator_min_rot).times(ratio).plus(k_elevator_min_rot);

        output = output.gt(k_elevator_max_rot) ? output : k_elevator_max_rot; 


        m_elevator_motor.setControl(m_elevator_motorOut.withPosition(output.in(Rotations)));
        

    }

    private void set_elevator_position_mm(Angle posision){

        // // gt is greater than 
        // if (posision.gt( k_max_Distance)){
         
        //     // logger.log(position + " requested is greater than the max position ");
        //     posision = k_max_Distance;

        // }
        // else if (posision.lt(k_min_Distance)){

        //     //logger.log(position + " requested is less than the minimum position");
        //     posision = k_min_Distance;

        // }

        // //calucaulate the conversion from meters to rotations
        // double ratio = (posision.minus(k_min_Distance)).div(k_max_Distance.minus(k_min_Distance)).baseUnitMagnitude();
    
        // Angle output = k_elevator_max_rot.minus(k_elevator_min_rot).times(ratio).plus(k_elevator_min_rot);

        // output = output.gt(k_elevator_max_rot) ? output : k_elevator_max_rot; 
        double ff_factor = posision.div(k_elevator_max_rot).magnitude()+k_default_kff_offset;

        Current ffCurrent = Amps.of(k_default_kff).times(ff_factor);

        System.out.println("position set "+ posision.in(Rotation) );

        m_elevator_motor.setControl(m_elevator_motorOut_mm.withPosition(posision.in(Rotations)).withFeedForward(ffCurrent));
        

    }



    public BooleanSupplier at_position(double tolerance){
        
        BooleanSupplier position_trigger = ()-> Math.abs(m_elevator_motor.getClosedLoopError().getValueAsDouble())<tolerance;
        
        return position_trigger;
    }

    public BooleanSupplier at_position(){

        return at_position(0.005);
    }
        

    // public Command set_position_command(Distance position ){
    //     return run(()->set_elevator_position_mm(position));
        
    // }

    public Command set_position_command_angle(Angle position ){
        return run(()->set_elevator_position_mm(position));
        
    }

    @Override
    public void simulationPeriodic() {
        
        m_elevatorSim.setInput(m_elevator_motorSim.getMotorVoltage());

        m_elevatorSim.update(0.020);

        
        var elevatorVelocity = 
            (m_elevatorSim.getVelocityMetersPerSecond()/k_windlass_radius.in(Meter));

        m_elevator_motorSim.setRawRotorPosition(m_elevatorSim.getPositionMeters());
        m_elevator_motorSim.setRotorVelocity(elevatorVelocity);

        m_elevatorMech2d.setLength(m_elevatorSim.getPositionMeters());
    }

    @Override
    public void periodic() {
        super.periodic();
        publish_data();
    }

    public void publish_data(){

        // put data important for charaterizing the data to the smart dashboard
        double set_point = m_elevator_motor.getClosedLoopReference().getValueAsDouble();
        double error = m_elevator_motor.getClosedLoopError().getValueAsDouble();
        double tcurrent = m_elevator_motor.getTorqueCurrent().getValueAsDouble();
        double velocity = m_elevator_motor.getVelocity().getValueAsDouble();
        double acceleration = m_elevator_motor.getAcceleration().getValueAsDouble();
        double position = m_elevator_motor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Elevator Set Point", set_point);
        SmartDashboard.putNumber("Elevator Error", error);
        SmartDashboard.putNumber("Elevator Torque Current", tcurrent);
        SmartDashboard.putNumber("Elevator Velocity", velocity);
        SmartDashboard.putNumber("Elevator Acceleration", acceleration);
        SmartDashboard.putNumber("Elevator Position", position);
        SmartDashboard.putData("Elevator Sim", m_mech2d);


        // SmartDashboard.putData("PID_Verification", m_elevator_motor.getClosedLoopSlot()
        
    }

    public void configure_from_dash(){
        // configure the motor from the smart dashboard
        m_elevator_config.Slot0.kP = SmartDashboard.getNumber("Elevator kP", k_default_kp); 
        m_elevator_config.Slot0.kI = SmartDashboard.getNumber("Elevator kI", k_default_ki);
        m_elevator_config.Slot0.kD = SmartDashboard.getNumber("Elevator kD", k_default_kd);
        m_elevator_config.Slot0.kG = SmartDashboard.getNumber("Elevator kG", k_default_kg);
        k_default_kff = SmartDashboard.getNumber("Elevator kFF", k_default_kff);
       
        
        m_elevator_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(SmartDashboard.getNumber("Elevator Current Limit", k_current_limit)))
        .withPeakReverseTorqueCurrent(Amps.of(-SmartDashboard.getNumber("Elevator Current Limit", k_current_limit)));

        m_elevator_config.MotionMagic.MotionMagicCruiseVelocity = SmartDashboard.getNumber("Elevator Cruise Velocity", k_default_cVelocity);
        m_elevator_config.MotionMagic.MotionMagicExpo_kV = SmartDashboard.getNumber("Elevator kV", k_default_kV);
        m_elevator_config.MotionMagic.MotionMagicExpo_kA = SmartDashboard.getNumber("Elevator kA", k_default_kA);


        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = m_elevator_motor.getConfigurator().apply(m_elevator_config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        
        System.out.println("pid Updated");


    }
    

}

