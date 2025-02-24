package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.databind.deser.impl.FailingDeserializer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

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

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Elevator extends SubsystemBase{
    // All hardware classes already have WPILib integration
    final TalonFX m_elevator_motor = new TalonFX(21);

    final CANcoder m_elevator_CANcoder = new CANcoder(26);
    

    final TalonFXSimState m_elevator_motorSim = m_elevator_motor.getSimState();

    final PositionTorqueCurrentFOC m_elevator_motorOut = new PositionTorqueCurrentFOC(0);

    final MotionMagicExpoTorqueCurrentFOC m_elevator_motorOut_mm = new MotionMagicExpoTorqueCurrentFOC(0);


    private final NeutralOut m_brake = new NeutralOut();

    final TalonFXConfiguration m_elevator_motorConfig = new TalonFXConfiguration();

    //if external cancoder isnt used
    public final double k_elevator_rotations = 4.7;
    public final double k_elevator_ratio = k_elevator_rotations*1;

    public final Angle k_elevator_min_rot = Rotations.of(0);
    public final Angle k_elevator_max_rot = Rotations.of(4.7);

    public final Distance k_min_Distance =  Meters.of(0);
    public final Distance k_max_Distance = Meter.of(3);
    public final Distance k_stowed =  Meters.of(0);



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



    public Elevator(){
        
        
        
        // motor configuration section
        

        TalonFXConfiguration  m_elevator_config = new TalonFXConfiguration();
       
        m_elevator_config.Slot0.kP = 60; // An error of 1 rotation results in 60 A output
        m_elevator_config.Slot0.kI = 0; // No output for integrated error
        m_elevator_config.Slot0.kD = 6; // A velocity of 1 rps results in 6 A output
        m_elevator_config.Slot0.kG = 0.1;
        m_elevator_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        // Peak output of 20 A
        m_elevator_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(5))
        .withPeakReverseTorqueCurrent(Amps.of(-5));

        //for motion magic controls
        m_elevator_config.MotionMagic.MotionMagicCruiseVelocity = 10;
        m_elevator_config.MotionMagic.MotionMagicExpo_kV = 1;
        m_elevator_config.MotionMagic.MotionMagicExpo_kA = 1;

        
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_elevator_motor.getConfigurator().apply(m_elevator_motorConfig);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }

        m_elevator_motor.setPosition(0);

        SmartDashboard.putData("Elevator Sim", m_mech2d);


    }

    private Angle validate_and_convert(Distance claw_heigt){

        // Takes in a value of Distance representing the claws center axel height from the groung
        // Validates the input to make sure that the hieght is within range 
        // Converts the axel hieght to travel distance and then to the rotaition positon of the motor/sensor
        // Then validate the rotaions to makes sure the motor is not set to an out of range position incase the calculations are bugged


        return Rotations.of(10);
      


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
        double ratio = (posision.minus(k_min_Distance)).div(k_max_Distance.minus(k_min_Distance)).baseUnitMagnitude();
    
        Angle output = k_elevator_max_rot.minus(k_elevator_min_rot).times(ratio).plus(k_elevator_min_rot);

        output = output.gt(k_elevator_max_rot) ? output : k_elevator_max_rot; 


        m_elevator_motor.setControl(m_elevator_motorOut.withPosition(output.in(Rotations)));
        

    }

    private void set_elevator_position_mm(Distance posision){

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
        double ratio = (posision.minus(k_min_Distance)).div(k_max_Distance.minus(k_min_Distance)).baseUnitMagnitude();
    
        Angle output = k_elevator_max_rot.minus(k_elevator_min_rot).times(ratio).plus(k_elevator_min_rot);

        output = output.gt(k_elevator_max_rot) ? output : k_elevator_max_rot; 


        m_elevator_motor.setControl(m_elevator_motorOut_mm.withPosition(output.in(Rotations)));
        

    }

    public Command set_position_command(Distance position ){
        return run(()->set_elevator_position(position));
        
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
}

