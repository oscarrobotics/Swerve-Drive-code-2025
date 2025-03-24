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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Map;
import java.util.function.BooleanSupplier;

import frc.robot.util.Constants.*;


public class Climber extends SubsystemBase {

    final TalonFX m_climber = new TalonFX(5);
    //motor request
    private final NeutralOut m_brake = new NeutralOut();


    final TalonFXSimState m_climberSim = m_climber.getSimState();


    final VelocityTorqueCurrentFOC m_climberFXOut_v_mm = new VelocityTorqueCurrentFOC(0);
    final MotionMagicExpoTorqueCurrentFOC m_climberFXOut_mm = new MotionMagicExpoTorqueCurrentFOC(0);


 
    


    TalonFXConfiguration  m_climber_config = new TalonFXConfiguration();

    // change values, imported from claw.java and intake.java
    // values for the climber motor controller
    
    
  

    // private final double k_default_intake_accel = 1;
    // private final double k_default_intake_jerk = 1; 

    // // mm_expo gains
    // private final double k_default_climber_kV = 10;
    // private final double k_default_climber_kA = 3;
    // private final double k_default_climber_cVelocity = 0.4; // used for both mm and mm_expo
    
   

    
    // shuffleboard entries for tuning
    private ShuffleboardTab  m_climber_tab = Shuffleboard.getTab("Climber Tuning");

    //moving motor 

    
        



    public Climber(){

        // configure motor controller
        
        
        //deploy velocity slot 0
        m_climber_config.Slot0.kS = k_climber.k_deploy_velocity_ks;
        m_climber_config.Slot0.kP = k_climber.k_deploy_velocity_kp;
        m_climber_config.Slot0.kI = k_climber.k_deploy_velocity_ki;
        m_climber_config.Slot0.kD = k_climber.k_deploy_velocity_kd;
        //climb velocity slot 1
        m_climber_config.Slot1.kS = k_climber.k_climb_velocity_ks;
        m_climber_config.Slot1.kP = k_climber.k_climb_velocity_kp;
        m_climber_config.Slot1.kI = k_climber.k_climb_velocity_ki;
        m_climber_config.Slot1.kD = k_climber.k_climb_velocity_kd;
        //deploy position slot 2
        m_climber_config.Slot2.kP = k_climber.k_deploy_position_kp;
        m_climber_config.Slot2.kI = k_climber.k_deploy_position_ki;
        m_climber_config.Slot2.kD = k_climber.k_deploy_position_kd;
        m_climber_config.Slot2.kG = k_climber.k_deploy_position_kg;
        //climb position slot 3
        // m_climber_config.Slot3.kP = k_climber.k_climb_position_kp;
        // m_climber_config.Slot3.kI = k_climber.k_climb_position_ki;
        // m_climber_config.Slot3.kD = k_climber.k_climb_position_kd;
        // m_climber_config.Slot3.kG = k_climber.k_climb_position_kg;

        m_climber_config.MotionMagic.MotionMagicExpo_kA = k_climber.kA;
        m_climber_config.MotionMagic.MotionMagicExpo_kV = k_climber.kV;
        m_climber_config.MotionMagic.MotionMagicCruiseVelocity = k_climber.deploy_cruse_velocity.in(RotationsPerSecond);

        // Peak output of 5 A
        m_climber_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(k_climber.climb_current_limit))
        .withPeakReverseTorqueCurrent(Amps.of(-k_climber.climb_current_limit));

        m_climber_config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        m_climber_config.MotorOutput.NeutralMode=NeutralModeValue.Brake;



        // m_fx.getConfigurator().apply(fx_cfg);
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = m_climber.getConfigurator().apply(m_climber_config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        
        m_climber.setPosition(0);

        // register();

    }

    private void set_climber_speed(AngularVelocity speed){
        m_climber.setControl(m_climberFXOut_v_mm.withVelocity(speed).withSlot(0));
    
    }







    public void stop_climber(){
        m_climber.setControl(m_brake);
    }
    


    // command to run the climber motor continously (need to change value 300)
    public Command climb_command(){
        return run(() -> set_climber_speed(AngularVelocity.ofBaseUnits(60, RPM)));
    }


    public Command stop(){
        return run(this::stop_climber); 


    }

    public Command deploy_climber(){
        return run(() -> m_climber.setControl(
            m_climberFXOut_mm.withPosition(
                k_climber.k_deploy_position)
                .withSlot(2)));
    }

    public Command reset_climber(){
        return run(() -> set_climber_speed(AngularVelocity.ofBaseUnits(-60, RPM))); 
    }



    

    
    
    
   
    }
