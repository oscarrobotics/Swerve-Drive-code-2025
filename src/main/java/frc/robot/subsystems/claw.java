package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class claw extends SubsystemBase{
     // All hardware classes already have WPILib integration
     final TalonFX m_intake = new TalonFX(31);
     final TalonFX m_mount = new TalonFX(32);
     
 
     final TalonFXSimState m_intakeSim = m_intake.getSimState();
     final TalonFXSimState m_mountSim = m_mount.getSimState();
 
     final DutyCycleOut m_intakeFXOut = new DutyCycleOut(0);
     final DutyCycleOut m_mountFXOut = new DutyCycleOut(0);
 
     final TalonFXConfiguration m_intakeConfig = new TalonFXConfiguration();
     final TalonFXConfiguration m_mountConfig = new TalonFXConfiguration();
     


     
     public claw(){

        m_talonFX.setControl(m_talonFXOut);
        // motor configuartion section

     }
 

    
}
