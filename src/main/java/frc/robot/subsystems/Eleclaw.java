package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Eleclaw extends SubsystemBase{

    private Elevator elevator;
    private Claw claw;
    private CommandSwerveDrivetrain drivetrain;


    public Eleclaw(Elevator elevator, Claw claw, CommandSwerveDrivetrain drivetrain){
        
        this.elevator = elevator;
        this.claw = claw;
        this.drivetrain = drivetrain; 
    }


    // public Command score_alge_1(){

    //     return run();
    // }
//                             11    0
//                           ___________
//                      10  /           \  1
//                     9   /             \  2     
//                        /               \
//                        \               /  
//                      8  \             /  3
//                        7 \___________/  4
///                            6    5          
///                 
/// 
///
///            Left                               Right
///           Station                            Station
/// 
/// 
/// 
    public Command score_coral_1(){

        
        return Commands.parallel(run(()->elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_1)).until(elevator.at_position()),
        run(()->claw.set_position_command_mm(claw.k_coral_position_1)).until(claw.at_position())
        ).andThen(
            claw.outtake_coral_command()
        )
        ;
        
    }




}