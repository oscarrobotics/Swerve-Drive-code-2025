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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.function.Supplier;

import javax.print.attribute.standard.JobHoldUntil;

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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.util.Constants.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class Eleclaw{

    private Elevator elevator;
    private Claw claw;

    private CommandSwerveDrivetrain drivetrain;

    private Intake intake;

    private CommandXboxController operator;
  


    private double height_adjust = 0;
    private double angle_adust = 0;


    HashMap<String, Waypoint> named_waypoints; 



    public Eleclaw(Elevator elevator, Claw claw,Intake intake, CommandSwerveDrivetrain drivetrain, CommandXboxController operator ){
        
        this.elevator = elevator;
        this.claw = claw;
        this.intake = intake;
        this.drivetrain = drivetrain; 
        this.operator = operator;
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
///
   
    

    public void load_linked_waypoint(){

        //create list to hold all paths
        //find all paths
        //
        //
        //load all paths


    }


    public Command retry_intake_coral(Time timeout){
        
        

        return intake.continuous_intake().until(intake.has_coral()).withTimeout(timeout);

    }

    public Command retry_intake_coral(){
        
        Time timeout = Seconds.of(13);
        return intake.continuous_intake().until(intake.has_coral()).withTimeout(timeout);

    }


    public Command score_coral_1(){

        

        return Commands.parallel(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_1).withTimeout(2),
        claw.set_position_command_mm(k_claw.k_coral_position_1).withTimeout(2)


        )
        ;
        
    }


    public Command position_coral_2_old(){

        return Commands.parallel(new RepeatCommand( elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_2)),
        new RepeatCommand(claw.set_position_command_mm(claw.k_coral_position_mid)), 
        new WaitCommand(100).until(()->operator.leftBumper().getAsBoolean()).andThen(intake.continuous_outake()));

       
        // return Commands.parallel(new RepeatCommand(run(()-> elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_2.plus(H_adj())))),
        // new RepeatCommand(run(()-> claw.set_position_command_mm(claw.k_coral_position_mid.plus(A_adj())))), new WaitCommand(100).until(()->operator.leftBumper().getAsBoolean()).andThen(intake.continuous_outake()));
       
        // return Commands.parallel(new RepeatCommand(run(()-> elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_2.plus(H_adj())))),
        // new RepeatCommand(run(()-> claw.set_position_command_mm(claw.k_coral_position_mid.plus(A_adj())))), 
        // new RepeatCommand(intake.continuous_outake().onlyWhile(()->operator.leftBumper().getAsBoolean()))
        // );


       
        
    }
    
    


       
        
    

    public Command position_coral_3_old(){

        return Commands.parallel(new RepeatCommand(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_3)),
        new RepeatCommand( claw.set_position_command_mm(claw.k_coral_position_mid)),new WaitCommand(100).until(()->operator.leftBumper().getAsBoolean()).andThen(intake.continuous_outake()));

        
        // return Commands.parallel(new RepeatCommand(run(()-> elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_3.plus(H_adj())))),
        // new RepeatCommand(run(()-> claw.set_position_command_mm(claw.k_coral_position_mid.plus(A_adj())))),
        //  new RepeatCommand(intake.continuous_outake().onlyWhile(()->operator.leftBumper().getAsBoolean()))
        //  );
        
    }
    public Command position_coral_4_old(){

        return Commands.parallel(new RepeatCommand(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_4)),
      new WaitCommand(1).andThen( new WaitCommand(10)).until(elevator.at_position(0.05)).andThen(new RepeatCommand(claw.set_position_command_mm(k_claw.k_coral_position_high))),
      new WaitCommand(100).until(()->operator.leftBumper().getAsBoolean()).andThen(intake.continuous_outake())

      );

        // return Commands.parallel(new RepeatCommand(run(()-> elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_4.plus(H_adj())))),
        // new RepeatCommand(run(()-> claw.set_position_command_mm(claw.k_coral_position_high.plus(A_adj())))), 
        // new RepeatCommand(intake.continuous_outake().onlyWhile(()->operator.leftBumper().getAsBoolean()))
        // );

    }
    public Command lower_alge_old(){
        return Commands.parallel(new RepeatCommand(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_2.plus(H_adj()))),
        new RepeatCommand(claw.set_position_command_mm(claw.k_coral_position_get_alge.plus(A_adj()))),new WaitCommand(100).until(()->operator.leftBumper().getAsBoolean()).andThen(intake.intake_alge_command())
        );

    }
    public Command upper_alge_old(){
        return Commands.parallel(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_3),
        claw.set_position_command_mm(claw.k_coral_position_get_alge),new WaitCommand(100).until(()->operator.leftBumper().getAsBoolean()).andThen(intake.intake_alge_command())
        );

    }

    public Command score_alge_old(){
        return Commands.parallel(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_4),
        claw.set_position_command_mm(claw.k_coral_position_shoot_alge),new WaitCommand(100).until(()->operator.leftBumper().getAsBoolean()).andThen(intake.outtake_alge_command())
        );

    }
       

    public Command position_load_old(){
        return Commands.parallel(elevator.set_position_command_angle(elevator.k_load),
        claw.set_position_command_mm(claw.k_load));

        // return Commands.parallel(new RepeatCommand(run(()->elevator.set_position_command_angle(elevator.k_load.plus(H_adj())))),
        // new RepeatCommand(run(()-> claw.set_position_command_mm(claw.k_load.plus(A_adj()))))
        // );
      
    }

    public void position_coral_1(){
        // System.out.println("pcoral2");
        CommandScheduler.getInstance().schedule( elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_1.plus(H_adj())));
        if (elevator.at_position(0.05).getAsBoolean()){
        CommandScheduler.getInstance().schedule(claw.set_position_command_mm(claw.k_coral_position_mid.plus(A_adj())));
        
        }
        intake.doing_coral();
        // if (operator.leftBumper().getAsBoolean() ){
        
        //     CommandScheduler.getInstance().schedule( intake.continuous_outake());
        // }
        // else if (operator.rightBumper().getAsBoolean()){
        //     CommandScheduler.getInstance().schedule(  intake.continuous_intake());
        // }
        // else {
        //     CommandScheduler.getInstance().cancel(intake.continuous_outake());
        //     CommandScheduler.getInstance().cancel(intake.continuous_intake());
        // }
    }


    public void position_coral_2(){
        // System.out.println("pcoral2");
        CommandScheduler.getInstance().schedule( elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_2.plus(H_adj())));
        
        if (elevator.at_position(0.05).getAsBoolean()){
        CommandScheduler.getInstance().schedule(claw.set_position_command_mm(k_claw.k_coral_position_mid.plus(A_adj())));
        
        }
        intake.doing_coral();
        // if (operator.leftBumper().getAsBoolean() ){
        
        //     CommandScheduler.getInstance().schedule( intake.continuous_outake());
        // }
        // else if (operator.rightBumper().getAsBoolean()){
        //     CommandScheduler.getInstance().schedule(  intake.continuous_intake());
        // }
        // else {
        //     CommandScheduler.getInstance().cancel(intake.continuous_outake());
        //     CommandScheduler.getInstance().cancel(intake.continuous_intake());
        // }
    }
    public void position_coral_3(){
        // System.out.println("pcoral3");
        CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_3.plus(H_adj())));
        if (elevator.at_position(0.05).getAsBoolean()){
        CommandScheduler.getInstance().schedule(claw.set_position_command_mm(claw.k_coral_position_mid.plus(A_adj())));
        
        }
        intake.doing_coral();
        // if (operator.leftBumper().getAsBoolean() ){
        
        //     CommandScheduler.getInstance().schedule( intake.continuous_outake());
        // }
        // else if (operator.rightBumper().getAsBoolean()){
        //     CommandScheduler.getInstance().schedule(  intake.continuous_intake());
        // }
        // else {
        //     CommandScheduler.getInstance().cancel(intake.continuous_outake());
        //     CommandScheduler.getInstance().cancel(intake.continuous_intake());
        // }

    }
    public void position_coral_4(){
        // System.out.println("pcoral4");
        CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_4.plus(H_adj())));
        if (elevator.at_position(0.05).getAsBoolean()){
        CommandScheduler.getInstance().schedule(claw.set_position_command_mm(k_claw.k_coral_position_high.plus(A_adj())));
        
        }
        intake.doing_coral();
        // if (operator.leftBumper().getAsBoolean() ){
        
        //     CommandScheduler.getInstance().schedule( intake.continuous_outake());
        // }
        // else if (operator.rightBumper().getAsBoolean()){
        //     CommandScheduler.getInstance().schedule(  intake.continuous_intake());
        // }
        // else {
        //     CommandScheduler.getInstance().cancel(intake.continuous_outake());
        //     CommandScheduler.getInstance().cancel(intake.continuous_intake());
        // }
    }

    public void pre_position_4(){
        CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_4.plus(H_adj())));
        if (elevator.at_position(0.05).getAsBoolean()){
        CommandScheduler.getInstance().schedule(claw.set_position_command_mm(k_claw.k_coral_position_high.plus(A_adj())));
        
        }
        intake.doing_coral();

    }
    public void position_load(){
      


        CommandScheduler.getInstance().schedule(  elevator.set_position_command_angle(elevator.k_load.plus(H_adj())));
        if (elevator.at_position(0.05).getAsBoolean()){
        CommandScheduler.getInstance().schedule(claw.set_position_command_mm(k_claw.k_load.plus(A_adj())));
        
        }
        intake.doing_coral();
        // if (operator.rightBumper().getAsBoolean() && intake.getCurrentCommand()==null){
        //     CommandScheduler.getInstance().schedule(  intake.continuous_intake());
        // }
        // else if (operator.leftBumper().getAsBoolean()&& intake.getCurrentCommand()==null ){
        
        //     CommandScheduler.getInstance().schedule( intake.continuous_outake());
        // }
        // else {
        //     CommandScheduler.getInstance().cancel(intake.continuous_outake());
        //     CommandScheduler.getInstance().cancel(intake.continuous_intake());
        // }

    }



    public void lower_alge(){
        // System.out.println("lower alge");
        CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_2.plus(H_adj())));
        if (elevator.at_position(0.05).getAsBoolean()){
        CommandScheduler.getInstance().schedule(claw.set_position_command_mm(claw.k_coral_position_get_alge.plus(A_adj())));
        
        }
        intake.doing_alge();
        // if (operator.leftBumper().getAsBoolean() ){
        
        //     CommandScheduler.getInstance().schedule( intake.outtake_alge_command());
        // }
        // else if (operator.rightBumper().getAsBoolean()){
        //     CommandScheduler.getInstance().schedule(  intake.intake_alge_command());
        // }
        // else {
        //     CommandScheduler.getInstance().cancel(intake.outtake_alge_command());
        //     CommandScheduler.getInstance().cancel(intake.intake_alge_command());
        // }
    }

    public void upper_alge(){
        CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_3.plus(H_adj())));
        if (elevator.at_position(0.05).getAsBoolean()){
        CommandScheduler.getInstance().schedule(claw.set_position_command_mm(claw.k_coral_position_get_alge.plus(A_adj())));
        
        }
        intake.doing_alge();
        // if (operator.leftBumper().getAsBoolean() ){
        
        //     CommandScheduler.getInstance().schedule( intake.outtake_alge_command());
        // }
        // else if (operator.rightBumper().getAsBoolean()){
        //     CommandScheduler.getInstance().schedule(  intake.intake_alge_command());
        // }
        // else {
        //     CommandScheduler.getInstance().cancel(intake.outtake_alge_command());
        //     CommandScheduler.getInstance().cancel(intake.intake_alge_command());
        // }

    }

    public void score_alge(){
        CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_4.plus(H_adj())));
        if (elevator.at_position(0.05).getAsBoolean()){
        CommandScheduler.getInstance().schedule(claw.set_position_command_mm(claw.k_coral_position_shoot_alge.plus(A_adj())));
        
        }
        intake.doing_alge();
        // if (operator.leftBumper().getAsBoolean() ){
        
        //     CommandScheduler.getInstance().schedule( intake.outtake_alge_command());
        // }
        // else if (operator.rightBumper().getAsBoolean()){
        //     CommandScheduler.getInstance().schedule(  intake.intake_alge_command());
        // }
        // else {
        //     CommandScheduler.getInstance().cancel(intake.outtake_alge_command());
        //     CommandScheduler.getInstance().cancel(intake.intake_alge_command());
        // }

    }

    // public void stow_alge(){
    //     CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_2.plus(H_adj())));
    //     CommandScheduler.getInstance().schedule(claw.set_position_command_mm(k_claw.k_alge_stowed.plus(A_adj())));
    //     intake.doing_alge();
    // }
        
    public void stow(){
       
        if(intake.coraling().getAsBoolean()){
        
            CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(k_elevator.k_stowed));
            CommandScheduler.getInstance().schedule(claw.set_position_command_mm(k_claw.k_stowed));
        }
        else{
            CommandScheduler.getInstance().schedule(claw.set_position_command_mm(k_claw.k_alge_stowed));
            if (claw.at_position(0.05).getAsBoolean()){
                CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(k_elevator.k_stowed));
            
            }
        }   
        
    }

    public void active_stow(){
        
            CommandScheduler.getInstance().schedule(claw.set_position_command_mm(k_claw.k_alge_stowed.plus(A_adj().times(1.5).plus(Rotation.of(0.03)))));
            if (claw.at_position(0.05).getAsBoolean()){
                CommandScheduler.getInstance().schedule(elevator.set_position_command_angle(k_elevator.k_stowed));
            
            }
        

    }
        
    




    private Angle H_adj(){

        double left = operator.getLeftY();

        double right = operator.getRightY();

        double value = Math.abs(right)>Math.abs(left)?right:left;

        value = Math.abs(value)>0.09? value:0;

        value = value/6;

        return Rotation.of(-value);


    }

    private Angle A_adj(){

        double left = operator.getLeftX();

        double right = operator.getRightX();

        double value = Math.abs(right)>Math.abs(left)?right:left;

        value = Math.abs(value)>0.09? value:0;

        value = value/6;
        
        return Rotation.of(-value);


    }


    

}