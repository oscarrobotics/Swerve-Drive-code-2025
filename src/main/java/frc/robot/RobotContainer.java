// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Eleclaw;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.VisionSubsystem;




import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
    
    




    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.07).withRotationalDeadband(MaxAngularRate * 0.07) // Add a 10% deadband
            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors/
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.RobotCentric strafe = new SwerveRequest.RobotCentric();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController drivestick = new CommandXboxController(0);
    private final CommandXboxController controlstick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final Elevator elevator = new Elevator();
    public final Claw claw = new Claw();
    public final Intake intake = new Intake();
    public final Lighting lighting = new Lighting();

    public final Climber climber = new Climber();
    public final VisionSubsystem vision = null ;

    public Eleclaw eleclaw;


    //Used to keep track of which direction the controller should drive the robot
    //in the condes curret state the orentation of the robot to the field can be messed up 
    //by certain parts of the executrion of autonomous, and the robot will drive opposite of 
    //what the driver expects, as a "temporary" fix, this will allow the driver to flip the direction 
    //of the axis to correct for this if needed, should not be needed after vision is implemented
    // but may still be kept just in case
    double forward_dir = -1;
    double side_dir = -1;


    double xfilter = 0;
    double yfilter = 0;
    double twistfilter = 0;

    final double k_xfilt_positive = 0.1;
    final double k_yfilt_positive = 0.1;
    final double k_tfilt_positive = 0.1;
    final double k_xfilt_negative = 0.5;
    final double k_yfilt_negative = 0.5;
    final double k_tfilt_negative = 1;

    // public final Eleclaw eleclaw = new Eleclaw(elevator, claw);



    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {

        eleclaw = new Eleclaw(elevator, claw, intake, drivetrain, controlstick);

        name_commands();
        autoChooser = AutoBuilder.buildAutoChooser("tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

       
        // vision = new VisionSubsystem(drivetrain);
        //code used for both intake and outake of coral from the claw during auto and teleop 

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        

        //sets the drive train to use the joystick for control by default action eveyloop
        //the speed control based on evelator position is also implemented here as well
        //as the driver orientation flip apllication

        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(joystick.getLeftY()* MaxSpeed*forward_dir *  ((elevator.is_stowed()&& !joystick.start().getAsBoolean()) ? (joystick.back().getAsBoolean() ? 1:0.7):0.3)) // Drive forward with negative Y (forward)
        //             .withVelocityY(joystick.getLeftX() * MaxSpeed *side_dir* ((elevator.is_stowed()&& !joystick.start().getAsBoolean()) ? (joystick.back().getAsBoolean() ? 1:0.7):0.3)) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );


        // joystick.rightStick().onTrue(new InstantCommand(()->
        //     drivetrain.setDefaultCommand(   
        //         // Drivetrain will execute this command periodically
        //         drivetrain.applyRequest(() ->
        //             drive.withVelocityX(drivestick.getLeftY()* MaxSpeed*forward_dir *  ((elevator.is_stowed()&& !drivestick.start().getAsBoolean()) ? (drivestick.back().getAsBoolean() ? 1:0.7):0.3)) // Drive forward with negative Y (forward)
        //                 .withVelocityY(drivestick.getLeftX() * MaxSpeed *side_dir* ((elevator.is_stowed()&& !drivestick.start().getAsBoolean()) ? (drivestick.back().getAsBoolean() ? 1:0.7):0.3)) // Drive left with negative X (left)
        //                 .withRotationalRate(-drivestick.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        //         );
            
        // // );
        // joystick.leftStick().onTrue(new InstantCommand(()->
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(()->smooth_drive())
                );
            // )
        // );
        

        // joystick.leftStick().toggleOnTrue( 
            // drivetrain.applyRequest(
            //     smooth_drive()
               
    
            // );
            
        // );

        // gives the driver the ability to strafe the robot in a robot centric manner to assit with lining up with field elements
        // may need to implement a way to adjust the speed of this to allow for more precise control

       
        drivestick.povUp().whileTrue(drivetrain.applyRequest(()->strafe.withVelocityX(MaxSpeed*(elevator.is_stowed()&& !drivestick.start().getAsBoolean() ? 0.3:0.1)).withVelocityY(0)));
        drivestick.povLeft().whileTrue(drivetrain.applyRequest(()->strafe.withVelocityY(MaxSpeed*(elevator.is_stowed()&& !drivestick.start().getAsBoolean() ? 0.3:0.1)).withVelocityX(0)));
        drivestick.povDown().whileTrue(drivetrain.applyRequest(()->strafe.withVelocityX(-MaxSpeed*(elevator.is_stowed()&& !drivestick.start().getAsBoolean() ? 0.3:0.1)).withVelocityY(0)));
        drivestick.povRight().whileTrue(drivetrain.applyRequest(()->strafe.withVelocityY(-MaxSpeed*(elevator.is_stowed()&& !drivestick.start().getAsBoolean() ? 0.3:0.1)).withVelocityX(0)));
        // joystick.rightTrigger().

        
        // toggles the values of the forward and side direction variables that control the direction of the robot
        drivestick.x().onTrue(new InstantCommand(()->flip_for()));
        drivestick.y().onTrue(new InstantCommand(()->flip_side()));
        

        // todo potentially
        // joystick.povDownLeft(). 
        // joystick.povDownRight().
        //  etc....
        

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(joystick.getLeftY(), joystick.getLeftX()))
        // ));


        
     
       


        // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        drivestick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivestick.rightTrigger().onTrue(climber.climb_command()).onFalse(climber.stop());
        drivestick.rightBumper().whileTrue(intake.continuous_outake());

        drivetrain.registerTelemetry(logger::telemeterize);

    //    controlstick.y().onTrue(elevator.get_posiCommand(25.0));
        // controlstick.leftBumper().onTrue(claw.position_command(claw.k_load_coral_position));
        //binds buttons to elevator position commands
        // controlstick.a().whileTrue(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_1));
        // controlstick.b().whileTrue(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_2));
        // controlstick.x().whileTrue(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_3));
        // controlstick.y().whileTrue(elevator.set_position_command_angle(elevator.k_coral_level_sense_postion_4));
        
     
        // controlstick.a().whileTrue(new RunCommand(()->elevator.set_elevator_position_mm(elevator.k_coral_level_sense_postion_1), elevator));
        // controlstick.b().whileTrue(new RunCommand(()->elevator.set_elevator_position_mm(elevator.k_coral_level_sense_postion_2), elevator));
        // controlstick.x().whileTrue(new RunCommand(()->elevator.set_elevator_position_mm(elevator.k_coral_level_sense_postion_3), elevator));
        // controlstick.y().whileTrue(new RunCommand(()->elevator.set_elevator_position_mm(elevator.k_coral_level_sense_postion_4), elevator));

        //TEMPORARILY COMMENTED OUT TO TEST DIFFERENT ANGLE
        controlstick.a().whileTrue(new RepeatCommand(new InstantCommand(eleclaw::position_coral_1))).onFalse(new InstantCommand(eleclaw::stow));
        controlstick.b().whileTrue(new RepeatCommand(new InstantCommand(eleclaw::position_coral_2))).onFalse(new InstantCommand(eleclaw::stow));
    
        controlstick.x().whileTrue(new RepeatCommand(new InstantCommand(eleclaw::position_coral_3))).onFalse(new InstantCommand(eleclaw::stow));
        
        //controlstick.y().onTrue(new RepeatCommand(new InstantCommand(eleclaw::position_coral_4))).onFalse(new InstantCommand(eleclaw::stow));
        controlstick.y()
                .whileTrue(
                new InstantCommand(eleclaw::pre_position_4)
                .andThen(new WaitCommand(0.55))
                .andThen(new RepeatCommand(new InstantCommand(eleclaw::position_coral_4)))
                )       
                .onFalse(new InstantCommand(eleclaw::pre_position_4)
                .andThen(new WaitCommand(0.4))
                .andThen(new InstantCommand(eleclaw::stow)));

        controlstick.povUp().whileTrue(new RepeatCommand(new InstantCommand(eleclaw::upper_alge)).repeatedly()).onFalse(new InstantCommand(eleclaw::stow));
        controlstick.povDown().whileTrue(new InstantCommand(eleclaw::lower_alge).repeatedly()).onFalse(new InstantCommand(eleclaw::stow));
        controlstick.povLeft().whileTrue(new InstantCommand(eleclaw::position_load).repeatedly()).onFalse(new InstantCommand(eleclaw::stow));
        controlstick.leftTrigger().whileTrue(new InstantCommand(eleclaw::score_alge).repeatedly()).onFalse(new InstantCommand(eleclaw::stow));
        // controlstick.rightTrigger().onTrue(climber.deploy_climber()).onFalse(climber.stop());

        //binds buttons to intake and outtake commands
        controlstick.leftBumper().whileTrue(intake.continuous_outake());
        controlstick.rightBumper().whileTrue(intake.continuous_intake());
        controlstick.rightStick().whileTrue(new InstantCommand(eleclaw::active_stow).repeatedly());
        // controlstick.leftStick().onTrue(climber.reset_climber()).onFalse(climber.stop());
        // controlstick.rightStick().whileTrue(new InstantCommand(()->claw.balance(drivetrain.getRotation3d().getMeasureY())).repeatedly());
        // controlstick.povUp().whileTrue(claw.set_position_command_mm(claw.k_coral_position_1));
        // controlstick.povLeft().whileTrue(claw.set_position_command_mm(claw.k_coral_position_2));
        //controlstick.povDown().whileTrue(claw.set_position_command_mm(claw.k_coral_position_high));
        //controlstick.povRight().whileTrue(claw.set_position_command_mm(claw.k_coral_position_floor));
        // controlstick.povRight().whileTrue(claw.set_position_command_mm(claw.k_coral_position_high));


        
        // controlstick.leftBumper().whileTrue(intake.continuous_outake());
        
        
        // controlstick.rightTrigger(0.5).onTrue(intake.co_intake());

        

        
        System.out.println("bindings configured");

        // controlstick.rightTrigger().onTrue(climber.climb_command()).onFalse(climber.stop());
        
        // 
      
        

        
        
        
        
        // controlstick.leftTrigger().onTrue(climber.climb_command()).onFalse(climber.stop());
       // SmartDashboard.putData("ResetClimber:", climber.reset_climber());
        // SmartDashboard.putData("DeployClimber:", climber.deploy_climber());
        
        // SmartDashboard.putData("Climb", climber.climb_command());
        // SmartDashboard.putData("StopClimber:", climber.stop());

        
        // controlstick.leftBumper().onTrue(claw.position_command(claw.k_load_coral_position));
// 
        // elevator.setDefaultCommand(elevator.set_position_command_angle(elevator.k_stowed));
        // // elevator.setDefaultCommand(new InstantCommand(elevator::configure_from_dash, elevator));
        // // SmartDashboard.putData("configure elevator", new InstantCommand(elevator::configure_from_dash));
        // claw.setDefaultCommand(claw.set_position_command_mm(claw.k_stowed));
            


    }

    private void name_commands() {
        NamedCommands.registerCommand("EjectCoral", intake.auto_outtake_coral_command());

        // NameCommands.registerCommand("TroughtEject", eleclaw.troft_eject());
        // NamedCommands.registerCommand("ScoreCoral1", intake.auto_outtake_coral_command());
        NamedCommands.registerCommand("ScoreCoral1",eleclaw.score_coral_1());//jsut position no score
        NamedCommands.registerCommand("PickCoral", intake.intake_coral_command());
        
    }

    void flip_for(){
        forward_dir=forward_dir*-1;

    }
    
    void flip_side(){
        side_dir=side_dir*-1;
    }

    

    private SwerveRequest smooth_drive(){

        double x_speed = drivestick.getLeftY() * MaxSpeed*forward_dir *  ((elevator.is_stowed()&& !drivestick.start().getAsBoolean()) ? (drivestick.back().getAsBoolean() ? 1:0.7):0.3);
        double y_speed =drivestick.getLeftX()  * MaxSpeed *side_dir* ((elevator.is_stowed()&& !drivestick.start().getAsBoolean()) ? (drivestick.back().getAsBoolean() ? 1:0.7):0.3);
        double t_speed =-drivestick.getRightX() * MaxAngularRate;

        xfilter = Math.abs(x_speed)>=Math.abs(xfilter)? x_speed*k_xfilt_positive+xfilter*(1-k_xfilt_positive): x_speed*k_xfilt_negative+xfilter*(1-k_xfilt_negative);
        yfilter = Math.abs(y_speed)>=Math.abs(yfilter)? y_speed*k_yfilt_positive+yfilter*(1-k_yfilt_positive): y_speed*k_yfilt_negative+yfilter*(1-k_yfilt_negative);
        twistfilter = Math.abs(t_speed)>=Math.abs(twistfilter)? t_speed*k_tfilt_positive+twistfilter*(1-k_tfilt_positive): t_speed*k_tfilt_negative+twistfilter*(1-k_tfilt_negative);

        SmartDashboard.putNumber("x_filter", xfilter);
        SmartDashboard.putNumber("y_filter", yfilter);
        SmartDashboard.putNumber("twist_filter", twistfilter);
        // System.out.println("smooth drive");
        
        SwerveRequest request = drive.withVelocityX(xfilter) // Drive forward with negative Y (forward)
        .withVelocityY( yfilter)// Drive left with negative X (left)
        .withRotationalRate(twistfilter);

        return request;


    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

   

}
