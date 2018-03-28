package org.usfirst.frc.team1318.robot.driver;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilib.IDigitalInput;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.AdvancedIntakeOuttakeTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.ConcurrentTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.DriveDistancePositionTimedTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.DriveDistanceTimedTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.ElevatorMovementTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.IntakeArmDownTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.IntakeArmUpTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.NavxTurnTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.OuttakeTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.PIDBrakeTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.SequentialTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.WaitTask;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import edu.wpi.first.wpilibj.DriverStation;

@Singleton
public class AutonomousRoutineSelector
{
    private static final String LogName = "auto";
    private final IDashboardLogger logger;

    private final IDigitalInput dipSwitchA;
    private final IDigitalInput dipSwitchB;
    private final IDigitalInput dipSwitchC;
    private final IDigitalInput dipSwitchD;
    private final IDigitalInput dipSwitchE;
    private final IDigitalInput dipSwitchF;

    private enum Position
    {
        Center, Left, Right, Special;
    }

    /**
     * Initializes a new AutonomousDriver
     */
    @Inject
    public AutonomousRoutineSelector(
        IDashboardLogger logger,
        IWpilibProvider provider)
    {
        // initialize robot parts that are used to select autonomous routine (e.g. dipswitches) here...
        this.logger = logger;
        this.dipSwitchA = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_A_DIGITAL_CHANNEL);
        this.dipSwitchB = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_B_DIGITAL_CHANNEL);
        this.dipSwitchC = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_C_DIGITAL_CHANNEL);
        this.dipSwitchD = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_D_DIGITAL_CHANNEL);
        this.dipSwitchE = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_E_DIGITAL_CHANNEL);
        this.dipSwitchF = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_F_DIGITAL_CHANNEL);
    }

    /**
     * Check what routine we want to use and return it
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine()
    {
        boolean switchA = !this.dipSwitchA.get();
        boolean switchB = !this.dipSwitchB.get();
        boolean switchC = !this.dipSwitchC.get();
        boolean switchD = !this.dipSwitchD.get();
        boolean switchE = !this.dipSwitchE.get();
        boolean switchF = !this.dipSwitchF.get();

        boolean isOpportunistic = switchC;  // Opportunistic if third switch flipped, fixed routine if not
        boolean prefersSwitch = switchD;  // Prefers switch if fourth switch flipped, prefers scale if not
        boolean twoCubeEnabled = switchE; // 2-cube mode is enabled if fifth switch flipped. 
        boolean twoCubePrefersSwitch = switchF; // 2nd cube should prefer switch if sixth switch flipped

        // add next base2 number (1, 2, 4, 8, 16, etc.) here based on number of dipswitches and which is on...
        int positionSelection = 0;
        if (switchA)
        {
            positionSelection += 1;
        }
        if (switchB)
        {
            positionSelection += 2;
        }

        // Robot position: 0 is center, 1 is left, and 2 is right (3 is reserved for special cases)
        Position position;
        switch (positionSelection)
        {
            case 0:
                position = Position.Center;
                break;
            case 1:
                position = Position.Left;
                break;
            case 2:
                position = Position.Right;
                break;
            case 3:
            default:
                position = Position.Special;
                break;
        }

        String rawSideData = DriverStation.getInstance().getGameSpecificMessage();

        // print routine parameters to the smartdash
        this.logger.logString(AutonomousRoutineSelector.LogName, "gameData", rawSideData);
        this.logger.logString(AutonomousRoutineSelector.LogName, "position", position.toString());
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "isOpportunistic", isOpportunistic);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "prefersSwitch", prefersSwitch);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "twoCubeEnabled", twoCubeEnabled);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "twoCubePrefersSwitch", twoCubePrefersSwitch);

        // handle special scenarios before trying to parse game data
        if (position == Position.Special)
        {
            if (isOpportunistic && prefersSwitch)
            {
                return CrossBaseLine();
            }
            else
            {
                return GetFillerRoutine();
            }
        }

        // parse game data
        boolean isSwitchSideLeft = false;
        boolean isScaleSideLeft = false;
        if (rawSideData != null && rawSideData.length() >= 2)
        {
            isSwitchSideLeft = (rawSideData.charAt(0) == 'L');
            isScaleSideLeft = (rawSideData.charAt(1) == 'L');
        }
        else
        {
            return CrossBaseLine();
        }

        // handle center scenario
        if (position == Position.Center)
        {
            if (isOpportunistic || prefersSwitch)
            {
                if (twoCubeEnabled)
                {
                    return PlaceTwoCubesOnSwitchFromMiddle(isSwitchSideLeft);
                }
                else
                {
                    return PlaceCubeOnSwitchFromMiddleOnly(isSwitchSideLeft);
                }
            }
            else
            {
                return PlaceCubeOnScaleFromMiddle(isScaleSideLeft);
            }
        }

        // handle left/right scenarios
        boolean isRobotLeft = position == Position.Left;
        if (isOpportunistic)
        {
            if (isRobotLeft == isSwitchSideLeft && isRobotLeft == isScaleSideLeft)
            {
                if (prefersSwitch)
                {
                    if (twoCubeEnabled)
                    {
                        return PlaceTwoCubesOnSameSideSwitch(isRobotLeft);
                    }
                    else
                    {
                        return PlaceCubeOnSameSideSwitchOnly(isRobotLeft);
                    }
                }
                else
                {
                    if (twoCubeEnabled)
                    {
                        if (twoCubePrefersSwitch)
                        {
                            return PlaceCubesOnSameSideScaleAndSwitch(isRobotLeft);
                        }
                        else
                        {
                            return PlaceTwoCubesOnSameSideScale(isRobotLeft);
                        }
                    }
                    else
                    {
                        return PlaceCubeOnSameSideScaleOnly(isRobotLeft);
                    }
                }
            }

            if (isRobotLeft == isScaleSideLeft)
            {
                if (twoCubeEnabled)
                {
                    return PlaceTwoCubesOnSameSideScale(isRobotLeft);
                }
                else
                {
                    return PlaceCubeOnSameSideScaleOnly(isRobotLeft);
                }
            }

            if (isRobotLeft == isSwitchSideLeft)
            {
                return PlaceCubeOnSameSideSwitchOnly(isRobotLeft);
            }

            return CrossBaseLine(); // prefersSwitch ? PlaceCubeOnOppositeSideSwitch(isRobotLeft) : PlaceCubeOnOppositeSideScale(isRobotLeft);
        }
        else // "always" mode
        {
            if (prefersSwitch)
            {
                if (isRobotLeft == isSwitchSideLeft)
                {
                    if (twoCubeEnabled)
                    {
                        return PlaceTwoCubesOnSameSideSwitch(isRobotLeft);
                    }
                    else
                    {
                        return PlaceCubeOnSameSideSwitchOnly(isRobotLeft);
                    }
                }
                else // switch is on opposite side
                {
                    return PlaceCubeOnOppositeSideSwitch(isRobotLeft);
                }
            }
            else // prefers scale
            {
                if (isRobotLeft == isScaleSideLeft)
                {
                    if (twoCubeEnabled)
                    {
                        if (!twoCubePrefersSwitch || isRobotLeft != isSwitchSideLeft)
                        {
                            return PlaceTwoCubesOnSameSideScale(isRobotLeft);
                        }
                        else
                        {
                            return PlaceCubesOnSameSideScaleAndSwitch(isRobotLeft);
                        }
                    }
                    else // single-cube mode
                    {
                        return PlaceCubeOnSameSideScaleOnly(isRobotLeft);
                    }
                }
                else // scale is on opposite side
                {
                    return PlaceCubeOnOppositeSideScale(isRobotLeft);
                }
            }
        }
    }

    /**
     * Gets an autonomous routine that does nothing
     * 
     * @return very long WaitTask
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0);
    }

    private static IControlTask PlaceCubeOnSameSideSwitchOnly(boolean startingLeft)
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(false),
            SequentialTask.Sequence(
                new DriveDistanceTimedTask(148.0, 3.0), // 3.5s
                ConcurrentTask.AllTasks(
                    SequentialTask.Sequence(
                        new NavxTurnTask(startingLeft ? 90.0 : -90.0),
                        new DriveDistanceTimedTask(24.0, 1.0)), // 18.5"
                    new ElevatorMovementTask(
                        1.25,
                        Operation.ElevatorSwitchPosition)),
                AutonomousRoutineSelector.DepositCubeOnly(),
                AutonomousRoutineSelector.PostRoutineBackUp()));
    }

    private static IControlTask PlaceTwoCubesOnSameSideSwitch(boolean startingLeft)
    {
        return SequentialTask.Sequence(PlaceCubeOnSameSideSwitchOnly(startingLeft),
            new NavxTurnTask(0),
            new DriveDistanceTimedTask(60.5, 1.5),
            new NavxTurnTask(startingLeft ? 145.0 : -145.0),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(18.0, 1),
                new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true)),
            ConcurrentTask.AllTasks(
                new ElevatorMovementTask(1.25, Operation.ElevatorSwitchPosition),
                SequentialTask.Sequence(
                    new IntakeArmUpTask(0.25),
                    new DriveDistanceTimedTask(10.0, 1))),
            AutonomousRoutineSelector.DepositCubeOnly(),
            AutonomousRoutineSelector.PostRoutineBackUp());
    }

    private static IControlTask PlaceCubeOnSameSideScaleOnly(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new DriveDistanceTimedTask(255.25, 3.5),
                    new NavxTurnTask(startingLeft ? 45.0 : -45.0),
                    new DriveDistanceTimedTask(14.5, 0.75)),
                SequentialTask.Sequence(
                    AutonomousRoutineSelector.InitialSetUp(true),
                    new WaitTask(2.75),
                    new ElevatorMovementTask(1.75, Operation.ElevatorHighScalePosition))),
            ConcurrentTask.AnyTasks(
                new PIDBrakeTask(),
                new OuttakeTask(0.75, true)),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(-14.5, 0.75),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    ConcurrentTask.AllTasks(
                        new IntakeArmDownTask(0.25),
                        new ElevatorMovementTask(0.25, Operation.ElevatorCarryPosition)))));
    }

    private static IControlTask PlaceTwoCubesOnSameSideScale(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            PlaceCubeOnSameSideScaleOnly(startingLeft),
            new NavxTurnTask(startingLeft ? 145.0 : -145.0),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(72.5, 1.75), // 76.5
                SequentialTask.Sequence(
                    new WaitTask(0.75),
                    new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true))),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new NavxTurnTask(startingLeft ? 145.0 : -145.0),
                    new DriveDistanceTimedTask(-72.5, 1.75), // 76.5
                    new NavxTurnTask(startingLeft ? 45.0 : -45.0),
                    new DriveDistanceTimedTask(14.5, 0.75)), //1s
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new ElevatorMovementTask(1.25, Operation.ElevatorHighScalePosition))),
            ConcurrentTask.AnyTasks(
                new PIDBrakeTask(),
                new OuttakeTask(1.0, true)),
            AutonomousRoutineSelector.PostRoutineBackUp());
    }

    private static IControlTask PlaceCubesOnSameSideScaleAndSwitch(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            PlaceCubeOnSameSideScaleOnly(startingLeft),
            new NavxTurnTask(startingLeft ? 145.0 : -145.0), // 150.0
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(76.5, 1.5), //1.75s
                SequentialTask.Sequence(
                    new WaitTask(0.75),
                    new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true))),
            ConcurrentTask.AllTasks(
                new IntakeArmUpTask(0.25),
                new ElevatorMovementTask(1.25, Operation.ElevatorSwitchPosition),
                SequentialTask.Sequence(
                    new DriveDistanceTimedTask(-12.0, 1.0),
                    new NavxTurnTask(startingLeft ? 160.0 : -160.0),
                    new DriveDistanceTimedTask(24.0, 1.0))),
            ConcurrentTask.AnyTasks(
                new PIDBrakeTask(),
                new OuttakeTask(1.0, true)),
            AutonomousRoutineSelector.PostRoutineBackUp());
    }

    private static IControlTask PlaceCubeOnOppositeSideSwitch(boolean startingLeft)
    {
        //general distance: 30 inches/sec
        //general turn speed: 45 degrees/sec
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(false),
            SequentialTask.Sequence(
                new DriveDistanceTimedTask(215.0, 3.0), // 210.0
                new NavxTurnTask(startingLeft ? 90.0 : -90.0),
                new DriveDistanceTimedTask(135.0, 2.0), //2.25s
                new NavxTurnTask(startingLeft ? 90.0 : -90.0), // over the bump, let's re-attain our desired angle
                new DriveDistanceTimedTask(90.0, 1.75), //2s
                new NavxTurnTask(startingLeft ? 180.0 : -180.0),
                ConcurrentTask.AllTasks(
                    new DriveDistanceTimedTask(23.0, 1.0), // 18.0
                    new ElevatorMovementTask(0.5, Operation.ElevatorSwitchPosition)),
                new NavxTurnTask(startingLeft ? 270.0 : -270.0),
                new DriveDistanceTimedTask(14.5, 0.5),
                AutonomousRoutineSelector.DepositCube(false),
                AutonomousRoutineSelector.PostRoutineBackUp()));
    }

    private static IControlTask PlaceCubeOnOppositeSideScale(boolean startingLeft)
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(true),
            SequentialTask.Sequence(
                new DriveDistanceTimedTask(215.0, 3.0), //3.5s, 210.0
                new NavxTurnTask(startingLeft ? 90.0 : -90.0),
                new DriveDistanceTimedTask(135.0, 2.0), //2.25s
                new NavxTurnTask(startingLeft ? 90.0 : -90.0), // over the bump, let's re-attain our desired angle
                new DriveDistanceTimedTask(90.0, 1.75), //2s
                new NavxTurnTask(startingLeft ? 0.0 : 0.0),
                new DriveDistanceTimedTask(44.0, 1.25), // 29.0 ??   1.75s
                new NavxTurnTask(startingLeft ? -45.0 : 45.0),
                ConcurrentTask.AllTasks(
                    new DriveDistanceTimedTask(12.0, 0.75), //.75s
                    new ElevatorMovementTask(0.75, Operation.ElevatorHighScalePosition)),
                AutonomousRoutineSelector.DepositCube(true),
                AutonomousRoutineSelector.PostRoutineBackUp()));
    }

    private static IControlTask PlaceCubeOnSwitchFromMiddleOnly(boolean switchIsLeft)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new DriveDistanceTimedTask(20.0, .75), //1s
                    new NavxTurnTask(switchIsLeft ? -45.0 : 37.5),
                    new DriveDistanceTimedTask(switchIsLeft ? 85.0 : 80.0, 1.5), //3s
                    new NavxTurnTask(false, 0.0),
                    new DriveDistanceTimedTask(switchIsLeft ? 20.0 : 18.0, .75)), //1.25s
                SequentialTask.Sequence(
                    AutonomousRoutineSelector.InitialSetUp(false),
                    new WaitTask(2.5),
                    new ElevatorMovementTask(1.25, Operation.ElevatorSwitchPosition))),
            AutonomousRoutineSelector.DepositCubeOnly(),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new WaitTask(0.75),
                    ConcurrentTask.AllTasks(
                        new IntakeArmDownTask(0.75),
                        new ElevatorMovementTask(0.75, Operation.ElevatorCarryPosition))),
                new DriveDistanceTimedTask(-24.0, 1.5)));
    }

    private static IControlTask PlaceTwoCubesOnSwitchFromMiddle(boolean switchIsLeft)
    {
        return SequentialTask.Sequence(
            PlaceCubeOnSwitchFromMiddleOnly(switchIsLeft),
            new NavxTurnTask(switchIsLeft ? 67.25 : -67.25),
            ConcurrentTask.AllTasks(
                new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true),
                new DriveDistanceTimedTask(45.0, 1.25)),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new WaitTask(1.25),
                    new ElevatorMovementTask(0.5, Operation.ElevatorSwitchPosition)),
                SequentialTask.Sequence(
                    new DriveDistanceTimedTask(-24.0, 1.25),
                    new NavxTurnTask(false, 0.0),
                    new DriveDistanceTimedTask(24.0, 1.0))), //1.25s
            AutonomousRoutineSelector.DepositCubeOnly(),
            AutonomousRoutineSelector.PostRoutineBackUp());
    }

    private static IControlTask PlaceCubeOnScaleFromMiddle(boolean scaleIsLeft)
    {
        return new WaitTask(0);
    }

    private static IControlTask CrossBaseLine()
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(true),
            new DriveDistancePositionTimedTask(0.5, 145.0, 5.0));
    }

    private static IControlTask InitialSetUp(boolean putArmDown)
    {
        if (putArmDown)
        {
            return ConcurrentTask.AllTasks(
                new IntakeArmDownTask(0.25),
                new ElevatorMovementTask(0.25, Operation.ElevatorCarryPosition));
        }

        return new ElevatorMovementTask(0.25, Operation.ElevatorCarryPosition);
    }

    private static IControlTask DepositCube(boolean isScale)
    {
        return ConcurrentTask.AnyTasks(
            new PIDBrakeTask(),
            SequentialTask.Sequence(
                new ElevatorMovementTask(
                    isScale ? 0.75 : 0.5,
                    isScale ? Operation.ElevatorHighScalePosition : Operation.ElevatorSwitchPosition),
                new OuttakeTask(2.0, true)));
    }

    private static IControlTask DepositCubeOnly()
    {
        return ConcurrentTask.AnyTasks(
            new PIDBrakeTask(),
            new OuttakeTask(1.0, true));
    }

    private static IControlTask PostRoutineBackUp()
    {
        return ConcurrentTask.AllTasks(
            SequentialTask.Sequence(
                new WaitTask(0.75),
                ConcurrentTask.AllTasks(
                    new IntakeArmDownTask(0.75),
                    new ElevatorMovementTask(0.75, Operation.ElevatorCarryPosition))),
            new DriveDistanceTimedTask(-24.0, 1.5));

    }
}

/*







































































































































                                      .                                                             
                                    .;+;+                                                           
                                    .+;;'   `,+'.                                                   
                                    ;';;+:..`` :+'+                                                 
                                    ,'+`    .+;;;;;+                                                
                                     ;,,, .+;;;;;'+++;                                              
                                     ;' `+;;;;;#+'+'+''#:.                                          
                                     '`+';;;'+;+;+++'''+'.                                          
                                     #';;;;#';+'+'''+''+'                                           
                                     ;;;;#;,+;;+;;;'''''':                                          
                                     ';'++'.`+;;'';;''+'',                                          
                                     :#'#+'``.'+++'#++'':`                                          
                                      `';++##```##+.''.##                                           
                                      +++#   #`#  `++++                                             
                                      +'#+ # :#: # ##'+                                             
                                      `#+#   +`+   #'#`                                             
                                       :,.+,+,`:+,+..,                                              
                                       `,:```,`,`.`;,                                               
                                        :+.;``.``;.#;                                               
                                        .'``'+'+'``'.                                               
                                         ,````````..                                                
                                          :```````:                                                 
                                          +``.:,``'                                                 
                                          :```````:                                                 
                                           +`````+                                                  
                                            ';+##                                                   
                                            '```'                                                   
                                           `'```'`                                                  
                                         .+''''''''                                                 
                                        +;;;;;;;;''#                                                
                                       :       `   `:                                               
                                      `,            '                                               
                                      +              '                                              
                                     ,;';,``.``.,,,:;#                                              
                                     +;;;;;;;;;;;;;;;'                                              
                                    ,';;;;;;;;;;;;;;;',                                             
                                    +:;;;;;;';;;;;;;;;+                                             
                                   `.   .:,;+;;:::;.``,                                             
                                   :`       #,       `.`                                            
                                   +       # ;        .;                                            
                                  .;;,`    ,         `,+                                            
                                  +;;;;;;''';;;;;;;';;';                                            
                                  +;;;;;;;';;;;;;;;;;'';;                                           
                                 `';;;;;;';;;;;;;;;;;';;+                                           
                                 + `:;;;;+;;;;;;;;';'''::                                           
                                 '     `:  ```````    ,  ,                                          
                                :       '             ;  +                                          
                                '`     ..             ,  ,                                          
                               ,;;;;;..+,`        ```.':;',                                         
                               +;;;;;;'+;;;;;;;;;;;;;;+;;;+                                         
                               ';;;;;;++;;;;;;;;;;;;;;';;;+                                         
                              `.:';;;;;#;;;;;;;;;;;;;;';;;;`                                        
                              ;    `,; ',:;;';;';;;;;:;``  +                                        
                              +      ; ;              ;    `                                        
                              ;      : +              '    `;                                       
                              ';:`` `` '              :`,:;;+                                       
                             `';;;;'+  +,..```````..:;#;;;;;;.                                      
                             `;;;;;;+  +;;;;;;;;;;;;;':';;;;;#                                      
                             .;;;;;;+  ';;;;;;;;;;;;;;,';;;;` .                                     
                             : `.;;'+  +;;;;;;;;;;;;;','.`    +                                     
                             '      ;  +.,,;:;:;;;,..`: ,     ``                                    
                             +      ,  '              : ;   .;'+                                    
                             +.`   ``  +              ;  ;:;;;;':                                   
                             ';;;';;`  +             .'  ;;;;;;;+                                   
                             ';;;;;'   :+++#++##+#+''',   +;;;;.`.                                  
                             +;;;;;'   +;;::;;;+:+;;'',   ,;;.   +                                  
                            ``:;;;;+   +;;:;;;:+;+;;++;    +     .`                                 
                             `   ``'   +;;;;;;;+;+;;'+;     ,   ;#,                                 
                            .      ;   ';;;;;;;;;;;;++'     + .+``.;                                
                            ``     ;   ';;;;;;+;';;;'+'      #`````:,                               
                             +++;,:.   ':;''++;:';:;'';      +``````,`                              
                             ,```,+    +;;';:;;+;;;;'';      +``````,+                              
                            .``````:   ;:;;++';;;;;;';,      ,``:#``+`.                             
                            ,``````'   `';;;;:;;;;;;+;`     '+``+:'`..'                             
                            ,``````'    +;;;;;;;;;;;''     ;:'``#;;.`++                             
                            ```````;    `;:;;;;;;;;;;#     ':'``++:+`+;                             
                            ```'`.`;     +;;;;;;;;;;;+    :::#``' +#`';                             
                            ,``'`:`#     `';;;;;;;;;;+    +:'.`,. ++`;;                             
                            +`.``+`'     :#;;;;;;;;;;;`   +:# ,`  +;`.'                             
                           ,.`+`.:.      ##;;;;;;;;;;;'   ,'`     ;:+#                              
                           '`;.`+`#      ##+;;;;;;;;;;+          ,::;                               
                           ,+,`:``,     :###;;;;;;;;;:'          +:;`                               
                            '`,,`+      ';##';;;;;;;;;;.         +:#                                
                             '+.+       +;;##;;;;;;;;;;'         ;:;                                
                               `       :;;;+#;;;;;;;;;;+        ;::`                                
                                       +;;;;#+;;;;;;;;;;        +:'                                 
                                       ';;;;+#;;;;;;;;;;.       ;:'                                 
                                      ,;;;;;;#;;;;;;;;;;+      +::.                                 
                                      +;;;;;;'';;;;;;;;;'      +:+                                  
                                     `;;;;;;;;#;;;;;;;;;;`    `;:+                                  
                                     ,;;;;;;;;+;;;;;;;;;;+    ':;,                                  
                                     +;;;;;;;;;+;;;;;;;;;'    +:+                                   
                                    .;;;;;;;;;+,;;;;;;;;;;`   ;;+                                   
                                    ';;;;;;;;;, ';;;;;;:;;,  +;:,                                   
                                    ';;;;;;;;'  +;;;;;;;;;'  +:+                                    
                                   ;;;;;;;;;;+  ,;;;;;;;;;+  ;:'                                    
                                   +;;;;;;;;;    ';;;;;;;;;`;:;`                                    
                                   ;;;;;;;;;+    +;;;;;;;;;+#:+                                     
                                  ';;;;;;;;;:    ;;;;;;;;;;';:'                                     
                                 `';;;;;;;:'      ';;;;;;;;;;:.                                     
                                 .;;;;;;;;;+      +;;;;;;;;;'+                                      
                                 +;;;;;;;;;       ';;;;;;;;;#+                                      
                                `;;;;;;;;;+       `;;;;;;;;;;`                                      
                                +;;;;;;;;;.        +;;;;;;;;;`                                      
                                ';;;;;;;:'         ;;;;;;;;;;;                                      
                               :;;;;;;;;;:         `;;;;;;;;;+                                      
                               +;;;;;;;;;           ';;;;;;;;;`                                     
                               ;;;;;;;;;+           ';;;;;;;;;:                                     
                              ';;;;;;;;;;           ,;;;;;;;;;+                                     
                              ':;;;;;;;'             +;;;;;;;;;                                     
                             .;:;;;;;;;'             +;;;;;;;;;:                                    
                             +;;;;;;;;;`             .;;;;;;;;;+                                    
                            `;;;;;;;;;+               ;:;;;;;;;;`                                   
                            ;;;;;;;;;;.               +;;;;;;;::.                                   
                            ';;;;;;;;'`               :;;;;;;;;:+                                   
                           :;;;;;;;;:'                ';;;;;;;;;'                                   
                           ';;;;;;;;'`                +#;;;;;;;;;`                                  
                          `;;;;;;;;;+                 '';;;;;;;;;+                                  
                          +;;;;;;;;;.                '::;;;;;;;;;+                                  
                          ;;;;;;;;;+                 #:'';;;;;;;;;`                                 
                         .#;;;;;;;;'                `;:+;;;;;;;;;;;                                 
                         ':'';;;;;;                 '::.,;;;;;;;;;+                                 
                        +::::+';;;+                 ':'  +:;;;;;;;;`                                
                       `;;;::::;#+:                `;:+  +;;;;;;;:;;      '#+,                      
                       +#::::::::;'`               +:;,  `;;;;:;;'#';;;;;::;:'`                     
                       ;:''::::::::#`              +:'    ';:;;+'::;;:;::::::''                     
                       +::;+':::::::'.            .:;+    '''+;::;:;:::;:::;':'                     
                        ';;:;'';:::::':           +::.     +:::::::::::::;#;:#                      
                         .''##;#;:;;:::'+        `+;'      ;:;::::::::;'+;:'+                       
                           ` `+:;+:;::;::+       +:;#      ';:::;:+#+';:::+.                        
                              ,+::+#';::;+       ';::      #:;;'+';'''++:`                          
                                '':::;'''#      ,:;;`      #';:;;:+                                 
                                 `:'++;;':       :++       .;;:;;#,                                 
                                       `                    '':``                                   


*/
