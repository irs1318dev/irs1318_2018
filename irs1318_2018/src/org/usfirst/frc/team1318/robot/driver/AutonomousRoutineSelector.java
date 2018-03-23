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
    private final IDigitalInput jumperE;

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

        this.jumperE = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_JUMPER_E_DIGITAL_CHANNEL);
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

        boolean jumperE = !this.jumperE.get();

        boolean isOpportunistic = switchC;  // Opportunistic if third switch flipped, fixed routine if not
        boolean prefersSwitch = switchD;  // Prefers switch if fourth switch flipped, prefers scale if not
        boolean twoCubeMode = jumperE; // attempt 2-cube mode if the jumper is in

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
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "twoCubeMode", twoCubeMode);

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

        if (position == Position.Special)
        {
            return specialRoutineSelection(isOpportunistic, prefersSwitch);
        }
        else if (isOpportunistic)
        {
            return opportunisticRoutineSelection(position, prefersSwitch, twoCubeMode, isSwitchSideLeft, isScaleSideLeft);
        }
        else
        {
            return fixedRoutineSelection(position, prefersSwitch, twoCubeMode, isSwitchSideLeft, isScaleSideLeft);
        }
    }

    /**
     * Special selection from both position switches being flipped
     * 
     * @return special routine non-dependent on position
     */
    private static IControlTask specialRoutineSelection(boolean switchC, boolean switchD)
    {
        if (switchC && switchD)
        {
            return CrossBaseLine();
        }
        else
        {
            return GetFillerRoutine();
        }
    }

    /**
     * Opportunistic selection from third, opportunistic switch being flipped
     * 
     * @return most efficient routine (in-line with starting side)
     */
    private static IControlTask opportunisticRoutineSelection(
        Position position,
        boolean prefersSwitch,
        boolean twoCubeMode,
        boolean isSwitchSideLeft,
        boolean isScaleSideLeft)
    {
        switch (position)
        {
            case Center:
                return PlaceCubeOnSwitchFromMiddle(isSwitchSideLeft);

            case Left:
                if (isSwitchSideLeft && isScaleSideLeft)
                {
                    return prefersSwitch ? PlaceCubeOnSameSideSwitch(true, twoCubeMode) : PlaceCubeOnSameSideScale(true, twoCubeMode);
                }

                if (isScaleSideLeft)
                {
                    return PlaceCubeOnSameSideScale(true, twoCubeMode);
                }

                if (isSwitchSideLeft)
                {
                    return PlaceCubeOnSameSideSwitch(true, twoCubeMode);
                }

                return CrossBaseLine(); // prefersSwitch ? PlaceCubeOnOppositeSideSwitch(true) : PlaceCubeOnOppositeSideScale(true);

            case Right:
                if (!isSwitchSideLeft && !isScaleSideLeft)
                {
                    return prefersSwitch ? PlaceCubeOnSameSideSwitch(false, twoCubeMode) : PlaceCubeOnSameSideScale(false, twoCubeMode);
                }

                if (!isScaleSideLeft)
                {
                    return PlaceCubeOnSameSideScale(false, twoCubeMode);
                }

                if (!isSwitchSideLeft)
                {
                    return PlaceCubeOnSameSideSwitch(false, twoCubeMode);
                }

                return CrossBaseLine(); // prefersSwitch ? PlaceCubeOnOppositeSideSwitch(false) : PlaceCubeOnOppositeSideScale(false);

            case Special:
            default:
                return GetFillerRoutine();
        }
    }

    /**
     * Fixed selection from third, opportunistic selection switch not being flipped
     * 
     * @return routine for either scale or switch, whichever was selected with the fourth switch
     */
    private static IControlTask fixedRoutineSelection(
        Position position,
        boolean prefersSwitch,
        boolean twoCubeMode,
        boolean isSwitchSideLeft,
        boolean isScaleSideLeft)
    {
        switch (position)
        {
            case Center:
                if (prefersSwitch)
                {
                    return PlaceCubeOnSwitchFromMiddle(isSwitchSideLeft);
                }
                else
                {
                    return PlaceCubeOnScaleFromMiddle(isScaleSideLeft);
                }

            case Left:
                if (prefersSwitch)
                {
                    return isSwitchSideLeft
                        ? PlaceCubeOnSameSideSwitch(true, twoCubeMode) : PlaceCubeOnOppositeSideSwitch(true, twoCubeMode);
                }
                else
                {
                    return isScaleSideLeft
                        ? PlaceCubeOnSameSideScale(true, twoCubeMode) : PlaceCubeOnOppositeSideScale(true, twoCubeMode);
                }

            case Right:
                if (prefersSwitch)
                {
                    return !isSwitchSideLeft
                        ? PlaceCubeOnSameSideSwitch(false, twoCubeMode) : PlaceCubeOnOppositeSideSwitch(false, twoCubeMode);
                }
                else
                {
                    return !isScaleSideLeft
                        ? PlaceCubeOnSameSideScale(false, twoCubeMode) : PlaceCubeOnOppositeSideScale(false, twoCubeMode);
                }

            case Special:
            default:
                return GetFillerRoutine();
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

    private static IControlTask PlaceCubeOnSameSideSwitch(boolean startingLeft, boolean twoCubeMode)
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(false),
            SequentialTask.Sequence(
                new DriveDistanceTimedTask(148.0, 3.5),
                new NavxTurnTask(startingLeft ? 90.0 : -90.0),
                ConcurrentTask.AllTasks(
                    new DriveDistanceTimedTask(24.0, 0.75), // 18.5
                    new ElevatorMovementTask(
                        0.75,
                        Operation.ElevatorSwitchPosition)),
                AutonomousRoutineSelector.DepositCube(false),
                AutonomousRoutineSelector.PostRoutineBackUp()));
    }

    private static IControlTask PlaceCubeOnSameSideScale(boolean startingLeft, boolean twoCubeMode)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new DriveDistanceTimedTask(255.25, 4.0),
                    new NavxTurnTask(startingLeft ? 45.0 : -45.0),
                    new DriveDistanceTimedTask(14.5, 1.0)),
                SequentialTask.Sequence(
                    AutonomousRoutineSelector.InitialSetUp(true),
                    new WaitTask(3.5),
                    new ElevatorMovementTask(
                        1.75,
                        Operation.ElevatorHighScalePosition))),
            ConcurrentTask.AnyTasks(
                new PIDBrakeTask(),
                new OuttakeTask(1.0, true)),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(-14.5, 1.0),
                SequentialTask.Sequence(
                    new WaitTask(0.75),
                    ConcurrentTask.AllTasks(
                        new IntakeArmDownTask(0.25),
                        new ElevatorMovementTask(0.25, Operation.ElevatorCarryPosition)))),
            !twoCubeMode
                ? new WaitTask(0.0)
                : SequentialTask.Sequence(
                    new NavxTurnTask(startingLeft ? 150.0 : -150.0),
                    ConcurrentTask.AllTasks(
                        new DriveDistanceTimedTask(76.5, 1.75),
                        new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true)),
                    ConcurrentTask.AllTasks(
                        SequentialTask.Sequence(
                            new DriveDistanceTimedTask(-76.5, 1.75),
                            new NavxTurnTask(startingLeft ? 45.0 : -45.0),
                            new DriveDistanceTimedTask(-14.5, 1.0)),
                        SequentialTask.Sequence(
                            new WaitTask(1.0),
                            new ElevatorMovementTask(Operation.ElevatorHighScalePosition))),
                    ConcurrentTask.AnyTasks(
                        new PIDBrakeTask(),
                        new OuttakeTask(1.0, true))));

        //        return ConcurrentTask.AllTasks(
        //            AutonomousRoutineSelector.InitialSetUp(true),
        //            SequentialTask.Sequence(
        //                new DriveDistanceTimedTask(303.65, 5.5),
        //                new NavxTurnTask(startingLeft ? 90.0 : -90.0),
        //                ConcurrentTask.AllTasks(
        //                    new DriveDistanceTimedTask(4.0, 0.75),
        //                    new ElevatorMovementTask(
        //                        0.75,
        //                        Operation.ElevatorHighScalePosition)),
        //                AutonomousRoutineSelector.DepositCube(true),
        //                AutonomousRoutineSelector.PostRoutineBackUp()));
    }

    private static IControlTask PlaceCubeOnOppositeSideSwitch(boolean startingLeft, boolean twoCubeMode)
    {
        //general distance: 30 inches/sec
        //general turn speed: 45 degrees/sec
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(false),
            SequentialTask.Sequence(
                new DriveDistanceTimedTask(210.0, 3.5),
                new NavxTurnTask(startingLeft ? 90.0 : -90.0),
                new DriveDistanceTimedTask(135.0, 2.25),
                new NavxTurnTask(startingLeft ? 90.0 : -90.0), // over the bump, let's re-attain our desired angle
                new DriveDistanceTimedTask(90.0, 2.0),
                new NavxTurnTask(startingLeft ? 180.0 : -180.0),
                ConcurrentTask.AllTasks(
                    new DriveDistanceTimedTask(18.0, 1.0), // 42.0
                    new ElevatorMovementTask(0.5, Operation.ElevatorSwitchPosition)),
                new NavxTurnTask(startingLeft ? 270.0 : -270.0),
                new DriveDistanceTimedTask(14.5, 0.5),
                AutonomousRoutineSelector.DepositCube(false),
                AutonomousRoutineSelector.PostRoutineBackUp()));
    }

    private static IControlTask PlaceCubeOnOppositeSideScale(boolean startingLeft, boolean twoCubeMode)
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(true),
            SequentialTask.Sequence(
                new DriveDistanceTimedTask(210.0, 3.5),
                new NavxTurnTask(startingLeft ? 90.0 : -90.0),
                new DriveDistanceTimedTask(135.0, 2.25),
                new NavxTurnTask(startingLeft ? 90.0 : -90.0), // over the bump, let's re-attain our desired angle
                new DriveDistanceTimedTask(90.0, 2.0),
                new NavxTurnTask(startingLeft ? 0.0 : 0.0),
                new DriveDistanceTimedTask(49.0, 1.75), // 22.0 ??
                new NavxTurnTask(startingLeft ? -45.0 : 45.0),
                ConcurrentTask.AllTasks(
                    new DriveDistanceTimedTask(12.0, 0.75),
                    new ElevatorMovementTask(0.75, Operation.ElevatorHighScalePosition)),
                AutonomousRoutineSelector.DepositCube(true),
                AutonomousRoutineSelector.PostRoutineBackUp()));
    }

    private static IControlTask PlaceCubeOnSwitchFromMiddle(boolean switchIsLeft)
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(false),
            SequentialTask.Sequence(
                new DriveDistanceTimedTask(20.0, 1.0),
                new NavxTurnTask(switchIsLeft ? -45.0 : 37.5),
                new DriveDistanceTimedTask(switchIsLeft ? 85.0 : 80.0, 3.0),
                new NavxTurnTask(false, 0.0),
                ConcurrentTask.AllTasks(
                    new DriveDistanceTimedTask(switchIsLeft ? 20.0 : 18.0, 1.25),
                    new ElevatorMovementTask(1.25, Operation.ElevatorSwitchPosition)),
                AutonomousRoutineSelector.DepositCubeOnly(),
                AutonomousRoutineSelector.PostRoutineBackUp()));
    }

    private static IControlTask PlaceCubeOnScaleFromMiddle(boolean scaleIsLeft)
    {
        return new WaitTask(0);
    }

    private static IControlTask PlaceSecondCubeOnScaleFromScale(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            new NavxTurnTask(startingLeft ? 145.0 : -145.0),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(86.0, 2.0),
                new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true)),
            new DriveDistanceTimedTask(-86.0, 2.0),
            ConcurrentTask.AllTasks(
                new ElevatorMovementTask(Operation.ElevatorHighScalePosition),
                SequentialTask.Sequence(
                    new NavxTurnTask(startingLeft ? 60.0 : -60.0),
                    new DriveDistanceTimedTask(24.0, 1.5))),
            AutonomousRoutineSelector.DepositCubeOnly());
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
            new OuttakeTask(2.0, true));
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
