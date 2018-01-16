package org.usfirst.frc.team1318.robot.driver;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilib.IDigitalInput;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.ConcurrentTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.DriveDistancePositionTimedTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.DriveDistanceTimedTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.SequentialTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.TurnTimedTask;
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
        this.dipSwitchA = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_A_CHANNEL);
        this.dipSwitchB = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_B_CHANNEL);
        this.dipSwitchC = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_C_CHANNEL);
        this.dipSwitchD = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_D_CHANNEL);
    }

    /**
     * Check what routine we want to use and return it
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine()
    {
        boolean isOpportunistic = this.dipSwitchC.get();  // Opportunistic if third switch flipped, fixed routine if not
        boolean prefersSwitch = this.dipSwitchD.get();  // Prefers switch if fourth switch flipped, prefers scale if not

        String rawSideData = DriverStation.getInstance().getGameSpecificMessage();
        boolean isSwitchSideLeft = (rawSideData.charAt(0) == 'L');
        boolean isScaleSideLeft = (rawSideData.charAt(1) == 'L');

        int positionSelection = 0; // Robot position where 0 is center, 1 is left, and 2 is right (3 is reserved for special cases)

        // add next base2 number (1, 2, 4, 8, 16, etc.) here based on number of dipswitches and which is on...
        if (this.dipSwitchA.get())
        {
            positionSelection += 1;
        }
        if (this.dipSwitchB.get())
        {
            positionSelection += 2;
        }

        // print routine parameters to the smartdash
        this.logger.logInteger(
            AutonomousRoutineSelector.LogName, "position",
            positionSelection);

        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "isOpportunistic",
            isOpportunistic);

        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "prefersSwitch",
            prefersSwitch);

        if (positionSelection == 3)
        {
            return specialRoutineSelection(isOpportunistic, prefersSwitch);
        }
        else if (isOpportunistic)
        {
            return opportunisticRoutineSelection(positionSelection, prefersSwitch, isSwitchSideLeft, isScaleSideLeft);
        }
        else
        {
            return setRoutineSelection(positionSelection, prefersSwitch, isSwitchSideLeft, isScaleSideLeft);
        }
    }

    /**
     * Special selection from both position switches being flipped
     * 
     * @return special routine non-dependent on position
     */
    private static IControlTask specialRoutineSelection(boolean switchThree, boolean switchFour)
    {
        if (switchThree && switchFour)
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
        int positionSelection, boolean prefersSwitch, boolean isSwitchSideLeft, boolean isScaleSideLeft)
    {
        switch (positionSelection)
        {
            case 0: // center
                return PlaceCubeOnSwitchFromMiddle();

            case 1: // left
                if (isSwitchSideLeft && isScaleSideLeft)
                {
                    return prefersSwitch ? PlaceCubeOnSameSideSwitch(true) : PlaceCubeOnSameSideScale(true);
                }
                else if (isScaleSideLeft)
                {
                    return PlaceCubeOnSameSideScale(true);
                }
                else
                {
                    return PlaceCubeOnSameSideSwitch(true);
                }
            case 2: // right
                if (!isSwitchSideLeft && !isScaleSideLeft)
                {
                    return prefersSwitch ? PlaceCubeOnSameSideSwitch(true) : PlaceCubeOnSameSideScale(true);
                }
                else if (!isScaleSideLeft)
                {
                    return PlaceCubeOnSameSideScale(true);
                }
                else
                {
                    return PlaceCubeOnSameSideSwitch(true);
                }
            default:
                return GetFillerRoutine();
        }
    }

    /**
     * Fixed selection from third, opportunistic selection switch not being flipped
     * 
     * @return routine for either scale or switch, whichever was selected with the fourth switch
     */
    private static IControlTask setRoutineSelection(
        int positionSelection, boolean prefersSwitch, boolean isSwitchSideLeft, boolean isScaleSideLeft)
    {
        switch (positionSelection)
        {
            case 0:
                if (prefersSwitch)
                {
                    return PlaceCubeOnSwitchFromMiddle();
                }
                else
                {
                    return PlaceCubeOnScaleFromMiddle();
                }
            case 1:
                if (prefersSwitch)
                {
                    return isSwitchSideLeft ? PlaceCubeOnSameSideSwitch(true) : PlaceCubeOnOppositeSideSwitch(true);
                }
                else
                {
                    return isScaleSideLeft ? PlaceCubeOnSameSideScale(true) : PlaceCubeOnOppositeSideScale(true);
                }
            case 2:
                if (prefersSwitch)
                {
                    return !isSwitchSideLeft ? PlaceCubeOnSameSideSwitch(false) : PlaceCubeOnOppositeSideSwitch(false);
                }
                else
                {
                    return !isScaleSideLeft ? PlaceCubeOnSameSideScale(false) : PlaceCubeOnOppositeSideScale(false);
                }
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

    private static IControlTask PlaceCubeOnSameSideSwitch(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            new DriveDistanceTimedTask(147.75, 5.0),
            new TurnTimedTask(startingLeft ? 90.0 : -90.0, 1.5),
            new DriveDistanceTimedTask(11.5, 0.5),
            AutonomousRoutineSelector.DepositCube());
    }

    private static IControlTask PlaceCubeOnSameSideScale(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            new DriveDistanceTimedTask(267.75, 9.0),
            new TurnTimedTask(startingLeft ? 70.0 : -70.0, 1.5),
            new DriveDistanceTimedTask(49.4, 2),
            AutonomousRoutineSelector.DepositCube());
    }

    private static IControlTask PlaceCubeOnOppositeSideSwitch(boolean startingLeft)
    {
        // TODO: function completion
        return new WaitTask(0);
    }

    private static IControlTask PlaceCubeOnOppositeSideScale(boolean startingLeft)
    {
        // TODO: function completion
        return new WaitTask(0);
    }

    private static IControlTask PlaceCubeOnSwitchFromMiddle()
    {
        // TODO: function completion
        return new WaitTask(0);
    }

    private static IControlTask PlaceCubeOnScaleFromMiddle()
    {
        // TODO: function completion
        return new WaitTask(0);
    }

    private static IControlTask DepositCube()
    {
        // TODO: function completion
        return new WaitTask(0);
    }

    private static IControlTask CrossBaseLine()
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(),
            new DriveDistancePositionTimedTask(0.5, 100, 5.0));
    }

    private static IControlTask InitialSetUp()
    {
        // TODO: elevator and intake setup from starting configuration
        return SequentialTask.Sequence();
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
