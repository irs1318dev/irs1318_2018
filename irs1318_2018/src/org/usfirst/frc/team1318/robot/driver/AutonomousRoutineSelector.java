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
import org.usfirst.frc.team1318.robot.driver.controltasks.DriveVelocityTimedTask;
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

        //boolean isOpportunistic = switchC;  // Opportunistic if third switch flipped, fixed routine if not
        //boolean prefersSwitch = switchD;  // Prefers switch if fourth switch flipped, prefers scale if not
        //boolean twoCubeEnabled = switchE; // 2-cube mode is enabled if fifth switch flipped. 
        //boolean twoCubePrefersSwitch = switchF; // 2nd cube should prefer switch if sixth switch flipped

        boolean nuclearOption = false;

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
        //this.logger.logBoolean(AutonomousRoutineSelector.LogName, "isOpportunistic", isOpportunistic);
        //this.logger.logBoolean(AutonomousRoutineSelector.LogName, "prefersSwitch", prefersSwitch);
        //this.logger.logBoolean(AutonomousRoutineSelector.LogName, "twoCubeEnabled", twoCubeEnabled);
        //this.logger.logBoolean(AutonomousRoutineSelector.LogName, "twoCubePrefersSwitch", twoCubePrefersSwitch);

        // handle special scenarios before trying to parse game data
        if (position == Position.Special)
        {
            if (!switchE || !switchF)
            {
                if (switchC)
                {
                    return CrossBaseLine();
                }
                else
                {
                    return GetFillerRoutine();
                }
            }

            // Nuclear option...
            // nuclearOption = true;
            //isOpportunistic = true;
            //prefersSwitch = false;
            //twoCubeEnabled = true;
            //twoCubePrefersSwitch = false;
            if (switchC && !switchD)
            {
                position = Position.Left;
            }
            else if (!switchC && switchD)
            {
                position = Position.Right;
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
            return PlaceTwoCubesOnSwitchFromMiddle(isSwitchSideLeft);
            /*            if (twoCubeEnabled)
            {
                if (twoCubePrefersSwitch)
                {
                }
                else
                {
                    return PlaceCubeOnSwitchPrepScaleFromMiddle(isSwitchSideLeft, isScaleSideLeft);
                }
            }
            else
            {
                return PlaceCubeOnSwitchFromMiddleOnly(isSwitchSideLeft);
            }
            */
        }

        boolean isRobotLeft = position == Position.Left;
        if (isRobotLeft == isSwitchSideLeft && isRobotLeft == isScaleSideLeft)
        {
            // both on our side
            return PlaceCubeOnSameSideSwitch(isRobotLeft); // 1-cube switch
        }
        else if (isRobotLeft == isScaleSideLeft)
        {
            // scale on our side
            return CrossBaseLine();
        }
        else if (isRobotLeft == isSwitchSideLeft)
        {
            // switch on our side
            return PlaceTwoCubesOnSameSideSwitch(isRobotLeft);
        }
        else
        {
            // neither on our side
            return OpportunisticCrossCenter(isRobotLeft);
        }

        /*
        // handle left/right scenarios
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
                        return PlaceCubeOnSameSideSwitch(isRobotLeft);
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
                        return PlaceCubeOnSameSideScaleCollaborative(isRobotLeft);
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
                    return PlaceCubeOnSameSideScaleCollaborative(isRobotLeft);
                }
            }
        
            if (nuclearOption)
            {
                return NuclearOption(isRobotLeft);
            }
        
            if (isRobotLeft == isSwitchSideLeft)
            {
                if (twoCubeEnabled)
                {
                    if (twoCubePrefersSwitch)
                    {
                        return PlaceTwoCubesOnSameSideSwitch(isRobotLeft);
                    }
                    else
                    {
                        return PlaceCubeOnSameSideSwitchPrepCubeForFarScale(isRobotLeft);
                    }
                }
                else
                {
                    return PlaceCubeOnSameSideSwitch(isRobotLeft);
                }
            }
        
            //return CrossBaseLine(); 
            return OpportunisticCrossCenter(isRobotLeft);
        }
        else // "always" mode
        {
            if (prefersSwitch)
            {
                if (isRobotLeft == isSwitchSideLeft)
                {
                    if (twoCubeEnabled)
                    {
                        if (twoCubePrefersSwitch || isRobotLeft == isScaleSideLeft)
                        {
                            return PlaceTwoCubesOnSameSideSwitch(isRobotLeft);
                        }
                        else
                        {
                            return PlaceCubeOnSameSideSwitchPrepCubeForFarScale(isRobotLeft);
                        }
                    }
                    else
                    {
                        return PlaceCubeOnSameSideSwitch(isRobotLeft);
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
                        return PlaceCubeOnSameSideScaleCollaborative(isRobotLeft);
                    }
                }
                else // scale is on opposite side
                {
                    return PlaceCubeOnOppositeSideScale(isRobotLeft);
                }
            }
        }
        */
    }

    private IControlTask OpportunisticCrossCenter(boolean startingLeft)
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(true),
            SequentialTask.Sequence(
                new DriveDistanceTimedTask(215.0, 3.0),
                new NavxTurnTask(startingLeft ? 90.0 : -90.0),
                new DriveDistanceTimedTask(96.0, 2.0),
                new NavxTurnTask(startingLeft ? 90.0 : -90.0)));
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
                AutonomousRoutineSelector.DepositCubeOnly(false),
                AutonomousRoutineSelector.PostRoutineBackUp()));
    }

    private static IControlTask PlaceFirstCubeOfTwoOnSameSideSwitchOnly(boolean startingLeft)
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(false),
            SequentialTask.Sequence(
                new DriveDistanceTimedTask(148.0, 3.0), // 3.5s
                ConcurrentTask.AllTasks(
                    SequentialTask.Sequence(
                        new NavxTurnTask(true, startingLeft ? 90.0 : -90.0),
                        new DriveDistanceTimedTask(24.0, 1.0)), // 18.5"
                    new ElevatorMovementTask(
                        1.25,
                        Operation.ElevatorSwitchPosition)),
                AutonomousRoutineSelector.DepositCubeOnly(false),
                ConcurrentTask.AllTasks(
                    SequentialTask.Sequence(
                        new WaitTask(0.5),
                        ConcurrentTask.AllTasks(
                            new IntakeArmDownTask(0.5),
                            new ElevatorMovementTask(0.5, Operation.ElevatorCarryPosition))),
                    new DriveDistanceTimedTask(-10.0, 1.0))));
    }

    private static IControlTask PlaceTwoCubesOnSameSideSwitch(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            PlaceFirstCubeOfTwoOnSameSideSwitchOnly(startingLeft),
            new NavxTurnTask(true, 0),
            new DriveDistanceTimedTask(60.0, 1.5),
            new NavxTurnTask(true, startingLeft ? 135.0 : -135.0),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(24.0, 1.0),
                ConcurrentTask.AnyTasks(
                    SequentialTask.Sequence(
                        new WaitTask(1.0),
                        new DriveVelocityTimedTask(2.0, 0.0, 0.15)), // move forward a little until we have the cube
                    new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true))),
            ConcurrentTask.AnyTasks(
                new DriveDistanceTimedTask(-5.0, 0.75),
                new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true, 0.0)),
            ConcurrentTask.AllTasks(
                new IntakeArmUpTask(0.25),
                new ElevatorMovementTask(1.25, Operation.ElevatorSwitchPosition)),
            new NavxTurnTask(startingLeft ? 165.0 : -165.0),
            new DriveDistanceTimedTask(18.0, 1.0),
            AutonomousRoutineSelector.DepositCubeOnly(false),
            AutonomousRoutineSelector.PostRoutineBackUp());
    }

    private static IControlTask PlaceCubeOnSameSideSwitchPrepCubeForFarScale(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            PlaceFirstCubeOfTwoOnSameSideSwitchOnly(startingLeft),
            new NavxTurnTask(true, 0),
            new DriveDistanceTimedTask(60.0, 1.5),
            new NavxTurnTask(true, startingLeft ? 135.0 : -135.0),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(24.0, 1.0),
                ConcurrentTask.AnyTasks(
                    SequentialTask.Sequence(
                        new WaitTask(1.0),
                        new DriveVelocityTimedTask(2.0, 0.0, 0.15)), // move forward a little until we have the cube
                    new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true))),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(-24.0, 1.0),
                new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true, 0.0)),
            new NavxTurnTask(true, startingLeft ? 90.0 : -90.0),
            new IntakeArmUpTask(0.25));
    }

    private static IControlTask PlaceCubeOnSameSideScaleCollaborative(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            //new DriveDistanceTimedTask(24.0, 0.5),
            //new NavxTurnTask(true, startingLeft ? -3.1 : 3.1),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new DriveDistanceTimedTask(304.0, 3.75),
                    new NavxTurnTask(startingLeft ? 90.0 : -90.0),
                    new DriveDistanceTimedTask(8.0, 0.5)),
                SequentialTask.Sequence(
                    new WaitTask(3.5),
                    new ElevatorMovementTask(1.5, Operation.ElevatorHighScalePosition))),
            ConcurrentTask.AnyTasks(
                new PIDBrakeTask(),
                new OuttakeTask(1.0, true)),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(-28.0, 1.5),
                SequentialTask.Sequence(
                    new WaitTask(1.25),
                    ConcurrentTask.AllTasks(
                        new IntakeArmDownTask(0.5),
                        new ElevatorMovementTask(0.5, Operation.ElevatorCarryPosition)))),
            new NavxTurnTask(startingLeft ? 45.0 : -45.0),
            new PIDBrakeTask());
    }

    private static IControlTask PlaceFirstOfTwoCubesOnSameSideScale(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new DriveDistanceTimedTask(248.00, 3.0), //255.25
                    new NavxTurnTask(startingLeft ? 45.0 : -45.0),
                    new DriveDistanceTimedTask(14.5, 0.5)),
                SequentialTask.Sequence(
                    AutonomousRoutineSelector.InitialSetUp(true),
                    new WaitTask(2.75),
                    new ElevatorMovementTask(1.75, Operation.ElevatorHighScalePosition))),
            ConcurrentTask.AnyTasks(
                new PIDBrakeTask(),
                new OuttakeTask(0.75, true)),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(-14.5, 0.5),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    ConcurrentTask.AllTasks(
                        new IntakeArmDownTask(0.25),
                        new ElevatorMovementTask(0.5, Operation.ElevatorCarryPosition)))));
    }

    private static IControlTask PlaceTwoCubesOnSameSideScale(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            PlaceFirstOfTwoCubesOnSameSideScale(startingLeft),
            new NavxTurnTask(false, startingLeft ? 150.0 : -150.0),
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(70.0, 1.4), // 76.5
                SequentialTask.Sequence(
                    new WaitTask(0.75),
                    ConcurrentTask.AnyTasks(
                        SequentialTask.Sequence(
                            new WaitTask(0.65),
                            new DriveVelocityTimedTask(1.0, 0.0, 0.15)), // move forward a little until we have the cube
                        new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true)))),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new NavxTurnTask(false, startingLeft ? 150.0 : -150.0),
                    new DriveDistanceTimedTask(-72.5, 1.5)), // 76.5, 1.75
                new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true)),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new NavxTurnTask(startingLeft ? 45.0 : -45.0),
                    new DriveDistanceTimedTask(14.5, 0.5)), //1s
                new ElevatorMovementTask(1.75, Operation.ElevatorHighScalePosition)),
            ConcurrentTask.AnyTasks(
                new PIDBrakeTask(),
                new OuttakeTask(1.0, true)),
            AutonomousRoutineSelector.PostRoutineBackUp());
    }

    private static IControlTask PlaceCubesOnSameSideScaleAndSwitch(boolean startingLeft)
    {
        return SequentialTask.Sequence(
            PlaceFirstOfTwoCubesOnSameSideScale(startingLeft),
            new NavxTurnTask(startingLeft ? 150.0 : -150.0), // 150.0
            ConcurrentTask.AllTasks(
                new DriveDistanceTimedTask(70.0, 1.4), // 76.5
                SequentialTask.Sequence(
                    new WaitTask(0.75),
                    ConcurrentTask.AnyTasks(
                        SequentialTask.Sequence(
                            new WaitTask(0.65),
                            new DriveVelocityTimedTask(2.0, 0.0, 0.15)), // move forward a little until we have the cube
                        new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true)))),
            ConcurrentTask.AllTasks(
                new IntakeArmUpTask(0.25),
                new ElevatorMovementTask(1.25, Operation.ElevatorSwitchPosition),
                SequentialTask.Sequence(
                    new DriveDistanceTimedTask(-12.0, 1.0),
                    new NavxTurnTask(startingLeft ? 160.0 : -160.0),
                    new DriveDistanceTimedTask(24.0, 1.0))),
            ConcurrentTask.AnyTasks(
                new PIDBrakeTask(),
                new OuttakeTask(1.0, true)), // 1.0
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
                    new DriveDistanceTimedTask(67.0, 1.5), // 18.0
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
                ConcurrentTask.AllTasks(
                    new NavxTurnTask(startingLeft ? -45.0 : 45.0),
                    new ElevatorMovementTask(1.0, Operation.ElevatorHighScalePosition)),
                new DriveDistanceTimedTask(14, 0.75), //12in, .75s
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
            AutonomousRoutineSelector.DepositCubeOnly(false),
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
                    new WaitTask(1.0),
                    new IntakeArmUpTask(0.25),
                    new ElevatorMovementTask(0.5, Operation.ElevatorSwitchPosition)),
                SequentialTask.Sequence(
                    new DriveDistanceTimedTask(-24.0, 1.25),
                    new NavxTurnTask(false, 0.0),
                    new DriveDistanceTimedTask(24.0, 1.0))), //1.25s
            AutonomousRoutineSelector.DepositCubeOnly(false),
            AutonomousRoutineSelector.PostRoutineBackUp());
    }

    private static IControlTask PlaceCubeOnSwitchPrepScaleFromMiddle(boolean switchIsLeft, boolean scaleIsLeft)
    {
        if (switchIsLeft == scaleIsLeft)
        {
            return SequentialTask.Sequence(
                PlaceCubeOnSwitchFromMiddleOnly(switchIsLeft),
                new NavxTurnTask(switchIsLeft ? 67.25 : -67.25),
                ConcurrentTask.AllTasks(
                    new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true),
                    new DriveDistanceTimedTask(45.0, 1.25)),
                new DriveDistanceTimedTask(-24.0, 0.75),
                new NavxTurnTask(true, switchIsLeft ? -90.0 : 90.0),
                new DriveDistanceTimedTask(70.0, 1.5),
                new NavxTurnTask(true, 0.0),
                new DriveDistanceTimedTask(150.0, 2.5));
        }
        else
        {
            return SequentialTask.Sequence(
                PlaceCubeOnSwitchFromMiddleOnly(switchIsLeft),
                new NavxTurnTask(switchIsLeft ? 67.25 : -67.25),
                ConcurrentTask.AllTasks(
                    new AdvancedIntakeOuttakeTask(Operation.ElevatorIntake, true),
                    new DriveDistanceTimedTask(45.0, 1.25)),
                new DriveDistanceTimedTask(-24.0, 0.75),
                new NavxTurnTask(true, 0.0),
                new DriveDistanceTimedTask(-50.0, 1.5),
                new NavxTurnTask(true, scaleIsLeft ? -90.0 : 90.0),
                new DriveDistanceTimedTask(160.0, 2.5),
                new NavxTurnTask(true, 0.0),
                new DriveDistanceTimedTask(160.0, 2.5));
        }
    }

    private static IControlTask CrossBaseLine()
    {
        return ConcurrentTask.AllTasks(
            AutonomousRoutineSelector.InitialSetUp(true),
            new DriveDistancePositionTimedTask(0.5, 145.0, 5.0));
    }

    private static IControlTask NuclearOption(boolean isRobotLeft)
    {
        return SequentialTask.Sequence(
            new DriveDistanceTimedTask(318.0, 5.0),
            ConcurrentTask.AnyTasks(
                new PIDBrakeTask(),
                new WaitTask(7.0)),
            new NavxTurnTask(true, 0.0),
            new DriveDistanceTimedTask(-70.0, 1.5));
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
                new OuttakeTask(1.0, true)));
    }

    private static IControlTask DepositCubeOnly(boolean isScale)
    {
        return ConcurrentTask.AnyTasks(
            new PIDBrakeTask(),
            new OuttakeTask(isScale ? 1.0 : 1.0, true));
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
