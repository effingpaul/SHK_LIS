import numpy as np
import robotic as ry
from typing import Tuple
from RobotEnviroment.robotMovement import moveBlocking, moveBlockingAndCheckForce

STANDARD_VELOCITY = .2


def standardKomo(C: ry.Config, phases: int, slicesPerPhase: int=20) -> ry.KOMO:

    q_now = C.getJointState()

    komo = ry.KOMO()
    komo.setConfig(C, True)

    komo.setTiming(phases, slicesPerPhase, 1., 2)

    komo.addControlObjective([], 0, 1e-2)
    komo.addControlObjective([], 1, 1e-1)
    komo.addControlObjective([], 2, 1e1)

    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    return komo


def giveRandomAllowedAngle(allowedSegments: [[float]]) -> float:

    # This could be made better I think
    total_len = 0
    for seg in allowedSegments:
        total_len += seg[1]-seg[0]
    nonAdjustedAngle = np.random.random()*total_len

    nonAdjustedAngle += allowedSegments[0][0]
    for i, seg in enumerate(allowedSegments[:-1]):
        if nonAdjustedAngle >= seg[0] and nonAdjustedAngle <= seg[1]:
            return nonAdjustedAngle
        nonAdjustedAngle += allowedSegments[i+1][0]-seg[1]
    
    return nonAdjustedAngle


def pushMotionWaypoints(point: np.ndarray,
                        normal: np.ndarray,
                        robot_pos: np.ndarray,
                        start_dist: float=.15,
                        end_dist_range: [float]=[.1, .15],
                        initial_elevation: float=.15,
                        config: ry.Config=None,
                        minHeight: float=.7) -> [np.ndarray]:

    robot2point = point-robot_pos
    dir = -1. if np.inner(robot2point, normal) < 0 else 1.
    start_pos = normal * start_dist * dir
    end_dist = np.random.rand() * (end_dist_range[1]-end_dist_range[0]) + end_dist_range[0]
    end_pos = normal * end_dist * dir
    initial = point.copy()+start_pos.copy()
    initial[2] += initial_elevation
    waypoints = [initial, point+start_pos, point, point-end_pos]
    if waypoints[-1][2] < minHeight:
        waypoints[-1][2] = minHeight
    
    if config:
        for i, w in enumerate(waypoints):
            way = config.getFrame(f"way{i}")

            if not way:
                way = config.addFrame(f"way{i}") \
                .setShape(ry.ST.marker, size=[.1])
                if i == 0:
                    way.setColor([1, 0, 0])
                elif i == len(waypoints)-1:
                    way.setColor([0, 0, 1])
                else:
                    way.setColor([0, 1, 0])

            way.setPosition(w)

    return waypoints


def specialPush(bot: ry.BotOp,
                C: ry.Config,
                direction: np.ndarray,
                velocity: float=STANDARD_VELOCITY,
                verbose: int=0) -> Tuple[bool, float]:
    
    """
    Creates a motion problem using "waypoint engineering" approach: define waypoints and motion relative to these
    - We assume the direction vector is normalised.
    """
    komo = standardKomo(C, 2)

    mat = np.eye(3) - np.outer(direction, direction)

    komo.addObjective([0, 1], ry.FS.negDistance, ['l_gripper', 'pushWayPoint'], ry.OT.ineq, [1], [-.1])
    komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', 'startWayPoint'], ry.OT.eq, [1e1])
    komo.addObjective([1, 2], ry.FS.positionDiff, ['l_gripper', 'startWayPoint'], ry.OT.eq, mat)

    komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', 'endWayPoint'], ry.OT.eq, [1e1])

    komo.addObjective([2], ry.FS.qItself, [], ry.OT.eq, [1e1], [], 1) # No motion derivative of q vector ergo the velocity = 0

    komo.addObjective([1, 2], ry.FS.vectorX, ['l_gripper'], ry.OT.eq, direction.reshape(1, 3))
    komo.addObjective([1, 2], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1], -direction)

    success, maxForce = moveBlockingAndCheckForce(bot, C, komo, velocity, verbose=verbose)

    return success, maxForce


def moveToInitialPushPoint(bot: ry.BotOp,
                           C: ry.Config,
                           initialPoint: np.ndarray,
                           direction: np.ndarray,
                           velocity: float=STANDARD_VELOCITY,
                           verbose: int=0) -> bool:

    komo = standardKomo(C, 1)

    komo.addObjective([1.], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], -direction)
    komo.addObjective([1.], ry.FS.scalarProductXZ, ['l_gripper', 'table'], ry.OT.eq, [1e1], [0.])
    komo.addObjective([1.], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.])

    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], initialPoint)
    
    return moveBlocking(bot, C, komo, velocity, verbose=verbose)


def moveThroughPushPath(bot: ry.BotOp,
                        C: ry.Config,
                        waypoints: [np.ndarray],
                        direction: np.ndarray,
                        velocity: float=.2,
                        verbose: int=0) -> Tuple[bool, float]:
    
    # We assume that the gripper is already at the starting waypoint and with the correct rotation
    komo = standardKomo(C, len(waypoints)-1)

    komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], -direction)
    komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', 'table'], ry.OT.eq, [1e1], [0.])
    komo.addObjective([], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.])

    for i, way in enumerate(waypoints[1:]):
        komo.addObjective([i+1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], way)
    
    success, maxForce = moveBlockingAndCheckForce(bot, C, komo, velocity, verbose=verbose)

    return success, maxForce


def moveBackAfterPush(bot: ry.BotOp,
                      C: ry.Config,
                      waypoints: [np.ndarray],
                      direction: np.ndarray,
                      velocity: float=.2,
                      verbose: int=0) -> bool:
    
    komo = standardKomo(C, len(waypoints)-1)

    # We assume that the gripper is already at the starting waypoint and with the correct rotation
    komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], -direction)
    komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', 'table'], ry.OT.eq, [1e1], [0.])
    komo.addObjective([], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.])

    for i in range(len(waypoints)-1):
        index = len(waypoints)-2 - i
        komo.addObjective([i+1], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[index]) 

    return moveBlocking(bot, C, komo, velocity, verbose=verbose)


def doPushThroughWaypoints(C: ry.Config,
                           bot: ry.BotOp,
                           ways: [np.ndarray],
                           verbose: int=0,
                           velocity: float=.2) -> Tuple[bool, float]:

    move_dir = ways[-1]-ways[1]
    pathLen = np.linalg.norm(move_dir) # Could use this to calculate the time the robot has to move for uniform velocities.
    move_dir /= pathLen

    ### GO TO INITIAL PUSH POINT ###
    success = moveToInitialPushPoint(bot, C,
                                     ways[0], move_dir,
                                     velocity=velocity, verbose=verbose)
    if not success:
        return False, .0
    
    ### EXECUTE PUSH ###
    success, maxForce = moveThroughPushPath(bot, C,
                                  ways, move_dir,
                                  velocity=velocity, verbose=verbose)
    if not success:
        return False, .0
    
    ### MOVE BACK ###
    # This is done so that the object is not disturbed after the push action
    success = moveBackAfterPush(bot, C,
                                  ways, move_dir,
                                  velocity=velocity, verbose=verbose)
    if not success:
        return False, .0
    
    return True, maxForce


def pokePoint(bot: ry.BotOp,
              C: ry.Config,
              point: np.ndarray,
              velocity: float=.1,
              maxPush: float=.02,
              verbose: int=0) -> Tuple[bool, float]:

    waypoints = [
        point + np.array([.0, .0, .1]),
        point + np.array([.0, .0, maxPush]),
        point - np.array([.0, .0, maxPush]),
    ]

    # Go to starting position
    komo = standardKomo(C, 1)

    komo.addObjective([1.], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [0., 0., 1.])
    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[0])

    success = moveBlocking(bot, C, komo, velocity, verbose=verbose)
    if not success:
        print("Failed getting to initial poking position!")
        return False, .0

    # Poke
    komo = standardKomo(C, 2)

    komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [0., 0., 1.])
    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[1])
    komo.addObjective([2.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[2])

    success, resultingForce = moveBlockingAndCheckForce(bot, C, komo, velocity, maxForceAllowed=1, verbose=verbose)
    if not success:
        print("Failed poking object!")
        return False, .0

    # Go back up
    komo = standardKomo(C, 2)

    komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [0., 0., 1.])
    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[1])
    komo.addObjective([2.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], waypoints[0])

    success = moveBlocking(bot, C, komo, velocity, verbose=verbose)
    if not success:
        print("Failed moving back from poking!")
        return False, .0

    return True, resultingForce
