import pybullet as p
import math


def tunnel(length, width, wall_width = 0.05, position=[0., 0., 0.], orientation=[0., 0., 0., 1.]):
    # create a rectangular tunnel as a building block of an obstacle parcour

    half_extents = [[length / 2, width / 2 + wall_width, wall_width / 2],
                    [length / 2, width / 2 + wall_width, wall_width / 2],
                    [length / 2, wall_width / 2, width / 2],
                    [length / 2, wall_width / 2, width / 2]]
    positions = [[0, 0., -width / 2 - wall_width / 2],
                 [0, 0., width / 2 + wall_width / 2],
                 [0, -width / 2 - wall_width / 2, 0.],
                [0, width / 2 + wall_width / 2, 0.]]

    visual_shape_id = p.createVisualShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                               halfExtents=half_extents,
                                               visualFramePositions=positions
                                               )
    collision_shape_id = p.createCollisionShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                                     halfExtents=half_extents,
                                                     collisionFramePositions=positions
                                                     )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collision_shape_id,
                           baseVisualShapeIndex=visual_shape_id,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb


def corner(width, wall_width=0.05, position=[0., 0., 0.], orientation=[0., 0., 0., 1.]):
    # create a corner (cube open at neg. x and pos. z) as a building block of an obstacle parcour
    half_extents = [[width / 2, width / 2 + wall_width, wall_width / 2],
                    [width / 2, wall_width / 2, width / 2],
                    [width / 2, wall_width / 2, width / 2],
                    [wall_width / 2, width / 2 + wall_width, width / 2]]
    positions = [[0, 0., -width / 2 - wall_width / 2],
                 [0, -width / 2 - wall_width / 2, 0.],
                 [0, width / 2 + wall_width / 2, 0.],
                 [width/2 + wall_width / 2, 0., 0.]]

    visual_shape_id = p.createVisualShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                               halfExtents=half_extents,
                                               visualFramePositions=positions
                                               )
    collision_shape_id = p.createCollisionShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                                     halfExtents=half_extents,
                                                     collisionFramePositions=positions
                                                     )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collision_shape_id,
                           baseVisualShapeIndex=visual_shape_id,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb


def cap(width, wall_width=0.05, position=[0, 0, 0,], orientation=[0, 0, 0]):
    # create a cap that can be used to close corners or tunnels.
    # By default it is a wall in the y-z-plane that can be rotated and translated to the desired position
    # create a corner (cube open at neg. x and pos. z) as a building block of an obstacle parcour
    half_extents = [wall_width / 2, width / 2 + wall_width, width / 2]
    pos = [0, 0., 0.]
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                          halfExtents=half_extents,
                                          visualFramePosition=pos
                                          )
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=half_extents,
                                                collisionFramePosition=pos
                                                )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collision_shape_id,
                           baseVisualShapeIndex=visual_shape_id,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb


def parcour():
    # create an obstacle parcour

    # width of the tunnels
    width = 1

    # create some orientation quaternions to rotate tunnels and corners
    tunnel_ornx = [0., 0., 0., 1.]
    tunnel_orny = p.getQuaternionFromAxisAngle(axis=[0, 0, 1], angle=math.pi / 2)
    tunnel_ornz = p.getQuaternionFromAxisAngle(axis=[0, 1, 0], angle=math.pi / 2)
    corner_negxposz = [0., 0., 0., 1.]
    corner_negzposy = p.getQuaternionFromEuler([math.pi, 0.0, -math.pi / 2])
    corner_negyposx = p.getQuaternionFromEuler([math.pi/2, 0.0, math.pi / 2])

    # create the parcour
    cap(width=width, position=[-1., 0., 2.])
    tunnel(length=5., width=width, position=[1.5, 0., 2.], orientation=tunnel_ornx)
    corner(width=width, position=[4.5, 0., 2.], orientation=corner_negxposz)
    tunnel(length=2., width=width, position=[4.5, 0.0, 3.5], orientation=tunnel_ornz)
    corner(width=width, position=[4.5, 0., 5.0], orientation=corner_negzposy)
    tunnel(length=4., width=width, position=[4.5, 2.5, 5], orientation=tunnel_orny)
    corner(width=width, position=[4.5, 5., 5.], orientation=corner_negyposx)
    tunnel(length=4., width=width, position=[7.0, 5.0, 5.], orientation=tunnel_ornx)
    cap(width=width, position=[9., 5., 5.])
