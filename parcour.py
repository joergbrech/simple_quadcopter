import pybullet as p
import math


def tunnel(length, width, wall_width = 0.05, position=[0., 0., 0.], orientation=[0., 0., 0., 1.]):
    # create a rectangular tunnel as a building block of an obstacle parcour
    wall1_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length / 2, width / 2 + wall_width, wall_width / 2])
    wall2_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length / 2, width / 2 + wall_width, wall_width / 2])
    wall3_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length / 2, wall_width / 2, width / 2])
    wall4_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length / 2, wall_width / 2, width / 2])

    visualShapeId = p.createVisualShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                             halfExtents=[[length / 2, width / 2 + wall_width, wall_width / 2],
                                                          [length / 2, width / 2 + wall_width, wall_width / 2],
                                                          [length / 2, wall_width / 2, width / 2],
                                                          [length / 2, wall_width / 2, width / 2]],
                                             visualFramePositions=[[0, 0., -width / 2 - wall_width / 2],
                                                                   [0, 0., width / 2 + wall_width / 2],
                                                                   [0, -width / 2 - wall_width / 2, 0.],
                                                                   [0, width / 2 + wall_width / 2, 0.]]
                                             )
    collisionShapeId = p.createCollisionShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                                   halfExtents=[[length / 2, width / 2 + wall_width, wall_width / 2],
                                                                [length / 2, width / 2 + wall_width, wall_width / 2],
                                                                [length / 2, wall_width / 2, width / 2],
                                                                [length / 2, wall_width / 2, width / 2]],
                                                   collisionFramePositions=[[0., 0., -width / 2 - wall_width / 2],
                                                                            [0., 0., width / 2 + wall_width / 2],
                                                                            [0., -width / 2 - wall_width / 2, 0.],
                                                                            [0., width / 2 + wall_width / 2, 0.]]
                                                   )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collisionShapeId,
                           baseVisualShapeIndex=visualShapeId,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb




def corner(width, wall_width=0.05, position=[0., 0., 0.], orientation=[0., 0., 0., 1.]):
    # create a corner (cube open at neg. x and pos. z) as a building block of an obstacle parcour
    # TODO
    pass


def parcour():
    # create an obstacle parcour

    # width of the tunnels
    width = 1

    ornx = [0., 0., 0., 1.]
    orny = p.getQuaternionFromAxisAngle(axis=[0, 0, 1], angle=math.pi / 2)
    ornz = p.getQuaternionFromAxisAngle(axis=[0, 1, 0], angle=math.pi / 2)

    tunnel(length=5., width=width, position=[1.5, 0., 2.], orientation=ornx)
    #corner(width=width, position=[4., 0., 0.])
    tunnel(length=2., width=width, position=[4.5, 0.0, 3.5], orientation=ornz)
    #corner(width=width, position=[4., 0., 0.])
    tunnel(length=4., width=width, position=[4.5, 2.5, 5], orientation=orny)
    #corner(width=width, position=[4., 0., 0.])
    tunnel(length=4., width=width, position=[7.0, 5.0, 5.], orientation=ornx)