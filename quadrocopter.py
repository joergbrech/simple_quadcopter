import pybullet as p
import time
import math


def tunnel(length, width, wallWidth = 0.05, position=[0., 0., 0.], orientation=[0., 0., 0., 1.]):
    # create a rectangular tunnel as a building block of an obstacle parcour
    wall1_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length / 2, width / 2 + wallWidth, wallWidth / 2])
    wall2_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length / 2, width / 2 + wallWidth, wallWidth / 2])
    wall3_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length / 2, wallWidth / 2, width / 2])
    wall4_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length / 2, wallWidth / 2, width / 2])

    visualShapeId = p.createVisualShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                             halfExtents=[[length / 2, width / 2 + wallWidth, wallWidth / 2],
                                                          [length / 2, width / 2 + wallWidth, wallWidth / 2],
                                                          [length / 2, wallWidth / 2, width / 2],
                                                          [length / 2, wallWidth / 2, width / 2]],
                                             visualFramePositions=[[length / 2, 0., -width / 2 - wallWidth / 2],
                                                                   [length / 2, 0., width / 2 + wallWidth / 2],
                                                                   [length / 2, -width / 2 - wallWidth / 2, 0.],
                                                                   [length / 2, width / 2 + wallWidth / 2, 0.]]
                                             )
    collisionShapeId = p.createCollisionShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                                   halfExtents=[[length / 2, width / 2 + wallWidth, wallWidth / 2],
                                                                [length / 2, width / 2 + wallWidth, wallWidth / 2],
                                                                [length / 2, wallWidth / 2, width / 2],
                                                                [length / 2, wallWidth / 2, width / 2]],
                                                   collisionFramePositions=[[length / 2, 0., -width / 2 - wallWidth / 2],
                                                                            [length / 2, 0., width / 2 + wallWidth / 2],
                                                                            [length / 2, -width / 2 - wallWidth / 2, 0.],
                                                                            [length / 2, width / 2 + wallWidth / 2, 0.]]
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




def corner(width, wallWidth=0.05, position=[0., 0., 0.], orientation=[0., 0., 0., 1.]):
    # create a corner (cube open at neg. x and pos. z) as a building block of an obstacle parcour
    # TODO
    pass


def parcour():
    # width of the tunnels
    width = 1

    ornx = [0., 0., 0., 1.]
    orny = p.getQuaternionFromAxisAngle(axis=[0, 0, 1], angle=math.pi / 2)
    ornz = p.getQuaternionFromAxisAngle(axis=[0, 1, 0], angle=math.pi / 2)

    tunnel(length=5., width=width, position=[-1, 0., 0.], orientation=ornx)
    #corner(width=width, position=[4., 0., 0.])
    tunnel(length=2., width=width, position=[4.5, 0.0, 2.5], orientation=ornz)
    #corner(width=width, position=[4., 0., 0.])
    tunnel(length=4., width=width, position=[4.5, 0.5, 3.], orientation=orny)
    #corner(width=width, position=[4., 0., 0.])
    tunnel(length=4., width=width, position=[5.0, 5.0, 3.], orientation=ornx)



if __name__ == '__main__':
    p.connect(p.GUI)
    p.setGravity(0, 0, -10)
    copter = p.loadURDF("data/quadrotor.urdf")
    p.resetBasePositionAndOrientation(copter, [0, 0, 0], [0, 0, 0, 1])
    p.changeDynamics(copter, -1, linearDamping=0.9)

    # create parcour
    parcour()

    # quadcopter color
    color = [0.4, 0.4, 0.4, 1]
    contactColor = [1., 0., 0., 1]

    cameraDistance = 1
    cameraYaw = 35
    cameraPitch = -35

    cameraDistanceSlider = p.addUserDebugParameter("cameraDistance",0.5,100,1)
    cameraYawSlider = p.addUserDebugParameter("cameraYaw", 0, 360, 35)

    baseForceSlider = p.addUserDebugParameter("baseForce", 0, 5, 0)
    force1 = [0, 0, 0]
    force2 = [0, 0, 0]
    force3 = [0, 0, 0]
    force4 = [0, 0, 0]
    force1Diff = 0.
    force2Diff = 0.
    force3Diff = 0.
    force4Diff = 0.

    while True:

        pos, orn = p.getBasePositionAndOrientation(copter)

        cameraTargetPosition = pos
        cameraDistance = p.readUserDebugParameter(cameraDistanceSlider)
        cameraYaw = p.readUserDebugParameter(cameraYawSlider)
        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

        keys = p.getKeyboardEvents()
        for k, v in keys.items():

            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                force1Diff = 0.25
            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED):
                force1Diff = 0.
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                force2Diff = 0.25
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED):
                force2Diff = 0.

            if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED):
                force3Diff = 0.25
            if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED):
                force3Diff = 0.
            if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED):
                force4Diff = 0.25
            if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED):
                force4Diff = 0.

        baseForce = p.readUserDebugParameter(baseForceSlider)
        force1 = [0., 0., force1Diff + baseForce]
        force2 = [0., 0., force2Diff + baseForce]
        force3 = [0., 0., force3Diff + baseForce]
        force4 = [0., 0., force4Diff + baseForce]

        p.applyExternalForce(copter, -1, force1, [0, 2.5, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(copter, -1, force2, [0, -2.5, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(copter, -1, force3, [2.5, 0, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(copter, -1, force4, [-2.5, 0, 0], flags=p.LINK_FRAME)

        if p.getContactPoints(copter):
            p.changeVisualShape(copter, -1, rgbaColor=contactColor)
        else:
            p.changeVisualShape(copter, -1, rgbaColor=color)

        p.stepSimulation()
        time.sleep(1. / 240.)
