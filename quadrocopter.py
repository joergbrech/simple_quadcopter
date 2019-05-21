import pybullet as p
import time


def tunnel(length, width, wallWidth = 0.05, position=[0., 0., 0.], orientation=[0., 0., 0., 1.]):
    # create a rectangular tunnel as a building block of an obstacle parcour
    for i in [0,1]:

        batchPositions = []

        if i % 2 == 0:

            halfExtents=[length / 2, width / 2 + wallWidth, wallWidth / 2]

            # lower wall
            batchPositions.append([length / 2, 0., -width / 2 - wallWidth / 2])

            # upper wall
            batchPositions.append([length / 2, 0., width / 2 + wallWidth / 2])
        else:

            halfExtents = [length / 2, wallWidth / 2, width / 2]

            # left wall
            batchPositions.append([length / 2, -width / 2 - wallWidth / 2, 0.])

            # right wall
            batchPositions.append([length / 2, width / 2 + wallWidth / 2, 0.])

        w = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)
        v = p.createVisualShape(shapeType=p.GEOM_BOX,
                                halfExtents=halfExtents,
                                rgbaColor=[1, 1, 1, 0.5],
                                specularColor=[0.4, .4, 0])

        b = p.createMultiBody(baseMass=0,
                              baseInertialFramePosition=[0, 0, 0],
                              baseCollisionShapeIndex=w,
                              baseVisualShapeIndex=v,
                              basePosition=[0., 0., 0.],
                              baseOrientation=[0., 0., 0., 1.],
                              batchPositions=batchPositions,
                              useMaximalCoordinates=True)
        p.resetBasePositionAndOrientation(b, position, orientation)


def corner(width, wallWidth=0.05, position=[0., 0., 0.], orientation=[0., 0., 0., 1.]):
    # create a corner (cube open at neg. x and pos. z) as a building block of an obstacle parcour
    # TODO
    pass


def parcour():
    # create tunnel that starts at (-1,0,0)
    tunnel(length=5., width=1, position=[-1., 0., 0.])


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
