import pybullet as p
import time

p.connect(p.GUI)
p.loadURDF("data/plane.urdf")
p.setGravity(0, 0, -10)
copter = p.loadURDF("data/quadrotor.urdf")
p.resetBasePositionAndOrientation(copter, [0, 0, 1], [0, 0, 0, 1])
p.changeDynamics(copter, -1, linearDamping=0.9)

color = [0.4, 0.4, 0.4, 1]
contactColor = [1., 0., 0., 1]

forwardVec = [2, 0, 0]
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
