import pybullet as p
import time
from parcour import parcour


if __name__ == '__main__':

    # initialize GUI and physics
    p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    # initialize the  quadrocopter and place into the world
    copter = p.loadURDF("data/quadrotor.urdf")
    p.resetBasePositionAndOrientation(copter, [0, 0, 2.], [0, 0, 0, 1])
    p.changeDynamics(copter, -1, linearDamping=0.9)

    # create parcour
    parcour()

    # quadcopter color
    color = [0.4, 0.4, 0.4, 1]
    contactColor = [1., 0., 0., 1]

    # create some sliders to manipulate the camera view
    cameraPitchSlider = p.addUserDebugParameter("cameraPitch", -90, 90, -35)
    cameraDistanceSlider = p.addUserDebugParameter("cameraDistance", 0.5, 100, 5)
    cameraYawSlider = p.addUserDebugParameter("cameraYaw", 0, 360, 35)

    # initialize force variables for the four rotors
    baseForceSlider = p.addUserDebugParameter("baseForce", 0, 5, 0)
    force1Diff = 0.
    force2Diff = 0.
    force3Diff = 0.
    force4Diff = 0.

    # simulation loop
    while True:

        pos, orn = p.getBasePositionAndOrientation(copter)

        # reset camera position
        cameraTargetPosition = pos
        cameraPitch = p.readUserDebugParameter(cameraPitchSlider)
        cameraDistance = p.readUserDebugParameter(cameraDistanceSlider)
        cameraYaw = p.readUserDebugParameter(cameraYawSlider)
        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

        # read user input to stear quadrocopter
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

        # apply force to the four rotors
        baseForce = p.readUserDebugParameter(baseForceSlider)
        force1 = [0., 0., force1Diff + baseForce]
        force2 = [0., 0., force2Diff + baseForce]
        force3 = [0., 0., force3Diff + baseForce]
        force4 = [0., 0., force4Diff + baseForce]

        p.applyExternalForce(copter, -1, force1, [0, 2.5, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(copter, -1, force2, [0, -2.5, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(copter, -1, force3, [2.5, 0, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(copter, -1, force4, [-2.5, 0, 0], flags=p.LINK_FRAME)

        # change color of copter in case of collision
        if p.getContactPoints(copter):
            p.changeVisualShape(copter, -1, rgbaColor=contactColor)
        else:
            p.changeVisualShape(copter, -1, rgbaColor=color)

        # advance the simulation
        p.stepSimulation()
        time.sleep(1. / 240.)
