import pybullet as pb
import pybullet_data
from transformation import Transformation
from time import sleep

from src.soccerbot import Soccerbot
from src.ramp import Ramp
import time
import matplotlib as plt

def main():
    plt.use('tkagg')

    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    soccerbot = Soccerbot([0, 0, 0], useFixedBase=False)
    ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0))  # change where the plane is lol ymes NO this will make things much harder in the future, move the robot not the floor
    pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.1])
    pb.setGravity(0, 0, -9.81)
    t1 = time.perf_counter()
    soccerbot.stand()
    soccerbot.getPath(Transformation([0.3, 0, 0]), show=True)
    t2 = time.perf_counter()
    print("duration: " + str(t2 - t1))
    soccerbot.calculate_angles(show=True)


    startup = True
    warmup = True
    # Step through simulation
    if startup:
        for i in range(100):
            sleep(0.004)
            pb.stepSimulation()
        print("Startup done")


    #step = soccerbot.step_sim(0.004) #true speed
    step = soccerbot.step_sim(sim_time_step=0.004, initial=0)

    if warmup:
        #next(step)
        for i in range(100):
            sleep(0.004)
            pb.stepSimulation()
        print("Warmup done")


    while (1):
        sleep(0.004)
        #sleep(0.01)
        pb.stepSimulation()
        next(step)

if __name__ == '__main__':
    main()