
from aerosandbox import numpy as np
from trajopt.problems import AircraftTrajectoryProblem2D as TrajProb
from cruiseExample import cruiseProblemTime

def howdoIwarmstart():
    """
    trying to figure out how to warm start different problems
    """

    # define problem grid
    time = np.linspace(0,15,101)

    # define two separate problems and set up each
    problem = TrajProb()
    gust_vel = problem.parameter(0)
    cruiseProblemTime(problem,time,gust_model_velocity=gust_vel)
    


    # solve problem 1
    sol = problem.solve()
    # change something about setup, run problem 1 again from existing solution
    problem.set_initial_from_sol(problem.CurrentSolution)
    problem.set_value(gust_vel,40)
    problem.solve()

    return problem


if __name__ == "__main__":
    from trajopt.dynamics import visualizeRun2D
    from matplotlib import pyplot as plt
    problem = howdoIwarmstart()

    fig_dict = visualizeRun2D(problem.Time,problem.LastSolution(problem.PhysicsModel),fig_dict=None)
    fig_dict = visualizeRun2D(problem.Time,problem.CurrentSolution(problem.PhysicsModel),fig_dict=fig_dict,case='2',casenum=1)

    plt.show()