# Continuous Control

## Introduction

In this project, the [Reacher][Reacher Environment] environment.

![Trained Agent][Trained Agent]

In this environment, a double-jointed arm can move to target locations. 
A reward of +0.1 is provided for each step that the agent's hand is in the goal location.
Thus, the goal of your agent is to **maintain its position at the target location for as many time steps as possible**.

The observation space consists of 33 variables corresponding to position, rotation, velocity, and angular velocities
of the arm.
Each action is a vector with 4 numbers, corresponding to torque applicable to two joints.
Every entry in the action vector should be a **number between -1 and 1**.

## Distributed Training

The environment contains 20 identical agents, each with its own copy of the environment.  
This is useful for algorithms like [PPO](https://arxiv.org/pdf/1707.06347.pdf),
[A3C](https://arxiv.org/pdf/1602.01783.pdf),
and [D4PG](https://openreview.net/pdf?id=SyZipzbCb) that use multiple (non-interacting, parallel) copies
of the same agent to distribute the task of gathering experience.  

## Solving the Environment

The barrier for solving the environment is to take into account the presence of many agents.
In particular, your agents must get an average score of +30 (over 100 consecutive episodes, and over all agents):

- After each episode, add up the rewards that each agent received (without discounting), to get a score for each agent. 
This yields 20 potentially different scores.
Then, take the average of these 20 scores. 
- This yields an **average score** for each episode (where the average is over all 20 agents).

The environment is considered solved, when the **average (over 100 episodes) of those average scores is at least +30**. 

## Getting Started

1. Download the environment from one of the links below. 
You need only select the environment that matches your operating system:

    - [Linux][Linux Environment]
    - [Mac OSX][Mac OSX Environment]
    - [Windows (32-bit)][Windows (32-bit) Environment]
    - [Windows (64-bit)][Windows (64-bit) Environment]
    
2. Place the file in the `continuous-control/` directory, and unzip (or decompress) the file. 

## Implementation

Follow the instructions in `Continuous_Control.ipynb` to see how it's been implemented.
Feel free to try your ideas!

[//]: # (images)
[Trained Agent]: https://user-images.githubusercontent.com/10624937/43851024-320ba930-9aff-11e8-8493-ee547c6af349.gif

[//]: # (links)
[Reacher Environment]: https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Learning-Environment-Examples.md#reacher
[Linux Environment]: https://s3-us-west-1.amazonaws.com/udacity-drlnd/P2/Reacher/Reacher_Linux.zip
[Mac OSX Environment]: https://s3-us-west-1.amazonaws.com/udacity-drlnd/P2/Reacher/Reacher.app.zip
[Windows (32-bit) Environment]: https://s3-us-west-1.amazonaws.com/udacity-drlnd/P2/Reacher/Reacher_Windows_x86.zip
[Windows (64-bit) Environment]: https://s3-us-west-1.amazonaws.com/udacity-drlnd/P2/Reacher/Reacher_Windows_x86_64.zip
