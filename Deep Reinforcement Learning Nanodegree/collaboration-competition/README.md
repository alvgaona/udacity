# Collaboration and Competition

![Trained Agent]

This project will work with the [Tennis][Tennis Environment] environment.

In this environment, two agents control rackets to bounce a ball over a net.
If an agent hits the ball over the net, it receives a reward of +0.1.
If an agent lets a ball hit the ground or hits the ball out of bounds, it receives a reward of -0.01.
Thus, the goal of each agent is to keep the ball in play.

The observation space consists of 8 variables corresponding to the position and velocity of the ball and racket.
Each agent receives its own, local observation. 
Two continuous actions are available, corresponding to movement toward (or away from) the net, and jumping. 

The task is episodic, and in order to solve the environment, the agents must get an average score of +0.5
(over 100 consecutive episodes, after taking the maximum over both agents). 

Specifically:

- After each episode, add up the rewards that each agent received (without discounting), to get a score for each agent.
This yields 2 (potentially different) scores.
Then, take the maximum of these 2 scores.
- This yields a single **score** for each episode.

The environment is considered solved, when the average (over 100 episodes) of those **scores** is at least +0.5.

## Getting Started

1. Download the environment from one of the links below.  You need only select the environment that matches your operating system:
    - [Linux]
    - [Mac OSX]
    - [Windows (32-bit)]
    - [Windows (64-bit)]
    
> If you'd like to train the agent on AWS (and have not [enabled a virtual screen][Virtual Screen]), 
> then please use [Linux NoVis] to obtain the "headless" version of the environment. 
> You will **not** be able to watch the agent without enabling a virtual screen, 
> but you will be able to train the agent.  
> To watch the agent, you should follow the instructions to enable a virtual screen, and then download the
> environment for the **Linux** operating system above.

2. Place the file in the DRLND GitHub repository, in the `p3_collab-compet/` folder, and unzip (or decompress) the file. 

[//]: # (Image References)

[Trained Agent]: https://user-images.githubusercontent.com/10624937/42135623-e770e354-7d12-11e8-998d-29fc74429ca2.gif
[Tennis Environment]: https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Learning-Environment-Examples.md#tennis

[//]: # (Links)
[Linux]: https://s3-us-west-1.amazonaws.com/udacity-drlnd/P3/Tennis/Tennis_Linux.zip
[Linux NoVis]: https://s3-us-west-1.amazonaws.com/udacity-drlnd/P3/Tennis/Tennis_Linux_NoVis.zip
[Mac OSX]: https://s3-us-west-1.amazonaws.com/udacity-drlnd/P3/Tennis/Tennis.app.zip
[Windows (32-bit)]: https://s3-us-west-1.amazonaws.com/udacity-drlnd/P3/Tennis/Tennis_Windows_x86.zip
[Windows (64-bit)]: https://s3-us-west-1.amazonaws.com/udacity-drlnd/P3/Tennis/Tennis_Windows_x86_64.zip
[Virtual Screen]: https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Training-on-Amazon-Web-Service.md