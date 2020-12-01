# Cooperative Robots
2 robots collaboratively move a rod out of a room through the opening. 
Fuzzy Logic and GA are leveraged to design an intelliegent control system.
* Author: Yufeng Sun @ IRAS Lab University of Cincinati
* Email: sunyf@mail.uc.edu

# Motivation
When two (or more) persons carry a large piece of furniture through a narrow door, they will
have to come up with a good strategy to smartly maneuver the furniture in all the 6 degrees of
freedom such that it can pass through the door. Understandably, the success and efficiency of
the job mainly depends on (1) how much experience the two people have and (2) how well the
two people collaborate with each other. What if two robots do the job? How do we control the
robots to successfully accomplish the job? The truth is that no robots in the world today are
smart enough to accomplish such a challenging job like humans. This has motivated many
researchers across the world to do research in intelligent robotics and hopefully future robots
will be able to do such jobs like humans do. Our project is to solve a similar but, of course,
much simplified problem as defined in the next section.

![A real world problem](/doc/realworldproblem.jpg)

# Problem definition
Our problem is limited to a 2D space (3 degrees of freedom). Consider a room with an opening
of width w and depth d, and a rod of length l, shown in Fig.2. Two robots, one at end A and the
other at end B, intend to move the rod from its initial location inside the room to outside
through the opening. The goal of this project is to train the two robots so that they can
collaboratively carry the rod through the opening to the outside room. The key for
collaboration is that each robot does not know how its partner robot moves but they have to
manage themselves to maintain the distance as they are fixed on each side of the rod. The 
center position is defined as the rod position and the angle of the rod with respect to the
horizontal line is defined as the rod orientation.

<img src="/doc/problemdef.jpg" alt="Problem definition" width="500">

# Formulation
* Environment:
	* room: x [-1, 1] y [-1, 0]
	* door: x [-0.1, 0.1] y [0 0.2]
	* rod length = 0.5
* Robots:
	* position: R1(x1, y1)  R2(x2, y2)
	* constraint: distance between two robots should always be rod length
	
<img src="/doc/problem_denotes.jpg" alt="Problem denotation" width="500">

# Kinematic Case
## Intelligent Control 
* Both robots knows its own position and the angle of the rod at any time, but does not know how will each other behave.
* Robot R1 controls the position of the rod and robot R2 controls the angle of the rod.
* Fuzzy Logic Control
	*  R1: inputs(x1, y1, angle of rod), outputs: velocity of rod
	*  R2: inputs(x2, y2, angle of rod), outputs: anglular velocity of rod
* Genetic Algorithm
	* Generation: 500
	* Population size: 100
	* Cost function: cost = 100*Cost1 + Cost2 (cost for each play with one case of initial state)
		* Cost1: if fail, the cost is the total distance of robots' positions to the predefined target position (outside room).
		* Cost2: if success, the cost is the total travel distance of two robots.
  
## Training and Test
* Training samples: 100 random cases of initial state 

<img src="/doc/random_training.jpg" alt="Sample Scenarios for Training" width="500">
* Training graph

<img src="/doc/kinematic/training_graph.jpg" alt="Sample Scenarios for Training" width="500">
* Test samples: 100 radndom cases of initial state

<img src="/doc/random_test.jpg" alt="Sample Scenarios for Training" width="500">
* Success rate: 91%

