This is our physics project proposal, we want to finish it:

Team Members: Alex Wang, Devon Parikh, Aiden Knapp 
Project Title: Optimizing Regenerative Braking of Electric Vehicles

Proposal:

Regenerative braking runs an electric vehicle's motor as a generator, converting kinetic energy back into chemical energy stored in the battery. It cannot always do all of the braking: the motor has torque and power limits, the battery accepts less charge as it nears full, and the tires have only so much grip. Our central question is how braking should be split between regenerative and friction braking to maximize recovered energy, and how that optimal split changes between straight driving and turning a corner.
By optimization, we mean choosing at each moment the fraction of braking force supplied by regen so that the total energy stored over a full drive cycle is as large as possible, while the car still slows down as the cycle requires. Whatever regen cannot supply comes from friction brakes and is lost as heat. Regen is capped by the smallest of four limits: motor torque at low speed, motor power at high speed, battery charge power, and tire grip. Grip matters because braking shifts weight forward, unloading the rear axle where many EVs regenerate, and because a tire's grip is shared between braking and cornering. More regen recovers more energy, but too much rear-axle regen can exceed rear grip and make the car unstable, so the optimal split balances energy recovered against grip available at each axle.
To keep the model manageable, we treat the car as a rigid body on two axles, described by its mass, wheelbase, and center-of-mass height. The road is flat and dry with a constant friction coefficient; we ignore weather and temperature, tire slip dynamics, suspension motion, and road grade. We keep aerodynamic drag and rolling resistance because they are simple to model and absorb a large share of energy. We will start with a point mass car braking in a straight line with only motor torque, and then we will add in drag force, rolling resistance, and corners. In VPython, we will write software to model the car, taking into account the simplified forces exerted on the car. We plan to create two models, one of which is a simplified straight-line braking model, and the other being one where the car turns a 90 degree corner at a constant speed. We will build these models using the momentum and energy principles, then run the same scenarios in Simulink to test our simplifying assumptions against a full vehicle simulation.

Milestones:

#1: Physics equations that involve conservation of energy (kinetic → electrical) 
#2: Simplified functional model of car in vpython: Script modeling vehicle speed, different braking forces, wheel torque, and energy recovery over time.
#3: A more advanced functional model that features a 90-degree turn and the car will take the midline through the turn, in which case we would better model lateral tire forces. This step is meant to add more complexity to our model by including more forces.
#4: Simulink Cross-Validation and Benchmarking: We will use MATLAB Simulink to run full vehicle simulations of our exact same driving scenarios and compare the results from its vehicle dynamics engine to our vpython model. 
#5: Conclude optimize brake torque splits and present our findings.