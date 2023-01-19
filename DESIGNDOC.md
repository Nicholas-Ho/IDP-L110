# IDP Higher level System goals:

## Objective
The ultimate objective is to find coloured blocks, determine their colour and deliver to the correct area based on their colour. The block can be delivered to the floor of the delivery area or onto the truck for extra points. The area contains white lines for guidance. An obstacle is situated between the starting and the drop off areas and the collection area which can be avoided by using either a ramp or tunnel. Each time a block is transported back across the obstacle a new block will be placed on one of the collection markers.

## Team Aims:
-	Key objectives to prioritise – overall team goals
-	Failure points for each objective
-	-Initial calibration and testing (have large amount of initial data collected early for sensor calibration)
-	Error logs and reporting (with time stamps – refer to software (i.e. motor torque struggling to climb, can identify if sensor problem or mechanical issue))
-	**Common calendar for team: use red for indication that you are busy – use green for availability alongside your name in parenthesis**

## Work scheduling: 
-	Honesty and own up with early mistakes
-	Continuous reporting of updates
-	Version control with individual branches in a common repository: datasets, images, detailed commit messages (not just software)
-	Initial brainstorm of ideas (basis of capability of each subteam)
1. Initial Brainstorm (sketches + feasibility - individually)
2. Combination of ideas (summation of initial sketches, and present the advantages and disadvantages)
- Murphy’s law – for manufacturing, expect details, things that can go wrong will go wrong

## Sub-teams:
1.	Electronics: 
- Identify i.e. ADCs, Amplification
- Timer constraints on Arduino
- LED indications (visual testing while robot is running – to identify state)*
2.	Mechanical:  
Length scales – line widths, bridge height and ramp scale. Torque figures for motors for initial hand calculations
3.	Software:
- State machine algorithm for line detection
- Exploration of overriding with CV (branch + end of line detection)
- Redundancy with state machine dependency
*Check if software can send data to a web server or similar…