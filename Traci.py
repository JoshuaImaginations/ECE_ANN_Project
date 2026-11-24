# Step 1: Add modules to provide access to specific libraries and functions
import os # Module provides functions to handle file paths, directories, environment variables
import sys # Module provides access to Python-specific system parameters and functions

# Go through the J_Edits_<number> comments to see which of the steps & segments can be changed others keep unchanged...J_Edit_1

# Step 2: Establish path to SUMO (SUMO_HOME)
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 3: Add Traci module to provide access to specific libraries and functions
import traci # Static network information (such as reading and analyzing network files)

# More Settings : For dynamically finding path to sumoconfig files. LOOK J_Edit_2 ---used Gemini AI by Google. J_Edit_7
# script_dir = os.path.dirname(os.path.abspath(__file__))  CHECK YT PLAYLIST
# config_path = os.path.join(script_dir, "Test6.sumocfg")

# Step 4: Define Sumo configuration
Sumo_config = [
    'sumo-gui',
    '-c', r"C:\Users\JOSHUA\Desktop\ANN_Project\Test6.sumocfg", # FORMER: 'Traci.sumocfg' ; NOW: 'Test6.sumocfg' . J_Edit_2
    '--step-length', '0.05',
    '--delay', '1000',
    '--lateral-resolution', '0.1'
]

# Step 5: Open connection between SUMO and Traci
traci.start(Sumo_config)

# APART FROM FILENAME CHANGE AT LINE-18, ONLY STEP 6 & 7 NEED CHANGES TO USE TRACI..KEEP OTHERS UNCHANGED...J_Edit_3

# Step 6: Define Variables

vehicle_speed = 0
total_speed = 0

# Step 7: Define Functions

# Define a function to get the speed of a vehicle by its ID ...................................................J_Edit_8
def update_speed():
    global vehicle_speed
    global total_speed
    if 'veh1' in traci.vehicle.getIDList():                                          # Any changes do here  
        vehicle_speed = traci.vehicle.getSpeed('veh1')                               # Any changes do here
        total_speed = total_speed + vehicle_speed                                    # Any changes do here
# step_count = step_count + 1                                                    # Any changes do here
        print(f"Vehicle speed: {vehicle_speed} m/s")                                     # Any changes do here.....J_Edit_6
    print(f"Total speed: {total_speed} m/s")                                               # Any changes do here.....J_Edit_10

# Step 8: Take simulation steps until there are no more vehicles in the network
while traci.simulation.getMinExpectedNumber() > 0:                                   # Don't change this line...J_Edit_4
    traci.simulationStep() # Move simulation forward 1 step                          # Don't change this line...J_Edit_5
    # Here you can decide what to do with simulation data at each step
    update_speed() # Call the function to update speed at each step............................................J_Edit_9

# Step 9: Close connection between SUMO and Traci
traci.close()