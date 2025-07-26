import subprocess
import pandas as pd
import os
import re
import argparse
import time
from pathlib import Path
import argparse

def parse_scen_file(scen_file, num_agents):
    """
    Parses the .scen file to extract start and goal positions for the given number of agents.

    Args:
        scen_file (str): Path to the .scen file.
        num_agents (int): Number of agents to extract.

    Returns:
        list of tuples: Start and goal positions for the agents.
    """
    positions = []
    with open(scen_file, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines[:num_agents]):
            if i == 0:
                continue
            # print(line)
            parts = line.strip().split('\t')
            start_x, start_y = int(parts[4]), int(parts[5])
            goal_x, goal_y = int(parts[6]), int(parts[7])
            positions.append((start_x, start_y, goal_x, goal_y))
    return positions

def modify_instance_file(instance_file, num_agents, positions, map_file, temp_instance_file):
    """
    Modifies the instance file with the given seed, number of agents, and positions, and saves it to a temporary file.

    Args:
        instance_file (str): Path to the original instance file.
        seed (int): The new seed value.
        num_agents (int): Number of agents.
        positions (list of tuples): Start and goal positions for the agents.
        temp_instance_file (str): Path to save the modified instance file.
        map_file (str): map file name
    """
    with open(instance_file, 'r') as infile, open(temp_instance_file, 'w') as outfile:
        for line in infile:
            if line.startswith("agents="):
                outfile.write(f"agents={num_agents}\n")
            elif line.startswith("map_file="):
                outfile.write(f"map_file={map_file}\n")
            elif line.startswith("random_problem="):
                outfile.write(f"random_problem={0}\n") 
            elif line.startswith("max_timestep="):
                outfile.write(f"max_timestep={100000}\n") 
            else:
                outfile.write(line)
        # Append start/goal positions to the file
        outfile.write("\n")
        for start_x, start_y, goal_x, goal_y in positions:
            outfile.write(f"{start_x},{start_y},{goal_x},{goal_y}\n")

def parse_result_txt(result_file):
    """
    Parses the result.txt file and extracts relevant fields.
    """
    patterns = {
        "agents": r"agents=(\d+)",
        "map_file": r"map_file=([\w\-.]+)",
        "solver": r"solver=([\w\-.]+)",
        "solved": r"solved=(\d+)",
        "soc": r"soc=(\d+)",
        "lb_soc": r"lb_soc=(\d+)",
        "makespan": r"makespan=(\d+)",
        "lb_makespan": r"lb_makespan=(\d+)",
        "sum_of_loss": r"sum_of_loss=(\d+)",
        "sum_of_loss_lb": r"sum_of_loss_lb=(\d+)",
        "comp_time": r"comp_time=(\d+)",
        "seed": r"seed=(\d+)"
    }

    extracted_data = {}

    try:
        with open(result_file, 'r') as file:
            content = file.read()

            for field, pattern in patterns.items():
                match = re.search(pattern, content)
                if match:
                    extracted_data[field] = int(match.group(1)) if match.group(1).isdigit() else match.group(1)

    except FileNotFoundError:
        print(f"Error: {result_file} not found.")

    return extracted_data


def run_experiment(modified_instance_file, output_file, solver_name, opti_deadline):
    """
    Runs the MAPF executable with the modified instance file.

    Args:
        modified_instance_file (str): Path to the modified instance file.
        output_file (str): Path to save the output log.
        solver_name (str): Solver name to use in the experiment.
        opti_deadline (str): opti_deadline

    Returns:
        tuple: stdout and stderr from the subprocess execution.
    """
    command = ['./build/mapf', '-i', modified_instance_file, '-o', output_file, '-s', solver_name, '-M', opti_deadline]
    try:
        # Run the subprocess with a timeout
        result = subprocess.run(command, capture_output=True, text=True, timeout=60)
        return result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        print(f"Process timed out after {60} seconds.")
        return None, None

def pibt_batch_runner(instance_file, output_csv, map_folder, max_time_threshold=60):
    """
    Runs pibt with varying opti_deadline values and logs results in a CSV file.
    Loops through different number of agents (N), checks if execution time exceeds threshold, and stops early if needed.
    """

    # Get all .map files in the map folder
    map_files = [f for f in os.listdir(map_folder) if f.endswith('.map')]
    if not map_files:
        print(f"No .map files found in {map_folder}. Please check the folder path.")
        return
    
    # Use the first map file (you can modify this part if you need to choose specific map files)
    map_file_path = os.path.join(map_folder, map_files[0])

    results = []

    # Get all .scen files in the map folder
    scen_files = [f for f in os.listdir(map_folder) if f.endswith('.scen')]
    if not scen_files:
        print(f"No .scen files found in {map_folder}. Please check the folder path.")
        return
    
    max_n_limit = 1000  # Initially, no limit
    max_processed_n = 20

    # Check the maximum processed agents for the current scenario
    existing_data = pd.read_csv(output_csv) if os.path.exists(output_csv) else pd.DataFrame()
    if not existing_data.empty:
        print(existing_data.columns)
        max_processed_n = existing_data["agents"].max()
        print(max_processed_n)

    # Loop through all .scen files in the map folder
    for N in range(max_processed_n, max_n_limit + 1, 40):  # Adjust these numbers as needed
        num_success_64 = 25
        num_success_256 = 25
        num_success = 25
        sceni = 0
        for scen_file in scen_files:
            scen_file_path = os.path.join(map_folder, scen_file)
            sceni += 1
            
            skip_loop = False
        
        # Loop through different numbers of agents (N) using range(20, 600, 40)
            for solver_name in ["PIBT", "PIBTOLD"]:
                opti_deadline = 0
                if solver_name == "PIBT":
                    for opti_deadline in [1000]:  # 1s

                        temp_instance_file = f"temp_instance_{scen_file}_{N}.txt"
                        positions = parse_scen_file(scen_file_path, N)

                        modify_instance_file(
                            instance_file=instance_file,
                            num_agents=N,
                            positions=positions,
                            map_file = map_files[0],
                            temp_instance_file=temp_instance_file,
                        )

                        start_time = time.time()  # Start timer

                        output_file = f"outputs/output_{solver_name}_scen_{scen_file}_na{N}.txt"
                        stdout, stderr = run_experiment(temp_instance_file, output_file, solver_name, str(opti_deadline))

                        cost_file_1 = f"outputs/times1htc_{sceni}_{N}.txt"
                        # Clean up the temporary file
                        os.remove(temp_instance_file)
                        # move the file if it exists
                        if os.path.exists("times1.txt"):
                            os.rename("times1.txt", cost_file_1)

                        # Parse the result file for output data
                        parsed_data = parse_result_txt(output_file)
                        print(parsed_data)
                        
                        if not parsed_data:
                            if opti_deadline == 4:
                                num_success_64 -= 1
                            elif opti_deadline == 256:
                                num_success_256 -= 1
                            print(f"Skipping experiment due to missing or invalid {output_file} for {scen_file}.")
                            continue
                        elif parsed_data["solved"] == 0:
                            if opti_deadline == 4:
                                num_success_64 -= 1
                            elif opti_deadline == 256:
                                num_success_256 -= 1
                        
                        # Add additional fields to parsed data
                        parsed_data["Opti_Deadline"] = opti_deadline
                        parsed_data["Scenario_File"] = scen_file

                        df = pd.DataFrame([parsed_data])
                        df.to_csv(output_csv, mode='a', header=not os.path.exists(output_csv), index=False)
                else: 
                    temp_instance_file = f"temp_instance_{scen_file}_{N}.txt"
                    positions = parse_scen_file(scen_file_path, N)

                    modify_instance_file(
                        instance_file=instance_file,
                        num_agents=N,
                        positions=positions,
                        map_file = map_files[0],
                        temp_instance_file=temp_instance_file,
                    )

                    start_time = time.time()  # Start timer

                    output_file = f"outputs/output_{solver_name}_scen_{scen_file}_na{N}.txt"
                    stdout, stderr = run_experiment(temp_instance_file, output_file, solver_name, str(opti_deadline))

                    # Clean up the temporary file
                    os.remove(temp_instance_file)

                    # Parse the result file for output data
                    parsed_data = parse_result_txt(output_file)
                    
                    if not parsed_data:
                        num_success -= 1
                        print(f"Skipping experiment due to missing or invalid {output_file} for {scen_file}.")
                        continue
                    elif parsed_data["solved"] == 0:
                        num_success -= 1
                    
                    # Add additional fields to parsed data
                    parsed_data["Opti_Deadline"] = opti_deadline
                    parsed_data["Scenario_File"] = scen_file

                    df = pd.DataFrame([parsed_data])
                    df.to_csv(output_csv, mode='a', header=not os.path.exists(output_csv), index=False)

        success_rate_64 = num_success_64 / 25 
        success_rate_256 = num_success_256 / 25 
        success_rate = num_success / 25
        print(f"Success rate for N={N}, opti_deadline 64: {success_rate_64:.2%}, opti_deadline 256: {success_rate_256:.2%}, pibtold: {success_rate:.2%}")
        
        if success_rate <= 0.05 and success_rate_64 <= 0.05 and success_rate_256 <= 0.05:
            print(f"Early termination: Success rate below 50% for N={N}")
            break

    print("Batch processing complete.")

def pibt_single_runner(instance_file, output_csv, map_folder, max_time_threshold=1):
    """
    Runs pibt with varying opti_deadline values and logs results in a CSV file.
    Loops through different number of agents (N), checks if execution time exceeds threshold, and stops early if needed.
    """

    # Get all .map files in the map folder
    map_files = [f for f in os.listdir(map_folder) if f.endswith('.map')]
    if not map_files:
        print(f"No .map files found in {map_folder}. Please check the folder path.")
        return
    
    # Use the first map file (you can modify this part if you need to choose specific map files)
    map_file_path = os.path.join(map_folder, map_files[0])

    results = []

    # Get all .scen files in the map folder
    scen_files = [f for f in os.listdir(map_folder) if f.endswith('.scen')]
    if not scen_files:
        print(f"No .scen files found in {map_folder}. Please check the folder path.")
        return
    
    max_n_limit = 200  # Initially, no limit
    sceni = 0

    # Check the maximum processed agents for the current scenario
    existing_data = pd.read_csv(output_csv) if os.path.exists(output_csv) else pd.DataFrame()

    # Loop through all .scen files in the map folder
    for scen_file in scen_files:
        scen_file_path = os.path.join(map_folder, scen_file)
        sceni +=1
        max_processed_n = 20
        skip_loop = False

        if not existing_data.empty:
            scenario_data = existing_data[existing_data["Scenario_File"] == scen_file]
            if not scenario_data.empty:
                max_processed_n = scenario_data["agents"].max()
                if max_processed_n >= max_n_limit:
                    continue
        
        # Loop through different numbers of agents (N) using range(20, 600, 40) range(max_processed_n, max_n_limit + 1, 40)
        for N in [500]:  # Adjust these numbers as needed , "PIBTOLD"
            for solver_name in ["PIBT"]:
                opti_deadline = 100000
                # for opti_deadline in [0,1,4,16,64,256]:  # From 0ms to 1s inclusive

                temp_instance_file = f"temp_instance_{scen_file}_{N}.txt"
                positions = parse_scen_file(scen_file_path, N)

                modify_instance_file(
                    instance_file=instance_file,
                    num_agents=N,
                    positions=positions,
                    map_file = map_files[0],
                    temp_instance_file=temp_instance_file,
                )

                start_time = time.time()  # Start timer

                output_file = f"outputs/output_{solver_name}_scen_{scen_file}_na{N}.txt"
                stdout, stderr = run_experiment(temp_instance_file, output_file, solver_name, str(opti_deadline))

                cost_file_1 = f"cost1_{sceni}.txt"

                # Rename the files
                os.rename("costs1.txt", cost_file_1)
                os.rename("costs20.txt", f"cost20_{sceni}.txt")
                os.rename("costs21.txt", f"cost21_{sceni}.txt")
                os.rename("costs22.txt", f"cost22_{sceni}.txt")
                os.rename("costs23.txt", f"cost23_{sceni}.txt")
                os.rename("costs24.txt", f"cost24_{sceni}.txt")
                os.rename("costs25.txt", f"cost25_{sceni}.txt")
                os.rename("costs26.txt", f"cost26_{sceni}.txt")

                # Clean up the temporary file
                os.remove(temp_instance_file)

    print("Batch processing complete.")
               



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Batch runner for pibt experiments.")
    parser.add_argument(
        "--output_csv",
        type=str,
        required=True,
        help="Path to the output CSV file where results will be saved."
    )
    parser.add_argument(
        "--map_folder",
        type=str,
        required=True,
        help="Path to the folder containing map and scenario files."
    )
    parser.add_argument(
        "--max_time_threshold",
        type=int,
        default=1,
        help="Maximum time (in seconds) allowed for an experiment. Default is 50 seconds."
    )

    args = parser.parse_args()

    # Run the batch runner
    pibt_batch_runner(
        instance_file = "instances/mapf/sample.txt",   # original_instance_file
        output_csv=args.output_csv,
        map_folder=args.map_folder,
        max_time_threshold=args.max_time_threshold
    )

    # pibt_single_runner(
    #     instance_file = "instances/mapf/sample.txt",   # original_instance_file
    #     output_csv=args.output_csv,
    #     map_folder=args.map_folder,
    #     max_time_threshold=args.max_time_threshold
    # )
