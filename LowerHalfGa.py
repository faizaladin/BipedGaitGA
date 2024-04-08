import random
import mujoco
import mujoco_viewer
import numpy as np
import random
from scipy.optimize import minimize
from numpy.random import randint

model = mujoco.MjModel.from_xml_path('humanoid.xml')
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

abdomen_y = data.actuator("abdomen_y")
hip_y_right = data.actuator("hip_y_right")
knee_right = data.actuator("knee_right")
ankle_y_right = data.actuator("ankle_y_right")
hip_y_left = data.actuator("hip_y_left")
knee_left = data.actuator("knee_left")
ankle_y_left = data.actuator("ankle_y_left")

abdomen_y.ctrl = 1

actuators = [hip_y_right, knee_right, ankle_y_right, hip_y_left, knee_left, ankle_y_left]

def create_chromosome(num_actuators, steps):
    chromosome = ""
    for x in range(steps):
        for i in range(num_actuators):
            rand_bit = "".join([random.choice(['0', '1']) for _ in range(2)])
            chromosome += rand_bit
    return chromosome

def mutate(chromosome):
    mutated_chromosome = ""
    for bit in chromosome:
        # Flip the bit with a probability of 1/300
        if random.random() < 1/300:
            mutated_chromosome += '0' if bit == '1' else '1'
        else:
            mutated_chromosome += bit
    return mutated_chromosome

def crossover(chromosome1, chromosome2):    
    crossover_point = random.randint(1, len(chromosome1) - 1)
    new_chromosome = chromosome1[:crossover_point] + chromosome2[crossover_point:]
    return new_chromosome

def roulette_selection(population, fitness_scores):
    total_fitness = sum(fitness_scores)
    selected_values = [random.random() * total_fitness for _ in range(2)]  # Select two random values
    selected_chromosomes = []

    for value in selected_values:
        cumulative_fitness = 0
        selected_chromosome = None
        for i, fitness in enumerate(fitness_scores):
            cumulative_fitness += fitness
            if cumulative_fitness >= value:
                selected_chromosome = population[i]
                break
        if selected_chromosome is not None:
            selected_chromosomes.append(selected_chromosome)

    # If only one chromosome is selected, duplicate it to have two parents
    if len(selected_chromosomes) == 1:
        selected_chromosomes.append(selected_chromosomes[0])

    return selected_chromosomes

def create_population(population_size, steps):
    population_list = []
    for i in range(population_size):
        population_list.append(create_chromosome(len(actuators), steps))
    return population_list

def decode(bit_string):
    # Convert the two-bit string to a number
    if bit_string == "00":
        return 0
    elif bit_string == "01":
        return 1
    elif bit_string == "10":
        return 0
    elif bit_string == "11":
        return -1

def convert_to_ctrl(bit_string, num_actuators):
    # Calculate the length of each encoded value
    value_length = 2  # Assuming each actuator value is encoded in 8 bits
    
    # Initialize an empty control matrix
    ctrl_matrix = []
    
    # Iterate through the bit string to decode values
    for i in range(0, len(bit_string), num_actuators * value_length):
        # Extract a chunk of bits for each step
        step_bits = bit_string[i:i + num_actuators * value_length]
        
        # Initialize a list to store decoded values for the current step
        step_values = []
        
        # Iterate through the chunk to decode values for each actuator
        for j in range(0, len(step_bits), value_length):
            # Extract the bits for the current actuator's value
            value_bits = step_bits[j:j + value_length]
            # Decode the value and append it to the list
            step_values.append(decode(value_bits))
        
        # Append the list of decoded values for the current step to the control matrix
        ctrl_matrix.append(step_values)
    
    return ctrl_matrix

def fitness(ctrl_matrix):
    mujoco.mj_resetData(model, data)
    point1 = [0, 0, 1.282]
    switch = False
    multiplier = 1
    for i in range(steps):
        for x in range(len(actuators)):
            actuators[x].ctrl = ctrl_matrix[i][x]
        mujoco.mj_step(model, data)
    point2 = data.body("torso").xpos
    head = data.body("head").xpos
    dist = np.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
    dist = dist * (1 + (head[2]*0.5))
    return dist

def new_generation(parents_list, population_size):
    population_list = []
    for i in range(population_size):
        child_chromosome = crossover(parents_list[0], parents_list[1])
        mutated_child_chromosome = mutate(child_chromosome)
        population_list.append(mutated_child_chromosome)
    return population_list

def main(population_list, fitness_scores):
    for i in range(len(population_list)):
        ctrl_matrix = convert_to_ctrl(population_list[i], len(actuators))
        fitness_score = fitness(ctrl_matrix)
        fitness_scores.append(fitness_score)
    parents_list = roulette_selection(population_list, fitness_scores)
    population_list = new_generation(parents_list, population_size)
    
    return population_list


steps = 500
population_size = 100
generations = 100
population_list = create_population(population_size, steps)
fitness_scores = []

print(len(population_list))

for i in range(generations):
    for x in range(len(population_list)):
        ctrl_matrix = convert_to_ctrl(population_list[x], len(actuators))
        fitness_score = fitness(ctrl_matrix)
        fitness_scores.append(fitness_score)
    parents_list = roulette_selection(population_list, fitness_scores)
    fitness_scores = []
    population_list = new_generation(parents_list, population_size)

fitness_score = 0

for l in range(len(population_list)):
        ctrl_matrix = convert_to_ctrl(population_list[l], len(actuators))
        temp_fitness_score = fitness(ctrl_matrix)
        if fitness_score < temp_fitness_score:
            chromosome = population_list[l]
            fitness_score = temp_fitness_score

ctrl_matrix = convert_to_ctrl(chromosome, len(actuators))

print(ctrl_matrix, fitness(ctrl_matrix))

mujoco.mj_resetData(model, data)
for z in range(steps):
    for x in range(len(actuators)):
        actuators[x].ctrl = ctrl_matrix[z][x]
        viewer.render()
        mujoco.mj_step(model, data)