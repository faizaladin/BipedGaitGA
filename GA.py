import random
import mujoco
import mujoco_viewer
import numpy as np
import random
from scipy.optimize import minimize
from numpy.random import randint

model = mujoco.MjModel.from_xml_path('humanoid.xml')
data = mujoco.MjData(model)
#viewer = mujoco_viewer.MujocoViewer(model, data)

abdomen_y = data.actuator("abdomen_y")
abdomen_z = data.actuator("abdomen_z")
abdomen_x = data.actuator("abdomen_x")
hip_x_right = data.actuator("hip_x_right")
hip_z_right = data.actuator("hip_z_right")
hip_y_right = data.actuator("hip_y_right")
knee_right = data.actuator("knee_right")
ankle_x_right = data.actuator("ankle_x_right")
ankle_y_right = data.actuator("ankle_y_right")
hip_x_left = data.actuator("hip_x_left")
hip_z_left = data.actuator("hip_z_left")
hip_y_left = data.actuator("hip_y_left")
knee_left = data.actuator("knee_left")
ankle_x_left = data.actuator("ankle_x_left")
ankle_y_left = data.actuator("ankle_y_left")
shoulder1_right = data.actuator("shoulder1_right")
shoulder2_right = data.actuator("shoulder2_right")
elbow_right = data.actuator("elbow_right")
shoulder1_left = data.actuator("shoulder1_left")
shoulder2_left = data.actuator("shoulder2_left")
elbow_left = data.actuator("elbow_left")

actuators = [abdomen_x, abdomen_y, abdomen_z, hip_x_right, hip_z_right, hip_y_right, knee_right, ankle_x_right, ankle_y_right, hip_x_left, hip_z_left, hip_y_left, knee_left, ankle_x_left, ankle_y_left, shoulder1_right, shoulder2_right, elbow_right, shoulder1_left, shoulder2_left, elbow_left]

def generate_random_actuator_value():
    # Generate a random float with two decimal places between -1 and 1
    return round(random.uniform(-1, 1), 2)

def encode_value(value):
    # Determine the sign bit
    sign_bit = '1' if value >= 0 else '0'
    
    # Scale the value to fit within the range [0, 99] (for two decimal places)
    scaled_value = int(abs(value) * 100)  # Scale to range [0, 99]
    
    # Convert the scaled value to a 7-bit binary string
    value_bits = format(scaled_value, '07b')
    
    # Concatenate the bits to form the bit string
    return sign_bit + value_bits

def initial_cycle(num_actuators, steps):
    # Generate a bit string for all the actuators for n steps
    bit_string = ''
    for _ in range(steps):
        for _ in range(num_actuators):
            # Append a random bit string for each actuator
            bit_string += encode_value(generate_random_actuator_value())  # Assuming 8 bits per actuator
    return bit_string

def decode_value(bit_string):
    # Extract the sign bit
    sign_bit = bit_string[0]
    
    # Extract the value bits
    value_bits = bit_string[1:]
    
    # Convert the value bits to an integer
    scaled_value = int(value_bits, 2)
    
    # Convert the scaled value to the original range [-1, 1]
    value = scaled_value / 100
    
    # Adjust the sign
    if sign_bit == '0':
        value *= -1
    
    return value

def convert_to_ctrl(bit_string, num_actuators):
    # Calculate the length of each encoded value
    value_length = 8  # Assuming each actuator value is encoded in 8 bits
    
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
            step_values.append(decode_value(value_bits))
        
        # Append the list of decoded values for the current step to the control matrix
        ctrl_matrix.append(step_values)
    
    return ctrl_matrix

def mutate(bit_string, num_bits):
    # Convert the bit string to a list to make it mutable
    bit_list = list(bit_string)
    
    # Generate indices to mutate
    indices_to_mutate = random.sample(range(len(bit_list)), num_bits)
    
    # Mutate the selected bits
    for index in indices_to_mutate:
        # Flip the bit (0 to 1 or 1 to 0)
        bit_list[index] = '1' if bit_list[index] == '0' else '0'
    
    # Convert the list back to a string
    mutated_bit_string = ''.join(bit_list)
    
    return mutated_bit_string

def main(ctrl_matrix, steps):
    mujoco.mj_resetData(model, data)
    point1 = [0, 0, 1.282]
    for i in range(steps):
        for x in range(len(actuators)):
            actuators[x].ctrl = ctrl_matrix[i][x]
        #viewer.render()
        mujoco.mj_step(model, data)
        point2 = data.body("torso").xpos
        distance = (point2[0]-point1[0])
        if (point2[2] < 1 or point2[2] > 2):
            return distance
    return distance

# Example usage
steps = 500
bit_string = ""
curr_distance = 0
counter = 0

while curr_distance < 20:
    if counter == 0:
        bit_string = initial_cycle(steps, len(actuators))
        ctrl_matrix = convert_to_ctrl(bit_string, 21)
        curr_distance = main(ctrl_matrix, steps)
        counter = counter + 1
    else:
        prev_bit = bit_string
        bit_string = mutate(bit_string, 252)
        ctrl_matrix = convert_to_ctrl(bit_string, 21)
        temp_distance = main(ctrl_matrix, steps)
        if (temp_distance >= curr_distance):
            curr_distance = temp_distance
        else:
            bit_string = prev_bit
    print(curr_distance)
print(ctrl_matrix)
