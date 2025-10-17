import argparse
import os

# Assumes data string is in 
# x,y,z format
# note, no parens
def string_to_vec3f(data_string):
    data = data_string.split(",")
    x = float(data[0])
    y = float(data[1])
    z = float(data[2])

    vector = (x,y,z)

    return vector

parser = argparse.ArgumentParser()

parser.add_argument("infile", help="in the endpoints directory, name of vector_xxx.txt")
parser.add_argument("outfile", help="in the endpoints/corrections directory, name of corrections_xxx.txt")

args = parser.parse_args()

path_base = os.getcwd()

# Open file stream for RESULTING CORRECCTION VECTORS
filestream = open(path_base + "/corrections/" + args.outfile, "w", encoding="utf-8")

# Read vector data
with open(path_base + "/" + args.infile) as f:
    endpoint_vectors = f.read().splitlines()

sample_num = len(endpoint_vectors)
#sample_num = 1

for i in range(0, sample_num):
    # Process data string
    data = endpoint_vectors[i] 
    data = data.replace("(","")
    data = data.replace(")","")
    data = data.split(";")

    voxel = data[0]

    correct_point = string_to_vec3f(data[1])
    offset_point = string_to_vec3f(data[2])

    '''
    print("Voxel Point: " + str(voxel))
    print("Correct Endpoint: " + str(correct_point))
    print("Offset Endoint: " + str(offset_point))
    '''

    # Calculate correction vector
    x_off = correct_point[0] - offset_point[0]
    y_off = correct_point[1] - offset_point[1]
    z_off = correct_point[2] - offset_point[2]

    correction_vector = (x_off, y_off, z_off)

    # Save point and correction vector to file
    write_data = voxel + ";" + str(correction_vector) + "\n"
    filestream.writelines(write_data)

print("Done. Saved " + str(sample_num) + " points")
