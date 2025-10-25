import argparse
import os
import math

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

def distance(x1, y1, z1, x2, y2, z2):
    d = 0.0
    d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    return d

parser = argparse.ArgumentParser()

parser.add_argument("infile", help="in the Output directory, name of for_distance_calc_passxxx.txt")
parser.add_argument("outfile", help="in the Output/distance_scalars/correction/ directory, name of corrected_passxxx.csv")

args = parser.parse_args()
path_base = os.getcwd()

# Open file stream for RESULTING CORRECCTION VECTORS
filestream = open(path_base + "/distance_scalars/correction/" + args.outfile, "w", encoding="utf-8")

# Add csv header
header_string = "x,y,z,d\n"
filestream.writelines(header_string)

# Read vector data
with open(path_base + "/" + args.infile) as f:
    endpoint_vectors = f.read().splitlines()

sample_num = len(endpoint_vectors)

for i in range(0, sample_num):
    # Process data string
    data = endpoint_vectors[i] 
    data = data.replace("(","")
    data = data.replace(")","")
    data = data.split(";")

    voxel = data[0]

    correct_point = string_to_vec3f(data[1])
    offset_point = string_to_vec3f(data[2])

    
    print("Voxel Point: " + str(voxel))
    print("Correct Endpoint: " + str(correct_point))
    print("Offset Endoint: " + str(offset_point))


    # Calculate distance scalar
    distance_scalar = distance(correct_point[0], correct_point[1], correct_point[2], offset_point[0], offset_point[1], offset_point[2])

    # Save point and distance to file
    write_data = voxel + "," + str(distance_scalar) + "\n"
    write_data = write_data.replace("(","")
    write_data = write_data.replace(")","")
    write_data = write_data.replace(" ","")
    filestream.writelines(write_data)


print("Done. Saved " + str(sample_num) + " points")
