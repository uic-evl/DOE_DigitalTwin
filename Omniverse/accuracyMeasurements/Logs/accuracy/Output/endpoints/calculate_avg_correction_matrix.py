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

parser.add_argument("outfile", help="in the ./endpoints directory, name of avg_correction_mat_xxx.txt")

args = parser.parse_args()
path_base = os.getcwd()

# Open file stream for RESULTING CORRECTION MATRIX
filestream = open(path_base + "/" + args.outfile, "w", encoding="utf-8")

# Find number of corrections to average -> input_num
infiles = os.listdir(path_base + "/corrections/")
input_num = len(infiles)

corrections_matrix = []

# Read input_num files 
for file in infiles:
    with open(path_base + "/corrections/" + file) as f:
        corrections = f.read().splitlines()

    #print("Opening " + path_base + "/corrections/" + file)

    pass_points = []
    voxels = []
    count = 0

    # process each cell
    for cell in corrections:
        data = cell.replace("(","")
        data = data.replace(")","")
        data = data.split(";")

        #print("Voxel is " + data[0] + "; Correction is: " + data[1])
        voxels.append(data[0])
        point = string_to_vec3f(data[1])
        pass_points.append(point)

    corrections_matrix.append(pass_points)

# This is assuming that all input files have the same number of voxels
# This SHOULD be true, but there is nothing in the code to stop/catch
# if this is not correct.
voxel_num = len(corrections_matrix[0])

'''
for thing in corrections_matrix:
    print("This path has " + str(len(thing)) + " voxels, and stores the following points: " + str(thing))
'''
# each line of the results will be in voxel(x,y,z);avg_fix(x,y,z)
result_mat = []

# For each cell in the result matrix
for i in range(0,voxel_num):
    x_vals = []
    y_vals = []
    z_vals = []

    for data_pass in range(0, input_num):
        x_vals.append(corrections_matrix[data_pass][i][0])
        y_vals.append(corrections_matrix[data_pass][i][1])
        z_vals.append(corrections_matrix[data_pass][i][2])

    x_avg = (sum(x_vals)) / input_num
    y_avg = (sum(x_vals)) / input_num
    z_avg = (sum(x_vals)) / input_num

    result_mat.append((x_avg, y_avg, z_avg))
    # Write voxel coords and avg_offset vectors to file
    write_data = voxels[i] + ";" + str((x_avg, y_avg, z_avg)) + "\n"
    filestream.writelines(write_data)

print("Done. Saved " + str(voxel_num) + " avg correction vectors")

