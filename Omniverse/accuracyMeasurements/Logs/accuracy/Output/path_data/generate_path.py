# Hardcode study area xform form OVIS
origin = [0.0, 0.21502056599432368, 0.13805]
scale = [0.1, 0.1, 0.1]

#Translate the origin from cube center to corner
sx = origin[0] + (scale[0] / 2) 
sy = origin[1] + (scale[1] / 2) 
sz = origin[2] + (scale[2] / 2)

step_size = 0.05 # give in METERS

x_max = scale[0] / step_size
y_max = scale[1] / step_size
z_max = scale[2] / step_size

# Open file stream
filestream = open("C:/Users/halle/Documents/DigitalTwin/Performance/accuracy/Output/path_data/output.txt", "w", encoding="utf-8")

# Create and fill path points
path = []
reverse_x = False
reverse_y = False


print("Starting point: " + str(sx) + "," + str(sy) + "," + str(sz))
print("Loop Maximums: " + str(x_max) + "," + str(y_max) + "," + str(z_max))

# Starting from the top corner
print("Starting Loop")

# Up-down loop from top to bottom
for z_ind in range (0,int(z_max)):
    #print("z:" + str(z_ind))
    
    '''
    min = 0
    max = 100
    step = 1
    if reverse_y is False:
        min = 0
        max = int(y_max) + 1
        step = 1
    else:
        min = int(y_max)
        max = 0
        step = -1
    '''

    # Back-Forth loop
    #for y_ind in range(min, max, step):
    for y_ind in range(0, int(y_max)):
        #print("y:" + str(y_ind))
        '''
        min = 0
        max = 100
        step = 1
        if reverse_x is False:
            min = 0
            max = int(x_max) + 1
            step = 1
        else:
            min = int(x_max)
            max = 0
            step = -1
        # Left-Right loop
        for x_ind in range(min, max, step):
        '''
        for x_ind in range(0, int(x_max)):
            #print("x:" + str(x_ind))

            z = sz - (step_size * z_ind)
            y = sy - (step_size * y_ind)
            x = sx - (step_size * x_ind)

            path.append((x,y,z))

            data = str((x,y,z)) + "\n"
            filestream.writelines(data) 

        reverse_x = not reverse_x
    reverse_y = not reverse_y

print("Done. Saved " + str(len(path)) + " points")
