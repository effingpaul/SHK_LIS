# read the images from a specified recordings folder and read the label files as well
# convert the images to a numpy array 


# INSTRCUTIONS:
# first record a trajectory with the cpp code in SHK_LIS/Teleoperation

# activate conda environment fish
# then convert the data to a pickled file with SHK_LIS/Teleoperation/convert.py

# copy the resulting expert_demos.pkl to the expert_demos folder in the FISH folder

import os
import numpy as np
import pickle
import cv2
import time
import math

def convert_images_to_numpy_array(folder_path, startFrame=0, endFrame=0, summarize_frames=1, height=84, width=84):
    images = []
    
    for i in range(startFrame, endFrame+1, summarize_frames):
        filename = "cam1_"+str(i)+".jpg"
        img = cv2.imread(os.path.join(folder_path + "/images", filename))
        #crop image to be quadratic
        if min(img.shape[0], img.shape[1]) != img.shape[0]:
            cutoff = (img.shape[0] - img.shape[1]) // 2
            img = img[cutoff:-cutoff, :]
        if min(img.shape[0], img.shape[1]) != img.shape[1]:
            cutoff = (img.shape[1] - img.shape[0]) // 2
            img = img[:, cutoff:-cutoff]
        # rescale images to height and width
        img = cv2.resize(img, (height, width))

        # reshape image to be of order (channels, height, width)
        img = np.moveaxis(img, -1, 0)

        images.append(img)

    return np.array(images)

def convert_poses_to_numpy_array(folder_path, startFrame=0, endFrame=0, summarize_frames=1, use_quat=True, normalize=False):

    def quatPoseToPRYPose(pos):
        quat = pos[3:]
        w = quat[0]
        x = quat[1]
        y = quat[2]
        z = quat[3]

        roll = math.atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z)
        pitch = math.asin(-2.0*(x*z - w*y))
        yaw = math.atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z)

        #convert pitch roll and yaw to angles
        roll = roll * 180 / math.pi
        pitch = pitch * 180 / math.pi
        yaw = yaw * 180 / math.pi

        pos = [pos[0], pos[1], pos[2], roll, pitch, yaw]
        return pos
    
    def quaternion_multiplication(q1, q2):
        q1 = normalize_quaternion(q1)
        q2 = normalize_quaternion(q2)
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return [w, x, y, z]
    
    def quaternion_division(q1, q2):
        q1 = normalize_quaternion(q1)
        q2 = normalize_quaternion(q2)
        w1, x1, y1, z1 = q1
        # form conjugate of q2
        w2, x2, y2, z2 = q2
        q2_conj = [w2, -x2, -y2, -z2]

        # form inverse of q2
        norm_squared = w2*w2 + x2*x2 + y2*y2 + z2*z2
        q2_inv = [q2_conj[0]/norm_squared, q2_conj[1]/norm_squared, q2_conj[2]/norm_squared, q2_conj[3]/norm_squared]

        # multiply q1 with conjugate of q2
        q = quaternion_multiplication(q2_inv, q1)
        return q 

    def normalize_quaternion(q):
        norm = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        q = [q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm]
        return q
    

    labels = []
    quatPoses = []
  
    minValues = [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]
    minDiff = [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]
    maxValues = [-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]
    maxDiff = [-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]
    if use_quat:
        minValues.append(np.inf)
        minDiff.append(np.inf)
        maxValues.append(-np.inf)
        maxDiff.append(-np.inf)
    
    file = open(folder_path+"/poses.txt", "r")
    lines = file.readlines()
    # skip lines until start frame
    for _ in range(startFrame):
        lines.pop(0)
    # read lines until end frame
    # data looks like this:
    # [0.00137941, 0.276708, 1.26097] [-0.780062, -0.195395, 0.14064, -0.577534]
    # this is saved in a 6d array [x, y, z, roll, pitch, yaw]
    prior_label = []
    for _ in range(startFrame, endFrame+1):
        line = lines.pop(0)
        line = line.replace("[", "")
        line = line.replace("]", "")
        line = line.replace(",", "")
        line = line.split()
        label = []
        for i in range(7):
            label.append(float(line[i]))

        quatPoses.append(label)
        if use_quat:
            label = label
        else:
            label = quatPoseToPRYPose(label)

        for i in range(len(label)):
            minValues[i] = min(minValues[i], label[i])
            maxValues[i] = max(maxValues[i], label[i])

        if len(prior_label) == 0:
            prior_label = label
        for i in range(len(label)):
            minDiff[i] = min(minDiff[i], label[i] - prior_label[i])
            maxDiff[i] = max(maxDiff[i], label[i] - prior_label[i])
        prior_label = label
    
        labels.append(label)

    start_pose = [0, 0, 0, 0, 0,0,0]
    for i in range(len(labels[0])):
        start_pose[i] = labels[0][i]
    end_pose = [0, 0, 0, 0, 0,0,0] 
    for i in range(len(labels[-1])):
        end_pose[i] = labels[-1][i]


    if normalize:
        # renormalize all labels by the difference of the respective min max values
        for i in range(len(labels)):
            for j in range(6):
                labels[i][j] = (labels[i][j] - minValues[j]) / (maxValues[j] - minValues[j])
    

    # redefine all labels as the difference between the current and the prior label
    prior_label = []
    for i in range(len(labels)):
        label = labels[i]
        if len(prior_label) == 0:
            prior_label = label
        
        tmp_label = [0, 0, 0, 0, 0, 0, 0]
        if use_quat:
            for i in range(len(label)):
                tmp_label[i] = label[i]
            

            label[3:] = quaternion_division(label[3:], prior_label[3:])
            for i in range(3):
                label[i] = label[i] - prior_label[i]
            print(label)
        else:
            for i in range(6):
                # the actions are actually only the diffs in poses
                tmp_label[i] = label[i]
                label[i] = label[i] - prior_label[i]
        prior_label = tmp_label

    # add up all lables in the summarize_frames 
    new_labels = []
    for i in range(0, len(labels), summarize_frames):
        label = labels[i]

        for j in range(1, summarize_frames):
            if i+j < len(labels):
                if use_quat:
                    label[3:] = quaternion_multiplication(label[3:], labels[i+j][3:])
                    label = [label[0] + labels[i+j][0], label[1] + labels[i+j][1], label[2] + labels[i+j][2], label[3], label[4], label[5], label[6]]
                else:
                    for k in range(6):
                        label[k] += labels[i+j][k]
        new_labels.append(label)

    labels = new_labels

    #print max and min array to a file in the folder
    with open(folder_path+"/min_max_values.txt", "w") as f:
        f.write("minValues: ")
        for i in range(len(minValues)):
            f.write(str(minValues[i]) + " ")
        f.write("\n")
        f.write("maxValues: ")
        for i in range(len(maxValues)):
            f.write(str(maxValues[i]) + " ")
        f.write("\n")


    #print max and min diff array to a file in the folder
    with open(folder_path+"/min_max_diff.txt", "w") as f:
        f.write("minDiff: ")
        for i in range(len(minDiff)):
            f.write(str(minDiff[i]) + " ")
        f.write("\n")
        f.write("maxDiff: ")
        for i in range(len(maxDiff)):
            f.write(str(maxDiff[i]) + " ")
        f.write("\n")

    #write all labels to new file names labelsPRY.txt
    with open(folder_path+"/labelsPRY.txt", "w") as f:
        for label in labels:
            for i in range(len(label)):
                f.write(str(label[i]) + " ")
            f.write("\n")


    if use_quat:
        # test quat fucntions

        q1 = [-0.77925, -0.199869, 0.144179, -0.576224]
        print("Quaternion: ", q1)
        q2 = [0.5, 0.5, 0.5, 0.5]
        print("Quaternion: ", q2)
        q3 = quaternion_multiplication(q1, q2)
        print("Multiplication: ", q3)
        q4 = quaternion_division(q3, q1)
        print("Division: ", q4)


        # test if all actions combined plus start pose result in the end pose
        print("Start pose: ", start_pose)
        print("End pose: ", end_pose)
        for i in range(len(labels)):
            if use_quat:
                start_pose[3:] = quaternion_multiplication(start_pose[3:], labels[i][3:])
                for j in range(3):
                    start_pose[j] += labels[i][j]
            else:
                for j in range(6):
                    start_pose[j] += labels[i][j]
        print("End pose caclulated: ", start_pose)



    return np.array(labels)


def get_number_of_frames(folder_path):
    file = open(folder_path+"/poses.txt", "r")
    num_lines = 0
    lines = file.readlines()
    for line in lines:
        num_lines += 1

    return num_lines


def pickle_the_data(images, labels, folder_path):
    # this method pickles the images and labels together to a file that then contains:
    # 1. images
    # 2. empty
    # 3. labels
    # 4. rewards (set to 0)
    # each of these has another dimension "trajectories" that is of size one for now

    data = [[images], [[]], [labels], [np.zeros(len(labels))]]

    with open(folder_path+"/expert_demos.pkl", "wb") as f:
        pickle.dump(data, f)


def read_pickled_data(folder_path):
    # this method tries to read the pickled data and shows one of the recovered images annotated with the corresponding label
    with open(folder_path+"/expert_demos.pkl", "rb") as f:
        data = pickle.load(f)
        print(data[0][0].shape)
        
        expert_demo, _, expert_demo_labels, rewards = data

    print("Number of images: ", len(expert_demo))
    print("Number of labels: ", len(expert_demo_labels))
    print("Number of rewards: ", len(rewards))
    print("First label: ", expert_demo_labels[0][0])
    print("First reward: ", rewards[0][0])
    print("image shape: ", expert_demo[0][0].shape)
    print("label shape: ", expert_demo_labels[0].shape)

    #rehsap images to be of order (height, width, channels)
    expert_demo[0] = np.moveaxis(expert_demo[0], 1, -1)

    # display a random image
    rand_id = np.random.randint(0, len(expert_demo[0]))
    print("Random image id: ", rand_id)
    img = expert_demo[0][rand_id]
    label = expert_demo_labels[0][rand_id]
    print("Label: ", label)
    cv2.imshow("Image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()





if __name__ == "__main__":
    folder_path = "recordings_1717521458"
    num_frames = get_number_of_frames(folder_path)
    startFrame = 22
    endFrame = 320
    summarize_frames = 8 # this is the number of frames that are combined to one label (all the n labels are added up while the first image is the input)

    startFrame = min(startFrame, num_frames)
    endFrame = min(endFrame, num_frames)

    labels = convert_poses_to_numpy_array(folder_path, startFrame, endFrame, summarize_frames, use_quat=True, normalize=False)
    
    images = convert_images_to_numpy_array(folder_path, startFrame, endFrame, summarize_frames, 84, 84)
    pickle_the_data(images, labels, folder_path)
    read_pickled_data(folder_path)





