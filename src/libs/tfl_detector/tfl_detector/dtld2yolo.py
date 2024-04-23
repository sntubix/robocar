# Copyright (c) 2024 University of Luxembourg, MIT License

import os
import numpy as np
import cv2 as cv
import json


def format(dtld_labels_path, data_path, mode):
    with open(dtld_labels_path) as file:
        images = json.load(file)

    images_path = data_path + "/images/" + mode
    if not os.path.exists(images_path):
        os.makedirs(images_path)

    labels_path = data_path + "/labels/" + mode
    if not os.path.exists(labels_path):
        os.makedirs(labels_path)

    for image in images["images"]:
        image_path = "dtld/" + image["image_path"]
        image_name = os.path.splitext(os.path.basename(image_path))[0]
        if not os.path.isfile(image_path):
            continue

        # write image
        img = cv.imread(image_path, cv.IMREAD_UNCHANGED)
        img = cv.cvtColor(img, cv.COLOR_BAYER_GB2BGR)
        img = np.right_shift(img, 4)
        img = img.astype(np.uint8)
        cv.imwrite(images_path + "/" + image_name + ".jpg", img)

        # write labels
        label_file = open(labels_path + "/" + image_name + ".txt", "a")
        for label in image["labels"]:
            attributes = label["attributes"]
            if attributes["relevance"] != "relevant":
                continue

            c = -1
            if attributes["state"] == "green":
                c = 0
            if attributes["state"] == "yellow":
                c = 1
            if attributes["state"] == "red":
                c = 2
            if c == -1:
                continue

            img_width = np.shape(img)[1]
            img_height = np.shape(img)[0]
            x = (label["x"] + (label["w"] / 2)) / img_width
            y = (label["y"] + (label["h"] / 2)) / img_height
            w = label["w"] / img_width
            h = label["h"] / img_height
            label_file.write(str(c) + " " + str(x) + " " + str(y)
                             + " " + str(w) + " " + str(h) + "\n")
        label_file.close()


if __name__ == "__main__":
    format(os.getcwd() + "/dtld/dtld_train.json", "./dataset", "train")
    format(os.getcwd() + "/dtld/dtld_test.json", "./dataset", "val")
