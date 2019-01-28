
# Run this script, to rename all the images in dataset_dir (specified by default.yaml) to the required filenames.

import yaml # for loading .yaml file
import os # for getting/setting filenames

# Open config file
stream = open("config/default.yaml", "r")

# !!!!!
# If the first sentence is "%YAML:1.0", skip it
# !!!!!
stream.readline() 

# Load yaml contents as a "dict"
doc = yaml.load(stream)

# Print all items
PRITN_ALL_ITEMS=False
if PRITN_ALL_ITEMS:
    for k,v in doc.items():
        print k, "->", v, " ", type(v)
    print "\n",

# Get the original image filenames
valid_image_suffixs = [".jpg",".gif",".png",".tga"]
dataset_dir = doc["dataset_dir"]
dataset_dir_filenames = sorted(os.listdir(dataset_dir))
src_filenames=list()
for filename in dataset_dir_filenames:
    suffix = os.path.splitext(filename)[1] # get suffix
    if suffix.lower() not in valid_image_suffixs:
        continue
    src_filenames.append(os.path.join(dataset_dir,filename))
print "\nPrinting first 5 image filenames:\n"
for s in src_filenames[:min(5,len(src_filenames))]:
    print s

# Set the target image filename
dataset_dir = doc["dataset_dir"]
s1="rgb_"
s2="{:05d}"
s3=".png"
dst_filename=s1+s2+s3
print "\ndst_filename format: "+dst_filename # rgb_{:05d}.png

# Rename all files
cnt=0
for s in src_filenames:
    os.rename(
        s, 
        os.path.join(dataset_dir,dst_filename.format(cnt))
    )
    cnt+=1
print "\nRename completed."
