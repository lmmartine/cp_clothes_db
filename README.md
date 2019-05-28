# cp_clothes_db

Run the nodes:

```bash
$ roscore
$ roslaunch cp_clothes_db cp_clothes_db_xtion1.launch
$ roslaunch cp_clothes_db cp_clothes_db_xtion2.launch
```
Activate the processing nodes:
```bash
$ rosservice call /xtion1_mask/active "select: true" 
$ rosservice call /xtion2_mask/active "select: true"
```
Play the rosbag
```bash
$ rosbag play towel1_move1.bag
$ rosbag play pant1_move2_topics.bag
```

Now you you see with rqt_image_view the mask published on the topic "/xtion2/mask/image_raw"




