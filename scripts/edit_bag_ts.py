# script usage: edit the timestamp in the tf msg in the bag
import rosbag

bag = rosbag.Bag('/home/jingyu/frog/zed_test_ws/src/d4_r10_zed.bag', 'r')
outbag = rosbag.Bag('/home/jingyu/frog/zed_test_ws/src/d4_r10_zed_fixed_ts.bag', 'w')

for topic, msg, t in bag.read_messages():
    if topic == "/tf" and msg.transforms:
        outbag.write(topic, msg, msg.transforms[0].header.stamp)
    else:
        outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)

bag.close()

