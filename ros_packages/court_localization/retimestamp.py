import roslib; roslib.load_manifest('court_localization')
from rospy import Time
import rosbag
#from std_msgs.msg import 
import sys

def main():
    if len(sys.argv) < 3:
        print 'Usage: %s [infile] [outfile]' % sys.argv[0]
    inbag = rosbag.Bag(sys.argv[1], 'r')

    inittime = None

    for topic, msg, t in inbag.read_messages():
        if inittime is None:
            inittime = t

        time = t - inittime
        print topic, '%d.%d' % (time.secs, time.nsecs)

        if time.secs > 5: break

    #outbag = rosbag.Bag(sys.argv[2], 'w')

if __name__ == '__main__':
    main()
