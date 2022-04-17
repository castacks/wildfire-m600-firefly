import pymap3d as pm
import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node("gt_test", anonymous=True)

pub = rospy.Publisher("gt_map", OccupancyGrid, queue_size=100)

rospy.sleep(1)

lat0 = 40.441751
lon0 = -79.9441072

lats = [40.4419103, 40.441784, 40.4419499, 40.4418931, 40.4420147]
lons = [-79.9441928, -79.944385, -79.9443914, -79.9446682, -79.9448107]

resolution = 0.5
minX = -100
maxX = 100
minY = -100
maxY = 100

output = OccupancyGrid()
output.header.frame_id = "world"
output.info.resolution = 0.5
output.info.width = 400
output.info.height = 400
output.info.origin.position.x = -100
output.info.origin.position.y = -100
output.data = [0] * (400*400)

for lat, lon in zip(lats, lons):
    xt, yt, _ = pm.enu.geodetic2enu(lat, lon, 279, lat0, lon0, 279)

    gtRow = int((yt-minY)/resolution)
    gtCol = int((xt-minX)/resolution)

    gtBin = gtCol + gtRow * 400

    output.data[gtBin] = 100

    print(xt)
    print(yt)
    print(gtBin)
    print()

pub.publish(output)
rospy.sleep(1)
