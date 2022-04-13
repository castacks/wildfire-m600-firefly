import pymap3d as pm
import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node("gt_test", anonymous=True)

pub = rospy.Publisher("gt_map", OccupancyGrid, queue_size=100)

rospy.sleep(1)

lat0 = 40.4795317421
lon0 = -79.8945689628

lats = [40.4795699, 40.4794344, 40.47932, 40.4794563, 40.4797508]
lons = [-79.8944861, -79.894423, -79.8943054, -79.8941351, -79.8942941]

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
