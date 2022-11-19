import utm
import argparse
import pprint
from bs4 import BeautifulSoup
import numpy as np

def parse_folder(folder):
    names = folder.find_all("name")

    origin_lat, origin_lon = find_placemark_lat_lon(names, "origin")

    start_lat, start_lon = find_placemark_lat_lon(names[1:], "start_pose")

    origin_x, origin_y, _, _ = utm.from_latlon(origin_lat, origin_lon)
    start_x, start_y, _, _ = utm.from_latlon(start_lat, start_lon)
    start_x_relative_to_origin = start_x - origin_x
    start_y_relative_to_origin = start_y - origin_y
    
    hole_dict = {"holes": []}
    
    for name in names[1:]:
        if name.next_element == "outer_polygon":
            print("Found search bounds")
            placemark = name.parent
            points_dict = extract_cartesian_points(placemark, origin_lat, origin_lon)
            outer_polygon_dict = {"outer_polygon": points_dict}
            # pprint.pprint(outer_polygon_dict)
        elif "hole" in name.next_element:
            placemark = name.parent
            points_dict = extract_cartesian_points(placemark, origin_lat, origin_lon)
            hole_dict["holes"].append(points_dict)
    ret = {}
    ret.update(outer_polygon_dict)
    ret.update(hole_dict)
    return ret

def approx_cartesian_offset_meters(lat1, lon1, lat2, lon2):
    """
    lat1: origin latitude
    lon1: origin longitude
    """
    # https://stackoverflow.com/a/14176034
    east1, north1, _, _ = lat1_utm = utm.from_latlon(lat1, lon1)
    east2, north2, _, _ = lat1_utm = utm.from_latlon(lat2, lon2)

    # already in meters
    x = east2 - east1
    y = north2 - north1

    # # https://stackoverflow.com/a/19356480
    # latMid = lat1 * math.pi /180
    
    # m_per_deg_lat = 111132.954 - 559.822 * math.cos( 2 * latMid ) + 1.175 * math.cos( 4 * latMid);
    # m_per_deg_lon = 111132.954 * math.cos ( latMid );
    
    # # https://gis.stackexchange.com/a/260673
    
    # x = (lon2 - lon1) * m_per_deg_lon
    # y = (lat2 - lat1) * m_per_deg_lat
    
    print("adding points  ", x, "  ", y)
    return x, y

def extract_cartesian_points(placemark, start_lat, start_lon):
    lon_lat_alt_list = str(placemark.find("coordinates").next_element).strip().split(" ")
    ret = {"points": []}
    for lon_lat_alt in lon_lat_alt_list:
        lon, lat, alt = lon_lat_alt.split(",")
        lon, lat, alt = float(lon), float(lat), float(alt)
        
        x, y = approx_cartesian_offset_meters(start_lat, start_lon, lat, lon)
        ret["points"].append({"x": x, "y":y, "z": 0})
    return ret

def find_placemark_lat_lon(names, search_name):
    latitude = None
    longitude = None
    
    for name in names:
        if name.next_element == search_name:
            placemark = name.parent
            longitude, latitude, altitude = str(placemark.find("coordinates").next_element).strip().split(",")
            longitude, latitude, altitude = float(longitude), float(latitude), float(altitude)
            
#             altitutde = float(placemark.find("altitute").next_element)
            print("Found lat lon for %s at %f,%f" % (search_name, latitude, longitude))
            break
    else:
        raise ValueError("Could not find %s lat lon" % search_name)
    
    return latitude, longitude

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("kml_file_path", type=str)
    args = parser.parse_args()

    with open(args.kml_file_path, "r") as f:
        kml_file = f.read()
    
    soup = BeautifulSoup(kml_file, "xml")
    folders = soup.find_all("Folder")

    print("Found {} mission folders".format(len(folders)))

    for folder in folders:
        mission_name = str(folder.find("name").next_element)
        print("\n===============================")
        print("Writing mission " + mission_name + " to yaml file")
        plan_request_dict = parse_folder(folder)
        pprint.pprint(plan_request_dict)
        np.save(mission_name + ".npy", plan_request_dict)
        print("===============================\n")

