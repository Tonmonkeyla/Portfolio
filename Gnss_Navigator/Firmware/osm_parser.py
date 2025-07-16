import math
from pathlib import Path

file = open("C:\\Users\\ben\\Desktop\\osm\\output3.osm", encoding="utf8")

def remove_duplicates(nodes):
    nodes.sort(key=lambda x: x.lat)
    for node in range(len(nodes)-1):
        nodex = 1
        if node+nodex < len(nodes):
            while (nodes[node+nodex].lat == nodes[node].lat):
                if (nodes[node+nodex].lon == nodes[node].lon):
                    nodes.pop(node+nodex)
                    if node+nodex >= len(nodes):
                        break
                elif node+nodex+1 < len(nodes):
                    nodex += 1
                else:
                    break



#TODO 3 seperate zoom detail level dictionaries, fill with nodes at 3 different levels of precision, prune, output to binary files in same directory as directories with same precision.
tile_dict3 = {}
tile_dict2 = {}
tile_dict1 = {}
class Node:
    #9999 jigabits of ram
    def __init__(self, lat, lon, precision=3):
        self.lat = round(float(lat), precision)
        self.lon = round(float(lon), precision)
        self.tile3 = (math.floor(float(lat)*10), math.floor(float(lon)*10))
        self.tile2 = (math.floor(float(lat)*1), math.floor(float(lon)*1))
        self.tile1 = (math.floor(float(lat)/10)*10, math.floor(float(lon)/10)*10)
    def __setprecision__(self, precision):
        self.lat = round(float(self.lat), precision)
        self.lon = round(float(self.lon), precision)


for line in file:
    if (("node" in line) and ("lat" in line) and ("lon" in line)):
        lat_index = line.index("lat=\"")
        lat_index += 5
        lat_out = ""
        while (line[lat_index] != "\""):
            lat_out += line[lat_index]
            lat_index += 1
        lon_index = line.index("lon=\"")
        lon_index += 5
        lon_out = ""
        while (line[lon_index] != "\""):
            lon_out += line[lon_index]
            lon_index += 1
        New_Node = Node(float(lat_out), float(lon_out), 3)
        if New_Node.tile3 in tile_dict3:
            tile_dict3[New_Node.tile3].append(New_Node)
        else:
            tile_dict3[New_Node.tile3] = [New_Node]
        if New_Node.tile2 in tile_dict2:
            tile_dict2[New_Node.tile2].append(New_Node)
        else:
            tile_dict2[New_Node.tile2] = [New_Node]
        if New_Node.tile1 in tile_dict1:
            tile_dict1[New_Node.tile1].append(New_Node)
        else:
            tile_dict1[New_Node.tile1] = [New_Node]
file.close()




for tile, nodes in tile_dict3.items():
    remove_duplicates(nodes)
    tile_dir_10deg = ((tile[0]//100)*10, (tile[1]//100)*10)
    tile_dir_1deg = (tile[0]//10, tile[1]//10)
    Path("C:\\Users\\ben\\Desktop\\osm\\tile_test2\\" + str(tile_dir_10deg) + "\\" + str(tile_dir_1deg)).mkdir(parents=True, exist_ok=True)
    file = open(("C:\\Users\\ben\\Desktop\\osm\\tile_test2\\" + str(tile_dir_10deg) + "\\" + str(tile_dir_1deg) + "\\" + str(tile) + ".bin"), "wb")
    file.write(len(nodes).to_bytes(4, 'big'))
    for node in nodes:
        file.write(int(node.lat*1000).to_bytes(4,'big',signed=True))
        file.write(int(node.lon*1000).to_bytes(4,'big',signed=True))
    file.close()

for tile, nodes in tile_dict2.items():
    remove_duplicates(nodes)
    tile_dir_10deg = ((tile[0]//100)*10, (tile[1]//100)*10)
    tile_dir_1deg = (tile[0]//10, tile[1]//10)
    Path("C:\\Users\\ben\\Desktop\\osm\\tile_test2\\" + str(tile_dir_10deg) + "\\" + str(tile_dir_1deg)).mkdir(parents=True, exist_ok=True)
    file = open(("C:\\Users\\ben\\Desktop\\osm\\tile_test2\\" + str(tile_dir_10deg) + "\\" + str(tile_dir_1deg) + "\\" + str(tile) + ".bin"), "wb")
    file.write(len(nodes).to_bytes(4, 'big'))
    for node in nodes:
        file.write(int(node.lat*1000).to_bytes(4,'big',signed=True))
        file.write(int(node.lon*1000).to_bytes(4,'big',signed=True))
    file.close()


