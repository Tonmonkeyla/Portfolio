#exec(open(r"C:\users\ben\documents\screw_adder.py").read())
import pcbnew
import math

board = pcbnew.GetBoard()
foot = board.GetFootprints()
screws = []
for i in foot:
	if "H" in i.GetReference():
		screws.append(i)
degrees = 15
for i in screws:
	start.x = 17.5 * math.sin(math.radians(degrees))
	start.y = 17.5 * math.cos(math.radians(degrees))
	i.SetPosition(pcbnew.wxPointMM(start.x,start.y))
	degrees = degrees - 120
pcbnew.Refresh()