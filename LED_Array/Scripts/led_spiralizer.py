#Spiralizes led footprints and creates zones between pads to create 7 individual strings. 
#exec(open(r"C:\users\ben\documents\led_spiralizer.py").read())
import pcbnew
import math
import cmath
class position(object):
	def __init__(self):
		self.x = 0
		self.y = 0
		self.r = 0
def lr(orient, d):
	left = orient + d
	if left > 1800:
		left = -3600+left 
	right = orient - d
	if right < -1800:
		right = 3600+right

	return left, right
board = pcbnew.GetBoard()
foot = board.GetFootprints()
led = [None]*50
for i in foot:
	if "D" in i.GetReference() and i.GetReference() != "D1":
		
		led[int(i.GetReference()[1:])] = i

led_int = 0
degrees = 0
led.remove(None)
led.remove(None)
dss = [50,40,25]
jss = [0,50,100]
while degrees<360:
	start = position()
	if degrees%60 == 0:
		start.x = 15 * math.sin(math.radians(degrees))
		start.y = 15 * math.cos(math.radians(degrees))
		dss = [20,40,30,(degrees*10)]
	else:
		start.x = 18 * math.sin(math.radians(degrees))
		start.y = 18 * math.cos(math.radians(degrees))
		dss = [40,30,20,(degrees*10)+200]
	start.r = degrees

	led[led_int].SetPosition(pcbnew.wxPointMM(start.x,start.y))  
	led[led_int].SetOrientation(dss[3])
	led_int += 1
	d_step = degrees
	for i in range(0,3):
		d_step += dss[i]
		
		start.x = start.x + (10 * math.sin(math.radians(d_step)))
		start.y = start.y + (10*math.cos(math.radians(d_step)))

		led[led_int].SetPosition(pcbnew.wxPointMM(start.x,start.y)) 
		p1 = led[led_int].GetPosition()
		tespos = led[led_int-1].GetPosition()
		
		p2 = led[led_int-1].GetPosition()
		print(p1,p2)

		v1_theta = math.atan2(p2.y, p1.y)
		v2_theta = math.atan2(p2.x, p1.x)

		ang = (v2_theta - v1_theta) * (180.0 / math.pi)



		print(ang)

		led[led_int].SetOrientation(d_step*10)

		led[led_int].Rotate(led[led_int].GetPosition(),200)
		led_int += 1
	degrees += 30



#zones
def carti(orient):
	x = 1 * math.cos(math.radians(orient/10))
	y = 1 * math.sin(math.radians(orient/10))
	return x,y
def rotate(x,y,d):
	rx = x+math.cos(math.radians(d))
	ry = y+math.sin(math.radians(d))
	return rx,ry
def _mm(m1, m2):
	return pcbnew.wxPoint(m1,m2)
def draw(start, end, layer=pcbnew.User_7):
	a = pcbnew.PCB_SHAPE()
	a.SetShape(pcbnew.S_SEGMENT)
	a.SetStart(start)
	a.SetEnd(end)
	a.SetLayer(layer)
	a.SetWidth(int(0.2*pcbnew.IU_PER_MM))
	board.Add(a)

led_int = 0
for i in led:
	
	if led_int > 46:
		break
	orient = i.GetOrientation()

	l,r = lr(orient, 900)
	pos = i.GetPosition()
	lx,ly = carti(l)
	rx,ry = carti(r)
	draw(_mm(pos.x,pos.y),_mm(pos.x+(ly*2*pcbnew.IU_PER_MM),pos.y+(lx*2*pcbnew.IU_PER_MM)))
	draw(_mm(pos.x,pos.y),_mm(pos.x+(ry*2*pcbnew.IU_PER_MM),pos.y+(rx*2*pcbnew.IU_PER_MM)))

	j = led[led_int+1]
	orient2 = j.GetOrientation()
	l2,r2 = lr(orient2,900)
	pos2 = j.GetPosition()
	lx2,ly2 = carti(l2)
	rx2,ry2 = carti(r2)
	
	

	offsets = [-.11,2.41,1.55,-1.2]
	offset =0
	offset2 = 0
	yseter = False
	z = pcbnew.ZONE(board)
	if (led_int % 4) == 0:
	


		z.SetPriority(int((led_int/4)+20))

		yset = 1.4
		offset = offsets[0]
		offset2 = offsets[1]

		#breaks for some reason
		fixer = True
		if led_int == len(led)-4:
			ref = led[0]
		else:

			ref = led[led_int + 4]
		reforient = ref.GetOrientation() 
		refpos = ref.GetPosition()
		xdxxd,l45 = lr(reforient,900)
		l45x,l45y = carti(l45)
		
		
		
		
	elif (led_int % 4) == 1:

		z.SetPriority(11)
		offset = offsets[1]
		offset2 = offsets[2]
		yset = 0

	elif (led_int % 4) == 2:
		ry2 *= 0.55
		rx2 *= 0.55
		ly2 *= 0.89
		lx2 *= 0.89
		z.SetPriority(42)
		offset = offsets[2]
		offset2 = offsets[3]
		yset = 0
		yseter = True

	else:
		led_int += 1
		continue

		offset = offsets[3]
		offset2 = offsets[3]

		

	
	
	z.SetLayer(pcbnew.F_Cu)
	z.SetIsFilled(True)
	z.SetNetCode(i.Pads()[1].GetNetCode())
	ol = z.Outline()
	ol.NewOutline()
	
	#fixer
	if fixer:
		ol.Append(int(refpos.x+(l45y*4*pcbnew.IU_PER_MM)),int(refpos.y+(l45x*4*pcbnew.IU_PER_MM)))
		fixer = False
	
	ol.Append(int(pos.x+(ly*(5+offset+yset)*pcbnew.IU_PER_MM)),int(pos.y+(lx*(5+offset+yset)*pcbnew.IU_PER_MM)))
	ol.Append(int(pos.x+(ry*(5+offset)*pcbnew.IU_PER_MM)),int(pos.y+(rx*(5+offset)*pcbnew.IU_PER_MM)))
	
	ol.Append(int(pos2.x+(ry2*(5+offset2)*pcbnew.IU_PER_MM)),int(pos2.y+(rx2*(5+offset2)*pcbnew.IU_PER_MM)))
	ol.Append(int(pos2.x+(ly2*(5+offset2)*pcbnew.IU_PER_MM)),int(pos2.y+(lx2*(5+offset2)*pcbnew.IU_PER_MM)))
	
	
	board.Add(z)

	led_int += 1

j = board.GetZoneList()
filler = pcbnew.ZONE_FILLER(board)
filler.Fill(board.Zones())

pcbnew.Refresh()
